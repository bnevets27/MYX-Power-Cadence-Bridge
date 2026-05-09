/*
 * ESP32 BLE Bike Power Meter Bridge
 * ==================================
 *
 * Connects to a MYX/BKSNSR power meter sensor that XOR-masks its BLE
 * Cycling Power Measurement (0x2A63) data with 0xAA, decodes it, and
 * re-broadcasts it as standard BLE Cycling Power + Cadence services.
 *
 * This build also exposes a local calibration portal. The portal can attempt
 * the OEM-style zero/tare procedure through the sensor's Cycling Power Control
 * Point, then provides local correction as a fallback for sensors that cannot
 * or will not tune themselves.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

// Configuration
static const char* TARGET_SENSOR_NAME = "BKSNSR";
static const char* BRIDGE_NAME = "MYX Bridge";
static const char* CAL_AP_NAME = "MYX-Bridge-Calibrate";
static const char* CAL_AP_PASS = "";
static const char* FW_VERSION = "1.1.0-cal";

static const uint8_t XOR_KEY = 0xAA;
static const float SMOOTHING_FACTOR = 0.5f;
static const uint32_t DROPOUT_MS = 2500;
static const uint32_t TUNE_TIMEOUT_MS = 6000;

static const uint8_t CP_OP_START_ENHANCED_OFFSET_COMPENSATION = 0x10;
static const uint8_t CP_OP_RESPONSE_CODE = 0x20;
static const uint8_t CP_RESPONSE_SUCCESS = 0x01;
static const uint8_t CP_RESPONSE_NOT_SUPPORTED = 0x02;
static const uint8_t CP_RESPONSE_OPERATION_FAILED = 0x04;

// Standard Bluetooth UUIDs
static BLEUUID CPS_SERVICE_UUID((uint16_t)0x1818);
static BLEUUID CPS_MEASUREMENT_UUID((uint16_t)0x2A63);
static BLEUUID CPS_FEATURE_UUID((uint16_t)0x2A65);
static BLEUUID SENSOR_LOCATION_UUID((uint16_t)0x2A5D);
static BLEUUID CPS_CONTROL_POINT_UUID((uint16_t)0x2A66);
static BLEUUID BATTERY_SERVICE_UUID((uint16_t)0x180F);
static BLEUUID BATTERY_LEVEL_UUID((uint16_t)0x2A19);
static BLEUUID CSC_SERVICE_UUID((uint16_t)0x1816);
static BLEUUID CSC_MEASUREMENT_UUID((uint16_t)0x2A5B);
static BLEUUID CSC_FEATURE_UUID((uint16_t)0x2A5C);

// Calibration settings stored in NVS.
struct CalibrationSettings {
    bool correctionEnabled;
    bool rawDiagnosticsEnabled;
    float localZeroOffset;
    float scaleFactor;
    float cadence[4];
    float cadenceOffset[4];
};

static CalibrationSettings calibration = {
    false,
    false,
    0.0f,
    1.0f,
    {60.0f, 70.0f, 80.0f, 90.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

enum TuneState {
    TUNE_IDLE,
    TUNE_NO_CONTROL_POINT,
    TUNE_STANDARD_SENT,
    TUNE_ENHANCED_SENT,
    TUNE_SUCCESS,
    TUNE_NOT_SUPPORTED,
    TUNE_FAILED,
    TUNE_TIMEOUT
};

// BLE client state: connection to the real sensor.
static BLEAdvertisedDevice* sensorDevice = nullptr;
static BLEClient* bleClient = nullptr;
static BLERemoteCharacteristic* remotePowerControlPoint = nullptr;
static bool sensorConnected = false;
static bool remoteControlPointReady = false;
static bool doConnect = false;
static bool doScan = true;
static bool scanRunning = false;
static bool calPortalStarted = false; // forward-declared so callbacks can reach it

// BLE server state: what apps connect to.
static BLEServer* bleServer = nullptr;
static BLECharacteristic* powerMeasurementChar = nullptr;
static BLECharacteristic* powerControlPointChar = nullptr;
static BLECharacteristic* cscMeasurementChar = nullptr;
static BLECharacteristic* batteryLevelChar = nullptr;
static int appConnectionCount = 0;

// Web and storage.
static Preferences preferences;
static WebServer webServer(80);

// Latest decoded/corrected data for logs and the calibration UI.
static volatile uint8_t lastBattery = 0xFF;
static volatile int16_t lastRawPower = 0;
static volatile int16_t lastZeroedPower = 0;
static volatile int16_t lastCorrectedPower = 0;
static volatile int16_t lastBroadcastPower = 0;
static volatile uint16_t lastCrankRev = 0;
static volatile uint16_t lastCrankTime = 0;
static volatile uint16_t lastEnergy = 0;
static volatile float lastCadence = 0.0f;
static volatile bool lastDropped = false;
static volatile float lastAppliedCadenceOffset = 0.0f;

// Cadence and smoothing state.
static uint16_t prevCrankRev = 0;
static uint16_t prevCrankTime = 0;
static float smoothedPower = 0.0f;
static uint32_t lastCrankMoveMs = 0;

// Sensor tune state.
static volatile TuneState tuneState = TUNE_IDLE;
static volatile uint8_t tuneLastOp = 0;
static volatile uint8_t tuneLastResponse = 0;
static volatile int16_t tuneReportedOffset = 0;
static volatile uint8_t tuneFailureReason = 0;
static bool tuneRequested = false;
static uint32_t tuneStartedMs = 0;

#ifndef LED_PIN
#define LED_PIN 2
#endif
static unsigned long lastBlinkTime = 0;
static bool ledState = false;

static const char* tuneStateText(TuneState state) {
    switch (state) {
        case TUNE_IDLE: return "idle";
        case TUNE_NO_CONTROL_POINT: return "sensor control point unavailable";
        case TUNE_STANDARD_SENT: return "standard tune requested";
        case TUNE_ENHANCED_SENT: return "enhanced tune requested";
        case TUNE_SUCCESS: return "tune complete";
        case TUNE_NOT_SUPPORTED: return "sensor tune not supported";
        case TUNE_FAILED: return "sensor tune failed";
        case TUNE_TIMEOUT: return "sensor tune timed out";
        default: return "unknown";
    }
}

static void loadCalibration() {
    preferences.begin("cal", true);
    calibration.correctionEnabled = preferences.getBool("enabled", calibration.correctionEnabled);
    calibration.rawDiagnosticsEnabled = preferences.getBool("rawdiag", calibration.rawDiagnosticsEnabled);
    calibration.localZeroOffset = preferences.getFloat("zero", calibration.localZeroOffset);
    calibration.scaleFactor = preferences.getFloat("scale", calibration.scaleFactor);
    for (int i = 0; i < 4; i++) {
        char key[12];
        snprintf(key, sizeof(key), "cad%d", i);
        calibration.cadence[i] = preferences.getFloat(key, calibration.cadence[i]);
        snprintf(key, sizeof(key), "off%d", i);
        calibration.cadenceOffset[i] = preferences.getFloat(key, calibration.cadenceOffset[i]);
    }
    preferences.end();

    if (!isfinite(calibration.scaleFactor) || calibration.scaleFactor < 0.05f || calibration.scaleFactor > 50.0f) {
        calibration.scaleFactor = 1.0f;
    }
}

static void saveCalibration() {
    preferences.begin("cal", false);
    preferences.putBool("enabled", calibration.correctionEnabled);
    preferences.putBool("rawdiag", calibration.rawDiagnosticsEnabled);
    preferences.putFloat("zero", calibration.localZeroOffset);
    preferences.putFloat("scale", calibration.scaleFactor);
    for (int i = 0; i < 4; i++) {
        char key[12];
        snprintf(key, sizeof(key), "cad%d", i);
        preferences.putFloat(key, calibration.cadence[i]);
        snprintf(key, sizeof(key), "off%d", i);
        preferences.putFloat(key, calibration.cadenceOffset[i]);
    }
    preferences.end();
}

static void resetCalibration() {
    calibration = {
        false,
        false,
        0.0f,
        1.0f,
        {60.0f, 70.0f, 80.0f, 90.0f},
        {0.0f, 0.0f, 0.0f, 0.0f}
    };
    saveCalibration();
}

static float getCadenceOffset(float cadence) {
    if (cadence <= 0.0f) return 0.0f;

    if (cadence <= calibration.cadence[0]) {
        float dCad = calibration.cadence[1] - calibration.cadence[0];
        if (fabs(dCad) < 0.001f) return calibration.cadenceOffset[0];
        float slope = (calibration.cadenceOffset[1] - calibration.cadenceOffset[0]) / dCad;
        return calibration.cadenceOffset[0] + slope * (cadence - calibration.cadence[0]);
    }

    for (int i = 0; i < 3; i++) {
        if (cadence >= calibration.cadence[i] && cadence <= calibration.cadence[i + 1]) {
            float dCad = calibration.cadence[i + 1] - calibration.cadence[i];
            if (fabs(dCad) < 0.001f) return calibration.cadenceOffset[i];
            float t = (cadence - calibration.cadence[i]) / dCad;
            return calibration.cadenceOffset[i] + t * (calibration.cadenceOffset[i + 1] - calibration.cadenceOffset[i]);
        }
    }

    float dCad = calibration.cadence[3] - calibration.cadence[2];
    if (fabs(dCad) < 0.001f) return calibration.cadenceOffset[3];
    float slope = (calibration.cadenceOffset[3] - calibration.cadenceOffset[2]) / dCad;
    return calibration.cadenceOffset[3] + slope * (cadence - calibration.cadence[3]);
}

static int16_t clampPowerToInt16(float watts) {
    if (!isfinite(watts)) return 0;
    if (watts < 0.0f) return 0;
    if (watts > 32767.0f) return 32767;
    return (int16_t)(watts + 0.5f);
}

static void xorDecode(uint8_t* data, size_t len, uint8_t key) {
    for (size_t i = 0; i < len; i++) {
        data[i] ^= key;
    }
}

static float calcCadence(uint16_t currRev, uint16_t currTime, uint16_t prevRev, uint16_t prevTime) {
    uint16_t dRev = (uint16_t)(currRev - prevRev);
    uint16_t dTime = (uint16_t)(currTime - prevTime);
    if (dRev == 0 || dTime == 0) return 0.0f;
    return 60.0f * 1024.0f * (float)dRev / (float)dTime;
}

static void jsonEscapePrint(const char* text) {
    while (*text) {
        char c = *text++;
        if (c == '"' || c == '\\') webServer.sendContent("\\");
        char out[2] = {c, '\0'};
        webServer.sendContent(out);
    }
}

static void remoteControlPointCallback(
    BLERemoteCharacteristic* pChar,
    uint8_t* pData,
    size_t length,
    bool isNotify)
{
    if (length < 3 || pData[0] != CP_OP_RESPONSE_CODE) return;

    tuneLastOp = pData[1];
    tuneLastResponse = pData[2];
    tuneFailureReason = 0;
    tuneReportedOffset = 0;

    if (length >= 5) {
        tuneReportedOffset = (int16_t)(pData[3] | (pData[4] << 8));
    } else if (length >= 4) {
        tuneFailureReason = pData[3];
    }

    if (tuneLastResponse == CP_RESPONSE_SUCCESS) {
        tuneState = TUNE_SUCCESS;
        Serial.printf("[TUNE] Sensor tune complete. Offset result: %d\n", tuneReportedOffset);
    } else if (tuneLastResponse == CP_RESPONSE_NOT_SUPPORTED) {
        tuneState = TUNE_NOT_SUPPORTED;
        Serial.printf("[TUNE] Sensor rejected op 0x%02X as not supported\n", tuneLastOp);
    } else if (tuneLastResponse == CP_RESPONSE_OPERATION_FAILED) {
        tuneState = TUNE_FAILED;
        Serial.printf("[TUNE] Sensor tune failed for op 0x%02X, reason 0x%02X\n", tuneLastOp, tuneFailureReason);
    } else {
        tuneState = TUNE_FAILED;
        Serial.printf("[TUNE] Sensor tune response for op 0x%02X: 0x%02X\n", tuneLastOp, tuneLastResponse);
    }
}

static void batteryNotifyCallback(
    BLERemoteCharacteristic* pChar,
    uint8_t* pData,
    size_t length,
    bool isNotify)
{
    if (length < 1) return;
    lastBattery = pData[0];
    Serial.printf("[BATTERY] Level: %d%%\n", lastBattery);
    if (batteryLevelChar != nullptr) {
        batteryLevelChar->setValue(pData, 1);
        batteryLevelChar->notify();
    }
}

static bool writeSensorTuneOp(uint8_t opCode, TuneState waitingState) {
    if (!remoteControlPointReady || remotePowerControlPoint == nullptr) {
        tuneState = TUNE_NO_CONTROL_POINT;
        Serial.println("[TUNE] Sensor Cycling Power Control Point unavailable");
        return false;
    }

    uint8_t payload[1] = {opCode};
    tuneLastOp = opCode;
    tuneLastResponse = 0;
    tuneReportedOffset = 0;
    tuneFailureReason = 0;
    tuneState = waitingState;
    tuneStartedMs = millis();
    remotePowerControlPoint->writeValue(payload, sizeof(payload), true);
    Serial.printf("[TUNE] Sent sensor tune op 0x%02X\n", opCode);
    return true;
}

static void serviceTuneState() {
    if (tuneRequested) {
        tuneRequested = false;
        writeSensorTuneOp(CP_OP_START_ENHANCED_OFFSET_COMPENSATION, TUNE_ENHANCED_SENT);
    }

    if ((tuneState == TUNE_STANDARD_SENT || tuneState == TUNE_ENHANCED_SENT) &&
        millis() - tuneStartedMs > TUNE_TIMEOUT_MS) {
        tuneState = TUNE_TIMEOUT;
        Serial.println("[TUNE] Enhanced tune timed out");
    }
}

class BridgeServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        appConnectionCount++;
        Serial.printf("[SERVER] App connected! (%d total)\n", appConnectionCount);
        BLEDevice::startAdvertising();
    }

    void onDisconnect(BLEServer* pServer) override {
        appConnectionCount--;
        if (appConnectionCount < 0) appConnectionCount = 0;
        Serial.printf("[SERVER] App disconnected. (%d remaining)\n", appConnectionCount);
        BLEDevice::startAdvertising();
    }
};

class ControlPointCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() == 0) return;

        uint8_t opCode = (uint8_t)value[0];
        uint8_t response[3] = {CP_OP_RESPONSE_CODE, opCode, CP_RESPONSE_NOT_SUPPORTED};
        pCharacteristic->setValue(response, sizeof(response));
        pCharacteristic->indicate();
        Serial.printf("[SERVER] App Control Point op 0x%02X -> not supported\n", opCode);
    }
};

static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify)
{
    if (length < 10) {
        Serial.printf("[SENSOR] Short packet (%d bytes), skipping\n", length);
        return;
    }

    uint8_t decoded[20];
    size_t copyLen = (length > sizeof(decoded)) ? sizeof(decoded) : length;
    memcpy(decoded, pData, copyLen);
    xorDecode(decoded, copyLen, XOR_KEY);

    int16_t rawPower = (int16_t)(decoded[2] | (decoded[3] << 8));
    uint16_t cRev = (uint16_t)(decoded[4] | (decoded[5] << 8));
    uint16_t cTime = (uint16_t)(decoded[6] | (decoded[7] << 8));
    uint16_t energy = (uint16_t)(decoded[8] | (decoded[9] << 8));

    if (cRev != prevCrankRev) {
        lastCrankMoveMs = millis();
    }

    bool dropped = (millis() - lastCrankMoveMs > DROPOUT_MS);
    int16_t zeroedPower = rawPower;
    int16_t correctedPower = rawPower;
    int16_t broadcastPower = 0;
    float cadence = 0.0f;
    float cadenceOffset = 0.0f;

    if (dropped) {
        smoothedPower = 0.0f;
    } else {
        cadence = calcCadence(cRev, cTime, prevCrankRev, prevCrankTime);
        if (!(cadence > 0.0f && cadence < 200.0f)) {
            cadence = (cRev != prevCrankRev) ? lastCadence : 0.0f;
        }

        float workingPower = (float)rawPower;
        if (calibration.correctionEnabled) {
            workingPower -= calibration.localZeroOffset;
            zeroedPower = (int16_t)roundf(workingPower);
            if (cadence > 0.0f) {
                cadenceOffset = getCadenceOffset(cadence);
                workingPower -= cadenceOffset;
            }
            workingPower *= calibration.scaleFactor;
        }

        correctedPower = clampPowerToInt16(workingPower);
        smoothedPower = (correctedPower * SMOOTHING_FACTOR) + (smoothedPower * (1.0f - SMOOTHING_FACTOR));
        broadcastPower = clampPowerToInt16(smoothedPower);
    }

    lastRawPower = rawPower;
    lastZeroedPower = zeroedPower;
    lastCorrectedPower = dropped ? 0 : correctedPower;
    lastBroadcastPower = broadcastPower;
    lastCrankRev = cRev;
    lastCrankTime = cTime;
    lastEnergy = energy;
    lastCadence = dropped ? 0.0f : cadence;
    lastDropped = dropped;
    lastAppliedCadenceOffset = cadenceOffset;

    prevCrankRev = cRev;
    prevCrankTime = cTime;

    uint8_t outBuf[8];
    uint16_t outFlags = 0x0020;
    outBuf[0] = outFlags & 0xFF;
    outBuf[1] = (outFlags >> 8) & 0xFF;
    outBuf[2] = (uint8_t)(broadcastPower & 0xFF);
    outBuf[3] = (uint8_t)((broadcastPower >> 8) & 0xFF);
    outBuf[4] = cRev & 0xFF;
    outBuf[5] = (cRev >> 8) & 0xFF;
    outBuf[6] = cTime & 0xFF;
    outBuf[7] = (cTime >> 8) & 0xFF;

    if (powerMeasurementChar != nullptr) {
        powerMeasurementChar->setValue(outBuf, sizeof(outBuf));
        powerMeasurementChar->notify();
    }

    if (cscMeasurementChar != nullptr) {
        uint8_t cscBuf[5];
        cscBuf[0] = 0x02;
        cscBuf[1] = cRev & 0xFF;
        cscBuf[2] = (cRev >> 8) & 0xFF;
        cscBuf[3] = cTime & 0xFF;
        cscBuf[4] = (cTime >> 8) & 0xFF;
        cscMeasurementChar->setValue(cscBuf, sizeof(cscBuf));
        cscMeasurementChar->notify();
    }

    static int16_t prevPrintPower = -999;
    static uint16_t prevPrintRev = 0xFFFF;
    if (broadcastPower != prevPrintPower || cRev != prevPrintRev || dropped) {
        if (calibration.rawDiagnosticsEnabled) {
            Serial.printf("[DATA] Raw: %d W | Zeroed: %d W | Corrected: %d W | Broadcast: %d W | Cadence: %.1f RPM | Offset: %.1f W | Scale: %.3f | Rev: %u | Energy: %u kJ%s\n",
                          rawPower, zeroedPower, correctedPower, broadcastPower, (double)lastCadence,
                          (double)cadenceOffset, (double)calibration.scaleFactor, cRev, energy,
                          dropped ? " | DROPPED" : "");
        } else {
            Serial.printf("[DATA] Power: %4d W | Cadence: %5.1f RPM | CrankRev: %5u | Energy: %5u kJ%s\n",
                          broadcastPower, (double)lastCadence, cRev, energy,
                          dropped ? " | DROPPED" : "");
        }
        prevPrintPower = broadcastPower;
        prevPrintRev = cRev;
    }
}

class SensorClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        Serial.println("[SENSOR] Connected to power meter!");
        sensorConnected = true;
    }

    void onDisconnect(BLEClient* pclient) override {
        Serial.println("[SENSOR] Disconnected from power meter!");
        sensorConnected = false;
        remoteControlPointReady = false;
        remotePowerControlPoint = nullptr;
        doScan = true;

        // Shut down the WiFi AP so BLE scanning runs without radio
        // contention. The AP will be restarted in loop() once the
        // sensor reconnects (calPortalStarted is reset below).
        if (calPortalStarted) {
            webServer.stop();
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_OFF);
            Serial.println("[WEB] Calibration AP stopped for clean BLE rescan");
            calPortalStarted = false;
        }
    }
};

class SensorScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        // Match by service UUID (preferred) or by device name as a fallback.
        // When WiFi AP is active it competes for radio time and BLE active-scan
        // responses — which carry the service UUID — can be missed.  Matching by
        // name alone lets us find the sensor even in that situation.
        bool hasCPS = advertisedDevice.haveServiceUUID() &&
                  advertisedDevice.isAdvertisingService(CPS_SERVICE_UUID);
        String devName = advertisedDevice.getName().c_str();
        bool hasName = advertisedDevice.haveName() && devName.length() > 0;
        bool nameMatch = strlen(TARGET_SENSOR_NAME) > 0 && devName.startsWith(TARGET_SENSOR_NAME);

        if (hasCPS || nameMatch || advertisedDevice.haveName()) {
            Serial.printf("[SCANDBG] Name:'%s' Addr:%s CPS:%s\n",
                          devName.c_str(),
                          advertisedDevice.getAddress().toString().c_str(),
                          hasCPS ? "yes" : "no");
        }

        if (!hasCPS && !nameMatch) return;  // Not our sensor

        // If CPS is present, allow unnamed devices (some advertisements do not
        // include Complete Local Name). Only enforce prefix filter when a name
        // is actually present.
        if (hasCPS && strlen(TARGET_SENSOR_NAME) > 0 && hasName && !nameMatch) {
            Serial.printf("[SCAN] Found CPS device '%s' but name does not match filter '%s'\n",
                          devName.c_str(), TARGET_SENSOR_NAME);
            return;
        }

        Serial.printf("[SCAN] Found target sensor: %s (%s)%s\n",
                      advertisedDevice.getName().c_str(),
                      advertisedDevice.getAddress().toString().c_str(),
                      hasCPS ? "" : " [matched by name, CPS UUID not in adv]");

        BLEDevice::getScan()->stop();
        scanRunning = false;
        sensorDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = false;
    }
};

static bool connectToSensor() {
    if (sensorDevice == nullptr) return false;

    Serial.printf("[SENSOR] Connecting to %s ...\n", sensorDevice->getAddress().toString().c_str());

    if (scanRunning) {
        BLEDevice::getScan()->stop();
        scanRunning = false;
    }

    bleClient = BLEDevice::createClient();
    bleClient->setClientCallbacks(new SensorClientCallbacks());

    if (!bleClient->connect(sensorDevice)) {
        Serial.println("[SENSOR] Connection failed!");
        return false;
    }

    BLERemoteService* remoteService = bleClient->getService(CPS_SERVICE_UUID);
    if (remoteService == nullptr) {
        Serial.println("[SENSOR] Cycling Power Service not found!");
        bleClient->disconnect();
        return false;
    }
    Serial.println("[SENSOR] Found Cycling Power Service (0x1818)");

    BLERemoteCharacteristic* remoteMeasurement = remoteService->getCharacteristic(CPS_MEASUREMENT_UUID);
    if (remoteMeasurement == nullptr) {
        Serial.println("[SENSOR] Power Measurement characteristic not found!");
        bleClient->disconnect();
        return false;
    }

    if (remoteMeasurement->canNotify()) {
        remoteMeasurement->registerForNotify(notifyCallback);
        Serial.println("[SENSOR] Subscribed to power notifications");
    } else {
        Serial.println("[SENSOR] WARNING: measurement characteristic does not notify");
    }

    remotePowerControlPoint = remoteService->getCharacteristic(CPS_CONTROL_POINT_UUID);
    remoteControlPointReady = false;
    if (remotePowerControlPoint != nullptr) {
        if (remotePowerControlPoint->canIndicate()) {
            remotePowerControlPoint->registerForNotify(remoteControlPointCallback, false);
            remoteControlPointReady = true;
            Serial.println("[TUNE] Remote Cycling Power Control Point found with indications");
        } else if (remotePowerControlPoint->canNotify()) {
            remotePowerControlPoint->registerForNotify(remoteControlPointCallback, true);
            remoteControlPointReady = true;
            Serial.println("[TUNE] Remote Cycling Power Control Point found with notifications");
        } else {
            Serial.println("[TUNE] Remote Cycling Power Control Point found but cannot indicate/notify");
        }
    } else {
        Serial.println("[TUNE] Remote Cycling Power Control Point not found");
        tuneState = TUNE_NO_CONTROL_POINT;
    }

    BLERemoteCharacteristic* remoteLocation = remoteService->getCharacteristic(SENSOR_LOCATION_UUID);
    if (remoteLocation != nullptr && remoteLocation->canRead()) {
        std::string locVal = remoteLocation->readValue();
        if (locVal.length() > 0) {
            Serial.printf("[SENSOR] Sensor Location: 0x%02X\n", (uint8_t)locVal[0]);
        }
    }

    BLERemoteCharacteristic* remoteFeature = remoteService->getCharacteristic(CPS_FEATURE_UUID);
    if (remoteFeature != nullptr && remoteFeature->canRead()) {
        std::string featVal = remoteFeature->readValue();
        Serial.print("[SENSOR] Feature value: ");
        for (size_t i = 0; i < featVal.length(); i++) {
            Serial.printf("%02X ", (uint8_t)featVal[i]);
        }
        Serial.println();
    }

    BLERemoteService* remoteBattService = bleClient->getService(BATTERY_SERVICE_UUID);
    if (remoteBattService != nullptr) {
        BLERemoteCharacteristic* remoteBattChar = remoteBattService->getCharacteristic(BATTERY_LEVEL_UUID);
        if (remoteBattChar != nullptr) {
            if (remoteBattChar->canRead()) {
                std::string battVal = remoteBattChar->readValue();
                if (battVal.length() > 0) {
                    lastBattery = (uint8_t)battVal[0];
                    Serial.printf("[SENSOR] Battery Level: %d%%\n", lastBattery);
                    if (batteryLevelChar != nullptr) {
                        uint8_t bv = lastBattery;
                        batteryLevelChar->setValue(&bv, 1);
                    }
                }
            }
            if (remoteBattChar->canNotify()) {
                remoteBattChar->registerForNotify(batteryNotifyCallback);
                Serial.println("[SENSOR] Subscribed to battery notifications");
            }
        }
    }

    lastCrankMoveMs = millis();
    sensorConnected = true;
    return true;
}

static void handleWebRoot() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html", "");
    webServer.sendContent(
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>MYX Bridge Calibration</title>"
        "<style>"
        ":root{font-family:Segoe UI,Arial,sans-serif;color:#17212b;background:#eef2f6}"
        "body{margin:0;padding:22px}main{max-width:980px;margin:auto}"
        "h1{margin:0 0 4px;font-size:28px}.sub{color:#5d6b78;margin:0 0 18px}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:12px}"
        ".card{background:#fff;border:1px solid #d7dde4;border-radius:8px;padding:14px;box-shadow:0 1px 3px #0001}"
        ".big{font-size:30px;font-weight:700}.muted{color:#64717f}.ok{color:#0b7f4f}.bad{color:#b32828}"
        "label{display:block;margin:8px 0 4px;font-size:13px;color:#3d4a57}"
        "input{box-sizing:border-box;width:100%;padding:9px;border:1px solid #c9d1d9;border-radius:6px;font-size:15px}"
        "input[type=checkbox]{width:auto;margin-right:6px}button,.btn{border:0;background:#0b5fff;color:#fff;padding:10px 13px;border-radius:6px;font-weight:700;cursor:pointer;text-decoration:none;display:inline-block}"
        "button.secondary,.btn.secondary{background:#4e5b67}button.warn{background:#aa3a2d}.row{display:flex;justify-content:space-between;gap:10px;margin:7px 0}"
        ".actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}.table{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
        "code{background:#edf1f5;padding:2px 5px;border-radius:4px}@media(max-width:560px){body{padding:14px}.big{font-size:24px}}"
        "</style></head><body><main>"
        "<h1>MYX Bridge Calibration</h1>"
        "<p class='sub'>Connect apps to MYX Bridge as usual. Use this page only for sensor tuning and correction.</p>"
        "<div class='grid'>"
        "<section class='card'><h2>Broadcast</h2><div class='big' id='broadcast'>0 W</div><div class='muted' id='cadence'>0 rpm</div></section>"
        "<section class='card'><h2>Raw Diagnostics</h2><div class='row'><span>Raw</span><b id='raw'>0 W</b></div><div class='row'><span>Zeroed</span><b id='zeroed'>0 W</b></div><div class='row'><span>Corrected</span><b id='corrected'>0 W</b></div><div class='row'><span>Applied cadence offset</span><b id='appliedOffset'>0 W</b></div></section>"
        "<section class='card'><h2>Connections</h2><div class='row'><span>Sensor</span><b id='sensor'>searching</b></div><div class='row'><span>Apps</span><b id='apps'>0</b></div><div class='row'><span>Battery</span><b id='battery'>?</b></div><div class='row'><span>Control point</span><b id='cp'>unknown</b></div></section>"
        "<section class='card'><h2>Sensor Tune</h2><p class='muted'>Pedal about 30 seconds, get off the bike, set the crank vertical, then press tune and stay off the bike.</p><div class='row'><span>Status</span><b id='tune'>idle</b></div><div class='row'><span>Returned offset</span><b id='tuneOffset'>0</b></div><div class='actions'><button onclick='tuneSensor()'>Tune Sensor</button></div></section>"
        "</div>"
        "<form class='card' method='post' action='/settings'><h2>Local Fallback Correction</h2>"
        "<label><input type='checkbox' name='enabled' id='enabled'>Correction enabled</label>"
        "<label><input type='checkbox' name='rawdiag' id='rawdiag'>Raw diagnostics enabled</label>"
        "<label>Local zero offset, watts<input name='zero' id='zero' type='number' step='0.1'></label>"
        "<label>Scale factor<input name='scale' id='scale' type='number' step='0.001' min='0.05' max='50'></label>"
        "<div class='table'>"
        "<label>Cadence 1<input name='cad0' id='cad0' type='number' step='0.1'></label><label>Offset 1<input name='off0' id='off0' type='number' step='0.1'></label>"
        "<label>Cadence 2<input name='cad1' id='cad1' type='number' step='0.1'></label><label>Offset 2<input name='off1' id='off1' type='number' step='0.1'></label>"
        "<label>Cadence 3<input name='cad2' id='cad2' type='number' step='0.1'></label><label>Offset 3<input name='off2' id='off2' type='number' step='0.1'></label>"
        "<label>Cadence 4<input name='cad3' id='cad3' type='number' step='0.1'></label><label>Offset 4<input name='off3' id='off3' type='number' step='0.1'></label>"
        "</div><div class='actions'><button type='submit'>Save settings</button><a class='btn secondary' href='/api/settings'>Export JSON</a><button class='warn' type='button' onclick='resetSettings()'>Reset defaults</button></div></form>"
        "</main><script>"
        "let initialized=false;"
        "function setVal(id,v){document.getElementById(id).textContent=v}"
        "function setInput(id,v){if(!initialized)document.getElementById(id).value=v}"
        "async function refresh(){let r=await fetch('/api/status');let d=await r.json();"
        "setVal('broadcast',d.broadcastPower+' W');setVal('cadence',d.cadence.toFixed(1)+' rpm');"
        "setVal('raw',d.rawPower+' W');setVal('zeroed',d.zeroedPower+' W');setVal('corrected',d.correctedPower+' W');setVal('appliedOffset',d.appliedCadenceOffset.toFixed(1)+' W');"
        "setVal('sensor',d.sensorConnected?'connected':'searching');setVal('apps',d.appConnections);setVal('battery',d.battery);setVal('cp',d.remoteControlPointReady?'ready':'unavailable');"
        "setVal('tune',d.tuneStatus);setVal('tuneOffset',d.tuneReportedOffset);"
        "if(!initialized){document.getElementById('enabled').checked=d.settings.enabled;document.getElementById('rawdiag').checked=d.settings.rawDiagnostics;"
        "setInput('zero',d.settings.localZeroOffset);setInput('scale',d.settings.scaleFactor);"
        "for(let i=0;i<4;i++){setInput('cad'+i,d.settings.cadence[i]);setInput('off'+i,d.settings.cadenceOffset[i]);}initialized=true;}}"
        "async function tuneSensor(){await fetch('/tune',{method:'POST'});refresh();}"
        "async function resetSettings(){if(confirm('Reset calibration settings?')){await fetch('/reset',{method:'POST'});initialized=false;refresh();}}"
        "refresh();setInterval(refresh,1000);</script></body></html>");
    webServer.sendContent("");
}

static float argFloat(const char* name, float fallback) {
    if (!webServer.hasArg(name)) return fallback;
    return webServer.arg(name).toFloat();
}

static void handleWebSettingsPost() {
    calibration.correctionEnabled = webServer.hasArg("enabled");
    calibration.rawDiagnosticsEnabled = webServer.hasArg("rawdiag");
    calibration.localZeroOffset = argFloat("zero", calibration.localZeroOffset);
    calibration.scaleFactor = argFloat("scale", calibration.scaleFactor);
    if (!isfinite(calibration.scaleFactor) || calibration.scaleFactor < 0.05f) calibration.scaleFactor = 1.0f;
    if (calibration.scaleFactor > 50.0f) calibration.scaleFactor = 50.0f;
    for (int i = 0; i < 4; i++) {
        char key[8];
        snprintf(key, sizeof(key), "cad%d", i);
        calibration.cadence[i] = argFloat(key, calibration.cadence[i]);
        snprintf(key, sizeof(key), "off%d", i);
        calibration.cadenceOffset[i] = argFloat(key, calibration.cadenceOffset[i]);
    }
    saveCalibration();
    webServer.sendHeader("Location", "/", true);
    webServer.send(303, "text/plain", "Saved");
}

static void sendSettingsJsonOnly() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "application/json", "");
    webServer.sendContent("{\"enabled\":");
    webServer.sendContent(calibration.correctionEnabled ? "true" : "false");
    webServer.sendContent(",\"rawDiagnostics\":");
    webServer.sendContent(calibration.rawDiagnosticsEnabled ? "true" : "false");
    webServer.sendContent(",\"localZeroOffset\":");
    webServer.sendContent(String(calibration.localZeroOffset, 3));
    webServer.sendContent(",\"scaleFactor\":");
    webServer.sendContent(String(calibration.scaleFactor, 6));
    webServer.sendContent(",\"cadence\":[");
    for (int i = 0; i < 4; i++) {
        if (i) webServer.sendContent(",");
        webServer.sendContent(String(calibration.cadence[i], 3));
    }
    webServer.sendContent("],\"cadenceOffset\":[");
    for (int i = 0; i < 4; i++) {
        if (i) webServer.sendContent(",");
        webServer.sendContent(String(calibration.cadenceOffset[i], 3));
    }
    webServer.sendContent("]}");
    webServer.sendContent("");
}

static void handleWebStatusJson() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "application/json", "");
    webServer.sendContent("{\"sensorConnected\":");
    webServer.sendContent(sensorConnected ? "true" : "false");
    webServer.sendContent(",\"appConnections\":");
    webServer.sendContent(String(appConnectionCount));
    webServer.sendContent(",\"battery\":\"");
    if (lastBattery == 0xFF) webServer.sendContent("?");
    else webServer.sendContent(String((int)lastBattery) + "%");
    webServer.sendContent("\",\"rawPower\":");
    webServer.sendContent(String((int)lastRawPower));
    webServer.sendContent(",\"zeroedPower\":");
    webServer.sendContent(String((int)lastZeroedPower));
    webServer.sendContent(",\"correctedPower\":");
    webServer.sendContent(String((int)lastCorrectedPower));
    webServer.sendContent(",\"broadcastPower\":");
    webServer.sendContent(String((int)lastBroadcastPower));
    webServer.sendContent(",\"cadence\":");
    webServer.sendContent(String((double)lastCadence, 3));
    webServer.sendContent(",\"crankRev\":");
    webServer.sendContent(String((unsigned int)lastCrankRev));
    webServer.sendContent(",\"crankTime\":");
    webServer.sendContent(String((unsigned int)lastCrankTime));
    webServer.sendContent(",\"energy\":");
    webServer.sendContent(String((unsigned int)lastEnergy));
    webServer.sendContent(",\"dropped\":");
    webServer.sendContent(lastDropped ? "true" : "false");
    webServer.sendContent(",\"appliedCadenceOffset\":");
    webServer.sendContent(String((double)lastAppliedCadenceOffset, 3));
    webServer.sendContent(",\"remoteControlPointReady\":");
    webServer.sendContent(remoteControlPointReady ? "true" : "false");
    webServer.sendContent(",\"tuneStatus\":\"");
    jsonEscapePrint(tuneStateText((TuneState)tuneState));
    webServer.sendContent("\",\"tuneLastOp\":");
    webServer.sendContent(String((int)tuneLastOp));
    webServer.sendContent(",\"tuneLastResponse\":");
    webServer.sendContent(String((int)tuneLastResponse));
    webServer.sendContent(",\"tuneReportedOffset\":");
    webServer.sendContent(String((int)tuneReportedOffset));
    webServer.sendContent(",\"tuneFailureReason\":");
    webServer.sendContent(String((int)tuneFailureReason));
    webServer.sendContent(",\"settings\":");
    webServer.sendContent("{\"enabled\":");
    webServer.sendContent(calibration.correctionEnabled ? "true" : "false");
    webServer.sendContent(",\"rawDiagnostics\":");
    webServer.sendContent(calibration.rawDiagnosticsEnabled ? "true" : "false");
    webServer.sendContent(",\"localZeroOffset\":");
    webServer.sendContent(String(calibration.localZeroOffset, 3));
    webServer.sendContent(",\"scaleFactor\":");
    webServer.sendContent(String(calibration.scaleFactor, 6));
    webServer.sendContent(",\"cadence\":[");
    for (int i = 0; i < 4; i++) {
        if (i) webServer.sendContent(",");
        webServer.sendContent(String(calibration.cadence[i], 3));
    }
    webServer.sendContent("],\"cadenceOffset\":[");
    for (int i = 0; i < 4; i++) {
        if (i) webServer.sendContent(",");
        webServer.sendContent(String(calibration.cadenceOffset[i], 3));
    }
    webServer.sendContent("]}}");
    webServer.sendContent("");
}

static void handleTunePost() {
    tuneRequested = true;
    tuneState = TUNE_IDLE;
    webServer.send(202, "application/json", "{\"status\":\"queued\"}");
}

static void handleResetPost() {
    resetCalibration();
    webServer.send(200, "application/json", "{\"status\":\"reset\"}");
}

static void handleNotFound() {
    webServer.send(404, "text/plain", "Not found");
}

static void setupCalibrationPortal() {
    WiFi.mode(WIFI_AP);
    bool apStarted = strlen(CAL_AP_PASS) > 0 ? WiFi.softAP(CAL_AP_NAME, CAL_AP_PASS) : WiFi.softAP(CAL_AP_NAME);
    Serial.printf("[WEB] Calibration AP %s: %s\n", apStarted ? "started" : "failed", CAL_AP_NAME);
    Serial.printf("[WEB] Calibration portal: http://%s/\n", WiFi.softAPIP().toString().c_str());

    webServer.on("/", HTTP_GET, handleWebRoot);
    webServer.on("/api/status", HTTP_GET, handleWebStatusJson);
    webServer.on("/api/settings", HTTP_GET, sendSettingsJsonOnly);
    webServer.on("/settings", HTTP_POST, handleWebSettingsPost);
    webServer.on("/tune", HTTP_POST, handleTunePost);
    webServer.on("/reset", HTTP_POST, handleResetPost);
    webServer.onNotFound(handleNotFound);
    webServer.begin();
}

static void setupBLEServer() {
    Serial.println("[SERVER] Setting up BLE Cycling Power Service...");

    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new BridgeServerCallbacks());

    BLEService* cpsService = bleServer->createService(CPS_SERVICE_UUID, 20);

    powerMeasurementChar = cpsService->createCharacteristic(
        CPS_MEASUREMENT_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    powerMeasurementChar->addDescriptor(new BLE2902());

    BLECharacteristic* powerFeatureChar = cpsService->createCharacteristic(
        CPS_FEATURE_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t featureValue[4] = {0x08, 0x00, 0x00, 0x00};
    powerFeatureChar->setValue(featureValue, 4);

    BLECharacteristic* sensorLocationChar = cpsService->createCharacteristic(
        SENSOR_LOCATION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t sensorLoc = 0x05;
    sensorLocationChar->setValue(&sensorLoc, 1);

    powerControlPointChar = cpsService->createCharacteristic(
        CPS_CONTROL_POINT_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE
    );
    powerControlPointChar->addDescriptor(new BLE2902());
    powerControlPointChar->setCallbacks(new ControlPointCallbacks());

    cpsService->start();

    BLEService* cscService = bleServer->createService(CSC_SERVICE_UUID, 12);
    cscMeasurementChar = cscService->createCharacteristic(
        CSC_MEASUREMENT_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    cscMeasurementChar->addDescriptor(new BLE2902());

    BLECharacteristic* cscFeatureChar = cscService->createCharacteristic(
        CSC_FEATURE_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t cscFeature[2] = {0x02, 0x00};
    cscFeatureChar->setValue(cscFeature, 2);

    BLECharacteristic* cscSensorLoc = cscService->createCharacteristic(
        SENSOR_LOCATION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t cscLoc = 0x05;
    cscSensorLoc->setValue(&cscLoc, 1);
    cscService->start();

    BLEService* disService = bleServer->createService(BLEUUID((uint16_t)0x180A));
    BLECharacteristic* mfgNameChar = disService->createCharacteristic(
        BLEUUID((uint16_t)0x2A29), BLECharacteristic::PROPERTY_READ);
    mfgNameChar->setValue("ESP32 Bike Bridge");
    BLECharacteristic* modelChar = disService->createCharacteristic(
        BLEUUID((uint16_t)0x2A24), BLECharacteristic::PROPERTY_READ);
    modelChar->setValue("BikePower Bridge v1");
    BLECharacteristic* fwRevChar = disService->createCharacteristic(
        BLEUUID((uint16_t)0x2A26), BLECharacteristic::PROPERTY_READ);
    fwRevChar->setValue(FW_VERSION);
    disService->start();

    BLEService* battService = bleServer->createService(BATTERY_SERVICE_UUID, 8);
    batteryLevelChar = battService->createCharacteristic(
        BATTERY_LEVEL_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    batteryLevelChar->addDescriptor(new BLE2902());
    uint8_t initBatt = 0;
    batteryLevelChar->setValue(&initBatt, 1);
    battService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(CPS_SERVICE_UUID);
    pAdvertising->addServiceUUID(CSC_SERVICE_UUID);
    pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
    pAdvertising->setAppearance(0x0484);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.printf("[SERVER] Advertising as '%s'\n", BRIDGE_NAME);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=========================================");
    Serial.println("  ESP32 BLE Bike Power Meter Bridge");
    Serial.printf("  Firmware: %s\n", FW_VERSION);
    Serial.println("=========================================");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    lastCrankMoveMs = millis();

    loadCalibration();
    Serial.printf("[CAL] Correction: %s | Raw diagnostics: %s | Zero: %.1f W | Scale: %.3f\n",
                  calibration.correctionEnabled ? "enabled" : "disabled",
                  calibration.rawDiagnosticsEnabled ? "enabled" : "disabled",
                  (double)calibration.localZeroOffset,
                  (double)calibration.scaleFactor);

    BLEDevice::init(BRIDGE_NAME);
    setupBLEServer();
    // WiFi AP is started lazily in loop() once the sensor is connected.
    // Starting the AP before scanning causes radio contention that prevents
    // the ESP32 from receiving BLE scan response packets (which carry the
    // sensor's service UUID and name).

    Serial.println("[SCAN] Starting continuous BLE scan for power meter...");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SensorScanCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(0, false);
    scanRunning = true;
    doScan = false;
    Serial.println("[SCAN] Continuous scan running");
}

void loop() {
    // Start the calibration AP the first time the sensor connects.
    // This keeps the WiFi radio silent during BLE scanning so scan
    // responses (service UUID + name) are not dropped.
    if (sensorConnected && !calPortalStarted) {
        setupCalibrationPortal();
        calPortalStarted = true;
    }

    if (calPortalStarted) webServer.handleClient();
    serviceTuneState();

    if (doConnect) {
        if (connectToSensor()) {
            Serial.println("[OK] Bridge is active! Power data will flow through.");
        } else {
            Serial.println("[ERROR] Failed to connect to sensor. Will retry...");
            doScan = true;
        }
        doConnect = false;
    }

    if (!sensorConnected && doScan) {
        Serial.println("[SCAN] Re-starting continuous scan for power meter...");
        BLEDevice::getScan()->start(0, false);
        scanRunning = true;
        doScan = false;
    }

    unsigned long now = millis();
    if (sensorConnected && appConnectionCount > 0) {
        if (!ledState) {
            digitalWrite(LED_PIN, HIGH);
            ledState = true;
        }
    } else if (sensorConnected) {
        if (now - lastBlinkTime > 250) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            lastBlinkTime = now;
        }
    } else {
        if (now - lastBlinkTime > 1000) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            lastBlinkTime = now;
        }
    }

    static unsigned long lastStatusTime = 0;
    if (now - lastStatusTime > 10000) {
        lastStatusTime = now;
        char battStr[8];
        if (lastBattery == 0xFF) snprintf(battStr, sizeof(battStr), "?");
        else snprintf(battStr, sizeof(battStr), "%d%%", lastBattery);
        Serial.printf("[STATUS] Sensor: %s | Apps: %d | Raw: %d W | Broadcast: %d W | Cadence: %.0f RPM | Battery: %s | Tune: %s\n",
                      sensorConnected ? "CONNECTED" : "SEARCHING",
                      appConnectionCount,
                      (int)lastRawPower,
                      (int)lastBroadcastPower,
                      (double)lastCadence,
                      battStr,
                      tuneStateText((TuneState)tuneState));
    }

    delay(10);
}
