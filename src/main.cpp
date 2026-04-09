/*
 * ESP32 BLE Bike Power Meter Bridge
 * ==================================
 *
 * Connects to a MYX/BKSNSR power meter sensor that XOR-masks its BLE
 * Cycling Power Measurement (0x2A63) data with 0xAA, decodes it, and
 * re-broadcasts as a standard-compliant BLE Cycling Power Service so
 * that apps like Zwift, TrainerRoad, Peloton, etc. can read it properly.
 *
 * Sensor details (from LightBlue log analysis):
 *   Name pattern:  BKSNSR*
 *   Service:       0x1818 (Cycling Power)
 *   Measurement:   0x2A63 (Notify) - XOR 0xAA masked
 *   Feature:       0x2A65 (Read)   - value 0x80000000
 *   Sensor Loc:    0x2A5D (Read)   - value 0x04 (Front Wheel)
 *   Manufacturer:  MYX_Power_Meter
 *   Battery:       0x180F / 0x2A19 (Read+Notify) — plain uint8, 0-100%
 *
 * Decoded 10-byte payload (after XOR 0xAA):
 *   bytes 0-1:  flags                  uint16 LE  (always 0x0820)
 *   bytes 2-3:  instantaneous_power    sint16 LE  (watts)
 *   bytes 4-5:  cumulative_crank_rev   uint16 LE
 *   bytes 6-7:  last_crank_event_time  uint16 LE  (1/1024 s)
 *   bytes 8-9:  accumulated_energy     uint16 LE  (kJ)
 *
 * Hardware: Any ESP32 dev board (ESP32-DevKitC, ESP32-WROOM, etc.)
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ─── Configuration ──────────────────────────────────────────────────────────

// Set to empty string "" to connect to any device advertising Cycling Power Service.
// Or set to your specific sensor name, e.g. "BSKNSRxxxxxxx"
static const char* TARGET_SENSOR_NAME = "BKSNSR";  // Match any BKSNSR* device

// The XOR key used by the sensor to mask data
static const uint8_t XOR_KEY = 0xAA;

// The name this ESP32 will advertise to apps (Zwift, etc.)
static const char* BRIDGE_NAME = "MYX Bridge";

// Firmware version
static const char* FW_VERSION = "1.0.2";

// ─── Power Smoothing ─────────────────────────────────────────────────────────
//
// Exponential moving average applied to power before broadcasting.
// Formula: smoothed = (raw * factor) + (smoothed * (1 - factor))
//
//   0.0 = maximum smoothing (output never changes — not useful)
//   0.5 = moderate smoothing (good balance of stability and responsiveness)
//   1.0 = no smoothing (raw values passed through unchanged)
//
// Lower values produce a smoother but slower-to-respond output.
// Higher values produce a more responsive but jumpier output.
// 0.5 is a recommended starting point.

static const float SMOOTHING_FACTOR = 0.5f;

// ─── Pedaling Dropout ────────────────────────────────────────────────────────
//
// If no new crank revolutions are detected within this window, power and
// cadence are immediately forced to zero and broadcast. This prevents the
// bridge from reporting stale non-zero power after you stop pedaling.
//
// The timer is only reset when cRev actually increments — not on every
// BLE packet — so the timer fires correctly even though the sensor keeps
// sending packets at rest.

static const uint32_t DROPOUT_MS = 2500;

// ─── BLE UUIDs ──────────────────────────────────────────────────────────────

// Standard Bluetooth Cycling Power Service
static BLEUUID CPS_SERVICE_UUID((uint16_t)0x1818);
static BLEUUID CPS_MEASUREMENT_UUID((uint16_t)0x2A63);  // Notify
static BLEUUID CPS_FEATURE_UUID((uint16_t)0x2A65);      // Read
static BLEUUID SENSOR_LOCATION_UUID((uint16_t)0x2A5D);  // Read
static BLEUUID CPS_CONTROL_POINT_UUID((uint16_t)0x2A66); // Write + Indicate

// Standard Bluetooth Battery Service
static BLEUUID BATTERY_SERVICE_UUID((uint16_t)0x180F);
static BLEUUID BATTERY_LEVEL_UUID((uint16_t)0x2A19);

// Standard Bluetooth Cycling Speed & Cadence Service
// Many apps read cadence from CSC rather than from CPS crank data
static BLEUUID CSC_SERVICE_UUID((uint16_t)0x1816);
static BLEUUID CSC_MEASUREMENT_UUID((uint16_t)0x2A5B);   // Notify
static BLEUUID CSC_FEATURE_UUID((uint16_t)0x2A5C);       // Read

// ─── Global State ───────────────────────────────────────────────────────────

// BLE Client (connection to the real sensor)
static BLEAdvertisedDevice* sensorDevice = nullptr;
static BLEClient* bleClient = nullptr;
static bool sensorConnected = false;
static bool doConnect = false;
static bool doScan = true;

// BLE Server (what apps connect to)
static BLEServer* bleServer = nullptr;
static BLECharacteristic* powerMeasurementChar = nullptr;
static BLECharacteristic* powerFeatureChar = nullptr;
static BLECharacteristic* sensorLocationChar = nullptr;
static BLECharacteristic* powerControlPointChar = nullptr;
static BLECharacteristic* cscMeasurementChar = nullptr;
static BLECharacteristic* batteryLevelChar = nullptr;
static bool clientSubscribed = false;
static int appConnectionCount = 0;

// Latest decoded data for serial output
static volatile uint8_t  lastBattery = 0xFF;  // 0xFF = unknown
static volatile int16_t  lastPower = 0;
static volatile uint16_t lastCrankRev = 0;
static volatile uint16_t lastCrankTime = 0;
static volatile uint16_t lastEnergy = 0;

// For cadence calculation
static uint16_t prevCrankRev = 0;
static uint16_t prevCrankTime = 0;
static float    lastCadence = 0.0f;

// ─── Smoothing + Dropout State ───────────────────────────────────────────────

// EMA accumulator — zeroed immediately on dropout so stale values from a
// previous effort don't bleed into the next one.
static float    smoothedPower   = 0.0f;

// Timestamp of the last crank revolution increment. Updated ONLY when cRev
// changes, not on every packet arrival — otherwise the dropout timer would
// never fire while the flywheel is coasting.
static uint32_t lastCrankMoveMs = 0;

// LED feedback
#ifndef LED_PIN
#define LED_PIN 2  // Built-in LED on most ESP32 boards
#endif
static unsigned long lastBlinkTime = 0;
static bool ledState = false;

// ─── Utility Functions ──────────────────────────────────────────────────────

/**
 * Called when the sensor sends a Battery Level notification.
 */
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

/**
 * XOR-decode a buffer in place with the mask byte.
 */
void xorDecode(uint8_t* data, size_t len, uint8_t key) {
    for (size_t i = 0; i < len; i++) {
        data[i] ^= key;
    }
}

/**
 * Calculate cadence in RPM from cumulative crank revolution data.
 * Handles uint16 wraparound.
 */
float calcCadence(uint16_t currRev, uint16_t currTime, uint16_t prevRev, uint16_t prevTime) {
    uint16_t dRev  = (uint16_t)(currRev - prevRev);
    uint16_t dTime = (uint16_t)(currTime - prevTime);
    if (dRev == 0 || dTime == 0) return 0.0f;
    return 60.0f * 1024.0f * (float)dRev / (float)dTime;
}

/**
 * Print a hex buffer to Serial.
 */
void printHex(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (i > 0) Serial.print(" ");
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
    }
}

// ─── BLE Server Callbacks (apps connecting to us) ───────────────────────────

class BridgeServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        appConnectionCount++;
        Serial.printf("[SERVER] App connected! (%d total)\n", appConnectionCount);

        // Allow more connections (for multiple apps)
        BLEDevice::startAdvertising();
    }

    void onDisconnect(BLEServer* pServer) override {
        appConnectionCount--;
        if (appConnectionCount < 0) appConnectionCount = 0;
        Serial.printf("[SERVER] App disconnected. (%d remaining)\n", appConnectionCount);

        // Re-start advertising
        BLEDevice::startAdvertising();
    }
};

// Minimal Cycling Power Control Point handler — responds "not supported" to
// every op-code so apps that probe capabilities don't see a BLE error.
class ControlPointCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            uint8_t opCode = value[0];
            // Response: [0x20 = Response Op Code, reqOpCode, 0x02 = Not Supported]
            uint8_t response[3] = {0x20, opCode, 0x02};
            pCharacteristic->setValue(response, 3);
            pCharacteristic->indicate();
            Serial.printf("[SERVER] Control Point op 0x%02X → not supported\n", opCode);
        }
    }
};

// ─── BLE Client Notification Callback (data from sensor) ────────────────────

/**
 * Called every time the sensor sends a Cycling Power Measurement notification.
 * This is the core decode + rebroadcast logic.
 */
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

    // Make a working copy
    uint8_t decoded[20];
    size_t copyLen = (length > sizeof(decoded)) ? sizeof(decoded) : length;
    memcpy(decoded, pData, copyLen);

    // XOR decode
    xorDecode(decoded, copyLen, XOR_KEY);

    // Parse fields (little-endian)
    uint16_t flags  = (uint16_t)(decoded[0] | (decoded[1] << 8));
    int16_t  power  = (int16_t)(decoded[2] | (decoded[3] << 8));
    uint16_t cRev   = (uint16_t)(decoded[4] | (decoded[5] << 8));
    uint16_t cTime  = (uint16_t)(decoded[6] | (decoded[7] << 8));
    uint16_t energy = (uint16_t)(decoded[8] | (decoded[9] << 8));

    // ── Dropout watchdog ──
    // Update the crank movement timestamp ONLY when cRev actually changes.
    // If we updated on every packet the timer would never fire while coasting.
    if (cRev != prevCrankRev) {
        lastCrankMoveMs = millis();
    }

    bool dropped = (millis() - lastCrankMoveMs > DROPOUT_MS);

    if (dropped) {
        // Immediate reset — zero everything and broadcast.
        // Zeroing smoothedPower prevents stale values bleeding into the next effort.
        power         = 0;
        smoothedPower = 0.0f;
        lastCadence   = 0.0f;
        lastPower     = 0;
    } else {
        // Clamp negative power to zero
        if (power < 0) power = 0;

        // Store for display
        lastPower     = power;
        lastCrankRev  = cRev;
        lastCrankTime = cTime;
        lastEnergy    = energy;

        // Calculate cadence
        float cadence = calcCadence(cRev, cTime, prevCrankRev, prevCrankTime);
        if (cadence > 0.0f && cadence < 200.0f) {
            lastCadence = cadence;
        } else if (cRev != prevCrankRev) {
            // Crank moved but cadence is out of range — keep last known
        } else {
            lastCadence = 0.0f;  // No movement
        }

        // ── Exponential moving average smoothing ──
        // Reduces packet-to-packet power jumpiness without significantly
        // slowing response to real efforts. Adjust SMOOTHING_FACTOR to taste.
        smoothedPower = (power * SMOOTHING_FACTOR) + (smoothedPower * (1.0f - SMOOTHING_FACTOR));
        power = (int16_t)smoothedPower;
    }

    prevCrankRev  = cRev;
    prevCrankTime = cTime;

    // ── Re-broadcast as standard Cycling Power Measurement ──
    // Build a PROPER BLE Cycling Power Measurement per Bluetooth spec.
    // Flags 0x0020 = bit 5 (Crank Revolution Data Present)
    // We intentionally strip the Accumulated Energy flag since most apps
    // don't use it and it keeps the packet cleaner.
    // If you want energy too, use flags 0x0820 and append 2 more bytes.
    //
    // Standard packet layout with crank data:
    //   [0-1]  Flags (uint16 LE)            = 0x0020
    //   [2-3]  Instantaneous Power (sint16 LE, watts)
    //   [4-5]  Cumulative Crank Revolutions (uint16 LE)
    //   [6-7]  Last Crank Event Time (uint16 LE, 1/1024 s)

    uint8_t outBuf[8];
    uint16_t outFlags = 0x0020;  // Crank Revolution Data Present

    outBuf[0] = outFlags & 0xFF;
    outBuf[1] = (outFlags >> 8) & 0xFF;
    outBuf[2] = (uint8_t)(power & 0xFF);
    outBuf[3] = (uint8_t)((power >> 8) & 0xFF);
    outBuf[4] = cRev & 0xFF;
    outBuf[5] = (cRev >> 8) & 0xFF;
    outBuf[6] = cTime & 0xFF;
    outBuf[7] = (cTime >> 8) & 0xFF;

    // Send to connected app(s)
    if (powerMeasurementChar != nullptr) {
        powerMeasurementChar->setValue(outBuf, sizeof(outBuf));
        powerMeasurementChar->notify();
    }

    // ── Also send CSC Measurement for cadence ──
    // Many apps only read cadence from the CSC Service (0x1816), not CPS.
    // CSC Measurement format:
    //   [0]    Flags (uint8)  = 0x02 (Crank Revolution Data Present)
    //   [1-2]  Cumulative Crank Revolutions (uint16 LE)
    //   [3-4]  Last Crank Event Time (uint16 LE, 1/1024 s)
    if (cscMeasurementChar != nullptr) {
        uint8_t cscBuf[5];
        cscBuf[0] = 0x02;  // Crank Revolution Data Present
        cscBuf[1] = cRev & 0xFF;
        cscBuf[2] = (cRev >> 8) & 0xFF;
        cscBuf[3] = cTime & 0xFF;
        cscBuf[4] = (cTime >> 8) & 0xFF;
        cscMeasurementChar->setValue(cscBuf, sizeof(cscBuf));
        cscMeasurementChar->notify();
    }

    // Serial debug output (throttled to unique changes)
    static int16_t prevPrintPower = -999;
    static uint16_t prevPrintRev = 0xFFFF;
    if (power != prevPrintPower || cRev != prevPrintRev) {
        if (dropped) {
            Serial.println("[DATA] Dropout — power and cadence zeroed.");
        } else {
            Serial.printf("[DATA] Power: %4d W | Cadence: %5.1f RPM | CrankRev: %5u | Energy: %5u kJ\n",
                          power, lastCadence, cRev, energy);
        }
        prevPrintPower = power;
        prevPrintRev = cRev;
    }
}

// ─── BLE Client Callbacks (connection to sensor) ────────────────────────────

class SensorClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        Serial.println("[SENSOR] Connected to power meter!");
        sensorConnected = true;
    }

    void onDisconnect(BLEClient* pclient) override {
        Serial.println("[SENSOR] Disconnected from power meter!");
        sensorConnected = false;
        doScan = true;  // Trigger re-scan
    }
};

// ─── BLE Scan Callbacks (finding the sensor) ────────────────────────────────

class SensorScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        // Check if device advertises the Cycling Power Service
        if (advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(CPS_SERVICE_UUID)) {

            // Optional: filter by name prefix
            String devName = advertisedDevice.getName().c_str();
            if (strlen(TARGET_SENSOR_NAME) > 0 && !devName.startsWith(TARGET_SENSOR_NAME)) {
                Serial.printf("[SCAN] Found CPS device '%s' but name doesn't match filter '%s'\n",
                              devName.c_str(), TARGET_SENSOR_NAME);
                return;
            }

            Serial.printf("[SCAN] Found target sensor: %s (%s)\n",
                          advertisedDevice.getName().c_str(),
                          advertisedDevice.getAddress().toString().c_str());

            BLEDevice::getScan()->stop();
            sensorDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            doScan = false;
        }
    }
};

// ─── Connect to the Sensor ──────────────────────────────────────────────────

bool connectToSensor() {
    if (sensorDevice == nullptr) return false;

    Serial.printf("[SENSOR] Connecting to %s ...\n", sensorDevice->getAddress().toString().c_str());

    bleClient = BLEDevice::createClient();
    bleClient->setClientCallbacks(new SensorClientCallbacks());

    // Connect
    if (!bleClient->connect(sensorDevice)) {
        Serial.println("[SENSOR] Connection failed!");
        return false;
    }
    Serial.println("[SENSOR] Connected!");

    // Get Cycling Power Service
    BLERemoteService* remoteService = bleClient->getService(CPS_SERVICE_UUID);
    if (remoteService == nullptr) {
        Serial.println("[SENSOR] Cycling Power Service not found!");
        bleClient->disconnect();
        return false;
    }
    Serial.println("[SENSOR] Found Cycling Power Service (0x1818)");

    // Get Power Measurement characteristic (0x2A63)
    BLERemoteCharacteristic* remoteMeasurement = remoteService->getCharacteristic(CPS_MEASUREMENT_UUID);
    if (remoteMeasurement == nullptr) {
        Serial.println("[SENSOR] Power Measurement characteristic not found!");
        bleClient->disconnect();
        return false;
    }
    Serial.println("[SENSOR] Found Power Measurement characteristic (0x2A63)");

    // Subscribe to notifications
    if (remoteMeasurement->canNotify()) {
        remoteMeasurement->registerForNotify(notifyCallback);
        Serial.println("[SENSOR] Subscribed to power notifications!");
    } else {
        Serial.println("[SENSOR] WARNING: Characteristic doesn't support Notify!");
    }

    // Read sensor location for debug info
    BLERemoteCharacteristic* remoteLocation = remoteService->getCharacteristic(SENSOR_LOCATION_UUID);
    if (remoteLocation != nullptr && remoteLocation->canRead()) {
        std::string locVal = remoteLocation->readValue();
        if (locVal.length() > 0) {
            Serial.printf("[SENSOR] Sensor Location: 0x%02X\n", (uint8_t)locVal[0]);
        }
    }

    // Read feature for debug info
    BLERemoteCharacteristic* remoteFeature = remoteService->getCharacteristic(CPS_FEATURE_UUID);
    if (remoteFeature != nullptr && remoteFeature->canRead()) {
        std::string featVal = remoteFeature->readValue();
        Serial.print("[SENSOR] Feature value: ");
        for (size_t i = 0; i < featVal.length(); i++) {
            Serial.printf("%02X ", (uint8_t)featVal[i]);
        }
        Serial.println();
    }

    // ── Battery Service (0x180F / 0x2A19) ──
    BLERemoteService* remoteBattService = bleClient->getService(BATTERY_SERVICE_UUID);
    if (remoteBattService != nullptr) {
        BLERemoteCharacteristic* remoteBattChar = remoteBattService->getCharacteristic(BATTERY_LEVEL_UUID);
        if (remoteBattChar != nullptr) {
            // Initial read
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
            // Subscribe to notifications
            if (remoteBattChar->canNotify()) {
                remoteBattChar->registerForNotify(batteryNotifyCallback);
                Serial.println("[SENSOR] Subscribed to battery notifications!");
            }
        } else {
            Serial.println("[SENSOR] Battery Level characteristic (0x2A19) not found");
        }
    } else {
        Serial.println("[SENSOR] Battery Service (0x180F) not found");
    }

    // Initialize dropout timer now that we're connected
    lastCrankMoveMs = millis();

    sensorConnected = true;
    return true;
}

// ─── Setup BLE Server (what apps see) ───────────────────────────────────────

void setupBLEServer() {
    Serial.println("[SERVER] Setting up BLE Cycling Power Service...");

    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new BridgeServerCallbacks());

    // Create Cycling Power Service (20 handles for all chars + descriptors)
    BLEService* cpsService = bleServer->createService(CPS_SERVICE_UUID, 20);

    // ── Power Measurement Characteristic (0x2A63) ──
    // Properties: Notify only (per BLE spec)
    powerMeasurementChar = cpsService->createCharacteristic(
        CPS_MEASUREMENT_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    // Add Client Characteristic Configuration Descriptor (required for Notify)
    powerMeasurementChar->addDescriptor(new BLE2902());

    // ── Power Feature Characteristic (0x2A65) ──
    // Properties: Read
    // Bit 3 = Crank Revolution Data Supported → 0x08
    powerFeatureChar = cpsService->createCharacteristic(
        CPS_FEATURE_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t featureValue[4] = {0x08, 0x00, 0x00, 0x00};  // Crank Rev Data Supported
    powerFeatureChar->setValue(featureValue, 4);

    // ── Sensor Location Characteristic (0x2A5D) ──
    // Value: 0x05 = Left Crank
    sensorLocationChar = cpsService->createCharacteristic(
        SENSOR_LOCATION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t sensorLoc = 0x05;  // Left Crank
    sensorLocationChar->setValue(&sensorLoc, 1);

    // ── Cycling Power Control Point (0x2A66) ──
    // Some apps probe this to discover sensor capabilities.
    // We respond "Op Code Not Supported" to every request, which is
    // spec-legal and stops apps from reporting a connection error.
    powerControlPointChar = cpsService->createCharacteristic(
        CPS_CONTROL_POINT_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE
    );
    powerControlPointChar->addDescriptor(new BLE2902());
    powerControlPointChar->setCallbacks(new ControlPointCallbacks());

    cpsService->start();

    // ── Cycling Speed & Cadence Service (0x1816) ──
    // Apps like Peloton, Zwift, Wahoo, etc. often read cadence from CSC
    // rather than from the crank data embedded in CPS measurements.
    BLEService* cscService = bleServer->createService(CSC_SERVICE_UUID, 12);

    // CSC Measurement (0x2A5B) — Notify
    cscMeasurementChar = cscService->createCharacteristic(
        CSC_MEASUREMENT_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    cscMeasurementChar->addDescriptor(new BLE2902());

    // CSC Feature (0x2A5C) — Read
    // Bit 1 = Crank Revolution Data Supported → 0x02
    BLECharacteristic* cscFeatureChar = cscService->createCharacteristic(
        CSC_FEATURE_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t cscFeature[2] = {0x02, 0x00};  // Crank Revolution Data Supported
    cscFeatureChar->setValue(cscFeature, 2);

    // CSC also uses Sensor Location — reuse the same value
    BLECharacteristic* cscSensorLoc = cscService->createCharacteristic(
        SENSOR_LOCATION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t cscLoc = 0x05;  // Left Crank
    cscSensorLoc->setValue(&cscLoc, 1);

    cscService->start();

    // ── Device Information Service (0x180A) ──
    // Helps apps identify this device and improves compatibility.
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

    // ── Battery Service (0x180F) ──
    // Re-broadcasts the sensor's battery level so apps can show it.
    BLEService* battService = bleServer->createService(BATTERY_SERVICE_UUID, 8);
    batteryLevelChar = battService->createCharacteristic(
        BATTERY_LEVEL_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    batteryLevelChar->addDescriptor(new BLE2902());
    uint8_t initBatt = 0;
    batteryLevelChar->setValue(&initBatt, 1);
    battService->start();

    // Set up advertising
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(CPS_SERVICE_UUID);
    pAdvertising->addServiceUUID(CSC_SERVICE_UUID);
    pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
    pAdvertising->setAppearance(0x0484);   // Cycling Power Sensor
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);   // Connection interval min (7.5 ms)
    pAdvertising->setMaxPreferred(0x12);   // Connection interval max (22.5 ms)
    BLEDevice::startAdvertising();

    Serial.printf("[SERVER] Advertising as '%s' with Cycling Power Service\n", BRIDGE_NAME);
    Serial.printf("[SERVER] Smoothing factor: %.1f | Dropout timeout: %d ms\n",
                  SMOOTHING_FACTOR, DROPOUT_MS);
}

// ─── Arduino Setup ──────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=========================================");
    Serial.println("  ESP32 BLE Bike Power Meter Bridge");
    Serial.printf ("  Firmware: %s\n", FW_VERSION);
    Serial.println("=========================================");
    Serial.println();
    Serial.println("This device connects to your MYX/BKSNSR");
    Serial.println("power meter, decodes the XOR-masked data,");
    Serial.println("and re-broadcasts it as a standard BLE");
    Serial.println("Cycling Power Service.");
    Serial.println();

    // LED setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Safe initial value for dropout timer
    lastCrankMoveMs = millis();

    // Initialize BLE with enough memory for both client and server roles
    BLEDevice::init(BRIDGE_NAME);

    // Set up the server side first (so apps can find us immediately)
    setupBLEServer();

    // Start scanning for the sensor
    Serial.println("[SCAN] Starting BLE scan for power meter...");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SensorScanCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(10, false);  // Scan for 10 seconds

    Serial.println("[SCAN] Initial scan started...");
}

// ─── Arduino Loop ───────────────────────────────────────────────────────────

void loop() {
    // ── Handle connection to sensor ──
    if (doConnect) {
        if (connectToSensor()) {
            Serial.println("[OK] Bridge is active! Power data will flow through.");
        } else {
            Serial.println("[ERROR] Failed to connect to sensor. Will retry...");
            doScan = true;
        }
        doConnect = false;
    }

    // ── Handle re-scan if disconnected ──
    if (!sensorConnected && doScan) {
        Serial.println("[SCAN] Re-scanning for power meter...");
        BLEDevice::getScan()->start(10, false);
        doScan = false;
    }

    // ── LED status feedback ──
    unsigned long now = millis();
    if (sensorConnected && appConnectionCount > 0) {
        // Solid LED = sensor connected AND app connected (full bridge active)
        if (!ledState) {
            digitalWrite(LED_PIN, HIGH);
            ledState = true;
        }
    } else if (sensorConnected) {
        // Fast blink = sensor connected, waiting for app
        if (now - lastBlinkTime > 250) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            lastBlinkTime = now;
        }
    } else {
        // Slow blink = searching for sensor
        if (now - lastBlinkTime > 1000) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            lastBlinkTime = now;
        }
    }

    // ── Periodic status print ──
    static unsigned long lastStatusTime = 0;
    if (now - lastStatusTime > 10000) {
        lastStatusTime = now;
        char battStr[8];
        if (lastBattery == 0xFF) snprintf(battStr, sizeof(battStr), "?");
        else snprintf(battStr, sizeof(battStr), "%d%%", lastBattery);
        Serial.printf("[STATUS] Sensor: %s | Apps: %d | Power: %d W | Cadence: %.0f RPM | Battery: %s\n",
                      sensorConnected ? "CONNECTED" : "SEARCHING",
                      appConnectionCount,
                      (int)lastPower,
                      lastCadence,
                      battStr);
    }

    delay(10);
}
