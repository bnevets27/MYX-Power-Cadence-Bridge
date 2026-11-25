/*
 * MYX Power & Cadence Bridge - Unified Firmware
 * =============================================
 *
 * One firmware image that:
 * - Bridges the MYX bike BLE sensor to standard cycling apps
 * - Provides sensor tune / diagnostics and local fallback correction
 * - Offers WiFi and MQTT configuration in one shared web UI
 * - Publishes ride data to MQTT with Home Assistant discovery
 * - Supports OTA firmware updates
 *
 * Core product rule: BLE bridge behavior is the top priority. Sensor loss
 * must trigger recovery, not require a reboot.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <nvs.h>
#include <math.h>

// Product configuration
static const char* TARGET_SENSOR_NAME = "BKSNSR";
static const char* BRIDGE_NAME = "MYX Bridge";
static const char* AP_NAME = "MYX-Bridge-Setup";
static char adminPassword[64];
static bool adminConfigured = false;
static bool setupWindow = false;
static bool setupCommitted = false;
static uint32_t setupStarted = 0;
static char setupToken[33];
static volatile bool bootButtonArmed = false;
static volatile uint32_t bootButtonDown = 0;
static volatile bool bootButtonPressed = false;
static volatile bool bootButtonEvent = false;
static portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR onBootButtonEdge() {
    portENTER_CRITICAL_ISR(&buttonMux);
    uint32_t now = millis();
    if (digitalRead(0) == HIGH) {
        if (bootButtonArmed && bootButtonPressed && now - bootButtonDown >= 5000) bootButtonEvent = true;
        bootButtonPressed = false;
        bootButtonArmed = true;
    } else if (bootButtonArmed) {
        bootButtonDown = now;
        bootButtonPressed = true;
    }
    portEXIT_CRITICAL_ISR(&buttonMux);
}
static const char* FW_VERSION = "2.1.4-unified";

static const uint8_t XOR_KEY = 0xAA;
static const float SMOOTHING_FACTOR = 0.5f;
static const uint32_t DROPOUT_MS = 2500;
// Cycling Power Profile procedure timeout is the 30-second ATT transaction timeout.
static const uint32_t TUNE_TIMEOUT_MS = 30000;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 15000;
static const uint32_t MQTT_RETRY_INTERVAL_MS = 5000;
static const uint32_t MQTT_PUBLISH_INTERVAL_MS = 2000;

static const uint8_t CP_OP_START_OFFSET_COMPENSATION = 0x0C;
static const uint8_t CP_OP_START_ENHANCED_OFFSET_COMPENSATION = 0x10;
static const uint8_t CP_OP_RESPONSE_CODE = 0x20;
// MYX has been observed using 0x10 as the response header.
static const uint8_t CP_OP_MYX_RESPONSE_CODE = 0x10;
static const uint32_t CP_FEATURE_ENHANCED_OFFSET = 1UL << 19;
static const uint8_t CP_RESPONSE_SUCCESS = 0x01;
static const uint8_t CP_RESPONSE_NOT_SUPPORTED = 0x02;
static const uint8_t CP_RESPONSE_OPERATION_FAILED = 0x04;
static const uint16_t MQTT_DEFAULT_PORT = 1883;
static const char* MQTT_DISCOVERY_PREFIX = "homeassistant";

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

struct CalibrationSettings {
    bool correctionEnabled;
    bool rawDiagnosticsEnabled;
    float localZeroOffset;
    float scaleFactor;
    float cadence[4];
    float cadenceOffset[4];
};

struct WiFiConfig {
    bool provisioned;
    char ssid[33];
    char password[65];
};

struct MqttConfig {
    bool enabled;
    char host[64];
    uint16_t port;
    char user[32];
    char pass[64];
};

struct CachedSensorConfig {
    bool valid;
    char address[18];
    uint8_t addrType;
    char lastName[32];
    char lastPrefix[16];
};

static WiFiConfig defaultWiFiConfig() {
    WiFiConfig cfg = {};
    cfg.provisioned = false;
    return cfg;
}

static MqttConfig defaultMqttConfig() {
    MqttConfig cfg = {};
    cfg.enabled = false;
    cfg.port = MQTT_DEFAULT_PORT;
    return cfg;
}

static CachedSensorConfig defaultCachedSensorConfig() {
    CachedSensorConfig cfg = {};
    cfg.valid = false;
    cfg.addrType = (uint8_t)BLE_ADDR_TYPE_PUBLIC;
    return cfg;
}

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

enum RecoveryState {
    RECOVERY_IDLE,
    RECOVERY_CACHED_RECONNECT,
    RECOVERY_SCANNING,
    RECOVERY_CONNECTING,
    RECOVERY_CONNECTED
};

static CalibrationSettings calibration = {
    false,
    false,
    0.0f,
    1.0f,
    {60.0f, 70.0f, 80.0f, 90.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

static WiFiConfig wifiConfig = defaultWiFiConfig();
static MqttConfig mqttConfig = defaultMqttConfig();
static CachedSensorConfig cachedSensor = defaultCachedSensorConfig();

// BLE client state: connection to the real sensor
static BLEAdvertisedDevice* sensorDevice = nullptr;
static BLEClient* bleClient = nullptr;
static BLERemoteCharacteristic* remotePowerControlPoint = nullptr;
static volatile bool sensorConnected = false;
static volatile bool remoteControlPointReady = false;
static volatile bool connectUsingCachedAddress = false;
static volatile bool doConnect = false;
static volatile bool doScan = false;
static volatile bool scanRunning = false;
static volatile bool scanAfterDisconnect = false;
static volatile bool disconnectPending = false;
static volatile bool disconnectDispatchComplete = false;
static volatile RecoveryState recoveryState = RECOVERY_IDLE;

// BLE server state: what apps connect to
static BLEServer* bleServer = nullptr;
static BLECharacteristic* powerMeasurementChar = nullptr;
static BLECharacteristic* powerControlPointChar = nullptr;
static BLECharacteristic* cscMeasurementChar = nullptr;
static BLECharacteristic* batteryLevelChar = nullptr;
static int appConnectionCount = 0;

// Web, MQTT, and storage
static Preferences preferences;
static WebServer webServer(80);
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static bool mqttConfigured = false;
static bool mqttConnected = false;
static bool webServerStarted = false;
static bool apRunning = false;
static bool routesRegistered = false;
static bool restartPending = false;
static bool otaComplete = false;
static uint32_t restartAtMs = 0;
static unsigned long lastMqttPublish = 0;
static unsigned long lastMqttReconnectAttempt = 0;
static unsigned long lastWiFiConnectAttempt = 0;
static bool wifiConnectInProgress = false;
static unsigned long wifiConnectStartedAt = 0;
static char deviceId[20];
static char mqttStateTopic[48];
static char mqttAvailTopic[48];

// Latest decoded/corrected data for logs/UI/MQTT
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
static volatile uint32_t lastSensorPacketMs = 0;
static volatile uint32_t lastPacketIntervalMs = 0;
static volatile uint16_t lastCrankRevDelta = 0;
static volatile uint16_t lastCrankTimeDelta = 0;
static volatile bool lastCadenceHeld = false;

// Cadence and smoothing state
static uint16_t prevCrankRev = 0;
static uint16_t prevCrankTime = 0;
static float smoothedPower = 0.0f;
static uint32_t lastCrankMoveMs = 0;

// Sensor tune state
static volatile TuneState tuneState = TUNE_IDLE;
static volatile uint8_t tuneLastOp = 0;
static volatile uint8_t tuneLastResponse = 0;
static volatile int16_t tuneReportedOffset = 0;
static volatile bool tuneOffsetReported = false;
static volatile uint8_t tuneFailureReason = 0;
static volatile bool tuneRequested = false;
static volatile bool tuneTimedOut = false;
static uint32_t tuneStartedMs = 0;
static portMUX_TYPE tuneMux = portMUX_INITIALIZER_UNLOCKED;
// Only fixed-size data is copied from BLE callbacks; formatting happens on the loop task.
struct TuneDiagnostics {
    int clientIf = -1;
    uint16_t connId = 0, cpHandle = 0, cccdHandle = 0;
    bool canWrite = false, canIndicate = false, canNotify = false;
    bool featureValid = false;
    uint32_t featureBits = 0;
    bool writeSeen = false, cccdSeen = false, registrationSeen = false;
    int writeStatus = -1, cccdStatus = -1, registrationStatus = -1;
    uint32_t writeStartedMs = 0, writeEventMs = 0, cccdEventMs = 0;
    uint32_t writeCount = 0, cccdCount = 0, responseCount = 0;
    uint32_t registrationCount = 0, registrationEventMs = 0;
    uint32_t responseMs = 0;
    size_t responseLength = 0, responseCaptured = 0;
    bool responseNotify = false, responseAccepted = false;
    uint8_t responseState = 0, responseExpectedOp = 0;
    uint8_t responseBytes[16] = {};
};
static TuneDiagnostics tuneDiag;
static uint32_t loggedTuneWriteCount = 0, loggedTuneCccdCount = 0, loggedTuneResponseCount = 0, loggedTuneRegistrationCount = 0;

// UI action state
static volatile bool reconnectRequested = false;
static volatile bool findAgainRequested = false;
static char lastUiMessage[160] = "Booting";
static char lastMqttTestResult[96] = "Not tested";
static constexpr uint8_t EVENT_LOG_CAPACITY = 64;
static constexpr size_t EVENT_LOG_LINE = 160;
static char eventLog[EVENT_LOG_CAPACITY][EVENT_LOG_LINE];
static portMUX_TYPE eventLogMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t eventLogNext = 0;
static uint8_t eventLogCount = 0;

#ifndef LED_PIN
#define LED_PIN 2
#endif
static unsigned long lastBlinkTime = 0;
static bool ledState = false;

static void scheduleRestart(const char* reason);
static void sendNoCacheHeaders();

// Allocate snapshots before locking; no I/O or allocation occurs in the critical section.
struct EventLogSnapshot {
    char lines[EVENT_LOG_CAPACITY][EVENT_LOG_LINE];
    uint8_t count;
};
static EventLogSnapshot* snapshotEventLog() {
    EventLogSnapshot* snapshot = (EventLogSnapshot*)malloc(sizeof(EventLogSnapshot));
    if (!snapshot) return nullptr;
    portENTER_CRITICAL(&eventLogMux);
    snapshot->count = eventLogCount;
    for (uint8_t i = 0; i < snapshot->count; ++i) {
        uint8_t idx = (eventLogNext + EVENT_LOG_CAPACITY - eventLogCount + i) % EVENT_LOG_CAPACITY;
        memcpy(snapshot->lines[i], eventLog[idx], EVENT_LOG_LINE);
    }
    portEXIT_CRITICAL(&eventLogMux);
    return snapshot;
}

static void setUiMessage(const char* text) {
    char line[EVENT_LOG_LINE];
    snprintf(line, sizeof(line), "[%lu ms] %s", (unsigned long)millis(), text);
    portENTER_CRITICAL(&eventLogMux);
    strlcpy(lastUiMessage, text, sizeof(lastUiMessage));
    strlcpy(eventLog[eventLogNext], line, sizeof(eventLog[eventLogNext]));
    eventLogNext = (uint8_t)((eventLogNext + 1) % EVENT_LOG_CAPACITY);
    if (eventLogCount < EVENT_LOG_CAPACITY) eventLogCount++;
    portEXIT_CRITICAL(&eventLogMux);
    Serial.printf("[UI] %s\n", line);
}

static const char* tuneStateText(TuneState state) {
    switch (state) {
        case TUNE_IDLE: return "idle";
        case TUNE_NO_CONTROL_POINT: return "sensor control point unavailable";
        case TUNE_STANDARD_SENT: return "standard tune requested";
        case TUNE_ENHANCED_SENT: return "enhanced tune requested";
        case TUNE_SUCCESS: return "tune complete";
        case TUNE_NOT_SUPPORTED: return "sensor tune not supported";
        case TUNE_FAILED: return "sensor tune failed";
        case TUNE_TIMEOUT: return "sensor tune timed out; reconnect sensor before retrying";
        default: return "unknown";
    }
}

static const char* recoveryStateText(RecoveryState state) {
    switch (state) {
        case RECOVERY_IDLE: return "idle";
        case RECOVERY_CACHED_RECONNECT: return "cached reconnect";
        case RECOVERY_SCANNING: return "scanning";
        case RECOVERY_CONNECTING: return "connecting";
        case RECOVERY_CONNECTED: return "connected";
        default: return "unknown";
    }
}

static const char* servingModeText() {
    if (wifiConfig.provisioned) {
        return WiFi.status() == WL_CONNECTED ? "lan" : "lan-waiting";
    }
    return apRunning ? "ap" : "offline";
}

static void buildDeviceId() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(deviceId, sizeof(deviceId), "myx_bridge_%02x%02x%02x", mac[3], mac[4], mac[5]);
    snprintf(mqttStateTopic, sizeof(mqttStateTopic), "%s/state", deviceId);
    snprintf(mqttAvailTopic, sizeof(mqttAvailTopic), "%s/availability", deviceId);
}

static void loadAdminPassword() {
    preferences.begin("security", true);
    String saved = preferences.getString("user_admin", "");
    adminConfigured = saved.length() >= 8 && saved.length() <= 63;
    strlcpy(adminPassword, saved.c_str(), sizeof(adminPassword));
    preferences.end();
}

static bool authorizeWeb() {
    if (setupWindow || !adminConfigured) {
        webServer.send(403, "text/plain", "Finish setup first");
        return false;
    }
    if (!webServer.authenticate("admin", adminPassword)) {
        webServer.requestAuthentication();
        return false;
    }
    if (webServer.method() == HTTP_POST && webServer.hasHeader("Origin")) {
        String expected = "http://" + webServer.header("Host");
        if (webServer.header("Origin") != expected) {
            webServer.send(403, "text/plain", "Origin rejected");
            return false;
        }
    }
    return true;
}

static String escapeHtml(const char* input) {
    String result;
    while (*input) {
        switch (*input) {
            case '&': result += "&amp;"; break;
            case '<': result += "&lt;"; break;
            case '>': result += "&gt;"; break;
            case '\'': result += "&#39;"; break;
            case '"': result += "&quot;"; break;
            default: result += *input;
        }
        ++input;
    }
    return result;
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

static void loadWiFiConfig() {
    preferences.begin("wifi", true);
    wifiConfig.provisioned = preferences.getBool("provisioned", false);
    strlcpy(wifiConfig.ssid, preferences.getString("ssid", "").c_str(), sizeof(wifiConfig.ssid));
    strlcpy(wifiConfig.password, preferences.getString("pass", "").c_str(), sizeof(wifiConfig.password));
    preferences.end();
    if (strlen(wifiConfig.ssid) == 0) {
        wifiConfig.provisioned = false;
    }
}

static void saveWiFiConfig() {
    preferences.begin("wifi", false);
    preferences.putBool("provisioned", wifiConfig.provisioned);
    preferences.putString("ssid", wifiConfig.ssid);
    preferences.putString("pass", wifiConfig.password);
    preferences.end();
}

static void clearWiFiConfig() {
    preferences.begin("wifi", false);
    preferences.clear();
    preferences.end();
    wifiConfig = defaultWiFiConfig();
}

static void loadMqttConfig() {
    preferences.begin("mqtt", true);
    mqttConfig.enabled = preferences.getBool("enabled", false);
    strlcpy(mqttConfig.host, preferences.getString("host", "").c_str(), sizeof(mqttConfig.host));
    mqttConfig.port = preferences.getUShort("port", MQTT_DEFAULT_PORT);
    strlcpy(mqttConfig.user, preferences.getString("user", "").c_str(), sizeof(mqttConfig.user));
    strlcpy(mqttConfig.pass, preferences.getString("pass", "").c_str(), sizeof(mqttConfig.pass));
    preferences.end();
    mqttConfigured = mqttConfig.enabled && strlen(mqttConfig.host) > 0;
}

static void saveMqttConfig() {
    preferences.begin("mqtt", false);
    preferences.putBool("enabled", mqttConfig.enabled);
    preferences.putString("host", mqttConfig.host);
    preferences.putUShort("port", mqttConfig.port);
    preferences.putString("user", mqttConfig.user);
    preferences.putString("pass", mqttConfig.pass);
    preferences.end();
    mqttConfigured = mqttConfig.enabled && strlen(mqttConfig.host) > 0;
}

static void clearMqttConfig() {
    preferences.begin("mqtt", false);
    preferences.clear();
    preferences.end();
    mqttConfig = defaultMqttConfig();
    mqttConfigured = false;
}

static void loadCachedSensor() {
    preferences.begin("sensor", true);
    cachedSensor.valid = preferences.getBool("valid", false);
    strlcpy(cachedSensor.address, preferences.getString("addr", "").c_str(), sizeof(cachedSensor.address));
    cachedSensor.addrType = preferences.getUChar("type", (uint8_t)BLE_ADDR_TYPE_PUBLIC);
    strlcpy(cachedSensor.lastName, preferences.getString("name", "").c_str(), sizeof(cachedSensor.lastName));
    strlcpy(cachedSensor.lastPrefix, preferences.getString("prefix", "").c_str(), sizeof(cachedSensor.lastPrefix));
    preferences.end();
    if (strlen(cachedSensor.address) == 0) {
        cachedSensor.valid = false;
    }
}

static void saveCachedSensor(const BLEAddress& address, esp_ble_addr_type_t addrType, const char* name) {
    cachedSensor.valid = true;
    BLEAddress addrCopy = address;
    strlcpy(cachedSensor.address, addrCopy.toString().c_str(), sizeof(cachedSensor.address));
    cachedSensor.addrType = (uint8_t)addrType;
    strlcpy(cachedSensor.lastName, name != nullptr ? name : "", sizeof(cachedSensor.lastName));
    strlcpy(cachedSensor.lastPrefix, TARGET_SENSOR_NAME, sizeof(cachedSensor.lastPrefix));

    preferences.begin("sensor", false);
    preferences.putBool("valid", cachedSensor.valid);
    preferences.putString("addr", cachedSensor.address);
    preferences.putUChar("type", cachedSensor.addrType);
    preferences.putString("name", cachedSensor.lastName);
    preferences.putString("prefix", cachedSensor.lastPrefix);
    preferences.end();
}

static void clearCachedSensor() {
    preferences.begin("sensor", false);
    preferences.clear();
    preferences.end();
    cachedSensor = defaultCachedSensorConfig();
}

static void fullReset() {
    clearWiFiConfig();
    clearMqttConfig();
    clearCachedSensor();
    resetCalibration();
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

static float calcCadence(uint16_t currRev, uint16_t currTime, uint16_t prevRevLocal, uint16_t prevTimeLocal) {
    uint16_t dRev = (uint16_t)(currRev - prevRevLocal);
    uint16_t dTime = (uint16_t)(currTime - prevTimeLocal);
    if (dRev == 0 || dTime == 0) return 0.0f;
    return 60.0f * 1024.0f * (float)dRev / (float)dTime;
}

static float argFloat(const char* name, float fallback) {
    if (!webServer.hasArg(name)) return fallback;
    return webServer.arg(name).toFloat();
}

static uint16_t argU16(const char* name, uint16_t fallback) {
    if (!webServer.hasArg(name)) return fallback;
    long value = webServer.arg(name).toInt();
    if (value <= 0 || value > 65535) return fallback;
    return (uint16_t)value;
}

static void jsonEscapePrint(const char* text) {
    while (*text) {
        char c = *text++;
        if (c == '"' || c == '\\') webServer.sendContent("\\");
        char out[2] = {c, '\0'};
        webServer.sendContent(out);
    }
}

static void scheduleRestart(const char* reason) {
    restartPending = true;
    restartAtMs = millis() + 1500;
    setUiMessage(reason);
}

static void cleanupSensorDevice() {
    if (sensorDevice != nullptr) {
        delete sensorDevice;
        sensorDevice = nullptr;
    }
}

static void cleanupBleClient() {
    if (bleClient != nullptr) {
        if (bleClient->isConnected()) {
            disconnectPending = true;
            bleClient->disconnect();
        }
    }
    remoteControlPointReady = false;
    remotePowerControlPoint = nullptr;
}

static void stopBleScan() {
    if (scanRunning) {
        BLEDevice::getScan()->stop();
        scanRunning = false;
    }
}

static void startBleScan() {
    if (scanRunning || sensorConnected) return;
    recoveryState = RECOVERY_SCANNING;
    setUiMessage("Scanning for bike sensor");
    if (BLEDevice::getScan()->start(0, nullptr, false)) {
        scanRunning = true;
        doScan = false;
    } else {
        setUiMessage("BLE scan start failed");
    }
}

static void stopAccessPoint() {
    if (!apRunning) return;
    if (webServerStarted && !wifiConfig.provisioned) {
        webServer.stop();
        webServerStarted = false;
        Serial.println("[WEB] Web server stopped");
    }
    WiFi.softAPdisconnect(true);
    apRunning = false;
    if (wifiConfig.provisioned) {
        WiFi.mode(WIFI_STA);
    } else {
        WiFi.mode(WIFI_OFF);
    }
    setUiMessage("Access point stopped while sensor is disconnected");
}

static void startAccessPointIfNeeded() {
    if (apRunning || (!setupWindow && (!adminConfigured || (!wifiConfig.provisioned && !sensorConnected)))) return;
    WiFi.mode(wifiConfig.provisioned ? WIFI_AP_STA : WIFI_AP);
    bool apStarted = setupWindow ? WiFi.softAP(AP_NAME) : WiFi.softAP(AP_NAME, adminPassword);
    if (apStarted) {
        apRunning = true;
        if (!webServerStarted) {
            webServer.begin();
            webServerStarted = true;
            Serial.println("[WEB] Web server started");
        }
        setUiMessage("Access point active");
        Serial.printf("[WEB] AP active: %s at http://%s/\n", AP_NAME, WiFi.softAPIP().toString().c_str());
    } else {
        setUiMessage("Access point failed to start");
    }
}

static void openSetupWindow() {
    if (apRunning) stopAccessPoint();
    setupCommitted = false;
    snprintf(setupToken, sizeof(setupToken), "%08lx%08lx%08lx%08lx", (unsigned long)esp_random(), (unsigned long)esp_random(), (unsigned long)esp_random(), (unsigned long)esp_random());
    setupWindow = true;
    setupStarted = millis();
    startAccessPointIfNeeded();
}

static bool setupRequestAllowed() {
    return setupWindow && !setupCommitted && millis() - setupStarted < 600000 && apRunning && webServer.client().localIP() == WiFi.softAPIP();
}

static void handleSetupPage() {
    if (!setupRequestAllowed()) { webServer.send(403, "text/plain", "Setup is only available on the setup network"); return; }
    String page = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>MYX Bridge setup</title><style>body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#e0e0e0;max-width:520px;margin:auto;padding:24px}h1{color:#00d4ff}form{background:#16213e;padding:20px;border-radius:12px}label{display:block;margin:16px 0;color:#cbd5e1}input{display:block;box-sizing:border-box;width:100%;padding:12px;margin-top:6px;border:1px solid #52637b;border-radius:6px;font-size:16px}button{background:#533483;color:white;border:0;border-radius:6px;padding:14px;font-size:16px;width:100%}</style></head><body><h1>MYX Bridge setup</h1><p>Choose an administrator password. You can optionally configure Wi-Fi now.</p><form method='post' action='/setup'><input type='hidden' name='token' value='";
    page += setupToken;
    page += "'><label>New password (8 to 63 characters) <input type='password' name='admin' minlength='8' maxlength='63' required></label><label>Confirm password <input type='password' name='confirm' minlength='8' maxlength='63' required></label><p>Leave the network name blank to keep existing Wi-Fi settings. If you enter a network name, leave its password blank only for an open network.</p><label>Wi-Fi network name (optional) <input name='ssid' maxlength='32'></label><label>Wi-Fi password <input type='password' name='wifi_password' maxlength='64'></label><button type='submit'>Save and restart</button></form></body></html>";
    sendNoCacheHeaders();
    webServer.send(200, "text/html", page);
}

static void handleSetupPost() {
    if (!setupRequestAllowed() || !webServer.hasArg("token") || webServer.arg("token") != setupToken ||
        (webServer.hasHeader("Origin") && webServer.header("Origin") != "http://" + webServer.header("Host"))) {
        webServer.send(403, "text/plain", "Setup request rejected"); return;
    }
    String pass = webServer.arg("admin");
    String ssid = webServer.arg("ssid");
    String wifiPass = webServer.arg("wifi_password");
    if (pass.length() < 8 || pass.length() > 63 || pass != webServer.arg("confirm") || ssid.length() > 32 || wifiPass.length() > 64) {
        webServer.send(400, "text/plain", "Check the passwords and Wi-Fi settings, then return and try again."); return;
    }
    for (size_t i = 0; i < pass.length(); ++i) {
        if ((uint8_t)pass[i] < 32 || (uint8_t)pass[i] > 126) {
            webServer.send(400, "text/plain", "Use printable ASCII characters for the password."); return;
        }
    }
    if (ssid.length()) {
        nvs_handle_t handle;
        if (nvs_open("wifi", NVS_READWRITE, &handle) != ESP_OK) { webServer.send(500, "text/plain", "Could not save Wi-Fi; try again."); return; }
        bool wifiSaved = nvs_set_str(handle, "ssid", ssid.c_str()) == ESP_OK &&
                         nvs_set_str(handle, "pass", wifiPass.c_str()) == ESP_OK &&
                         nvs_set_u8(handle, "provisioned", 1) == ESP_OK &&
                         nvs_commit(handle) == ESP_OK;
        nvs_close(handle);
        if (!wifiSaved) { webServer.send(500, "text/plain", "Could not save Wi-Fi; try again."); return; }
        strlcpy(wifiConfig.ssid, ssid.c_str(), sizeof(wifiConfig.ssid));
        strlcpy(wifiConfig.password, wifiPass.c_str(), sizeof(wifiConfig.password));
        wifiConfig.provisioned = true;
    }
    if (!preferences.begin("security", false)) { webServer.send(500, "text/plain", "Could not save password; try again."); return; }
    bool saved = preferences.putString("user_admin", pass) == pass.length();
    preferences.end();
    if (!saved) { webServer.send(500, "text/plain", "Could not save password; try again."); return; }
    strlcpy(adminPassword, pass.c_str(), sizeof(adminPassword));
    adminConfigured = true;
    setupCommitted = true;
    webServer.send(200, "text/html", "<p>Settings saved. The bridge is restarting. Reconnect to your Wi-Fi network, or to MYX-Bridge-Setup using your new password, then sign in as admin.</p>");
    scheduleRestart("Setup complete, restarting");
}

static void beginWiFiConnect() {
    if (!wifiConfig.provisioned) return;
    WiFi.mode(apRunning ? WIFI_AP_STA : WIFI_STA);
    WiFi.begin(wifiConfig.ssid, wifiConfig.password);
    wifiConnectInProgress = true;
    wifiConnectStartedAt = millis();
    lastWiFiConnectAttempt = millis();
    setUiMessage("Connecting to WiFi");
    Serial.printf("[WIFI] Connecting to %s\n", wifiConfig.ssid);
}

static void ensureWiFiState() {
    if (setupWindow) return;
    if (!wifiConfig.provisioned) {
        if (WiFi.getMode() != WIFI_OFF && !apRunning) {
            WiFi.mode(WIFI_OFF);
        }
        return;
    }

    wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
        if (apRunning) stopAccessPoint();
        if (wifiConnectInProgress) {
            wifiConnectInProgress = false;
            setUiMessage("WiFi connected");
            Serial.printf("[WIFI] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
        }
        if (!webServerStarted) {
            webServer.begin();
            webServerStarted = true;
            Serial.println("[WEB] Web server started");
        }
        return;
    }

    if (!wifiConnectInProgress && millis() - lastWiFiConnectAttempt >= WIFI_RETRY_INTERVAL_MS) {
        beginWiFiConnect();
    }

    if (wifiConnectInProgress && millis() - wifiConnectStartedAt > WIFI_CONNECT_TIMEOUT_MS) {
        wifiConnectInProgress = false;
        WiFi.disconnect();
        setUiMessage("WiFi connect timed out, retrying");
        startAccessPointIfNeeded();
    }
}

static bool writeSensorTuneOp(uint8_t opCode, TuneState waitingState) {
    BLERemoteCharacteristic* controlPoint = remotePowerControlPoint;
    if (!sensorConnected || !remoteControlPointReady || controlPoint == nullptr) {
        tuneState = TUNE_NO_CONTROL_POINT;
        Serial.println("[TUNE] Sensor Cycling Power Control Point unavailable");
        return false;
    }

    uint8_t payload[1] = {opCode};
    portENTER_CRITICAL(&tuneMux);
    if (!sensorConnected || !remoteControlPointReady) {
        portEXIT_CRITICAL(&tuneMux);
        return false;
    }
    tuneLastOp = opCode;
    tuneLastResponse = 0;
    tuneReportedOffset = 0;
    tuneOffsetReported = false;
    tuneFailureReason = 0;
    tuneState = waitingState;
    tuneStartedMs = millis();
    tuneDiag.writeStartedMs = tuneStartedMs;
    tuneDiag.writeSeen = false;
    tuneDiag.writeStatus = -1;
    portEXIT_CRITICAL(&tuneMux);
    // Client destruction is deferred until the disconnect event has completed.
    controlPoint->writeValue(payload, sizeof(payload), true);
    Serial.printf("[TUNE] Requested sensor tune op 0x%02X\n", opCode);
    return true;
}

static void serviceTuneState() {
    TuneDiagnostics snapshot;
    portENTER_CRITICAL(&tuneMux);
    snapshot = tuneDiag;
    if ((snapshot.cccdSeen && snapshot.cccdStatus != ESP_GATT_OK) ||
        (snapshot.registrationSeen && snapshot.registrationStatus != ESP_GATT_OK)) remoteControlPointReady = false;
    if (snapshot.writeSeen && snapshot.writeStatus != ESP_GATT_OK &&
        (tuneState == TUNE_STANDARD_SENT || tuneState == TUNE_ENHANCED_SENT)) {
        tuneState = TUNE_FAILED;
    }
    portEXIT_CRITICAL(&tuneMux);
    char detail[96];
    if (snapshot.cccdCount != loggedTuneCccdCount) {
        loggedTuneCccdCount = snapshot.cccdCount;
        snprintf(detail, sizeof(detail), "Tune CCCD write status 0x%02X at %lu ms", snapshot.cccdStatus, (unsigned long)snapshot.cccdEventMs);
        setUiMessage(detail);
    }
    if (snapshot.registrationCount != loggedTuneRegistrationCount) {
        loggedTuneRegistrationCount = snapshot.registrationCount;
        snprintf(detail, sizeof(detail), "Tune notification registration status 0x%02X at %lu ms", snapshot.registrationStatus, (unsigned long)snapshot.registrationEventMs);
        setUiMessage(detail);
    }
    if (snapshot.writeCount != loggedTuneWriteCount) {
        loggedTuneWriteCount = snapshot.writeCount;
        snprintf(detail, sizeof(detail), "Tune op 0x%02X write status 0x%02X at %lu ms", (unsigned)tuneLastOp, snapshot.writeStatus, (unsigned long)snapshot.writeEventMs);
        setUiMessage(detail);
    }
    if (snapshot.responseCount != loggedTuneResponseCount) {
        loggedTuneResponseCount = snapshot.responseCount;
        snprintf(detail, sizeof(detail), "Tune response %s len %u code 0x%02X at %lu ms", snapshot.responseAccepted ? "accepted" : "ignored", (unsigned)snapshot.responseLength, snapshot.responseCaptured >= 3 ? snapshot.responseBytes[2] : 0, (unsigned long)snapshot.responseMs);
        setUiMessage(detail);
    }
    if (tuneRequested) {
        tuneRequested = false;
        // Prefer enhanced only when the sensor advertises it. Otherwise try
        // standard offset, including when the feature characteristic is absent.
        if (snapshot.featureValid && (snapshot.featureBits & CP_FEATURE_ENHANCED_OFFSET))
            writeSensorTuneOp(CP_OP_START_ENHANCED_OFFSET_COMPENSATION, TUNE_ENHANCED_SENT);
        else
            writeSensorTuneOp(CP_OP_START_OFFSET_COMPENSATION, TUNE_STANDARD_SENT);
    }

    portENTER_CRITICAL(&tuneMux);
    bool tuneExpired = (tuneState == TUNE_STANDARD_SENT || tuneState == TUNE_ENHANCED_SENT) &&
        millis() - tuneStartedMs >= TUNE_TIMEOUT_MS;
    if (tuneExpired) {
        tuneState = TUNE_TIMEOUT;
        tuneTimedOut = true;
    }
    portEXIT_CRITICAL(&tuneMux);
    if (tuneExpired) {
        setUiMessage("Sensor tune timed out");
        Serial.println("[TUNE] Sensor tune timed out");
    }

    if (tuneState == TUNE_NOT_SUPPORTED && tuneLastOp == CP_OP_START_ENHANCED_OFFSET_COMPENSATION) {
        setUiMessage("Enhanced tune unsupported, trying standard tune");
        Serial.println("[TUNE] Enhanced tune unsupported; trying standard tune");
        writeSensorTuneOp(CP_OP_START_OFFSET_COMPENSATION, TUNE_STANDARD_SENT);
    }
}

class BridgeServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        (void)pServer;
        appConnectionCount++;
        Serial.printf("[SERVER] App connected! (%d total)\n", appConnectionCount);
        BLEDevice::startAdvertising();
    }

    void onDisconnect(BLEServer* pServer) override {
        (void)pServer;
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

static void remoteControlPointCallback(
    BLERemoteCharacteristic* pChar,
    uint8_t* pData,
    size_t length,
    bool isNotify)
{
    (void)pChar;
    Serial.printf("[TUNE] %s received (%u bytes):", isNotify ? "Notification" : "Indication", (unsigned)length);
    for (size_t i = 0; i < length && i < 16; ++i) Serial.printf(" %02X", pData[i]);
    if (length > 16) Serial.print(" ...");
    Serial.println();
    portENTER_CRITICAL(&tuneMux);
    // Accept only the two observed forms using the nonstandard MYX header.
    bool myxUnsupported = length == 3 && pData[0] == CP_OP_MYX_RESPONSE_CODE &&
        pData[1] == CP_OP_START_ENHANCED_OFFSET_COMPENSATION &&
        pData[2] == CP_RESPONSE_NOT_SUPPORTED;
    bool myxStandardSuccess = length == 3 && pData[0] == CP_OP_MYX_RESPONSE_CODE &&
        pData[1] == CP_OP_START_OFFSET_COMPENSATION &&
        pData[2] == CP_RESPONSE_SUCCESS;
    bool accepted = length >= 3 &&
        (pData[0] == CP_OP_RESPONSE_CODE || myxUnsupported || myxStandardSuccess) &&
        (tuneState == TUNE_STANDARD_SENT || tuneState == TUNE_ENHANCED_SENT) &&
        pData[1] == tuneLastOp && pData[2] >= CP_RESPONSE_SUCCESS &&
        pData[2] <= CP_RESPONSE_OPERATION_FAILED;
    tuneDiag.responseCount++;
    tuneDiag.responseMs = millis();
    tuneDiag.responseLength = length;
    tuneDiag.responseCaptured = length < sizeof(tuneDiag.responseBytes) ? length : sizeof(tuneDiag.responseBytes);
    memcpy(tuneDiag.responseBytes, pData, tuneDiag.responseCaptured);
    tuneDiag.responseNotify = isNotify;
    tuneDiag.responseAccepted = accepted;
    tuneDiag.responseState = (uint8_t)tuneState;
    tuneDiag.responseExpectedOp = tuneLastOp;
    if (!accepted) {
        portEXIT_CRITICAL(&tuneMux);
        Serial.printf("[TUNE] Ignored response (state %u, expected op 0x%02X)\n", (unsigned)tuneState, tuneLastOp);
        return;
    }

    tuneLastOp = pData[1];
    tuneLastResponse = pData[2];
    tuneFailureReason = 0;
    tuneReportedOffset = 0;
    tuneOffsetReported = false;

    if (tuneLastResponse == CP_RESPONSE_OPERATION_FAILED && length >= 4) {
        tuneFailureReason = pData[3];
    }

    // Enhanced success includes company ID and a length-prefixed manufacturer payload.
    bool malformedSuccess = tuneLastResponse == CP_RESPONSE_SUCCESS &&
        ((tuneLastOp == CP_OP_START_OFFSET_COMPENSATION && length != 5 && !myxStandardSuccess) ||
        (tuneLastOp == CP_OP_START_ENHANCED_OFFSET_COMPENSATION &&
        (length < 8 || length != (size_t)8 + pData[7])));
    if (tuneLastResponse == CP_RESPONSE_SUCCESS && !malformedSuccess) {
        if (!myxStandardSuccess) {
            tuneReportedOffset = (int16_t)(pData[3] | (pData[4] << 8));
            tuneOffsetReported = true;
        }
        tuneState = TUNE_SUCCESS;
    } else if (tuneLastResponse == CP_RESPONSE_NOT_SUPPORTED) {
        tuneState = TUNE_NOT_SUPPORTED;
    } else {
        tuneState = TUNE_FAILED;
    }
    portEXIT_CRITICAL(&tuneMux);

    if (malformedSuccess) {
        setUiMessage("Sensor tune returned a malformed success response");
        Serial.println("[TUNE] Success response has invalid result length");
    } else if (tuneLastResponse == CP_RESPONSE_SUCCESS) {
        setUiMessage("Sensor tune complete");
        if (tuneOffsetReported) Serial.printf("[TUNE] Sensor tune complete. Offset result: %d\n", tuneReportedOffset);
        else Serial.println("[TUNE] Sensor tune complete. Offset not reported");
    } else if (tuneLastResponse == CP_RESPONSE_NOT_SUPPORTED) {
        setUiMessage("Sensor tune opcode not supported");
        Serial.printf("[TUNE] Sensor rejected op 0x%02X as not supported\n", tuneLastOp);
    } else if (tuneLastResponse == CP_RESPONSE_OPERATION_FAILED) {
        setUiMessage("Sensor tune failed");
        Serial.printf("[TUNE] Sensor tune failed for op 0x%02X, reason 0x%02X\n", tuneLastOp, tuneFailureReason);
    } else {
        setUiMessage("Sensor tune returned an unknown response");
        Serial.printf("[TUNE] Sensor tune response for op 0x%02X: 0x%02X\n", tuneLastOp, tuneLastResponse);
    }
}

static void batteryNotifyCallback(
    BLERemoteCharacteristic* pChar,
    uint8_t* pData,
    size_t length,
    bool isNotify)
{
    (void)pChar;
    (void)isNotify;
    if (length < 1) return;
    lastBattery = pData[0];
    Serial.printf("[BATTERY] Level: %d%%\n", lastBattery);
    if (batteryLevelChar != nullptr) {
        batteryLevelChar->setValue(pData, 1);
        batteryLevelChar->notify();
    }
}

static void publishSensorData();

static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify)
{
    (void)pBLERemoteCharacteristic;
    (void)isNotify;
    if (length < 10) {
        Serial.printf("[SENSOR] Short packet (%u bytes), skipping\n", (unsigned)length);
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
    uint32_t nowMs = millis();
    uint32_t packetInterval = lastSensorPacketMs == 0 ? 0 : nowMs - lastSensorPacketMs;
    uint16_t crankRevDelta = (uint16_t)(cRev - prevCrankRev);
    uint16_t crankTimeDelta = (uint16_t)(cTime - prevCrankTime);
    lastSensorPacketMs = nowMs;
    lastPacketIntervalMs = packetInterval;
    lastCrankRevDelta = crankRevDelta;
    lastCrankTimeDelta = crankTimeDelta;

    if (cRev != prevCrankRev) {
        lastCrankMoveMs = nowMs;
    }

    bool dropped = (nowMs - lastCrankMoveMs > DROPOUT_MS);
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
            cadence = lastCadence;
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
    lastCadenceHeld = !dropped && cRev == prevCrankRev && cadence > 0.0f;

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

static void requestSensorRecovery(bool preferCached, const char* reason) {
    doConnect = false;
    stopBleScan();
    connectUsingCachedAddress = preferCached && cachedSensor.valid;
    if (connectUsingCachedAddress) {
        recoveryState = RECOVERY_CACHED_RECONNECT;
        setUiMessage(reason);
        doConnect = true;
        doScan = false;
    } else {
        recoveryState = RECOVERY_SCANNING;
        setUiMessage(reason);
        doScan = true;
    }
}

class SensorClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        (void)pclient;
        Serial.println("[SENSOR] Connected to power meter!");
        sensorConnected = true;
        recoveryState = RECOVERY_CONNECTED;
        setUiMessage("Sensor connected");
        if (!wifiConfig.provisioned) {
            startAccessPointIfNeeded();
        }
    }

    void onDisconnect(BLEClient* pclient) override {
        (void)pclient;
        disconnectPending = true;
        Serial.println("[SENSOR] Disconnected from power meter!");
        sensorConnected = false;
        lastBroadcastPower = 0;
        lastCorrectedPower = 0;
        lastCadence = 0;
        lastSensorPacketMs = 0;
        smoothedPower = 0;
        remoteControlPointReady = false;
        remotePowerControlPoint = nullptr;
        portENTER_CRITICAL(&tuneMux);
        tuneRequested = false;
        tuneState = TUNE_IDLE;
        tuneTimedOut = false;
        portEXIT_CRITICAL(&tuneMux);
        if (!wifiConfig.provisioned && !setupWindow) {
            stopAccessPoint();
        }
        bool preferCached = !scanAfterDisconnect;
        scanAfterDisconnect = false;
        requestSensorRecovery(preferCached, "Sensor disconnected, recovering");
    }
};

// The library invokes this hook only after its client and service event dispatch
// has completed. Reclaim the client on the loop task after this point.
static void onGattClientEventComplete(esp_gattc_cb_event_t event, esp_gatt_if_t clientIf,
                                      esp_ble_gattc_cb_param_t* param) {
    if (param && event == ESP_GATTC_REG_FOR_NOTIFY_EVT) {
        portENTER_CRITICAL(&tuneMux);
        if ((int)clientIf == tuneDiag.clientIf && tuneDiag.cpHandle && param->reg_for_notify.handle == tuneDiag.cpHandle) {
            tuneDiag.registrationSeen = true;
            tuneDiag.registrationStatus = param->reg_for_notify.status;
            tuneDiag.registrationEventMs = millis();
            tuneDiag.registrationCount++;
        }
        portEXIT_CRITICAL(&tuneMux);
    }
    if (param && (event == ESP_GATTC_WRITE_CHAR_EVT || event == ESP_GATTC_WRITE_DESCR_EVT)) {
        portENTER_CRITICAL(&tuneMux);
        if ((int)clientIf == tuneDiag.clientIf && param->write.conn_id == tuneDiag.connId) {
            if (event == ESP_GATTC_WRITE_CHAR_EVT && tuneDiag.cpHandle && param->write.handle == tuneDiag.cpHandle) {
                tuneDiag.writeSeen = true;
                tuneDiag.writeStatus = param->write.status;
                tuneDiag.writeEventMs = millis();
                tuneDiag.writeCount++;
            } else if (event == ESP_GATTC_WRITE_DESCR_EVT && tuneDiag.cccdHandle && param->write.handle == tuneDiag.cccdHandle) {
                tuneDiag.cccdSeen = true;
                tuneDiag.cccdStatus = param->write.status;
                tuneDiag.cccdEventMs = millis();
                tuneDiag.cccdCount++;
            }
        }
        portEXIT_CRITICAL(&tuneMux);
    }
    if (event == ESP_GATTC_DISCONNECT_EVT && disconnectPending) {
        disconnectDispatchComplete = true;
        disconnectPending = false;
    }
}

class SensorScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (sensorConnected || doConnect) {
            return;
        }

        bool hasCPS = advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(CPS_SERVICE_UUID);
        String devName = advertisedDevice.getName().c_str();
        bool hasName = advertisedDevice.haveName() && devName.length() > 0;
        bool nameMatch = strlen(TARGET_SENSOR_NAME) > 0 && hasName && devName.startsWith(TARGET_SENSOR_NAME);
        bool cachedMatch = cachedSensor.valid &&
            strcmp(advertisedDevice.getAddress().toString().c_str(), cachedSensor.address) == 0;

        if (!nameMatch && !cachedMatch) return;

        Serial.printf("[SCAN] Candidate sensor: '%s' (%s)\n",
                      devName.c_str(),
                      advertisedDevice.getAddress().toString().c_str());

        if (cachedSensor.valid && !cachedMatch && strlen(cachedSensor.address) > 0) {
            if (nameMatch || hasCPS) {
                Serial.printf("[SCAN] Cached target not yet found, saw alternate candidate %s\n",
                              advertisedDevice.getAddress().toString().c_str());
            }
        }

        if (cachedSensor.valid && !cachedMatch && (nameMatch || hasCPS)) {
            if (recoveryState == RECOVERY_SCANNING && !connectUsingCachedAddress) {
                // scan fallback can connect to any valid candidate
            } else {
                return;
            }
        }

        cleanupSensorDevice();
        sensorDevice = new BLEAdvertisedDevice(advertisedDevice);
        connectUsingCachedAddress = false;
        doConnect = true;
        doScan = false;
        setUiMessage("Sensor found, connecting");
    }
};

static bool connectToSensor() {
    if (sensorConnected) return true;

    stopBleScan();
    if (disconnectPending) return false;
    if (disconnectDispatchComplete) {
        delete bleClient;
        bleClient = nullptr;
        disconnectDispatchComplete = false;
    }
    if (bleClient == nullptr) {
        static SensorClientCallbacks callbacks;
        bleClient = BLEDevice::createClient();
        bleClient->setClientCallbacks(&callbacks);
    }

    bool connected = false;
    if (connectUsingCachedAddress && cachedSensor.valid && strlen(cachedSensor.address) > 0) {
        Serial.printf("[SENSOR] Cached reconnect to %s (type %u)\n", cachedSensor.address, cachedSensor.addrType);
        connected = bleClient->connect(BLEAddress(std::string(cachedSensor.address)),
                                       (esp_ble_addr_type_t)cachedSensor.addrType);
    } else {
        if (sensorDevice == nullptr) return false;
        Serial.printf("[SENSOR] Connecting to %s ...\n", sensorDevice->getAddress().toString().c_str());
        connected = bleClient->connect(sensorDevice);
    }

    connectUsingCachedAddress = false;

    if (!connected) {
        Serial.println("[SENSOR] Connection failed");
        cleanupBleClient();
        return false;
    }

    BLERemoteService* remoteService = bleClient->getService(CPS_SERVICE_UUID);
    if (remoteService == nullptr) {
        Serial.println("[SENSOR] Cycling Power Service not found");
        cleanupBleClient();
        return false;
    }

    BLERemoteCharacteristic* remoteMeasurement = remoteService->getCharacteristic(CPS_MEASUREMENT_UUID);
    if (remoteMeasurement == nullptr) {
        Serial.println("[SENSOR] Power Measurement characteristic not found");
        cleanupBleClient();
        return false;
    }

    if (remoteMeasurement->canNotify()) {
        remoteMeasurement->registerForNotify(notifyCallback);
        Serial.println("[SENSOR] Subscribed to power notifications");
    } else {
        Serial.println("[SENSOR] WARNING: measurement characteristic does not notify");
        cleanupBleClient();
        return false;
    }

    remotePowerControlPoint = remoteService->getCharacteristic(CPS_CONTROL_POINT_UUID);
    remoteControlPointReady = false;
    portENTER_CRITICAL(&tuneMux);
    tuneDiag.clientIf = (int)bleClient->getGattcIf();
    tuneDiag.connId = bleClient->getConnId();
    tuneDiag.cpHandle = 0;
    tuneDiag.cccdHandle = 0;
    tuneDiag.cccdSeen = false;
    tuneDiag.cccdStatus = -1;
    tuneDiag.writeSeen = false;
    tuneDiag.writeStatus = -1;
    tuneDiag.registrationSeen = false;
    tuneDiag.registrationStatus = -1;
    tuneDiag.featureValid = false;
    tuneDiag.canWrite = tuneDiag.canIndicate = tuneDiag.canNotify = false;
    portEXIT_CRITICAL(&tuneMux);
    if (remotePowerControlPoint != nullptr) {
        BLERemoteDescriptor* cpCccd = remotePowerControlPoint->getDescriptor(BLEUUID((uint16_t)0x2902));
        portENTER_CRITICAL(&tuneMux);
        tuneDiag.cpHandle = remotePowerControlPoint->getHandle();
        tuneDiag.cccdHandle = cpCccd ? cpCccd->getHandle() : 0;
        tuneDiag.canWrite = remotePowerControlPoint->canWrite();
        tuneDiag.canIndicate = remotePowerControlPoint->canIndicate();
        tuneDiag.canNotify = remotePowerControlPoint->canNotify();
        portEXIT_CRITICAL(&tuneMux);
        if (!remotePowerControlPoint->canWrite()) {
            Serial.println("[TUNE] Remote Cycling Power Control Point does not support write with response");
        } else if (cpCccd == nullptr) {
            Serial.println("[TUNE] Remote Cycling Power Control Point has no client configuration descriptor");
        } else if (remotePowerControlPoint->canIndicate()) {
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
    }
    if (!remoteControlPointReady) tuneState = TUNE_NO_CONTROL_POINT;

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
        if (featVal.length() >= 4) {
            uint32_t bits = (uint8_t)featVal[0] | ((uint32_t)(uint8_t)featVal[1] << 8) |
                            ((uint32_t)(uint8_t)featVal[2] << 16) | ((uint32_t)(uint8_t)featVal[3] << 24);
            portENTER_CRITICAL(&tuneMux);
            tuneDiag.featureBits = bits;
            tuneDiag.featureValid = true;
            portEXIT_CRITICAL(&tuneMux);
        }
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

    std::string connectedName = sensorDevice != nullptr ? sensorDevice->getName() : std::string(cachedSensor.lastName);
    saveCachedSensor(bleClient->getPeerAddress(),
                     sensorDevice != nullptr ? sensorDevice->getAddressType() : (esp_ble_addr_type_t)cachedSensor.addrType,
                     connectedName.c_str());

    lastCrankMoveMs = millis();
    lastSensorPacketMs = millis();
    sensorConnected = true;
    recoveryState = RECOVERY_CONNECTED;
    publishSensorData();
    return true;
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
    modelChar->setValue("BikePower Bridge Unified");
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

static void publishDiscovery() {
    if (!mqttClient.connected()) return;

    struct {
        const char* name;
        const char* uidSuffix;
        const char* valueTpl;
        const char* unit;
        const char* devClass;
        const char* stateClass;
        const char* icon;
    } sensors[] = {
        {"Power", "power", "{{ value_json.power }}", "W", "power", "measurement", nullptr},
        {"Cadence", "cadence", "{{ value_json.cadence }}", "RPM", nullptr, "measurement", "mdi:bicycle"},
        {"Battery", "battery", "{{ value_json.battery }}", "%", "battery", "measurement", nullptr},
        {"Energy", "energy", "{{ value_json.energy }}", "kJ", "energy", "total_increasing", nullptr},
    };

    for (auto& s : sensors) {
        JsonDocument doc;

        char uid[48];
        snprintf(uid, sizeof(uid), "%s_%s", deviceId, s.uidSuffix);
        char name[48];
        snprintf(name, sizeof(name), "MYX Bridge %s", s.name);

        doc["name"] = name;
        doc["unique_id"] = uid;
        doc["state_topic"] = mqttStateTopic;
        doc["value_template"] = s.valueTpl;
        doc["unit_of_measurement"] = s.unit;
        if (s.devClass) doc["device_class"] = s.devClass;
        if (s.stateClass) doc["state_class"] = s.stateClass;
        if (s.icon) doc["icon"] = s.icon;
        doc["availability_topic"] = mqttAvailTopic;
        doc["payload_available"] = "online";
        doc["payload_not_available"] = "offline";

        JsonObject dev = doc["device"].to<JsonObject>();
        dev["identifiers"].to<JsonArray>().add(deviceId);
        dev["name"] = "MYX Bridge";
        dev["manufacturer"] = "ESP32 DIY";
        dev["model"] = "MYX Power Bridge";
        dev["sw_version"] = FW_VERSION;

        char topic[128];
        snprintf(topic, sizeof(topic), "%s/sensor/%s/config", MQTT_DISCOVERY_PREFIX, uid);

        String payload;
        serializeJson(doc, payload);
        if (!mqttClient.publish(topic, payload.c_str(), true)) Serial.println("[MQTT] Discovery publish failed");
    }

    JsonDocument doc;
    char uid[48];
    snprintf(uid, sizeof(uid), "%s_ble_connected", deviceId);
    doc["name"] = "MYX Bridge Sensor Connected";
    doc["unique_id"] = uid;
    doc["state_topic"] = mqttStateTopic;
    doc["value_template"] = "{{ value_json.ble_connected }}";
    doc["device_class"] = "connectivity";
    doc["payload_on"] = "ON";
    doc["payload_off"] = "OFF";
    doc["availability_topic"] = mqttAvailTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"].to<JsonArray>().add(deviceId);
    dev["name"] = "MYX Bridge";
    dev["manufacturer"] = "ESP32 DIY";
    dev["model"] = "MYX Power Bridge";
    dev["sw_version"] = FW_VERSION;

    char topic[128];
    snprintf(topic, sizeof(topic), "%s/binary_sensor/%s/config", MQTT_DISCOVERY_PREFIX, uid);

    String payload;
    serializeJson(doc, payload);
    if (!mqttClient.publish(topic, payload.c_str(), true)) Serial.println("[MQTT] Discovery publish failed");
}

static void publishSensorData() {
    if (!mqttClient.connected()) return;

    JsonDocument doc;
    doc["power"] = (int)lastBroadcastPower;
    doc["cadence"] = round(lastCadence * 10.0f) / 10.0f;
    if (lastBattery == 0xFF) doc["battery"] = nullptr;
    else doc["battery"] = (int)lastBattery;
    doc["energy"] = (int)lastEnergy;
    doc["ble_connected"] = sensorConnected ? "ON" : "OFF";

    char payload[256];
    serializeJson(doc, payload, sizeof(payload));
    mqttClient.publish(mqttStateTopic, payload);
}

static bool mqttConnect() {
    if (!mqttConfigured || !mqttConfig.enabled) return false;
    if (WiFi.status() != WL_CONNECTED) return false;

    mqttClient.setServer(mqttConfig.host, mqttConfig.port);

    char clientId[32];
    snprintf(clientId, sizeof(clientId), "%s_%04X", deviceId, (uint16_t)random(0xFFFF));

    bool connected = false;
    if (strlen(mqttConfig.user) > 0) {
        connected = mqttClient.connect(clientId, mqttConfig.user, mqttConfig.pass,
                                       mqttAvailTopic, 1, true, "offline");
    } else {
        connected = mqttClient.connect(clientId, nullptr, nullptr,
                                       mqttAvailTopic, 1, true, "offline");
    }

    mqttConnected = connected;
    if (connected) {
        mqttClient.publish(mqttAvailTopic, "online", true);
        publishDiscovery();
        publishSensorData();
        setUiMessage("MQTT connected");
        Serial.println("[MQTT] Connected");
        return true;
    }

    Serial.printf("[MQTT] Failed, rc=%d\n", mqttClient.state());
    return false;
}

static bool testMqttConnection(const char* host, uint16_t port, const char* user, const char* pass) {
    if (WiFi.status() != WL_CONNECTED) {
        strlcpy(lastMqttTestResult, "WiFi not connected", sizeof(lastMqttTestResult));
        return false;
    }

    WiFiClient testClient;
    PubSubClient testPub(testClient);
    testPub.setServer(host, port);
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "%s_test_%04X", deviceId, (uint16_t)random(0xFFFF));

    bool ok = false;
    if (strlen(user) > 0) {
        ok = testPub.connect(clientId, user, pass);
    } else {
        ok = testPub.connect(clientId);
    }

    if (ok) {
        strlcpy(lastMqttTestResult, "MQTT test succeeded", sizeof(lastMqttTestResult));
        testPub.disconnect();
        return true;
    }

    snprintf(lastMqttTestResult, sizeof(lastMqttTestResult), "MQTT test failed (rc=%d)", testPub.state());
    return false;
}

static String batteryText() {
    if (lastBattery == 0xFF) return "?";
    return String((int)lastBattery) + "%";
}

static String wifiStatusText() {
    if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
    if (wifiConfig.provisioned) return "Waiting";
    return "Not provisioned";
}

static String mqttStatusText() {
    if (mqttConnected) return "Connected";
    if (mqttConfig.enabled) return "Configured";
    return "Disabled";
}

static void sendNoCacheHeaders() {
    webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    webServer.sendHeader("Pragma", "no-cache");
    webServer.sendHeader("Expires", "0");
}

static void sendPageStart(const char* title, bool autoRefresh) {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    sendNoCacheHeaders();
    webServer.send(200, "text/html", "");
    webServer.sendContent(
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    webServer.sendContent(
        "<title>MYX Bridge</title>"
        "<style>"
        ":root{--bg:#131a27;--card:#16213e;--line:#0f3460;--text:#e7edf7;--muted:#93a1b8;--cyan:#00d4ff;--green:#00ff88;--purple:#533483;--purple2:#6c44a0;--red:#c0392b;}"
        "*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#1e2a44 0,#131a27 45%,#0e1420 100%);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif}"
        "main{max-width:1080px;margin:0 auto;padding:20px}"
        "h1{color:var(--cyan);text-align:center;margin:0 0 6px;font-size:34px}"
        ".subtitle{text-align:center;color:var(--muted);margin:0 0 18px}"
        ".hero{background:linear-gradient(135deg,#16213e 0,#1a2750 100%);border:1px solid #224269;border-radius:18px;padding:18px 18px 12px;box-shadow:0 10px 30px #0005}"
        ".nav{display:flex;gap:10px;flex-wrap:wrap;justify-content:center;margin:16px 0 6px}"
        ".nav a{border:1px solid #29538a;background:#17233f;color:var(--text);padding:10px 14px;border-radius:999px;font-weight:700;cursor:pointer;text-decoration:none}"
        ".nav a.active{background:var(--purple);border-color:#7b59b9}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px;margin-top:18px}"
        ".card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:16px;box-shadow:0 8px 24px #0003}"
        ".card h2{margin:0 0 12px;color:var(--cyan);font-size:1.05em}"
        ".big{font-size:2.2em;text-align:center;color:var(--green);margin:10px 0 4px;font-weight:800}"
        ".small{text-align:center;color:var(--muted)}"
        ".row{display:flex;justify-content:space-between;gap:10px;padding:7px 0;border-bottom:1px solid #1d3256}"
        ".row:last-child{border-bottom:none}"
        ".label{color:var(--muted)}.value{font-weight:700;color:#fff}"
        ".note{color:var(--muted);font-size:.95em;line-height:1.45}"
        "label{display:block;margin:10px 0 6px;font-size:13px;color:var(--muted)}"
        "input{width:100%;padding:11px 12px;border:1px solid #2b456d;border-radius:10px;font-size:15px;background:#101a2f;color:#f3f7ff}"
        "input[type=checkbox]{width:auto;margin-right:8px}"
        ".inline-check{display:flex;align-items:center;gap:8px;color:#fff;margin-top:6px}"
        ".table{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
        ".actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:14px}"
        "button,.btn{display:inline-block;border:none;background:var(--purple);color:#fff;padding:11px 14px;border-radius:10px;font-weight:700;cursor:pointer;text-decoration:none}"
        "button:hover,.btn:hover{background:var(--purple2)}"
        "button.secondary,.btn.secondary{background:#27476f}"
        "button.warn,.btn.warn{background:var(--red)}"
        "pre{margin:0;background:#10192c;border:1px solid #22395d;border-radius:12px;padding:12px;color:#cfe6ff;min-height:170px;white-space:pre-wrap}"
        "@media(max-width:700px){main{padding:14px}.table{grid-template-columns:1fr}.big{font-size:1.9em}}"
        "</style></head><body><main><div class='hero'>"
        "<h1>MYX Bridge</h1><p class='subtitle'>Unified firmware with BLE bridge, sensor tools, network setup, MQTT, and OTA.</p>"
        "<div class='nav'>");
    webServer.sendContent(String(title) == "Overview"
        ? "<a class='active' href='/'>Overview</a><a href='/diag'>Diagnostics</a><a href='/network'>Network Settings</a><a href='/update'>OTA</a>"
        : String(title) == "Diagnostics"
            ? "<a href='/'>Overview</a><a class='active' href='/diag'>Diagnostics</a><a href='/network'>Network Settings</a><a href='/update'>OTA</a>"
            : "<a href='/'>Overview</a><a href='/diag'>Diagnostics</a><a class='active' href='/network'>Network Settings</a><a href='/update'>OTA</a>");
    webServer.sendContent("</div></div>");
}

static void sendPageEnd() {
    webServer.sendContent("</main></body></html>");
    webServer.sendContent("");
}

static void mqttDisconnectCleanly() {
    if (mqttClient.connected()) {
        mqttClient.publish(mqttAvailTopic, "offline", true);
        mqttClient.disconnect();
    }
}

static void ensureMqttState() {
    if (!mqttConfigured || !mqttConfig.enabled) {
        mqttDisconnectCleanly();
        mqttConnected = false;
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        mqttDisconnectCleanly();
        mqttConnected = false;
        return;
    }

    if (!mqttClient.connected()) {
        mqttConnected = false;
        if (millis() - lastMqttReconnectAttempt >= MQTT_RETRY_INTERVAL_MS) {
            lastMqttReconnectAttempt = millis();
            mqttConnect();
        }
        return;
    }

    mqttConnected = true;
    mqttClient.loop();
    if (millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL_MS) {
        lastMqttPublish = millis();
        publishSensorData();
    }
}

static void handleWebRoot() {
    unsigned long uptime = millis() / 1000;
    unsigned long days = uptime / 86400;
    unsigned long hours = (uptime % 86400) / 3600;
    unsigned long mins = (uptime % 3600) / 60;
    unsigned long secs = uptime % 60;

    char uptimeStr[32];
    if (days > 0) snprintf(uptimeStr, sizeof(uptimeStr), "%lud %02lu:%02lu:%02lu", days, hours, mins, secs);
    else snprintf(uptimeStr, sizeof(uptimeStr), "%02lu:%02lu:%02lu", hours, mins, secs);

    char battStr[8];
    if (lastBattery == 0xFF) snprintf(battStr, sizeof(battStr), "?");
    else snprintf(battStr, sizeof(battStr), "%d%%", lastBattery);

    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    sendNoCacheHeaders();
    webServer.send(200, "text/html", "");
    webServer.sendContent(
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>MYX Bridge</title>"
        "<style>"
        "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;max-width:680px;margin:0 auto;padding:20px;background:#1a1a2e;color:#e0e0e0}"
        "h1{color:#00d4ff;text-align:center;margin-bottom:5px}"
        ".subtitle{text-align:center;color:#888;margin-bottom:20px}"
        ".card{background:#16213e;border-radius:12px;padding:16px;margin:12px 0;border:1px solid #0f3460}"
        ".card h2{margin:0 0 12px;font-size:1.1em;color:#00d4ff}"
        ".row{display:flex;justify-content:space-between;gap:12px;padding:6px 0;border-bottom:1px solid #0f3460}"
        ".row:last-child{border-bottom:none}"
        ".label{color:#888}.value{font-weight:600;color:#fff}"
        ".big{font-size:2.2em;text-align:center;color:#00ff88;margin:8px 0}"
        ".online{color:#00ff88}.offline{color:#ff4444}"
        ".note{color:#9aa3b4;font-size:.95em;line-height:1.45;margin:0 0 10px}"
        ".btn{display:block;width:100%;padding:12px;margin:12px 0 0;background:#533483;color:#fff;border:none;border-radius:8px;font-size:1em;cursor:pointer;text-align:center;text-decoration:none;box-sizing:border-box}"
        ".btn:hover{background:#6c44a0}.btn:disabled{opacity:.5;cursor:not-allowed}"
        ".btn-secondary{background:#0f3460}.btn-secondary:hover{background:#1a4a7a}"
        ".btn-danger{background:#c0392b}.btn-danger:hover{background:#e74c3c}"
        ".btn-row{display:grid;grid-template-columns:1fr;gap:10px;margin-top:10px}"
        ".inline-form{margin:0}"
        ".nav{display:flex;gap:10px;flex-wrap:wrap;justify-content:center;margin:16px 0 18px}"
        ".nav a{border:1px solid #29538a;background:#17233f;color:#e0e0e0;padding:10px 14px;border-radius:999px;font-weight:700;text-decoration:none}"
        ".nav a.active{background:#533483;border-color:#7b59b9}"
        ".status-banner{margin-top:12px;padding:10px 12px;border-radius:10px;background:#101a2f;border:1px solid #2b456d;color:#cfe6ff;font-size:.95em}"
        "@media(min-width:620px){.btn-row.two{grid-template-columns:1fr 1fr}.btn-row.three{grid-template-columns:1fr 1fr 1fr}}"
        "</style></head><body>");

    char buf[256];
    snprintf(buf, sizeof(buf), "<h1>MYX Bridge</h1><div class='subtitle'>v%s &bull; Live data</div>", FW_VERSION);
    webServer.sendContent(buf);
    webServer.sendContent(
        "<div class='nav'>"
        "<a class='active' href='/'>Overview</a>"
        "<a href='/diag'>Diagnostics</a>"
        "<a href='/network'>Network Settings</a>"
        "<a href='/update'>OTA</a>"
        "</div>");

    snprintf(buf, sizeof(buf), "<div class='card'><h2>Power</h2><div class='big' id='val-power'>%d W</div></div>", (int)lastBroadcastPower);
    webServer.sendContent(buf);

    webServer.sendContent("<div class='card'><h2>Sensor Data</h2>");
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Cadence</span><span class='value' id='val-cadence'>%.1f RPM</span></div>", lastCadence);
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Battery</span><span class='value' id='val-battery'>%s</span></div>", battStr);
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Energy</span><span class='value' id='val-energy'>%d kJ</span></div>", (int)lastEnergy);
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Crank Revs</span><span class='value' id='val-cranks'>%d</span></div>", (int)lastCrankRev);
    webServer.sendContent(buf);
    webServer.sendContent("</div>");

    webServer.sendContent("<div class='card'><h2>Connections</h2>");
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>BLE Sensor</span><span class='value %s' id='val-ble-sensor'>%s</span></div>",
             sensorConnected ? "online" : "offline",
             sensorConnected ? "Connected" : "Searching");
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Recovery</span><span class='value' id='val-recovery'>%s</span></div>", recoveryStateText(recoveryState));
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>BLE Apps</span><span class='value' id='val-ble-apps'>%d</span></div>", appConnectionCount);
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>WiFi</span><span class='value %s' id='val-wifi'>%s</span></div>",
             WiFi.status() == WL_CONNECTED ? "online" : "offline",
             (WiFi.status() == WL_CONNECTED ? "Connected" : (wifiConfig.provisioned ? "Waiting" : "Not provisioned")));
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>IP Address</span><span class='value' id='val-wifi-ip'>%s</span></div>", WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "?");
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>MQTT</span><span class='value %s' id='val-mqtt'>%s</span></div>",
             mqttConnected ? "online" : "offline",
             mqttStatusText().c_str());
    webServer.sendContent(buf);
    if (mqttConfig.enabled && strlen(mqttConfig.host) > 0) {
        snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>MQTT Broker</span><span class='value' id='val-broker'>%s:%u</span></div>",
                 escapeHtml(mqttConfig.host).c_str(), (unsigned int)mqttConfig.port);
        webServer.sendContent(buf);
    }
    webServer.sendContent("</div>");

    webServer.sendContent("<div class='card'><h2>Calibrate Sensor</h2><p class='note'>Pedal to wake the sensor, get off the bike, set the crank vertical, then run tune.</p>");
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Tune Status</span><span class='value' id='val-tune'>%s</span></div>", tuneStateText((TuneState)tuneState));
    webServer.sendContent(buf);
    char offsetText[24];
    if (tuneOffsetReported) snprintf(offsetText, sizeof(offsetText), "%d", (int)tuneReportedOffset);
    else snprintf(offsetText, sizeof(offsetText), "Not reported");
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Returned Offset</span><span class='value' id='val-tune-offset'>%s</span></div>", offsetText);
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Control Point</span><span class='value %s' id='val-cp'>%s</span></div>",
             remoteControlPointReady ? "online" : "offline",
             remoteControlPointReady ? "Ready" : "Unavailable");
    webServer.sendContent(buf);
    webServer.sendContent(
        "<div class='status-banner' id='action-status'>Ready</div>"
        "<div class='btn-row three'>"
        "<button class='btn' id='tune-button' disabled type='button' onclick=\"sendAction('/tune','Tune request sent. Stay off the bike while calibration runs.')\">Tune Sensor</button>"
        "<button class='btn btn-secondary' type='button' onclick=\"sendAction('/reconnect','Reconnect request sent.')\">Reconnect Sensor</button>"
        "<button class='btn btn-secondary' type='button' onclick=\"sendAction('/find-sensor','Sensor search requested.')\">Find Sensor Again</button>"
        "</div></div>");

    webServer.sendContent("<div class='card'><h2>Device</h2>");
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Mode</span><span class='value' id='val-mode'>%s</span></div>", servingModeText());
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Cached Sensor</span><span class='value' id='val-cached'>%s</span></div>", cachedSensor.valid ? cachedSensor.address : "No");
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Uptime</span><span class='value' id='val-uptime'>%s</span></div>", uptimeStr);
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Free Heap</span><span class='value' id='val-heap'>%d KB</span></div>", (int)(ESP.getFreeHeap() / 1024));
    webServer.sendContent(buf);
    webServer.sendContent("</div>");

    webServer.sendContent(
        "<script>"
        "function fmtUptime(s){var d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),m=Math.floor((s%3600)/60),sc=s%60;var p=function(n){return n<10?'0'+n:n;};return d>0?d+'d '+p(h)+':'+p(m)+':'+p(sc):p(h)+':'+p(m)+':'+p(sc);}"
        "function upd(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}"
        "function setClass(id,cls){var e=document.getElementById(id);if(e)e.className='value '+cls;}"
        "function setAction(msg){upd('action-status',msg);}"
        "function sendAction(path,msg){setAction('Sending request...');fetch(path,{method:'POST'}).then(function(r){return r.json().then(function(d){if(!r.ok)throw new Error(d.message||d.status||'request failed');return d;});}).then(function(){setAction(msg);refresh();}).catch(function(e){setAction('Action request failed: '+e.message);});}"
        "function refresh(){fetch('/api/live?ts='+(new Date().getTime())).then(function(r){return r.json();}).then(function(d){"
        "upd('val-power',d.broadcastPower+' W');"
        "upd('val-cadence',Number(d.cadence).toFixed(1)+' RPM');"
        "upd('val-battery',d.battery);"
        "upd('val-energy',d.energy+' kJ');"
        "upd('val-cranks',d.crankRev);"
        "upd('val-recovery',d.recoveryState);"
        "upd('val-ble-sensor',d.sensorConnected?'Connected':'Searching');"
        "setClass('val-ble-sensor',d.sensorConnected?'online':'offline');"
        "upd('val-ble-apps',d.appConnections);"
        "upd('val-wifi',d.wifiConnected?'Connected':(d.wifiProvisioned?'Waiting':'Not provisioned'));upd('val-wifi-ip',d.wifiConnected?d.wifiIp:'?');"
        "setClass('val-wifi',d.wifiConnected?'online':'offline');"
        "upd('val-mqtt',d.mqttConnected?'Connected':(d.mqttEnabled?'Configured':'Disabled'));"
        "setClass('val-mqtt',d.mqttConnected?'online':'offline');"
        "upd('val-broker',(d.mqttEnabled&&d.mqttHost)?(d.mqttHost+':'+d.mqttPort):'');"
        "upd('val-tune',d.tuneStatus);document.getElementById('tune-button').disabled=!d.tuneCanStart;"
        "upd('val-tune-offset',d.tuneOffsetReported?d.tuneReportedOffset:'Not reported');"
        "upd('val-cp',d.remoteControlPointReady?'Ready':'Unavailable');"
        "setClass('val-cp',d.remoteControlPointReady?'online':'offline');"
        "upd('val-mode',d.servingMode);"
        "upd('val-cached',d.cachedValid?d.cachedAddress:'No');"
        "upd('val-uptime',fmtUptime(d.uptimeSec));"
        "upd('val-heap',Math.round(d.freeHeap/1024)+' KB');"
        "}).catch(function(){});}"
        "refresh();setInterval(refresh,500);"
        "</script></body></html>");
    webServer.sendContent("");
}

static void handleDiagnosticsRoot() {
    sendPageStart("Diagnostics", false);
    webServer.sendContent(
        "<div class='grid'>"
        "<section class='card'><h2>Raw Diagnostics</h2>"
        "<div class='row'><span class='label'>Raw</span><span class='value' id='raw'>");
    webServer.sendContent(String((int)lastRawPower) + " W");
    webServer.sendContent(
        "</span></div><div class='row'><span class='label'>Zeroed</span><span class='value' id='zeroed'>");
    webServer.sendContent(String((int)lastZeroedPower) + " W");
    webServer.sendContent(
        "</span></div><div class='row'><span class='label'>Corrected</span><span class='value' id='corrected'>");
    webServer.sendContent(String((int)lastCorrectedPower) + " W");
    webServer.sendContent(
        "</span></div><div class='row'><span class='label'>Cadence Offset</span><span class='value' id='appliedOffset'>");
    webServer.sendContent(String((double)lastAppliedCadenceOffset, 1) + " W");
    webServer.sendContent(
        "</span></div><div class='row'><span class='label'>Battery</span><span class='value' id='battery'>");
    webServer.sendContent(batteryText());
    webServer.sendContent(
        "</span></div><div class='row'><span class='label'>Energy</span><span class='value' id='energy'>");
    webServer.sendContent(String((unsigned int)lastEnergy) + " kJ");
    webServer.sendContent(
        "</span></div></section>"
        "<section class='card'><h2>Status / Logs</h2>"
        "<div class='row'><span class='label'>Last Event</span><span class='value' id='msg'>");
    if (lastUiMessage[0]) webServer.sendContent(lastUiMessage);
    webServer.sendContent(
        "</span></div><div class='row'><span class='label'>Firmware</span><span class='value'>");
    webServer.sendContent(FW_VERSION);
    webServer.sendContent("</span></div><a href='/api/logs' download='myx-bridge-logs.txt'>Download Logs</a><pre id='eventLog'></pre><h3>Packet Statistics</h3><pre id='packetStats'></pre>");
    webServer.sendContent(
        "</section>"
        "<section class='card'><h2>Local Fallback Correction</h2><form method='post' action='/settings/cal'>"
        "<label class='inline-check'><input type='checkbox' name='enabled'");
    if (calibration.correctionEnabled) webServer.sendContent(" checked");
    webServer.sendContent(
        ">Correction enabled</label>"
        "<label class='inline-check'><input type='checkbox' name='rawdiag'");
    if (calibration.rawDiagnosticsEnabled) webServer.sendContent(" checked");
    webServer.sendContent(
        ">Raw diagnostics enabled</label>"
        "<label>Local zero offset, watts<input name='zero' type='number' step='0.1' value='");
    webServer.sendContent(String(calibration.localZeroOffset, 3));
    webServer.sendContent(
        "'></label><label>Scale factor<input name='scale' type='number' step='0.001' min='0.05' max='50' value='");
    webServer.sendContent(String(calibration.scaleFactor, 6));
    webServer.sendContent("'></label><div class='table'>");
    for (int i = 0; i < 4; i++) {
        webServer.sendContent("<label>Cadence ");
        webServer.sendContent(String(i + 1));
        webServer.sendContent("<input name='cad");
        webServer.sendContent(String(i));
        webServer.sendContent("' type='number' step='0.1' value='");
        webServer.sendContent(String(calibration.cadence[i], 3));
        webServer.sendContent("'></label><label>Offset ");
        webServer.sendContent(String(i + 1));
        webServer.sendContent("<input name='off");
        webServer.sendContent(String(i));
        webServer.sendContent("' type='number' step='0.1' value='");
        webServer.sendContent(String(calibration.cadenceOffset[i], 3));
        webServer.sendContent("'></label>");
    }
    webServer.sendContent(
        "</div><div class='actions'><button type='submit'>Save Correction</button></form>"
        "<form method='post' action='/reset-cal'><button type='submit' class='warn'>Reset Correction</button></form></div>"
        "</section></div>"
        "<script>"
        "function setVal(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}"
        "function refresh(){var x=new XMLHttpRequest();x.open('GET','/api/live?ts='+(new Date().getTime()),true);x.onreadystatechange=function(){if(x.readyState!==4||x.status!==200)return;try{var d=JSON.parse(x.responseText);setVal('raw',d.rawPower+' W');setVal('zeroed',d.zeroedPower+' W');setVal('corrected',d.correctedPower+' W');setVal('appliedOffset',Number(d.appliedCadenceOffset).toFixed(1)+' W');setVal('battery',d.battery);setVal('energy',d.energy+' kJ');setVal('msg',d.lastMessage);var held=d.cadenceHeld?'yes':'no';var drop=d.dropped?'yes':'no';document.getElementById('packetStats').textContent='Packet interval: '+d.packetIntervalMs+' ms\\nCrank rev delta: '+d.crankRevDelta+'\\nCrank time delta: '+d.crankTimeDelta+'\\nCadence held: '+held+'\\nDropped: '+drop;}catch(e){}};x.send();}"
        "function refreshLogs(){fetch('/api/status').then(function(r){return r.json();}).then(function(d){setVal('eventLog',d.eventLog.join('\\n'));}).catch(function(){});}"
        "refresh();refreshLogs();setInterval(refresh,500);setInterval(refreshLogs,3000);"
        "</script>");
    sendPageEnd();
}

static void handleNetworkRoot() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    sendNoCacheHeaders();
    webServer.send(200, "text/html", "");
    webServer.sendContent(
        "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>MYX Bridge - Network</title>"
        "<style>"
        "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;max-width:760px;margin:0 auto;padding:20px;background:#1a1a2e;color:#e0e0e0}"
        "h1{color:#00d4ff;text-align:center;margin:0 0 5px}.subtitle{text-align:center;color:#888;margin-bottom:18px}"
        ".nav{display:flex;gap:10px;flex-wrap:wrap;justify-content:center;margin:16px 0 18px}.nav a{border:1px solid #29538a;background:#17233f;color:#e0e0e0;padding:10px 14px;border-radius:999px;font-weight:700;text-decoration:none}.nav a.active{background:#533483;border-color:#7b59b9}"
        ".card{background:#16213e;border-radius:8px;padding:16px;margin:12px 0;border:1px solid #0f3460}.card h2{margin:0 0 12px;font-size:1.1em;color:#00d4ff}"
        "label{display:block;margin:10px 0 6px;color:#aab3c5;font-size:.95em}input{width:100%;box-sizing:border-box;padding:12px;border:1px solid #2b456d;border-radius:8px;background:#101a2f;color:#fff;font-size:16px}input[type=checkbox]{width:auto;margin-right:8px}.check{display:flex;align-items:center;color:#fff}"
        ".row{display:flex;justify-content:space-between;gap:10px;padding:7px 0;border-bottom:1px solid #0f3460}.row:last-child{border-bottom:none}.label{color:#888}.value{font-weight:600;color:#fff}"
        ".actions{display:flex;flex-wrap:wrap;gap:10px;margin-top:14px}button{border:none;background:#533483;color:#fff;padding:12px 16px;border-radius:8px;font-weight:700;cursor:pointer}button.secondary{background:#0f3460}button.warn{background:#c0392b}"
        ".note{color:#9aa3b4;line-height:1.45;margin:0 0 12px}"
        "</style></head><body><h1>MYX Bridge</h1><div class='subtitle'>Network Settings</div>"
        "<div class='nav'><a href='/'>Overview</a><a href='/diag'>Diagnostics</a><a class='active' href='/network'>Network Settings</a><a href='/update'>OTA</a></div>"
        "<section class='card'><h2>WiFi Settings</h2><p class='note'>Saving WiFi changes only WiFi credentials. MQTT settings stay separate.</p>"
        "<form method='post' action='/settings/wifi'>"
        "<label class='check'><input type='checkbox' name='provisioned'");
    if (wifiConfig.provisioned) webServer.sendContent(" checked");
    webServer.sendContent(
        ">WiFi enabled / provisioned</label>"
        "<label>SSID<input name='ssid' maxlength='32' value='");
    if (wifiConfig.ssid[0]) webServer.sendContent(escapeHtml(wifiConfig.ssid));
    webServer.sendContent(
        "'></label>"
        "<label>Password<input name='password' type='password' maxlength='64' value='");
    webServer.sendContent(
        "' placeholder='Leave blank to keep saved password'></label><label><input type='checkbox' name='clear_password'>Clear WiFi password</label>"
        "<div class='actions'><button type='submit'>Save WiFi</button></div>"
        "</form>"
        "<form method='post' action='/reset-wifi'><div class='actions'><button type='submit' class='warn'>Reset WiFi</button></div></form>"
        "</section>"
        "<section class='card' id='mqtt'><h2>MQTT Settings</h2><p class='note'>Saving MQTT does not touch WiFi. Use Test MQTT for a broker check while connected on LAN.</p>"
        "<form id='mqtt-form' method='post' action='/settings/mqtt'>"
        "<label class='check'><input type='checkbox' name='enabled'");
    if (mqttConfig.enabled) webServer.sendContent(" checked");
    webServer.sendContent(
        ">MQTT enabled</label>"
        "<label>Host<input name='host' maxlength='63' value='");
    if (mqttConfig.host[0]) webServer.sendContent(escapeHtml(mqttConfig.host));
    webServer.sendContent(
        "'></label>"
        "<label>Port<input name='port' type='number' min='1' max='65535' value='");
    webServer.sendContent(String(mqttConfig.port));
    webServer.sendContent(
        "'></label>"
        "<label>Username<input name='user' maxlength='31' value='");
    if (mqttConfig.user[0]) webServer.sendContent(escapeHtml(mqttConfig.user));
    webServer.sendContent(
        "'></label>"
        "<label>Password<input name='pass' type='password' maxlength='63' value='");
    webServer.sendContent(
        "' placeholder='Leave blank to keep saved password'></label><label><input type='checkbox' name='clear_pass'>Clear MQTT password</label>"
        "<div class='row'><span class='label'>MQTT Test</span><span class='value' id='mqttTest'>");
    if (lastMqttTestResult[0]) webServer.sendContent(lastMqttTestResult);
    webServer.sendContent(
        "</span></div>"
        "<div class='actions'><button type='submit'>Save MQTT</button><button type='button' class='secondary' onclick='testMqtt()'>Test MQTT</button></div>"
        "</form>"
        "<form method='post' action='/reset-mqtt'><div class='actions'><button type='submit' class='warn'>Reset MQTT</button></div></form>"
        "</section>"
        "<section class='card'><h2>Reset</h2><form method='post' action='/full-reset'><button type='submit' class='warn' onclick=\"return confirm('Full reset WiFi, MQTT, cached sensor, and correction settings?')\">Full Reset</button></form></section>"
        "<script>"
        "function encodeForm(form){var els=form.elements,parts=[],i,e;for(i=0;i<els.length;i++){e=els[i];if(!e.name)continue;if((e.type==='checkbox'||e.type==='radio')&&!e.checked)continue;parts.push(encodeURIComponent(e.name)+'='+encodeURIComponent(e.value));}return parts.join('&');}"
        "function testMqtt(){var form=document.getElementById('mqtt-form');var xhr=new XMLHttpRequest();xhr.open('POST','/test-mqtt',true);xhr.setRequestHeader('Content-Type','application/x-www-form-urlencoded');xhr.onreadystatechange=function(){if(xhr.readyState!==4)return;if(xhr.status!==200){document.getElementById('mqttTest').textContent='Test request failed';return;}try{var d=JSON.parse(xhr.responseText);document.getElementById('mqttTest').textContent=d.result;}catch(e){document.getElementById('mqttTest').textContent='Test failed';}};xhr.send(encodeForm(form));}"
        "</script></body></html>");
    webServer.sendContent("");
}

static void sendSettingsJsonOnly() {
    JsonDocument doc;
    doc["enabled"] = calibration.correctionEnabled;
    doc["rawDiagnostics"] = calibration.rawDiagnosticsEnabled;
    doc["localZeroOffset"] = calibration.localZeroOffset;
    doc["scaleFactor"] = calibration.scaleFactor;
    JsonArray cadence = doc["cadence"].to<JsonArray>();
    JsonArray cadenceOffset = doc["cadenceOffset"].to<JsonArray>();
    for (int i = 0; i < 4; i++) {
        cadence.add(calibration.cadence[i]);
        cadenceOffset.add(calibration.cadenceOffset[i]);
    }
    String payload;
    serializeJson(doc, payload);
    sendNoCacheHeaders();
    webServer.send(200, "application/json", payload);
}

static void handleWebLiveJson() {
    // Reuse the status endpoint's JSON serializer, including correct escaping.
    JsonDocument doc;
    doc["broadcastPower"] = (int)lastBroadcastPower;
    doc["cadence"] = (float)lastCadence;
    doc["battery"] = batteryText();
    doc["rawPower"] = (int)lastRawPower;
    doc["zeroedPower"] = (int)lastZeroedPower;
    doc["correctedPower"] = (int)lastCorrectedPower;
    doc["appliedCadenceOffset"] = (float)lastAppliedCadenceOffset;
    doc["energy"] = (unsigned)lastEnergy;
    doc["crankRev"] = (unsigned)lastCrankRev;
    doc["crankTime"] = (unsigned)lastCrankTime;
    doc["packetIntervalMs"] = (unsigned long)lastPacketIntervalMs;
    doc["crankRevDelta"] = (unsigned)lastCrankRevDelta;
    doc["crankTimeDelta"] = (unsigned)lastCrankTimeDelta;
    doc["cadenceHeld"] = (bool)lastCadenceHeld;
    doc["dropped"] = (bool)lastDropped;
    doc["sensorConnected"] = (bool)sensorConnected;
    doc["appConnections"] = appConnectionCount;
    doc["recoveryState"] = recoveryStateText(recoveryState);
    doc["remoteControlPointReady"] = (bool)remoteControlPointReady;
    doc["tuneStatus"] = tuneStateText((TuneState)tuneState);
    doc["tuneCanStart"] = sensorConnected && remoteControlPointReady && remotePowerControlPoint != nullptr && tuneState != TUNE_ENHANCED_SENT && tuneState != TUNE_STANDARD_SENT && !tuneRequested && !tuneTimedOut;
    doc["tuneOffsetReported"] = (bool)tuneOffsetReported;
    if (tuneOffsetReported) doc["tuneReportedOffset"] = (int)tuneReportedOffset;
    else doc["tuneReportedOffset"] = nullptr;
    doc["servingMode"] = servingModeText();
    doc["wifiConnected"] = WiFi.status() == WL_CONNECTED;
    doc["wifiProvisioned"] = wifiConfig.provisioned;
    doc["wifiIp"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("");
    doc["mqttConnected"] = mqttConnected;
    doc["mqttEnabled"] = mqttConfig.enabled;
    doc["mqttHost"] = mqttConfig.host;
    doc["mqttPort"] = mqttConfig.port;
    doc["mqttTestResult"] = lastMqttTestResult;
    doc["cachedValid"] = cachedSensor.valid;
    doc["cachedAddress"] = cachedSensor.address;
    doc["uptimeSec"] = millis() / 1000;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["lastMessage"] = lastUiMessage;
    String json;
    serializeJson(doc, json);
    sendNoCacheHeaders();
    webServer.send(200, "application/json", json);
    return;
}

static void handleWebStatusJson() {
    JsonDocument doc;
    doc["sensorConnected"] = sensorConnected;
    doc["appConnections"] = appConnectionCount;
    doc["uptimeSec"] = (uint32_t)(millis() / 1000);
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["battery"] = lastBattery == 0xFF ? "?" : String((int)lastBattery) + "%";
    doc["rawPower"] = (int)lastRawPower;
    doc["zeroedPower"] = (int)lastZeroedPower;
    doc["correctedPower"] = (int)lastCorrectedPower;
    doc["broadcastPower"] = (int)lastBroadcastPower;
    doc["cadence"] = (double)lastCadence;
    doc["crankRev"] = (unsigned int)lastCrankRev;
    doc["crankTime"] = (unsigned int)lastCrankTime;
    doc["packetIntervalMs"] = (unsigned long)lastPacketIntervalMs;
    doc["crankRevDelta"] = (unsigned int)lastCrankRevDelta;
    doc["crankTimeDelta"] = (unsigned int)lastCrankTimeDelta;
    doc["cadenceHeld"] = lastCadenceHeld;
    doc["energy"] = (unsigned int)lastEnergy;
    doc["dropped"] = lastDropped;
    doc["appliedCadenceOffset"] = (double)lastAppliedCadenceOffset;
    doc["remoteControlPointReady"] = remoteControlPointReady;
    doc["tuneStatus"] = tuneStateText((TuneState)tuneState);
    doc["tuneLastOp"] = (int)tuneLastOp;
    doc["tuneLastResponse"] = (int)tuneLastResponse;
    doc["tuneOffsetReported"] = (bool)tuneOffsetReported;
    if (tuneOffsetReported) doc["tuneReportedOffset"] = (int)tuneReportedOffset;
    else doc["tuneReportedOffset"] = nullptr;
    doc["tuneFailureReason"] = (int)tuneFailureReason;
    TuneDiagnostics diag;
    portENTER_CRITICAL(&tuneMux);
    diag = tuneDiag;
    portEXIT_CRITICAL(&tuneMux);
    JsonObject td = doc["tuneDiagnostics"].to<JsonObject>();
    td["clientIf"] = diag.clientIf;
    td["connId"] = diag.connId;
    td["controlPointHandle"] = diag.cpHandle;
    td["cccdHandle"] = diag.cccdHandle;
    td["canWrite"] = diag.canWrite;
    td["canIndicate"] = diag.canIndicate;
    td["canNotify"] = diag.canNotify;
    td["featureValid"] = diag.featureValid;
    td["featureBits"] = diag.featureBits;
    td["writeSeen"] = diag.writeSeen;
    td["writeStatus"] = diag.writeStatus;
    td["writeStartedMs"] = diag.writeStartedMs;
    td["writeEventMs"] = diag.writeEventMs;
    td["writeCount"] = diag.writeCount;
    td["cccdSeen"] = diag.cccdSeen;
    td["cccdStatus"] = diag.cccdStatus;
    td["cccdEventMs"] = diag.cccdEventMs;
    td["cccdCount"] = diag.cccdCount;
    td["registrationSeen"] = diag.registrationSeen;
    td["registrationStatus"] = diag.registrationStatus;
    td["registrationEventMs"] = diag.registrationEventMs;
    td["registrationCount"] = diag.registrationCount;
    td["responseCount"] = diag.responseCount;
    td["responseMs"] = diag.responseMs;
    td["responseLength"] = diag.responseLength;
    td["responseNotify"] = diag.responseNotify;
    td["responseAccepted"] = diag.responseAccepted;
    td["responseState"] = diag.responseState;
    td["responseExpectedOp"] = diag.responseExpectedOp;
    JsonArray raw = td["responseBytes"].to<JsonArray>();
    for (size_t i = 0; i < diag.responseCaptured; ++i) raw.add(diag.responseBytes[i]);
    doc["recoveryState"] = recoveryStateText(recoveryState);
    doc["servingMode"] = servingModeText();
    doc["lastMessage"] = lastUiMessage;

    JsonObject settings = doc["settings"].to<JsonObject>();
    settings["enabled"] = calibration.correctionEnabled;
    settings["rawDiagnostics"] = calibration.rawDiagnosticsEnabled;
    settings["localZeroOffset"] = calibration.localZeroOffset;
    settings["scaleFactor"] = calibration.scaleFactor;
    JsonArray cadence = settings["cadence"].to<JsonArray>();
    JsonArray cadenceOffset = settings["cadenceOffset"].to<JsonArray>();
    for (int i = 0; i < 4; i++) {
        cadence.add(calibration.cadence[i]);
        cadenceOffset.add(calibration.cadenceOffset[i]);
    }

    JsonObject wifi = doc["wifi"].to<JsonObject>();
    wifi["provisioned"] = wifiConfig.provisioned;
    wifi["connected"] = WiFi.status() == WL_CONNECTED;
    wifi["ssid"] = wifiConfig.ssid;
    wifi["ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";

    JsonObject mqtt = doc["mqtt"].to<JsonObject>();
    mqtt["enabled"] = mqttConfig.enabled;
    mqtt["configured"] = mqttConfigured;
    mqtt["connected"] = mqttConnected;
    mqtt["host"] = mqttConfig.host;
    mqtt["port"] = mqttConfig.port;
    mqtt["user"] = mqttConfig.user;
    mqtt["testResult"] = lastMqttTestResult;

    JsonObject cached = doc["cachedSensor"].to<JsonObject>();
    cached["valid"] = cachedSensor.valid;
    cached["address"] = cachedSensor.address;
    cached["name"] = cachedSensor.lastName;

    JsonArray eventLogJson = doc["eventLog"].to<JsonArray>();
    EventLogSnapshot* logs = snapshotEventLog();
    if (logs) {
        for (uint8_t i = 0; i < logs->count; ++i) eventLogJson.add(logs->lines[i]);
        free(logs);
    }


    String payload;
    serializeJson(doc, payload);
    sendNoCacheHeaders();
    webServer.send(200, "application/json", payload);
}

static void handleLogsDownload() {
    EventLogSnapshot* logs = snapshotEventLog();
    if (!logs) { webServer.send(503, "text/plain", "Log snapshot unavailable"); return; }
    TuneDiagnostics diag;
    portENTER_CRITICAL(&tuneMux);
    diag = tuneDiag;
    portEXIT_CRITICAL(&tuneMux);
    sendNoCacheHeaders();
    webServer.sendHeader("Content-Disposition", "attachment; filename=\"myx-bridge-logs.txt\"");
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/plain; charset=utf-8", "");
    char line[256];
    snprintf(line, sizeof(line), "MYX Bridge firmware %s\nUptime: %lu ms\nEvents (oldest first):\n", FW_VERSION, (unsigned long)millis());
    webServer.sendContent(line);
    for (uint8_t i = 0; i < logs->count; ++i) { webServer.sendContent(logs->lines[i]); webServer.sendContent("\n"); }
    free(logs);
    snprintf(line, sizeof(line), "\nTune diagnostics: write status=%d count=%lu; CCCD status=%d count=%lu; registration status=%d count=%lu\nResponse: length=%u captured=%u accepted=%u notify=%u expected opcode=0x%02X state=%u bytes=", diag.writeStatus, (unsigned long)diag.writeCount, diag.cccdStatus, (unsigned long)diag.cccdCount, diag.registrationStatus, (unsigned long)diag.registrationCount, (unsigned)diag.responseLength, (unsigned)diag.responseCaptured, (unsigned)diag.responseAccepted, (unsigned)diag.responseNotify, diag.responseExpectedOp, diag.responseState);
    webServer.sendContent(line);
    for (size_t i = 0; i < diag.responseCaptured && i < sizeof(diag.responseBytes); ++i) { snprintf(line, sizeof(line), "%s%02X", i ? " " : "", diag.responseBytes[i]); webServer.sendContent(line); }
    webServer.sendContent("\n");
    webServer.sendContent("");
}

static void handleTunePost() {
    enum TuneRequestResult { QUEUED, DISCONNECTED, NO_CONTROL_POINT, TIMED_OUT, BUSY };
    TuneRequestResult result;
    portENTER_CRITICAL(&tuneMux);
    if (!sensorConnected) result = DISCONNECTED;
    else if (!remoteControlPointReady || remotePowerControlPoint == nullptr) result = NO_CONTROL_POINT;
    else if (tuneTimedOut) result = TIMED_OUT;
    else if (tuneState == TUNE_ENHANCED_SENT || tuneState == TUNE_STANDARD_SENT || tuneRequested) result = BUSY;
    else {
        tuneRequested = true;
        tuneState = TUNE_IDLE;
        result = QUEUED;
    }
    portEXIT_CRITICAL(&tuneMux);
    if (result == DISCONNECTED) {
        webServer.send(409, "application/json", "{\"status\":\"unavailable\",\"message\":\"Sensor is not connected. Pedal to wake it and wait for it to connect.\"}");
        return;
    }
    if (result == NO_CONTROL_POINT) {
        webServer.send(409, "application/json", "{\"status\":\"unavailable\",\"message\":\"Sensor control point is unavailable.\"}");
        return;
    }
    if (result == TIMED_OUT) {
        webServer.send(409, "application/json", "{\"status\":\"unavailable\",\"message\":\"The previous tune timed out. Reconnect the sensor before retrying.\"}");
        return;
    }
    if (result == BUSY) {
        webServer.send(409, "application/json", "{\"status\":\"busy\",\"message\":\"A sensor tune is already in progress.\"}");
        return;
    }
    setUiMessage("Tune requested");
    webServer.send(202, "application/json", "{\"status\":\"queued\",\"message\":\"Tune requested\"}");
}

static void handleReconnectPost() {
    reconnectRequested = true;
    setUiMessage("Reconnect requested");
    webServer.send(202, "application/json", "{\"status\":\"reconnect-requested\",\"message\":\"Reconnect requested\"}");
}

static void handleFindAgainPost() {
    findAgainRequested = true;
    setUiMessage("Find sensor requested");
    webServer.send(202, "application/json", "{\"status\":\"find-again-requested\",\"message\":\"Find sensor requested\"}");
}

static void handleStartConfigPortalPost() {
    webServer.sendHeader("Location", "/network", true);
    webServer.send(303, "text/plain", "Use network settings");
}

static void handleCalibrationPost() {
    CalibrationSettings previous = calibration;
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
    bool valid = isfinite(calibration.localZeroOffset) && fabs(calibration.localZeroOffset) <= 32767.0f;
    for (int i = 0; i < 4; i++) {
        valid = valid && isfinite(calibration.cadence[i]) && calibration.cadence[i] >= 0 && calibration.cadence[i] <= 250;
        valid = valid && isfinite(calibration.cadenceOffset[i]) && fabs(calibration.cadenceOffset[i]) <= 32767.0f;
        if (i > 0) valid = valid && calibration.cadence[i] > calibration.cadence[i - 1];
    }
    if (!valid) {
        calibration = previous;
        webServer.send(400, "text/plain", "Invalid calibration values: use finite offsets and increasing cadence points from 0 to 250 RPM.");
        return;
    }
    saveCalibration();
    setUiMessage("Correction settings saved");
    webServer.sendHeader("Location", "/diag", true);
    webServer.send(303, "text/plain", "Saved");
}

static void handleResetCalibrationPost() {
    resetCalibration();
    setUiMessage("Correction settings reset");
    webServer.send(200, "application/json", "{\"status\":\"reset\"}");
}

static void handleWiFiSettingsPost() {
    bool wasProvisioned = wifiConfig.provisioned;
    char oldSsid[sizeof(wifiConfig.ssid)];
    strlcpy(oldSsid, wifiConfig.ssid, sizeof(oldSsid));

    wifiConfig.provisioned = webServer.hasArg("provisioned") && webServer.arg("ssid").length() > 0;
    strlcpy(wifiConfig.ssid, webServer.arg("ssid").c_str(), sizeof(wifiConfig.ssid));
    if (webServer.hasArg("clear_password")) wifiConfig.password[0] = '\0';
    else if (webServer.arg("password").length()) strlcpy(wifiConfig.password, webServer.arg("password").c_str(), sizeof(wifiConfig.password));
    saveWiFiConfig();

    if (!wifiConfig.provisioned) {
        setUiMessage("WiFi saved as disabled");
    } else if (!wasProvisioned || strcmp(oldSsid, wifiConfig.ssid) != 0) {
        setUiMessage("WiFi saved, restarting to apply");
    } else {
        setUiMessage("WiFi credentials updated, restarting to reconnect");
    }

    webServer.sendHeader("Location", "/network", true);
    webServer.send(303, "text/plain", "WiFi saved");
    scheduleRestart("WiFi settings changed, restarting");
}

static void handleResetWiFiPost() {
    mqttDisconnectCleanly();
    mqttConnected = false;
    clearWiFiConfig();
    setUiMessage("WiFi settings reset");
    webServer.send(200, "application/json", "{\"status\":\"wifi-reset\"}");
    scheduleRestart("WiFi reset, restarting");
}

static void handleMqttSettingsPost() {
    mqttConfig.enabled = webServer.hasArg("enabled");
    strlcpy(mqttConfig.host, webServer.arg("host").c_str(), sizeof(mqttConfig.host));
    mqttConfig.port = argU16("port", MQTT_DEFAULT_PORT);
    strlcpy(mqttConfig.user, webServer.arg("user").c_str(), sizeof(mqttConfig.user));
    if (webServer.hasArg("clear_pass")) mqttConfig.pass[0] = '\0';
    else if (webServer.arg("pass").length()) strlcpy(mqttConfig.pass, webServer.arg("pass").c_str(), sizeof(mqttConfig.pass));
    if (strlen(mqttConfig.host) == 0) {
        mqttConfig.enabled = false;
    }
    saveMqttConfig();
    mqttDisconnectCleanly();
    mqttConnected = false;
    setUiMessage("MQTT settings saved");
    webServer.sendHeader("Location", "/network", true);
    webServer.send(303, "text/plain", "MQTT saved");
}

static void handleResetMqttPost() {
    clearMqttConfig();
    mqttDisconnectCleanly();
    mqttConnected = false;
    strlcpy(lastMqttTestResult, "Not tested", sizeof(lastMqttTestResult));
    setUiMessage("MQTT settings reset");
    webServer.send(200, "application/json", "{\"status\":\"mqtt-reset\"}");
}

static void handleTestMqttPost() {
    MqttConfig temp = mqttConfig;
    if (webServer.hasArg("host")) temp.enabled = webServer.hasArg("enabled");
    if (webServer.hasArg("host")) strlcpy(temp.host, webServer.arg("host").c_str(), sizeof(temp.host));
    if (webServer.hasArg("port")) temp.port = argU16("port", temp.port);
    if (webServer.hasArg("user")) strlcpy(temp.user, webServer.arg("user").c_str(), sizeof(temp.user));
    if (webServer.hasArg("clear_pass")) temp.pass[0] = '\0';
    else if (webServer.arg("pass").length()) strlcpy(temp.pass, webServer.arg("pass").c_str(), sizeof(temp.pass));

    bool ok = false;
    if (!temp.enabled || strlen(temp.host) == 0) {
        strlcpy(lastMqttTestResult, "MQTT is disabled or host is empty", sizeof(lastMqttTestResult));
    } else {
        ok = testMqttConnection(temp.host, temp.port, temp.user, temp.pass);
    }

    JsonDocument doc;
    doc["ok"] = ok;
    doc["result"] = lastMqttTestResult;
    String payload;
    serializeJson(doc, payload);
    webServer.send(200, "application/json", payload);
}

static void handleFullResetPost() {
    mqttDisconnectCleanly();
    mqttConnected = false;
    fullReset();
    setUiMessage("Full reset complete");
    webServer.send(200, "application/json", "{\"status\":\"full-reset\"}");
    scheduleRestart("Full reset, restarting");
}

static void handleOtaPage() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html", "");
    webServer.sendContent("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
    webServer.sendContent("<meta name='viewport' content='width=device-width,initial-scale=1'>");
    webServer.sendContent("<title>MYX Bridge - Firmware Update</title>");
    webServer.sendContent(
        "<style>"
        "body{font-family:Segoe UI,Arial,sans-serif;max-width:680px;margin:0 auto;padding:20px;background:#eef2f6;color:#17212b}"
        "h1{margin-bottom:8px}.card{background:#fff;border:1px solid #d7dde4;border-radius:10px;padding:20px}"
        ".btn{display:inline-block;padding:12px 20px;margin-top:12px;background:#0b5fff;color:#fff;border:none;border-radius:8px;cursor:pointer;text-decoration:none}"
        ".btn.secondary{background:#4e5b67}#status{margin-top:14px;font-weight:700}#barwrap{display:none;background:#d7dde4;border-radius:8px;height:18px;overflow:hidden;margin-top:12px}#bar{height:100%;width:0;background:#0b5fff}"
        "</style></head><body><h1>Firmware Update</h1><div class='card'>");
    webServer.sendContent("<p>Current version: <b>");
    webServer.sendContent(FW_VERSION);
    webServer.sendContent("</b></p>");
    webServer.sendContent(
        "<form id='uf'><input type='file' id='fbin' name='firmware' accept='.bin' required>"
        "<br><button type='submit' class='btn'>Upload &amp; Flash</button></form>"
        "<div id='barwrap'><div id='bar'></div></div>"
        "<div id='status'></div></div><p><a href='/' class='btn secondary'>Back</a></p>"
        "<script>"
        "function st(m){document.getElementById('status').textContent=m;}"
        "function bar(p){document.getElementById('bar').style.width=p+'%';}"
        "document.getElementById('uf').addEventListener('submit',function(e){"
        "e.preventDefault();var file=document.getElementById('fbin').files[0];if(!file){st('Select a .bin file');return;}"
        "document.getElementById('barwrap').style.display='block';bar(0);st('Uploading...');"
        "var fd=new FormData();fd.append('firmware',file);var xhr=new XMLHttpRequest();"
        "xhr.upload.onprogress=function(ev){if(ev.lengthComputable){bar(Math.round(ev.loaded/ev.total*100));}};"
        "xhr.onload=function(){if(xhr.status===200){bar(100);st('Flash complete. Rebooting...');setTimeout(function(){location.href='/';},12000);}else{st('Flash failed: '+xhr.responseText);}};"
        "xhr.onerror=function(){st('Connection dropped. Device may be rebooting...');setTimeout(function(){location.href='/';},15000);};"
        "xhr.open('POST','/update');xhr.send(fd);});"
        "</script></body></html>");
    webServer.sendContent("");
}

static void handleOtaResult() {
    if (!otaComplete || Update.hasError()) {
        webServer.send(500, "text/plain", "OTA failed");
    } else {
        webServer.send(200, "text/plain", "OK");
        scheduleRestart("OTA complete, restarting");
    }
    otaComplete = false;
}

static void handleOtaUpload() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
        otaComplete = false;
        Serial.printf("[OTA] Begin: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            otaComplete = true;
            Serial.printf("[OTA] Success: %u bytes\n", (unsigned)upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaComplete = false;
    }
}

static void handleNotFound() {
    webServer.send(404, "text/plain", "Not found");
}

static void registerWebRoutes() {
    if (routesRegistered) return;
    const char* headers[] = {"Origin", "Host"};
    webServer.collectHeaders(headers, 2);
    webServer.on("/", HTTP_GET, [](){ if (setupWindow) handleSetupPage(); else if (authorizeWeb()) handleWebRoot(); });
    webServer.on("/setup", HTTP_GET, handleSetupPage);
    webServer.on("/setup", HTTP_POST, handleSetupPost);
    webServer.on("/diag", HTTP_GET, [](){ if (authorizeWeb()) handleDiagnosticsRoot(); });
    webServer.on("/network", HTTP_GET, [](){ if (authorizeWeb()) handleNetworkRoot(); });
    webServer.on("/api/live", HTTP_GET, [](){ if (authorizeWeb()) handleWebLiveJson(); });
    webServer.on("/api/status", HTTP_GET, [](){ if (authorizeWeb()) handleWebStatusJson(); });
    webServer.on("/api/logs", HTTP_GET, [](){ if (authorizeWeb()) handleLogsDownload(); });
    webServer.on("/api/settings", HTTP_GET, [](){ if (authorizeWeb()) sendSettingsJsonOnly(); });
    webServer.on("/settings/cal", HTTP_POST, [](){ if (authorizeWeb()) handleCalibrationPost(); });
    webServer.on("/settings/wifi", HTTP_POST, [](){ if (authorizeWeb()) handleWiFiSettingsPost(); });
    webServer.on("/settings/mqtt", HTTP_POST, [](){ if (authorizeWeb()) handleMqttSettingsPost(); });
    webServer.on("/tune", HTTP_POST, [](){ if (authorizeWeb()) handleTunePost(); });
    webServer.on("/reconnect", HTTP_POST, [](){ if (authorizeWeb()) handleReconnectPost(); });
    webServer.on("/find-sensor", HTTP_POST, [](){ if (authorizeWeb()) handleFindAgainPost(); });
    webServer.on("/start-config-portal", HTTP_POST, [](){ if (authorizeWeb()) handleStartConfigPortalPost(); });
    webServer.on("/reset-cal", HTTP_POST, [](){ if (authorizeWeb()) handleResetCalibrationPost(); });
    webServer.on("/reset-wifi", HTTP_POST, [](){ if (authorizeWeb()) handleResetWiFiPost(); });
    webServer.on("/reset-mqtt", HTTP_POST, [](){ if (authorizeWeb()) handleResetMqttPost(); });
    webServer.on("/test-mqtt", HTTP_POST, [](){ if (authorizeWeb()) handleTestMqttPost(); });
    webServer.on("/full-reset", HTTP_POST, [](){ if (authorizeWeb()) handleFullResetPost(); });
    webServer.on("/update", HTTP_GET, [](){ if (authorizeWeb()) handleOtaPage(); });
    webServer.on("/update", HTTP_POST, [](){ if (authorizeWeb()) handleOtaResult(); }, [](){ if (!setupWindow && adminConfigured && webServer.authenticate("admin", adminPassword) && (!webServer.hasHeader("Origin") || webServer.header("Origin") == "http://" + webServer.header("Host"))) handleOtaUpload(); });
    webServer.onNotFound(handleNotFound);
    routesRegistered = true;
}

static void setupWebServer() {
    registerWebRoutes();
}

static void serviceRecoveryActions() {
    if (reconnectRequested) {
        reconnectRequested = false;
        if (!sensorConnected) {
            requestSensorRecovery(true, "Manual reconnect requested");
        } else if (bleClient != nullptr) {
            setUiMessage("Reconnecting sensor");
            disconnectPending = true;
            bleClient->disconnect();
        }
    }

    if (findAgainRequested) {
        findAgainRequested = false;
        cleanupSensorDevice();
        if (sensorConnected && bleClient != nullptr) {
            setUiMessage("Re-finding sensor");
            scanAfterDisconnect = true;
            disconnectPending = true;
            bleClient->disconnect();
        } else {
            requestSensorRecovery(false, "Manual scan requested");
        }
    }

    if (doConnect && !disconnectPending) {
        doConnect = false;
        recoveryState = RECOVERY_CONNECTING;
        stopBleScan();
        Serial.println("[SENSOR] Starting connect attempt");
        if (connectToSensor()) {
            Serial.println("[OK] Bridge active - power data flowing");
            if (!wifiConfig.provisioned) {
                startAccessPointIfNeeded();
            }
        } else {
            Serial.println("[ERROR] Sensor connection failed");
            requestSensorRecovery(false, "Reconnect failed, scanning");
        }
    }

    if (!sensorConnected && doScan) {
        startBleScan();
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=========================================");
    Serial.println("  MYX Power & Cadence Bridge");
    Serial.printf("  Firmware: %s\n", FW_VERSION);
    Serial.println("=========================================");

    pinMode(LED_PIN, OUTPUT);
    pinMode(0, INPUT_PULLUP);
    bootButtonArmed = digitalRead(0) == HIGH;
    attachInterrupt(digitalPinToInterrupt(0), onBootButtonEdge, CHANGE);
    digitalWrite(LED_PIN, LOW);
    lastCrankMoveMs = millis();
    WiFi.persistent(false);

    buildDeviceId();
    loadAdminPassword();
    loadCalibration();
    loadWiFiConfig();
    loadMqttConfig();
    loadCachedSensor();

    Serial.printf("[CAL] Correction: %s | Raw diagnostics: %s | Zero: %.1f W | Scale: %.3f\n",
                  calibration.correctionEnabled ? "enabled" : "disabled",
                  calibration.rawDiagnosticsEnabled ? "enabled" : "disabled",
                  (double)calibration.localZeroOffset,
                  (double)calibration.scaleFactor);
    Serial.printf("[WIFI] Provisioned: %s | SSID: %s\n",
                  wifiConfig.provisioned ? "yes" : "no",
                  wifiConfig.provisioned ? wifiConfig.ssid : "(none)");
    Serial.printf("[MQTT] Enabled: %s | Host: %s\n",
                  mqttConfig.enabled ? "yes" : "no",
                  strlen(mqttConfig.host) ? mqttConfig.host : "(none)");
    if (cachedSensor.valid) {
        Serial.printf("[SENSOR] Cached target: %s (%s)\n", cachedSensor.address, cachedSensor.lastName);
    }

    BLEDevice::init(BRIDGE_NAME);
    BLEDevice::setCustomGattcHandler(onGattClientEventComplete);
    setupBLEServer();
    setupWebServer();
    if (!adminConfigured) openSetupWindow();

    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SensorScanCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);

    mqttClient.setBufferSize(1024);

    if (wifiConfig.provisioned && !setupWindow) {
        beginWiFiConnect();
    }

    requestSensorRecovery(true, wifiConfig.provisioned
        ? "Provisioned boot, recovering sensor in background"
        : "Waiting for sensor before enabling AP");
}

void loop() {
    portENTER_CRITICAL(&buttonMux);
    bool buttonTriggered = bootButtonEvent || (bootButtonArmed && bootButtonPressed && millis() - bootButtonDown >= 5000);
    if (buttonTriggered) {
        bootButtonEvent = false;
        bootButtonArmed = digitalRead(0) == HIGH;
        bootButtonPressed = false;
    }
    portEXIT_CRITICAL(&buttonMux);
    if (buttonTriggered) openSetupWindow();
    if (setupWindow && millis() - setupStarted >= 600000 && !restartPending) {
        setupWindow = false;
        stopAccessPoint();
    }
    if (sensorConnected && !disconnectPending && !tuneRequested &&
        tuneState != TUNE_STANDARD_SENT && tuneState != TUNE_ENHANCED_SENT && lastSensorPacketMs != 0 &&
        millis() - lastSensorPacketMs > 15000) {
        setUiMessage("Sensor data timed out, reconnecting");
        cleanupBleClient();
    }
    ensureWiFiState();
    if (!wifiConfig.provisioned && sensorConnected) {
        startAccessPointIfNeeded();
    }

    if (webServerStarted) {
        webServer.handleClient();
    }
    serviceTuneState();
    if (!setupWindow) serviceRecoveryActions();
    if (!setupWindow) ensureMqttState();
    else if (mqttClient.connected()) mqttClient.loop();

    if (restartPending && millis() >= restartAtMs) {
        delay(100);
        ESP.restart();
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
        Serial.printf("[STATUS] Sensor: %s | Recovery: %s | Mode: %s | Apps: %d | Broadcast: %d W | Cadence: %.0f RPM | Battery: %s | WiFi: %s | MQTT: %s\n",
                      sensorConnected ? "CONNECTED" : "SEARCHING",
                      recoveryStateText(recoveryState),
                      servingModeText(),
                      appConnectionCount,
                      (int)lastBroadcastPower,
                      (double)lastCadence,
                      battStr,
                      WiFi.status() == WL_CONNECTED ? "CONNECTED" : (wifiConfig.provisioned ? "WAITING" : "OFF"),
                      mqttConnected ? "CONNECTED" : (mqttConfigured ? "WAITING" : "OFF"));
    }

    delay(10);
}
