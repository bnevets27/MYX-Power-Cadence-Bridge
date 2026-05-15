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
#include <WiFiManager.h>
#include <Update.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <math.h>

// Product configuration
static const char* TARGET_SENSOR_NAME = "BKSNSR";
static const char* BRIDGE_NAME = "MYX Bridge";
static const char* AP_NAME = "MYX-Bridge-Setup";
static const char* AP_PASS = "";
static const char* FW_VERSION = "2.1.0-unified";

static const uint8_t XOR_KEY = 0xAA;
static const float SMOOTHING_FACTOR = 0.5f;
static const uint32_t DROPOUT_MS = 2500;
static const uint32_t TUNE_TIMEOUT_MS = 6000;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 15000;
static const uint32_t MQTT_RETRY_INTERVAL_MS = 5000;
static const uint32_t MQTT_PUBLISH_INTERVAL_MS = 2000;

static const uint8_t CP_OP_START_OFFSET_COMPENSATION = 0x0C;
static const uint8_t CP_OP_START_ENHANCED_OFFSET_COMPENSATION = 0x10;
static const uint8_t CP_OP_RESPONSE_CODE = 0x20;
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
static volatile uint8_t tuneFailureReason = 0;
static bool tuneRequested = false;
static uint32_t tuneStartedMs = 0;

// UI action state
static volatile bool reconnectRequested = false;
static volatile bool findAgainRequested = false;
static volatile bool configPortalRequested = false;
static uint32_t configPortalStartAtMs = 0;
static char lastUiMessage[160] = "Booting";
static char lastMqttTestResult[96] = "Not tested";
static char eventLog[10][96];
static uint8_t eventLogNext = 0;
static uint8_t eventLogCount = 0;

#ifndef LED_PIN
#define LED_PIN 2
#endif
static unsigned long lastBlinkTime = 0;
static bool ledState = false;

static void scheduleRestart(const char* reason);

static void setUiMessage(const char* text) {
    strlcpy(lastUiMessage, text, sizeof(lastUiMessage));
    strlcpy(eventLog[eventLogNext], text, sizeof(eventLog[eventLogNext]));
    eventLogNext = (uint8_t)((eventLogNext + 1) % 10);
    if (eventLogCount < 10) {
        eventLogCount++;
    }
    Serial.printf("[UI] %s\n", lastUiMessage);
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
        case TUNE_TIMEOUT: return "sensor tune timed out";
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

static void runWiFiManagerPortal() {
    setUiMessage("Starting WiFiManager setup portal");
    Serial.printf("[WIFI] Starting WiFiManager portal: %s\n", AP_NAME);

    if (webServerStarted) {
        webServer.stop();
        webServerStarted = false;
    }
    if (apRunning) {
        WiFi.softAPdisconnect(true);
        apRunning = false;
    }

    char mqttEnabledValue[2];
    snprintf(mqttEnabledValue, sizeof(mqttEnabledValue), "%d", mqttConfig.enabled ? 1 : 0);
    char mqttPortValue[6];
    snprintf(mqttPortValue, sizeof(mqttPortValue), "%u", (unsigned int)mqttConfig.port);

    WiFiManager wm;
    WiFiManagerParameter mqttHeader("<br><h3>MQTT Settings</h3>");
    WiFiManagerParameter mqttEnabledParam("mqtt_enabled", "MQTT Enabled (1 or 0)", mqttEnabledValue, 1);
    WiFiManagerParameter mqttHostParam("mqtt_host", "MQTT Host", mqttConfig.host, 63);
    WiFiManagerParameter mqttPortParam("mqtt_port", "MQTT Port", mqttPortValue, 5);
    WiFiManagerParameter mqttUserParam("mqtt_user", "MQTT Username", mqttConfig.user, 31);
    WiFiManagerParameter mqttPassParam("mqtt_pass", "MQTT Password", mqttConfig.pass, 63);

    wm.addParameter(&mqttHeader);
    wm.addParameter(&mqttEnabledParam);
    wm.addParameter(&mqttHostParam);
    wm.addParameter(&mqttPortParam);
    wm.addParameter(&mqttUserParam);
    wm.addParameter(&mqttPassParam);
    wm.setConfigPortalTimeout(300);
    wm.setConnectTimeout(20);
    wm.setBreakAfterConfig(true);

    bool ok = wm.startConfigPortal(AP_NAME);
    if (ok || WiFi.status() == WL_CONNECTED) {
        String ssid = WiFi.SSID();
        String pass = WiFi.psk();
        wifiConfig.provisioned = ssid.length() > 0;
        strlcpy(wifiConfig.ssid, ssid.c_str(), sizeof(wifiConfig.ssid));
        strlcpy(wifiConfig.password, pass.c_str(), sizeof(wifiConfig.password));
        saveWiFiConfig();

        mqttConfig.enabled = atoi(mqttEnabledParam.getValue()) != 0;
        strlcpy(mqttConfig.host, mqttHostParam.getValue(), sizeof(mqttConfig.host));
        mqttConfig.port = (uint16_t)atoi(mqttPortParam.getValue());
        if (mqttConfig.port == 0) mqttConfig.port = MQTT_DEFAULT_PORT;
        strlcpy(mqttConfig.user, mqttUserParam.getValue(), sizeof(mqttConfig.user));
        strlcpy(mqttConfig.pass, mqttPassParam.getValue(), sizeof(mqttConfig.pass));
        if (strlen(mqttConfig.host) == 0) mqttConfig.enabled = false;
        saveMqttConfig();

        Serial.printf("[WIFI] WiFiManager saved SSID '%s'; MQTT %s\n",
                      wifiConfig.ssid, mqttConfig.enabled ? "enabled" : "disabled");
        scheduleRestart("Network setup saved, restarting");
    } else {
        Serial.println("[WIFI] WiFiManager portal closed without connection");
        scheduleRestart("Network setup closed, restarting");
    }
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
            bleClient->disconnect();
        }
        delete bleClient;
        bleClient = nullptr;
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
    if (wifiConfig.provisioned || apRunning || !sensorConnected) return;
    WiFi.mode(WIFI_AP);
    bool apStarted = strlen(AP_PASS) > 0 ? WiFi.softAP(AP_NAME, AP_PASS) : WiFi.softAP(AP_NAME);
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

static void beginWiFiConnect() {
    if (!wifiConfig.provisioned) return;
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiConfig.ssid, wifiConfig.password);
    wifiConnectInProgress = true;
    wifiConnectStartedAt = millis();
    lastWiFiConnectAttempt = millis();
    setUiMessage("Connecting to WiFi");
    Serial.printf("[WIFI] Connecting to %s\n", wifiConfig.ssid);
}

static void ensureWiFiState() {
    if (!wifiConfig.provisioned) {
        if (WiFi.getMode() != WIFI_OFF && !apRunning) {
            WiFi.mode(WIFI_OFF);
        }
        return;
    }

    if (apRunning) {
        stopAccessPoint();
    }

    wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
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
        writeSensorTuneOp(CP_OP_START_OFFSET_COMPENSATION, TUNE_STANDARD_SENT);
    }

    if ((tuneState == TUNE_STANDARD_SENT || tuneState == TUNE_ENHANCED_SENT) &&
        millis() - tuneStartedMs > TUNE_TIMEOUT_MS) {
        if (tuneState == TUNE_STANDARD_SENT) {
            setUiMessage("Standard tune timed out, trying enhanced tune");
            Serial.println("[TUNE] Standard tune timed out; trying enhanced tune");
            writeSensorTuneOp(CP_OP_START_ENHANCED_OFFSET_COMPENSATION, TUNE_ENHANCED_SENT);
        } else {
            tuneState = TUNE_TIMEOUT;
            setUiMessage("Sensor tune timed out");
            Serial.println("[TUNE] Enhanced tune timed out");
        }
    }

    if (tuneState == TUNE_NOT_SUPPORTED && tuneLastOp == CP_OP_START_OFFSET_COMPENSATION) {
        setUiMessage("Standard tune unsupported, trying enhanced tune");
        Serial.println("[TUNE] Standard tune unsupported; trying enhanced tune");
        writeSensorTuneOp(CP_OP_START_ENHANCED_OFFSET_COMPENSATION, TUNE_ENHANCED_SENT);
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
    (void)isNotify;
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
        setUiMessage("Sensor tune complete");
        Serial.printf("[TUNE] Sensor tune complete. Offset result: %d\n", tuneReportedOffset);
    } else if (tuneLastResponse == CP_RESPONSE_NOT_SUPPORTED) {
        tuneState = TUNE_NOT_SUPPORTED;
        setUiMessage("Sensor tune opcode not supported");
        Serial.printf("[TUNE] Sensor rejected op 0x%02X as not supported\n", tuneLastOp);
    } else if (tuneLastResponse == CP_RESPONSE_OPERATION_FAILED) {
        tuneState = TUNE_FAILED;
        setUiMessage("Sensor tune failed");
        Serial.printf("[TUNE] Sensor tune failed for op 0x%02X, reason 0x%02X\n", tuneLastOp, tuneFailureReason);
    } else {
        tuneState = TUNE_FAILED;
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
        Serial.println("[SENSOR] Disconnected from power meter!");
        sensorConnected = false;
        remoteControlPointReady = false;
        remotePowerControlPoint = nullptr;
        tuneState = TUNE_IDLE;
        if (!wifiConfig.provisioned) {
            stopAccessPoint();
        }
        requestSensorRecovery(true, "Sensor disconnected, recovering");
    }
};

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

        if (!hasCPS && !nameMatch && !cachedMatch) return;

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
    cleanupBleClient();

    bleClient = BLEDevice::createClient();
    bleClient->setClientCallbacks(new SensorClientCallbacks());

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

    const char* connectedName = sensorDevice != nullptr ? sensorDevice->getName().c_str() : cachedSensor.lastName;
    saveCachedSensor(bleClient->getPeerAddress(),
                     sensorDevice != nullptr ? sensorDevice->getAddressType() : (esp_ble_addr_type_t)cachedSensor.addrType,
                     connectedName);

    lastCrankMoveMs = millis();
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

        char payload[512];
        serializeJson(doc, payload, sizeof(payload));
        mqttClient.publish(topic, payload, true);
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

    char payload[512];
    serializeJson(doc, payload, sizeof(payload));
    mqttClient.publish(topic, payload, true);
}

static void publishSensorData() {
    if (!mqttClient.connected()) return;

    JsonDocument doc;
    doc["power"] = (int)lastBroadcastPower;
    doc["cadence"] = round(lastCadence * 10.0f) / 10.0f;
    doc["battery"] = (lastBattery == 0xFF) ? 0 : (int)lastBattery;
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

static void ensureMqttState() {
    if (!mqttConfigured || !mqttConfig.enabled) {
        if (mqttClient.connected()) mqttClient.disconnect();
        mqttConnected = false;
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        if (mqttClient.connected()) mqttClient.disconnect();
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
        ".btn:hover{background:#6c44a0}"
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
             wifiStatusText().c_str());
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>MQTT</span><span class='value %s' id='val-mqtt'>%s</span></div>",
             mqttConnected ? "online" : "offline",
             mqttStatusText().c_str());
    webServer.sendContent(buf);
    if (mqttConfig.enabled && strlen(mqttConfig.host) > 0) {
        snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>MQTT Broker</span><span class='value' id='val-broker'>%s:%u</span></div>",
                 mqttConfig.host, (unsigned int)mqttConfig.port);
        webServer.sendContent(buf);
    }
    webServer.sendContent("</div>");

    webServer.sendContent("<div class='card'><h2>Calibrate Sensor</h2><p class='note'>Pedal to wake the sensor, get off the bike, set the crank vertical, then run tune.</p>");
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Tune Status</span><span class='value' id='val-tune'>%s</span></div>", tuneStateText((TuneState)tuneState));
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Returned Offset</span><span class='value' id='val-tune-offset'>%d</span></div>", (int)tuneReportedOffset);
    webServer.sendContent(buf);
    snprintf(buf, sizeof(buf), "<div class='row'><span class='label'>Control Point</span><span class='value %s' id='val-cp'>%s</span></div>",
             remoteControlPointReady ? "online" : "offline",
             remoteControlPointReady ? "Ready" : "Unavailable");
    webServer.sendContent(buf);
    webServer.sendContent(
        "<div class='status-banner' id='action-status'>Ready</div>"
        "<div class='btn-row three'>"
        "<button class='btn' type='button' onclick=\"sendAction('/tune','Tune request sent. Stay off the bike while calibration runs.')\">Tune Sensor</button>"
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
        "function sendAction(path,msg){setAction(msg);fetch(path,{method:'POST'}).then(function(r){return r.json();}).then(function(d){if(d&&d.status){setAction(msg);refresh();}}).catch(function(){setAction('Action request failed');});}"
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
        "upd('val-wifi',d.wifiConnected?d.wifiIp:(d.wifiProvisioned?'Waiting':'Not provisioned'));"
        "setClass('val-wifi',d.wifiConnected?'online':'offline');"
        "upd('val-mqtt',d.mqttConnected?'Connected':(d.mqttEnabled?'Configured':'Disabled'));"
        "setClass('val-mqtt',d.mqttConnected?'online':'offline');"
        "upd('val-broker',(d.mqttEnabled&&d.mqttHost)?(d.mqttHost+':'+d.mqttPort):'');"
        "upd('val-tune',d.tuneStatus);"
        "upd('val-tune-offset',d.tuneReportedOffset);"
        "upd('val-cp',d.remoteControlPointReady?'Ready':'Unavailable');"
        "setClass('val-cp',d.remoteControlPointReady?'online':'offline');"
        "upd('val-mode',d.servingMode);"
        "upd('val-cached',d.cachedValid?d.cachedAddress:'No');"
        "upd('val-uptime',fmtUptime(d.uptimeSec));"
        "upd('val-heap',Math.round(d.freeHeap/1024)+' KB');"
        "if(d.lastMessage){setAction(d.lastMessage);}"
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
    webServer.sendContent(lastUiMessage);
    webServer.sendContent(
        "</span></div><div class='row'><span class='label'>MQTT Test</span><span class='value' id='mqttTest'>");
    webServer.sendContent(lastMqttTestResult);
    webServer.sendContent(
        "</span></div><div class='row'><span class='label'>Firmware</span><span class='value'>");
    webServer.sendContent(FW_VERSION);
    webServer.sendContent("</span></div><pre id='eventLog'>");
    for (uint8_t i = 0; i < eventLogCount; i++) {
        int idx = (int)eventLogNext - (int)eventLogCount + i;
        while (idx < 0) idx += 10;
        webServer.sendContent(eventLog[idx]);
        if (i + 1 < eventLogCount) webServer.sendContent("\n");
    }
    webServer.sendContent(
        "</pre></section>"
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
        "function refresh(){var x=new XMLHttpRequest();x.open('GET','/api/live?ts='+(new Date().getTime()),true);x.onreadystatechange=function(){if(x.readyState!==4||x.status!==200)return;try{var d=JSON.parse(x.responseText);setVal('raw',d.rawPower+' W');setVal('zeroed',d.zeroedPower+' W');setVal('corrected',d.correctedPower+' W');setVal('appliedOffset',Number(d.appliedCadenceOffset).toFixed(1)+' W');setVal('battery',d.battery);setVal('energy',d.energy+' kJ');setVal('msg',d.lastMessage);setVal('mqttTest','See Network Settings');var held=d.cadenceHeld?'yes':'no';var drop=d.dropped?'yes':'no';document.getElementById('eventLog').textContent='Packet interval: '+d.packetIntervalMs+' ms\\nCrank rev delta: '+d.crankRevDelta+'\\nCrank time delta: '+d.crankTimeDelta+'\\nCadence held: '+held+'\\nDropped: '+drop;}catch(e){}};x.send();}"
        "refresh();setInterval(refresh,500);"
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
        "<section class='card'><h2>Guided Setup Portal</h2><p class='note'>Use the standard WiFiManager portal for WiFi and MQTT entry. The bridge will restart after saving.</p>"
        "<form method='post' action='/start-config-portal'><button type='submit'>Launch WiFi/MQTT Setup Portal</button></form></section>"
        "<section class='card'><h2>WiFi Settings</h2><p class='note'>Saving WiFi changes only WiFi credentials. MQTT settings stay separate.</p>"
        "<form method='post' action='/settings/wifi'>"
        "<label class='check'><input type='checkbox' name='provisioned'");
    if (wifiConfig.provisioned) webServer.sendContent(" checked");
    webServer.sendContent(
        ">WiFi enabled / provisioned</label>"
        "<label>SSID<input name='ssid' maxlength='32' value='");
    webServer.sendContent(wifiConfig.ssid);
    webServer.sendContent(
        "'></label>"
        "<label>Password<input name='password' type='password' maxlength='64' value='");
    webServer.sendContent(wifiConfig.password);
    webServer.sendContent(
        "'></label>"
        "<div class='actions'><button type='submit'>Save WiFi</button></div>"
        "</form>"
        "<form method='post' action='/reset-wifi'><div class='actions'><button type='submit' class='warn'>Reset WiFi</button></div></form>"
        "</section>"
        "<section class='card'><h2>MQTT Settings</h2><p class='note'>Saving MQTT does not touch WiFi. Use Test MQTT for a broker check while connected on LAN.</p>"
        "<form id='mqtt-form' method='post' action='/settings/mqtt'>"
        "<label class='check'><input type='checkbox' name='enabled'");
    if (mqttConfig.enabled) webServer.sendContent(" checked");
    webServer.sendContent(
        ">MQTT enabled</label>"
        "<label>Host<input name='host' maxlength='63' value='");
    webServer.sendContent(mqttConfig.host);
    webServer.sendContent(
        "'></label>"
        "<label>Port<input name='port' type='number' min='1' max='65535' value='");
    webServer.sendContent(String(mqttConfig.port));
    webServer.sendContent(
        "'></label>"
        "<label>Username<input name='user' maxlength='31' value='");
    webServer.sendContent(mqttConfig.user);
    webServer.sendContent(
        "'></label>"
        "<label>Password<input name='pass' type='password' maxlength='63' value='");
    webServer.sendContent(mqttConfig.pass);
    webServer.sendContent(
        "'></label>"
        "<div class='row'><span class='label'>MQTT Test</span><span class='value' id='mqttTest'>");
    webServer.sendContent(lastMqttTestResult);
    webServer.sendContent(
        "</span></div>"
        "<div class='actions'><button type='submit'>Save MQTT</button><button type='button' class='secondary' onclick='testMqtt()'>Test MQTT</button></div>"
        "</form>"
        "<form method='post' action='/reset-mqtt'><div class='actions'><button type='submit' class='warn'>Reset MQTT</button></div></form>"
        "</section>"
        "<section class='card'><h2>Reset</h2><form method='post' action='/full-reset'><button type='submit' class='warn' onclick=\"return confirm('Full reset WiFi, MQTT, cached sensor, and correction settings?')\">Full Reset</button></form></section>"
        "<script>"
        "function encodeForm(form){var els=form.elements,parts=[],i,e;for(i=0;i<els.length;i++){e=els[i];if(!e.name)continue;if((e.type==='checkbox'||e.type==='radio')&&!e.checked)continue;parts.push(encodeURIComponent(e.name)+'='+encodeURIComponent(e.value));}return parts.join('&');}"
        "function testMqtt(){var form=document.getElementById('mqtt-form');var xhr=new XMLHttpRequest();xhr.open('POST','/test-mqtt',true);xhr.setRequestHeader('Content-Type','application/x-www-form-urlencoded');xhr.onreadystatechange=function(){if(xhr.readyState!==4||xhr.status!==200)return;try{var d=JSON.parse(xhr.responseText);document.getElementById('mqttTest').textContent=d.result;}catch(e){document.getElementById('mqttTest').textContent='Test failed';}};xhr.send(encodeForm(form));}"
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
    char payload[1024];
    char battery[8];
    if (lastBattery == 0xFF) snprintf(battery, sizeof(battery), "?");
    else snprintf(battery, sizeof(battery), "%d%%", (int)lastBattery);

    snprintf(payload, sizeof(payload),
             "{\"broadcastPower\":%d,\"cadence\":%.1f,\"battery\":\"%s\","
             "\"rawPower\":%d,\"zeroedPower\":%d,\"correctedPower\":%d,"
             "\"appliedCadenceOffset\":%.1f,\"energy\":%u,\"crankRev\":%u,"
             "\"crankTime\":%u,\"packetIntervalMs\":%lu,\"crankRevDelta\":%u,"
             "\"crankTimeDelta\":%u,\"cadenceHeld\":%s,\"dropped\":%s,"
             "\"sensorConnected\":%s,"
             "\"appConnections\":%d,\"recoveryState\":\"%s\",\"remoteControlPointReady\":%s,"
             "\"tuneStatus\":\"%s\",\"tuneReportedOffset\":%d,\"servingMode\":\"%s\","
             "\"wifiConnected\":%s,\"wifiProvisioned\":%s,\"wifiIp\":\"%s\","
             "\"mqttConnected\":%s,\"mqttEnabled\":%s,\"mqttHost\":\"%s\",\"mqttPort\":%u,"
             "\"cachedValid\":%s,\"cachedAddress\":\"%s\",\"uptimeSec\":%lu,"
             "\"freeHeap\":%u,\"lastMessage\":\"%s\"}",
             (int)lastBroadcastPower,
             (double)lastCadence,
             battery,
             (int)lastRawPower,
             (int)lastZeroedPower,
             (int)lastCorrectedPower,
             (double)lastAppliedCadenceOffset,
             (unsigned int)lastEnergy,
             (unsigned int)lastCrankRev,
             (unsigned int)lastCrankTime,
             (unsigned long)lastPacketIntervalMs,
             (unsigned int)lastCrankRevDelta,
             (unsigned int)lastCrankTimeDelta,
             lastCadenceHeld ? "true" : "false",
             lastDropped ? "true" : "false",
             sensorConnected ? "true" : "false",
             appConnectionCount,
             recoveryStateText(recoveryState),
             remoteControlPointReady ? "true" : "false",
             tuneStateText((TuneState)tuneState),
             (int)tuneReportedOffset,
             servingModeText(),
             WiFi.status() == WL_CONNECTED ? "true" : "false",
             wifiConfig.provisioned ? "true" : "false",
             WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "",
             mqttConnected ? "true" : "false",
             mqttConfig.enabled ? "true" : "false",
             mqttConfig.host,
             (unsigned int)mqttConfig.port,
             cachedSensor.valid ? "true" : "false",
             cachedSensor.address,
             (unsigned long)(millis() / 1000),
             (unsigned int)ESP.getFreeHeap(),
             lastUiMessage);
    sendNoCacheHeaders();
    webServer.send(200, "application/json", payload);
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
    doc["tuneReportedOffset"] = (int)tuneReportedOffset;
    doc["tuneFailureReason"] = (int)tuneFailureReason;
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
    wifi["password"] = wifiConfig.password;
    wifi["ip"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";

    JsonObject mqtt = doc["mqtt"].to<JsonObject>();
    mqtt["enabled"] = mqttConfig.enabled;
    mqtt["configured"] = mqttConfigured;
    mqtt["connected"] = mqttConnected;
    mqtt["host"] = mqttConfig.host;
    mqtt["port"] = mqttConfig.port;
    mqtt["user"] = mqttConfig.user;
    mqtt["pass"] = mqttConfig.pass;
    mqtt["testResult"] = lastMqttTestResult;

    JsonObject cached = doc["cachedSensor"].to<JsonObject>();
    cached["valid"] = cachedSensor.valid;
    cached["address"] = cachedSensor.address;
    cached["name"] = cachedSensor.lastName;

    JsonArray eventLogJson = doc["eventLog"].to<JsonArray>();
    for (uint8_t i = 0; i < eventLogCount; i++) {
        int idx = (int)eventLogNext - (int)eventLogCount + i;
        while (idx < 0) idx += 10;
        eventLogJson.add(eventLog[idx]);
    }

    String payload;
    serializeJson(doc, payload);
    sendNoCacheHeaders();
    webServer.send(200, "application/json", payload);
}

static void handleTunePost() {
    tuneRequested = true;
    tuneState = TUNE_IDLE;
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
    configPortalRequested = true;
    configPortalStartAtMs = millis() + 750;
    webServer.send(200, "text/html",
                   "<!doctype html><html><head><meta charset='utf-8'>"
                   "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                   "<meta http-equiv='refresh' content='8;url=/'>"
                   "<title>Starting Setup</title></head>"
                   "<body style='font-family:sans-serif;background:#1a1a2e;color:#e0e0e0;padding:30px'>"
                   "<h2>Starting WiFi/MQTT setup portal...</h2>"
                   "<p>If the page changes networks, reconnect to <b>MYX-Bridge-Setup</b> and open <b>192.168.4.1</b>.</p>"
                   "</body></html>");
}

static void handleCalibrationPost() {
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
    strlcpy(wifiConfig.password, webServer.arg("password").c_str(), sizeof(wifiConfig.password));
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
    clearWiFiConfig();
    WiFi.disconnect(true);
    setUiMessage("WiFi settings reset");
    webServer.send(200, "application/json", "{\"status\":\"wifi-reset\"}");
    scheduleRestart("WiFi reset, restarting");
}

static void handleMqttSettingsPost() {
    mqttConfig.enabled = webServer.hasArg("enabled");
    strlcpy(mqttConfig.host, webServer.arg("host").c_str(), sizeof(mqttConfig.host));
    mqttConfig.port = argU16("port", MQTT_DEFAULT_PORT);
    strlcpy(mqttConfig.user, webServer.arg("user").c_str(), sizeof(mqttConfig.user));
    strlcpy(mqttConfig.pass, webServer.arg("pass").c_str(), sizeof(mqttConfig.pass));
    if (strlen(mqttConfig.host) == 0) {
        mqttConfig.enabled = false;
    }
    saveMqttConfig();
    mqttClient.disconnect();
    mqttConnected = false;
    setUiMessage("MQTT settings saved");
    webServer.sendHeader("Location", "/network", true);
    webServer.send(303, "text/plain", "MQTT saved");
}

static void handleResetMqttPost() {
    clearMqttConfig();
    mqttClient.disconnect();
    mqttConnected = false;
    strlcpy(lastMqttTestResult, "Not tested", sizeof(lastMqttTestResult));
    setUiMessage("MQTT settings reset");
    webServer.send(200, "application/json", "{\"status\":\"mqtt-reset\"}");
}

static void handleTestMqttPost() {
    MqttConfig temp = mqttConfig;
    temp.enabled = webServer.hasArg("enabled");
    if (webServer.hasArg("host")) strlcpy(temp.host, webServer.arg("host").c_str(), sizeof(temp.host));
    if (webServer.hasArg("port")) temp.port = argU16("port", temp.port);
    if (webServer.hasArg("user")) strlcpy(temp.user, webServer.arg("user").c_str(), sizeof(temp.user));
    if (webServer.hasArg("pass")) strlcpy(temp.pass, webServer.arg("pass").c_str(), sizeof(temp.pass));

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
    fullReset();
    WiFi.disconnect(true);
    mqttClient.disconnect();
    mqttConnected = false;
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
    if (Update.hasError()) {
        webServer.send(500, "text/plain", "OTA failed");
    } else {
        webServer.send(200, "text/plain", "OK");
        scheduleRestart("OTA complete, restarting");
    }
}

static void handleOtaUpload() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
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
            Serial.printf("[OTA] Success: %u bytes\n", (unsigned)upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    }
}

static void handleNotFound() {
    webServer.send(404, "text/plain", "Not found");
}

static void registerWebRoutes() {
    if (routesRegistered) return;
    webServer.on("/", HTTP_GET, handleWebRoot);
    webServer.on("/diag", HTTP_GET, handleDiagnosticsRoot);
    webServer.on("/network", HTTP_GET, handleNetworkRoot);
    webServer.on("/api/live", HTTP_GET, handleWebLiveJson);
    webServer.on("/api/status", HTTP_GET, handleWebStatusJson);
    webServer.on("/api/settings", HTTP_GET, sendSettingsJsonOnly);
    webServer.on("/settings/cal", HTTP_POST, handleCalibrationPost);
    webServer.on("/settings/wifi", HTTP_POST, handleWiFiSettingsPost);
    webServer.on("/settings/mqtt", HTTP_POST, handleMqttSettingsPost);
    webServer.on("/tune", HTTP_POST, handleTunePost);
    webServer.on("/reconnect", HTTP_POST, handleReconnectPost);
    webServer.on("/find-sensor", HTTP_POST, handleFindAgainPost);
    webServer.on("/start-config-portal", HTTP_POST, handleStartConfigPortalPost);
    webServer.on("/reset-cal", HTTP_POST, handleResetCalibrationPost);
    webServer.on("/reset-wifi", HTTP_POST, handleResetWiFiPost);
    webServer.on("/reset-mqtt", HTTP_POST, handleResetMqttPost);
    webServer.on("/test-mqtt", HTTP_POST, handleTestMqttPost);
    webServer.on("/full-reset", HTTP_POST, handleFullResetPost);
    webServer.on("/update", HTTP_GET, handleOtaPage);
    webServer.on("/update", HTTP_POST, handleOtaResult, handleOtaUpload);
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
        } else {
            setUiMessage("Sensor already connected");
        }
    }

    if (findAgainRequested) {
        findAgainRequested = false;
        cleanupSensorDevice();
        if (sensorConnected && bleClient != nullptr) {
            setUiMessage("Re-finding sensor");
            bleClient->disconnect();
        } else {
            requestSensorRecovery(false, "Manual scan requested");
        }
    }

    if (doConnect) {
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
    digitalWrite(LED_PIN, LOW);
    lastCrankMoveMs = millis();
    WiFi.persistent(false);

    buildDeviceId();
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
    setupBLEServer();
    setupWebServer();

    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SensorScanCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);

    mqttClient.setBufferSize(1024);

    if (wifiConfig.provisioned) {
        beginWiFiConnect();
    }

    requestSensorRecovery(true, wifiConfig.provisioned
        ? "Provisioned boot, recovering sensor in background"
        : "Waiting for sensor before enabling AP");
}

void loop() {
    if (configPortalRequested && millis() >= configPortalStartAtMs) {
        configPortalRequested = false;
        runWiFiManagerPortal();
    }

    ensureWiFiState();
    if (!wifiConfig.provisioned && sensorConnected) {
        startAccessPointIfNeeded();
    }

    if (webServerStarted) {
        webServer.handleClient();
    }
    serviceTuneState();
    serviceRecoveryActions();
    ensureMqttState();

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
