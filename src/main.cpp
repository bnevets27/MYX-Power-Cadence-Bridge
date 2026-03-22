/*
 * ESP32 BLE Bike Power Meter Bridge + Home Assistant
 * ===================================================
 *
 * This firmware does three things:
 *
 * 1. BLE BRIDGE: Connects to a MYX/BKSNSR power meter sensor that
 *    XOR-masks its Cycling Power Measurement (0x2A63) data with 0xAA,
 *    decodes it, and re-broadcasts as a standard BLE Cycling Power
 *    Service for apps (Zwift, TrainerRoad, Peloton, etc.)
 *
 * 2. HOME ASSISTANT: Publishes power, cadence, battery, and energy
 *    data to an MQTT broker with HA auto-discovery. Sensors appear
 *    automatically in Home Assistant — no YAML needed.
 *
 * 3. DEBUG WEB UI: Serves a simple status page on your LAN showing
 *    live sensor data, connection states, and device info.
 *
 * First-time setup:
 *   1. Power on the ESP32
 *   2. Connect to the "MYX-Bridge-Setup" WiFi network
 *   3. A captive portal opens — pick your WiFi network and enter
 *      your MQTT broker details (IP, port, username, password)
 *   4. The device saves credentials and connects automatically
 *
 * Sensor details (from LightBlue log analysis):
 *   Name pattern:  BKSNSR*
 *   Service:       0x1818 (Cycling Power)
 *   Measurement:   0x2A63 (Notify) - XOR 0xAA masked
 *   Feature:       0x2A65 (Read)   - value 0x80000000
 *   Sensor Loc:    0x2A5D (Read)   - value 0x04 (Front Wheel)
 *   Manufacturer:  MYX_Power_Meter
 *   Battery:       0x180F / 0x2A19 (Read+Notify) — uint8, 0-100%
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

// ── BLE ──
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ── WiFi & Web & OTA ──
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Update.h>

// ── BT memory release ──
#include "esp_bt.h"

// ── MQTT ──
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── Persistent storage ──
#include <Preferences.h>

// ─── Version ────────────────────────────────────────────────────────────────

static const char* FW_VERSION = "2.0.1";

// ─── Configuration ──────────────────────────────────────────────────────────

// BLE sensor name filter (prefix match). Empty string = any CPS device.
static const char* TARGET_SENSOR_NAME = "BKSNSR";

// XOR key the sensor uses to mask data
static const uint8_t XOR_KEY = 0xAA;

// BLE advertised name for apps (Zwift, etc.)
static const char* BRIDGE_NAME = "MYX Bridge";

// WiFi captive portal AP name
static const char* AP_NAME = "MYX-Bridge-Setup";

// MQTT defaults
static const uint16_t MQTT_DEFAULT_PORT = 1883;
static const char* MQTT_DISCOVERY_PREFIX = "homeassistant";

// How often to publish sensor data to MQTT (ms)
static const unsigned long MQTT_PUBLISH_INTERVAL = 2000;

// Web server port
static const uint16_t WEB_PORT = 80;

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
static BLEUUID CSC_SERVICE_UUID((uint16_t)0x1816);
static BLEUUID CSC_MEASUREMENT_UUID((uint16_t)0x2A5B);   // Notify
static BLEUUID CSC_FEATURE_UUID((uint16_t)0x2A5C);       // Read

// ─── MQTT Configuration (saved to flash) ────────────────────────────────────

struct MqttConfig {
    char server[64];
    uint16_t port;
    char user[32];
    char pass[64];
};

static MqttConfig mqttConfig = {"", MQTT_DEFAULT_PORT, "", ""};
static Preferences preferences;

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

// Latest decoded sensor data
static volatile uint8_t  lastBattery = 0xFF;  // 0xFF = unknown
static volatile int16_t  lastPower = 0;
static volatile uint16_t lastCrankRev = 0;
static volatile uint16_t lastCrankTime = 0;
static volatile uint16_t lastEnergy = 0;

// For cadence calculation
static uint16_t prevCrankRev = 0;
static uint16_t prevCrankTime = 0;
static unsigned long lastCrankEventMs = 0;  // millis() of last actual crank movement
static float    lastCadence = 0.0f;

// WiFi & MQTT
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static bool mqttConfigured = false;
static bool mqttConnected = false;
static unsigned long lastMqttPublish = 0;
static unsigned long lastMqttReconnectAttempt = 0;
static char deviceId[20];       // "myx_bridge_aabbcc"
static char mqttStateTopic[48]; // "myx_bridge_aabbcc/state"
static char mqttAvailTopic[48]; // "myx_bridge_aabbcc/availability"

// Web server
static WebServer webServer(WEB_PORT);

// LED feedback
static const int LED_PIN = 2;  // Built-in LED on most ESP32 boards
static unsigned long lastBlinkTime = 0;
static bool ledState = false;

// ─── MQTT Config Persistence ────────────────────────────────────────────────

void loadMqttConfig() {
    preferences.begin("mqtt", true);  // read-only
    strlcpy(mqttConfig.server, preferences.getString("server", "").c_str(), sizeof(mqttConfig.server));
    mqttConfig.port = preferences.getUShort("port", MQTT_DEFAULT_PORT);
    strlcpy(mqttConfig.user, preferences.getString("user", "").c_str(), sizeof(mqttConfig.user));
    strlcpy(mqttConfig.pass, preferences.getString("pass", "").c_str(), sizeof(mqttConfig.pass));
    preferences.end();

    mqttConfigured = strlen(mqttConfig.server) > 0;
    Serial.printf("[MQTT] Config loaded — server: '%s', port: %d, user: '%s', configured: %s\n",
                  mqttConfig.server, mqttConfig.port, mqttConfig.user,
                  mqttConfigured ? "YES" : "NO");
}

void saveMqttConfig() {
    preferences.begin("mqtt", false);  // read-write
    preferences.putString("server", mqttConfig.server);
    preferences.putUShort("port", mqttConfig.port);
    preferences.putString("user", mqttConfig.user);
    preferences.putString("pass", mqttConfig.pass);
    preferences.end();
    Serial.println("[MQTT] Config saved to flash");
}

// ─── Device ID (based on ESP32 MAC) ────────────────────────────────────────

void buildDeviceId() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(deviceId, sizeof(deviceId), "myx_bridge_%02x%02x%02x", mac[3], mac[4], mac[5]);
    snprintf(mqttStateTopic, sizeof(mqttStateTopic), "%s/state", deviceId);
    snprintf(mqttAvailTopic, sizeof(mqttAvailTopic), "%s/availability", deviceId);
    Serial.printf("[MQTT] Device ID: %s\n", deviceId);
}

// ─── Utility Functions ──────────────────────────────────────────────────────

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
        BLEDevice::startAdvertising();
    }

    void onDisconnect(BLEServer* pServer) override {
        appConnectionCount--;
        if (appConnectionCount < 0) appConnectionCount = 0;
        Serial.printf("[SERVER] App disconnected. (%d remaining)\n", appConnectionCount);
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
            uint8_t response[3] = {0x20, opCode, 0x02};
            pCharacteristic->setValue(response, 3);
            pCharacteristic->indicate();
            Serial.printf("[SERVER] Control Point op 0x%02X -> not supported\n", opCode);
        }
    }
};

// ─── BLE Notification Callbacks ─────────────────────────────────────────────

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

    // Make a working copy and XOR decode
    uint8_t decoded[20];
    size_t copyLen = (length > sizeof(decoded)) ? sizeof(decoded) : length;
    memcpy(decoded, pData, copyLen);
    xorDecode(decoded, copyLen, XOR_KEY);

    // Parse fields (little-endian)
    uint16_t flags  = (uint16_t)(decoded[0] | (decoded[1] << 8));
    int16_t  power  = (int16_t)(decoded[2] | (decoded[3] << 8));
    uint16_t cRev   = (uint16_t)(decoded[4] | (decoded[5] << 8));
    uint16_t cTime  = (uint16_t)(decoded[6] | (decoded[7] << 8));
    uint16_t energy = (uint16_t)(decoded[8] | (decoded[9] << 8));

    // Clamp negative power to zero
    if (power < 0) power = 0;

    // Store for display / MQTT
    lastPower    = power;
    lastCrankRev = cRev;
    lastCrankTime = cTime;
    lastEnergy   = energy;

    // Calculate cadence — only update on actual crank movement,
    // and only zero out after 3 seconds of no new crank events
    if (cRev != prevCrankRev) {
        float cadence = calcCadence(cRev, cTime, prevCrankRev, prevCrankTime);
        if (cadence > 0.0f && cadence < 200.0f) {
            lastCadence = cadence;
        }
        lastCrankEventMs = millis();
        prevCrankRev  = cRev;
        prevCrankTime = cTime;
    } else if (lastCadence > 0.0f && (millis() - lastCrankEventMs > 3000)) {
        lastCadence = 0.0f;  // No movement for 3 s
    }

    // ── Re-broadcast as standard Cycling Power Measurement ──
    // Flags 0x0020 = bit 5 (Crank Revolution Data Present)
    uint8_t outBuf[8];
    uint16_t outFlags = 0x0020;
    outBuf[0] = outFlags & 0xFF;
    outBuf[1] = (outFlags >> 8) & 0xFF;
    outBuf[2] = (uint8_t)(power & 0xFF);
    outBuf[3] = (uint8_t)((power >> 8) & 0xFF);
    outBuf[4] = cRev & 0xFF;
    outBuf[5] = (cRev >> 8) & 0xFF;
    outBuf[6] = cTime & 0xFF;
    outBuf[7] = (cTime >> 8) & 0xFF;

    if (powerMeasurementChar != nullptr) {
        powerMeasurementChar->setValue(outBuf, sizeof(outBuf));
        powerMeasurementChar->notify();
    }

    // ── Also send CSC Measurement for cadence ──
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
        Serial.printf("[DATA] Power: %4d W | Cadence: %5.1f RPM | CrankRev: %5u | Energy: %5u kJ\n",
                      power, lastCadence, cRev, energy);
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
        doScan = true;
    }
};

// ─── BLE Scan Callbacks (finding the sensor) ────────────────────────────────

class SensorScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(CPS_SERVICE_UUID)) {

            String devName = advertisedDevice.getName().c_str();
            if (strlen(TARGET_SENSOR_NAME) > 0 && !devName.startsWith(TARGET_SENSOR_NAME)) {
                Serial.printf("[SCAN] Found CPS device '%s' but name doesn't match '%s'\n",
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

    // Get Power Measurement characteristic
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
                Serial.println("[SENSOR] Subscribed to battery notifications!");
            }
        } else {
            Serial.println("[SENSOR] Battery Level characteristic (0x2A19) not found");
        }
    } else {
        Serial.println("[SENSOR] Battery Service (0x180F) not found");
    }

    sensorConnected = true;
    return true;
}

// ─── Setup BLE Server (what apps see) ───────────────────────────────────────

void setupBLEServer() {
    Serial.println("[SERVER] Setting up BLE Cycling Power Service...");

    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new BridgeServerCallbacks());

    // ── Cycling Power Service (0x1818) ──
    BLEService* cpsService = bleServer->createService(CPS_SERVICE_UUID, 20);

    powerMeasurementChar = cpsService->createCharacteristic(
        CPS_MEASUREMENT_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    powerMeasurementChar->addDescriptor(new BLE2902());

    powerFeatureChar = cpsService->createCharacteristic(
        CPS_FEATURE_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t featureValue[4] = {0x08, 0x00, 0x00, 0x00};  // Crank Rev Data Supported
    powerFeatureChar->setValue(featureValue, 4);

    sensorLocationChar = cpsService->createCharacteristic(
        SENSOR_LOCATION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t sensorLoc = 0x05;  // Left Crank
    sensorLocationChar->setValue(&sensorLoc, 1);

    powerControlPointChar = cpsService->createCharacteristic(
        CPS_CONTROL_POINT_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE
    );
    powerControlPointChar->addDescriptor(new BLE2902());
    powerControlPointChar->setCallbacks(new ControlPointCallbacks());

    cpsService->start();

    // ── Cycling Speed & Cadence Service (0x1816) ──
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
    uint8_t cscFeature[2] = {0x02, 0x00};  // Crank Revolution Data Supported
    cscFeatureChar->setValue(cscFeature, 2);

    BLECharacteristic* cscSensorLoc = cscService->createCharacteristic(
        SENSOR_LOCATION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    uint8_t cscLoc = 0x05;  // Left Crank
    cscSensorLoc->setValue(&cscLoc, 1);

    cscService->start();

    // ── Device Information Service (0x180A) ──
    BLEService* disService = bleServer->createService(BLEUUID((uint16_t)0x180A));
    BLECharacteristic* mfgNameChar = disService->createCharacteristic(
        BLEUUID((uint16_t)0x2A29), BLECharacteristic::PROPERTY_READ);
    mfgNameChar->setValue("ESP32 Bike Bridge");
    BLECharacteristic* modelChar = disService->createCharacteristic(
        BLEUUID((uint16_t)0x2A24), BLECharacteristic::PROPERTY_READ);
    modelChar->setValue("BikePower Bridge v2");
    BLECharacteristic* fwRevChar = disService->createCharacteristic(
        BLEUUID((uint16_t)0x2A26), BLECharacteristic::PROPERTY_READ);
    fwRevChar->setValue(FW_VERSION);
    disService->start();

    // ── Battery Service (0x180F) ──
    BLEService* battService = bleServer->createService(BATTERY_SERVICE_UUID, 8);
    batteryLevelChar = battService->createCharacteristic(
        BATTERY_LEVEL_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    batteryLevelChar->addDescriptor(new BLE2902());
    uint8_t initBatt = 0;
    batteryLevelChar->setValue(&initBatt, 1);
    battService->start();

    // ── Advertising ──
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(CPS_SERVICE_UUID);
    pAdvertising->addServiceUUID(CSC_SERVICE_UUID);
    pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
    pAdvertising->setAppearance(0x0484);   // Cycling Power Sensor
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);   // 7.5 ms
    pAdvertising->setMaxPreferred(0x12);   // 22.5 ms
    BLEDevice::startAdvertising();

    Serial.printf("[SERVER] Advertising as '%s'\n", BRIDGE_NAME);
}

// ─── MQTT: Publish HA Auto-Discovery ────────────────────────────────────────

void publishDiscovery() {
    // Sensor entities: power, cadence, battery, energy
    struct {
        const char* name;
        const char* uid_suffix;
        const char* value_tpl;
        const char* unit;
        const char* dev_class;   // nullptr if none
        const char* state_class;
        const char* icon;        // nullptr to use default
    } sensors[] = {
        {"Power",   "power",   "{{ value_json.power }}",   "W",   "power",   "measurement",      nullptr},
        {"Cadence", "cadence", "{{ value_json.cadence }}", "RPM", nullptr,   "measurement",      "mdi:bicycle"},
        {"Battery", "battery", "{{ value_json.battery }}", "%",   "battery", "measurement",      nullptr},
        {"Energy",  "energy",  "{{ value_json.energy }}",  "kJ",  "energy",  "total_increasing", nullptr},
    };

    for (auto& s : sensors) {
        JsonDocument doc;

        char uid[48];
        snprintf(uid, sizeof(uid), "%s_%s", deviceId, s.uid_suffix);
        char name[48];
        snprintf(name, sizeof(name), "MYX Bridge %s", s.name);

        doc["name"] = name;
        doc["unique_id"] = uid;
        doc["state_topic"] = mqttStateTopic;
        doc["value_template"] = s.value_tpl;
        doc["unit_of_measurement"] = s.unit;
        if (s.dev_class)  doc["device_class"] = s.dev_class;
        if (s.state_class) doc["state_class"] = s.state_class;
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
        mqttClient.publish(topic, payload, true);  // retained
        Serial.printf("[MQTT] Discovery: %s\n", topic);
    }

    // Binary sensor for BLE connection status
    {
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
        Serial.printf("[MQTT] Discovery: %s\n", topic);
    }
}

void publishSensorData() {
    if (!mqttClient.connected()) return;

    JsonDocument doc;
    doc["power"] = (int)lastPower;
    doc["cadence"] = round(lastCadence * 10.0f) / 10.0f;
    doc["battery"] = (lastBattery == 0xFF) ? 0 : (int)lastBattery;
    doc["energy"] = (int)lastEnergy;
    doc["ble_connected"] = sensorConnected ? "ON" : "OFF";

    char payload[256];
    serializeJson(doc, payload, sizeof(payload));
    mqttClient.publish(mqttStateTopic, payload);
}

// ─── MQTT: Connect ──────────────────────────────────────────────────────────

bool mqttConnect() {
    if (!mqttConfigured) return false;
    if (WiFi.status() != WL_CONNECTED) return false;

    Serial.printf("[MQTT] Connecting to %s:%d ...\n", mqttConfig.server, mqttConfig.port);

    mqttClient.setServer(mqttConfig.server, mqttConfig.port);

    char clientId[32];
    snprintf(clientId, sizeof(clientId), "%s_%04X", deviceId, (uint16_t)random(0xFFFF));

    bool connected;
    if (strlen(mqttConfig.user) > 0) {
        connected = mqttClient.connect(clientId, mqttConfig.user, mqttConfig.pass,
                                       mqttAvailTopic, 1, true, "offline");
    } else {
        connected = mqttClient.connect(clientId, nullptr, nullptr,
                                       mqttAvailTopic, 1, true, "offline");
    }

    if (connected) {
        Serial.println("[MQTT] Connected!");
        mqttClient.publish(mqttAvailTopic, "online", true);
        publishDiscovery();
        publishSensorData();
        mqttConnected = true;
        return true;
    } else {
        Serial.printf("[MQTT] Failed, rc=%d\n", mqttClient.state());
        mqttConnected = false;
        return false;
    }
}

// ─── WiFi Setup (WiFiManager captive portal) ───────────────────────────────

static bool shouldSaveMqttConfig = false;

void saveConfigCallback() {
    shouldSaveMqttConfig = true;
}

void setupWiFi() {
    WiFiManager wm;

    // Custom MQTT parameters shown in the captive portal
    WiFiManagerParameter mqtt_header("<br><h3>MQTT Settings (for Home Assistant)</h3>");
    WiFiManagerParameter mqtt_server("mqtt_server", "MQTT Server", mqttConfig.server, 63);
    char portStr[6];
    snprintf(portStr, sizeof(portStr), "%d", mqttConfig.port);
    WiFiManagerParameter mqtt_port("mqtt_port", "MQTT Port", portStr, 5);
    WiFiManagerParameter mqtt_user("mqtt_user", "MQTT Username", mqttConfig.user, 31);
    WiFiManagerParameter mqtt_pass("mqtt_pass", "MQTT Password", mqttConfig.pass, 63);

    wm.addParameter(&mqtt_header);
    wm.addParameter(&mqtt_server);
    wm.addParameter(&mqtt_port);
    wm.addParameter(&mqtt_user);
    wm.addParameter(&mqtt_pass);

    wm.setSaveConfigCallback(saveConfigCallback);
    wm.setConfigPortalTimeout(300);  // 5 min portal timeout, then retry
    wm.setConnectTimeout(15);

    Serial.printf("[WIFI] Starting WiFiManager (AP: %s) ...\n", AP_NAME);

    if (!wm.autoConnect(AP_NAME)) {
        Serial.println("[WIFI] Failed to connect. Restarting...");
        delay(3000);
        ESP.restart();
    }

    Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

    // Save MQTT config if it was entered/changed in the portal
    if (shouldSaveMqttConfig) {
        strlcpy(mqttConfig.server, mqtt_server.getValue(), sizeof(mqttConfig.server));
        mqttConfig.port = atoi(mqtt_port.getValue());
        if (mqttConfig.port == 0) mqttConfig.port = MQTT_DEFAULT_PORT;
        strlcpy(mqttConfig.user, mqtt_user.getValue(), sizeof(mqttConfig.user));
        strlcpy(mqttConfig.pass, mqtt_pass.getValue(), sizeof(mqttConfig.pass));
        saveMqttConfig();
        mqttConfigured = strlen(mqttConfig.server) > 0;
    }
}

// ─── Web Server Handlers ────────────────────────────────────────────────────

void handleWebRoot() {
    // Snapshot values for the initial paint so the page is useful before
    // the first JS fetch fires.
    unsigned long uptime = millis() / 1000;
    unsigned long days  = uptime / 86400;
    unsigned long hours = (uptime % 86400) / 3600;
    unsigned long mins  = (uptime % 3600) / 60;
    unsigned long secs  = uptime % 60;

    char battStr[8];
    if (lastBattery == 0xFF) snprintf(battStr, sizeof(battStr), "?");
    else snprintf(battStr, sizeof(battStr), "%d%%", lastBattery);

    char uptimeStr[32];
    if (days > 0) snprintf(uptimeStr, sizeof(uptimeStr), "%lud %02lu:%02lu:%02lu", days, hours, mins, secs);
    else snprintf(uptimeStr, sizeof(uptimeStr), "%02lu:%02lu:%02lu", hours, mins, secs);

    // Stream response in fixed-size chunks — no large heap allocations
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html", "");

    // ── Head + CSS ──────────────────────────────────────────────────────────
    webServer.sendContent(
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>MYX Bridge</title>"
        "<style>"
        "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;"
        "max-width:600px;margin:0 auto;padding:20px;background:#1a1a2e;color:#e0e0e0}"
        "h1{color:#00d4ff;text-align:center;margin-bottom:5px}"
        ".subtitle{text-align:center;color:#888;margin-bottom:20px}"
        ".card{background:#16213e;border-radius:12px;padding:16px;margin:12px 0;"
        "border:1px solid #0f3460}"
        ".card h2{margin:0 0 12px;font-size:1.1em;color:#00d4ff}"
        ".row{display:flex;justify-content:space-between;padding:6px 0;"
        "border-bottom:1px solid #0f3460}"
        ".row:last-child{border-bottom:none}"
        ".label{color:#888}"
        ".value{font-weight:600;color:#fff}"
        ".big{font-size:2.2em;text-align:center;color:#00ff88;margin:8px 0}"
        ".online{color:#00ff88}"
        ".offline{color:#ff4444}"
        ".btn{display:block;width:100%;padding:12px;margin:16px 0 0;background:#533483;"
        "color:#fff;border:none;border-radius:8px;font-size:1em;cursor:pointer;"
        "text-align:center;text-decoration:none;box-sizing:border-box}"
        ".btn:hover{background:#6c44a0}"
        ".btn-danger{background:#c0392b}"
        ".btn-danger:hover{background:#e74c3c}"
        "</style></head><body>"
    );

    // ── Header ───────────────────────────────────────────────────────────────
    char buf[256];
    snprintf(buf, sizeof(buf),
        "<h1>MYX Bridge</h1>"
        "<div class='subtitle'>v%s &bull; Live data</div>",
        FW_VERSION);
    webServer.sendContent(buf);

    // ── Power card (initial server-side value, updated by JS) ────────────────
    snprintf(buf, sizeof(buf),
        "<div class='card'><h2>Power</h2>"
        "<div class='big' id='val-power'>%d W</div>"
        "</div>",
        (int)lastPower);
    webServer.sendContent(buf);

    // ── Sensor Data card (one row per sendContent to stay within buf[256]) ──
    webServer.sendContent("<div class='card'><h2>Sensor Data</h2>");

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>Cadence</span>"
        "<span class='value' id='val-cadence'>%.1f RPM</span></div>",
        lastCadence);
    webServer.sendContent(buf);

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>Battery</span>"
        "<span class='value' id='val-battery'>%s</span></div>",
        battStr);
    webServer.sendContent(buf);

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>Energy</span>"
        "<span class='value' id='val-energy'>%d kJ</span></div>",
        (int)lastEnergy);
    webServer.sendContent(buf);

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>Crank Revs</span>"
        "<span class='value' id='val-cranks'>%d</span></div>",
        (int)lastCrankRev);
    webServer.sendContent(buf);

    webServer.sendContent("</div>");  // end Sensor Data card

    // ── Connections card (split into small chunks to stay within buf[256]) ──
    webServer.sendContent("<div class='card'><h2>Connections</h2>");

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>BLE Sensor</span>"
        "<span class='value %s' id='val-ble-sensor'>%s</span></div>",
        sensorConnected ? "online" : "offline",
        sensorConnected ? "Connected" : "Searching");
    webServer.sendContent(buf);

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>BLE Apps</span>"
        "<span class='value' id='val-ble-apps'>%d</span></div>",
        appConnectionCount);
    webServer.sendContent(buf);

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>WiFi</span>"
        "<span class='value online' id='val-wifi'>%s</span></div>",
        WiFi.localIP().toString().c_str());
    webServer.sendContent(buf);

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>MQTT</span>"
        "<span class='value %s' id='val-mqtt'>%s</span></div>",
        mqttConnected ? "online" : "offline",
        mqttConnected ? "Connected" : "Disconnected");
    webServer.sendContent(buf);

    if (mqttConfigured) {
        snprintf(buf, sizeof(buf),
            "<div class='row'><span class='label'>MQTT Broker</span>"
            "<span class='value'>%s:%d</span></div>",
            mqttConfig.server, mqttConfig.port);
        webServer.sendContent(buf);
    }
    webServer.sendContent("</div>");  // end Connections card

    // ── Device card (one row per sendContent) ────────────────────────────────
    webServer.sendContent("<div class='card'><h2>Device</h2>");

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>Uptime</span>"
        "<span class='value' id='val-uptime'>%s</span></div>",
        uptimeStr);
    webServer.sendContent(buf);

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>Device ID</span>"
        "<span class='value'>%s</span></div>",
        deviceId);
    webServer.sendContent(buf);

    snprintf(buf, sizeof(buf),
        "<div class='row'><span class='label'>Free Heap</span>"
        "<span class='value' id='val-heap'>%d KB</span></div>",
        (int)(ESP.getFreeHeap() / 1024));
    webServer.sendContent(buf);

    webServer.sendContent("</div>");  // end Device card

    // ── Buttons ──────────────────────────────────────────────────────────────
    webServer.sendContent(
        "<a href='/update' class='btn'>Firmware Update (OTA)</a>"
        "<a href='/reset' class='btn btn-danger'"
        " onclick=\"return confirm('Reset WiFi and MQTT settings? Device will restart in AP mode.')\">"
        "Reset WiFi &amp; MQTT Settings</a>"
    );

    // ── JS live-update (no full page reload) ─────────────────────────────────
    // fetch /api/status every 5 s and patch only the changed DOM nodes
    webServer.sendContent(
        "<script>"
        "function fmtUptime(s){"
        "var d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),"
        "m=Math.floor((s%3600)/60),sc=s%60;"
        "var p=function(n){return n<10?'0'+n:n;};"
        "return d>0?d+'d '+p(h)+':'+p(m)+':'+p(sc):p(h)+':'+p(m)+':'+p(sc);}"
        "function upd(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}"
        "function setClass(id,cls){"
        "var e=document.getElementById(id);"
        "if(e){e.className='value '+cls;}}"
        "function refresh(){"
        "fetch('/api/status').then(function(r){return r.json();})"
        ".then(function(d){"
        "upd('val-power',d.power+' W');"
        "upd('val-cadence',d.cadence.toFixed(1)+' RPM');"
        "upd('val-battery',d.battery<0?'?':d.battery+'%');"
        "upd('val-energy',d.energy+' kJ');"
        "upd('val-cranks',d.crank_revs);"
        "var sc=d.ble_sensor?'online':'offline';"
        "upd('val-ble-sensor',d.ble_sensor?'Connected':'Searching');"
        "setClass('val-ble-sensor',sc);"
        "upd('val-ble-apps',d.ble_apps);"
        "var mc=d.mqtt_connected?'online':'offline';"
        "upd('val-mqtt',d.mqtt_connected?'Connected':'Disconnected');"
        "setClass('val-mqtt',mc);"
        "upd('val-uptime',fmtUptime(d.uptime_s));"
        "upd('val-heap',Math.round(d.free_heap/1024)+' KB');"
        "}).catch(function(){});}"
        "setInterval(refresh,5000);"
        "</script>"
    );

    webServer.sendContent("</body></html>");
    webServer.sendContent("");  // signal end of chunked response
}

void handleWebJson() {
    JsonDocument doc;
    doc["power"] = (int)lastPower;
    doc["cadence"] = round(lastCadence * 10.0f) / 10.0f;
    doc["battery"] = (lastBattery == 0xFF) ? -1 : (int)lastBattery;
    doc["energy"] = (int)lastEnergy;
    doc["crank_revs"] = (int)lastCrankRev;
    doc["ble_sensor"] = sensorConnected;
    doc["ble_apps"] = appConnectionCount;
    doc["mqtt_connected"] = mqttConnected;
    doc["uptime_s"] = millis() / 1000;
    doc["free_heap"] = ESP.getFreeHeap();

    char buf[256];
    serializeJson(doc, buf, sizeof(buf));
    webServer.send(200, "application/json", buf);
}

void handleWebReset() {
    String resetHtml = "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='5;url=/'>"
        "<title>Resetting...</title></head>"
        "<body style='font-family:sans-serif;text-align:center;padding:50px;background:#1a1a2e;color:#e0e0e0'>"
        "<h2>Settings Cleared</h2>"
        "<p>Device will restart in AP mode.<br>"
        "Connect to <b>" + String(AP_NAME) + "</b> to reconfigure.</p></body></html>";
    webServer.send(200, "text/html", resetHtml);

    delay(1000);

    // Clear WiFi credentials
    WiFiManager wm;
    wm.resetSettings();

    // Clear MQTT config
    preferences.begin("mqtt", false);
    preferences.clear();
    preferences.end();

    delay(1000);
    ESP.restart();
}

// ─── OTA Firmware Update Handlers ───────────────────────────────────────────

void handleOtaPage() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html", "");

    // Send head and CSS as plain string literals — no snprintf, no buffer overflow risk
    webServer.sendContent("<!DOCTYPE html><html><head>");
    webServer.sendContent("<meta charset='UTF-8'>");
    webServer.sendContent("<meta name='viewport' content='width=device-width,initial-scale=1'>");
    webServer.sendContent("<title>MYX Bridge - Firmware Update</title>");
    webServer.sendContent(
        "<style>"
        "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;"
        "max-width:600px;margin:0 auto;padding:20px;background:#1a1a2e;color:#e0e0e0}"
        "h1{color:#00d4ff;text-align:center}"
        ".card{background:#16213e;border-radius:12px;padding:20px;margin:16px 0;"
        "border:1px solid #0f3460;text-align:center}"
        ".card p{color:#888;margin:8px 0 16px}"
        "input[type=file]{color:#e0e0e0;margin:12px 0;display:block;width:100%}"
        ".btn{display:inline-block;padding:12px 32px;margin:12px 4px 0;background:#533483;"
        "color:#fff;border:none;border-radius:8px;font-size:1em;cursor:pointer;text-decoration:none}"
        ".btn:hover{background:#6c44a0}"
        ".btn-upload{background:#0f3460}.btn-upload:hover{background:#1a4a7a}"
        "progress{width:100%;margin:12px 0;display:none}"
        "#status{margin:12px 0;font-weight:600}"
        ".warn{color:#f39c12;font-size:0.9em}"
        "</style></head><body>"
    );

    webServer.sendContent("<h1>Firmware Update</h1><div class='card'>");

    // Only version string needs snprintf
    char vbuf[64];
    snprintf(vbuf, sizeof(vbuf), "<p>Current version: <b>%s</b></p>", FW_VERSION);
    webServer.sendContent(vbuf);

    webServer.sendContent(
        "<form id='uf'>"
        "<input type='file' id='fbin' name='firmware' accept='.bin' required>"
        "<br>"
        "<button type='submit' class='btn btn-upload' id='btn'>Upload &amp; Flash</button>"
        "</form>"
        "<div id='bar-wrap' style='display:none;background:#0f3460;border-radius:8px;margin:16px 0;height:24px;overflow:hidden'>"
        "<div id='bar' style='height:100%;width:0;background:#00d4ff;transition:width 0.3s'></div>"
        "</div>"
        "<div id='pct' style='display:none;color:#888;font-size:0.9em;margin:-8px 0 8px'></div>"
        "<div id='status' style='font-size:1.1em;font-weight:600;min-height:1.5em'></div>"
        "<p class='warn'>&#9888; Do not power off during update!</p>"
        "</div>"
    );

    webServer.sendContent("<a href='/' class='btn' id='back'>&#8592; Back</a>");

    webServer.sendContent(
        "<script>"
        "function setStatus(msg,col){"
        "var s=document.getElementById('status');"
        "s.textContent=msg;s.style.color=col||'#e0e0e0';}"
        "function setBar(pct){"
        "document.getElementById('bar').style.width=pct+'%';"
        "document.getElementById('pct').textContent=pct+'%';}"
        "document.getElementById('uf').addEventListener('submit',function(e){"
        "e.preventDefault();"
        "var file=document.getElementById('fbin').files[0];"
        "if(!file){setStatus('Please select a .bin file.','#ff4444');return;}"
        "document.getElementById('btn').disabled=true;"
        "document.getElementById('back').style.pointerEvents='none';"
        "document.getElementById('back').style.opacity='0.4';"
        "document.getElementById('bar-wrap').style.display='block';"
        "document.getElementById('pct').style.display='block';"
        "setBar(0);"
        "setStatus('Uploading firmware...','#00d4ff');"
        "var fd=new FormData();"
        "fd.append('firmware',file);"
        "var xhr=new XMLHttpRequest();"
        "xhr.upload.onprogress=function(ev){"
        "if(ev.lengthComputable){"
        "var p=Math.round(ev.loaded/ev.total*100);"
        "setBar(p);"
        "if(p<100){setStatus('Uploading... '+p+'%','#00d4ff');}"
        "else{setStatus('Upload complete. Flashing...','#f39c12');}}};"
        "xhr.onload=function(){"
        "if(xhr.status===200){"
        "setBar(100);"
        "setStatus('&#10003; Flash complete! Rebooting...','#00ff88');"
        "var c=12;"
        "var t=setInterval(function(){"
        "c--;setStatus('&#10003; Flash complete! Redirecting in '+c+'s...','#00ff88');"
        "if(c<=0){clearInterval(t);location.href='/';}},1000);"
        "}else{"
        "setStatus('Flash failed: '+xhr.responseText,'#ff4444');"
        "document.getElementById('btn').disabled=false;}};"
        "xhr.onerror=function(){"
        "setStatus('Connection lost - device may be rebooting...','#f39c12');"
        "var c=15;"
        "var t=setInterval(function(){"
        "c--;setStatus('Waiting for device... '+c+'s','#f39c12');"
        "if(c<=0){clearInterval(t);location.href='/';}},1000);};"
        "xhr.open('POST','/update');"
        "xhr.send(fd);});"
        "</script>"
        "</body></html>"
    );

    webServer.sendContent("");  // end chunked
}

void handleOtaUpload() {
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
            Serial.printf("[OTA] Success! %u bytes written\n", upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    }
}

void handleOtaResult() {
    if (Update.hasError()) {
        String msg = "Update failed.";
        webServer.send(400, "text/plain", msg);
    } else {
        webServer.send(200, "text/plain", "OK");
        delay(1000);
        ESP.restart();
    }
}

void handleWebNotFound() {
    webServer.send(404, "text/plain", "Not found");
}

void setupWebServer() {
    webServer.on("/", handleWebRoot);
    webServer.on("/api/status", handleWebJson);
    webServer.on("/update", HTTP_GET, handleOtaPage);
    webServer.on("/update", HTTP_POST, handleOtaResult, handleOtaUpload);
    webServer.on("/reset", handleWebReset);
    webServer.onNotFound(handleWebNotFound);
    webServer.begin();
    Serial.printf("[WEB] Server started on http://%s:%d\n",
                  WiFi.localIP().toString().c_str(), WEB_PORT);
}

// ─── Arduino Setup ──────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=============================================");
    Serial.println("  MYX Bridge - BLE Power Meter + Home Assistant");
    Serial.println("=============================================");
    Serial.printf("  Firmware: %s\n", FW_VERSION);
    Serial.println();

    // LED setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Build unique device ID from ESP32 MAC
    buildDeviceId();

    // Load saved MQTT config from flash
    loadMqttConfig();

    // Initialize BLE (before WiFi for proper coexistence)
    // Release Classic BT (BR/EDR) memory — we only need BLE.
    // This frees ~70KB of heap that bluedroid holds for classic BT.
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    BLEDevice::init(BRIDGE_NAME);

    // Set up the BLE server side first (so apps can find us immediately)
    setupBLEServer();

    // Connect to WiFi (captive portal on first boot or if saved network is gone)
    setupWiFi();

    // Setup MQTT connection
    if (mqttConfigured) {
        mqttClient.setServer(mqttConfig.server, mqttConfig.port);
        mqttClient.setBufferSize(1024);  // Needed for HA discovery payloads
        mqttConnect();
    } else {
        Serial.println("[MQTT] Not configured. Enter MQTT details via the setup portal.");
    }

    // Start debug web server
    setupWebServer();

    // Start BLE scan for the sensor
    Serial.println("[SCAN] Starting BLE scan for power meter...");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SensorScanCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(10, false);

    Serial.println("[OK] Setup complete!");
}

// ─── Arduino Loop ───────────────────────────────────────────────────────────

void loop() {
    unsigned long now = millis();

    // ── BLE: handle sensor connection ──
    if (doConnect) {
        if (connectToSensor()) {
            Serial.println("[OK] Bridge active — power data flowing!");
        } else {
            Serial.println("[ERROR] Sensor connection failed. Retrying...");
            doScan = true;
        }
        doConnect = false;
    }

    if (!sensorConnected && doScan) {
        Serial.println("[SCAN] Re-scanning for power meter...");
        BLEDevice::getScan()->start(10, false);
        doScan = false;
    }

    // ── MQTT: maintain connection and publish data ──
    if (mqttConfigured && WiFi.status() == WL_CONNECTED) {
        if (!mqttClient.connected()) {
            mqttConnected = false;
            // Retry every 5 seconds
            if (now - lastMqttReconnectAttempt > 5000) {
                lastMqttReconnectAttempt = now;
                mqttConnect();
            }
        } else {
            mqttClient.loop();
            // Publish sensor data periodically
            if (now - lastMqttPublish > MQTT_PUBLISH_INTERVAL) {
                lastMqttPublish = now;
                publishSensorData();
            }
        }
    }

    // ── Web server ──
    webServer.handleClient();

    // ── LED status feedback ──
    if (sensorConnected && appConnectionCount > 0) {
        // Solid LED = full bridge active (sensor + app)
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
        Serial.printf("[STATUS] Sensor: %s | Apps: %d | Power: %d W | Cadence: %.0f RPM | Battery: %s | MQTT: %s\n",
                      sensorConnected ? "CONNECTED" : "SEARCHING",
                      appConnectionCount,
                      (int)lastPower,
                      lastCadence,
                      battStr,
                      mqttConnected ? "CONNECTED" : "DISCONNECTED");
    }

    delay(10);
}
