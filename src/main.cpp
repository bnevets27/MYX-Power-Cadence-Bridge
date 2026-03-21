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

// ── MQTT ──
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── Persistent storage ──
#include <Preferences.h>

// ─── Version ────────────────────────────────────────────────────────────────

static const char* FW_VERSION = "2.0.0";

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

    // Calculate cadence
    float cadence = calcCadence(cRev, cTime, prevCrankRev, prevCrankTime);
    if (cadence > 0.0f && cadence < 200.0f) {
        lastCadence = cadence;
    } else if (cRev != prevCrankRev) {
        // Crank moved but cadence out of range — keep last known
    } else {
        lastCadence = 0.0f;  // No movement
    }
    prevCrankRev  = cRev;
    prevCrankTime = cTime;

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
    unsigned long uptime = millis() / 1000;
    unsigned long days  = uptime / 86400;
    unsigned long hours = (uptime % 86400) / 3600;
    unsigned long mins  = (uptime % 3600) / 60;
    unsigned long secs  = uptime % 60;

    char battStr[8];
    if (lastBattery == 0xFF) snprintf(battStr, sizeof(battStr), "?");
    else snprintf(battStr, sizeof(battStr), "%d%%", lastBattery);

    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<meta http-equiv='refresh' content='3'>";
    html += "<title>MYX Bridge</title>";
    html += "<style>";
    html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;";
    html += "max-width:600px;margin:0 auto;padding:20px;background:#1a1a2e;color:#e0e0e0}";
    html += "h1{color:#00d4ff;text-align:center;margin-bottom:5px}";
    html += ".subtitle{text-align:center;color:#888;margin-bottom:20px}";
    html += ".card{background:#16213e;border-radius:12px;padding:16px;margin:12px 0;";
    html += "border:1px solid #0f3460}";
    html += ".card h2{margin:0 0 12px;font-size:1.1em;color:#00d4ff}";
    html += ".row{display:flex;justify-content:space-between;padding:6px 0;";
    html += "border-bottom:1px solid #0f3460}";
    html += ".row:last-child{border-bottom:none}";
    html += ".label{color:#888}.value{font-weight:600;color:#fff}";
    html += ".big{font-size:2.2em;text-align:center;color:#00ff88;margin:8px 0}";
    html += ".online{color:#00ff88}.offline{color:#ff4444}";
    html += ".btn{display:block;width:100%;padding:12px;margin:16px 0 0;background:#533483;";
    html += "color:#fff;border:none;border-radius:8px;font-size:1em;cursor:pointer;text-align:center;text-decoration:none}";
    html += ".btn:hover{background:#6c44a0}";
    html += ".btn-danger{background:#c0392b}.btn-danger:hover{background:#e74c3c}";
    html += "</style></head><body>";

    html += "<h1>MYX Bridge</h1>";
    html += "<div class='subtitle'>v" + String(FW_VERSION) + " &bull; Refreshes every 3s</div>";

    // Power (big display)
    html += "<div class='card'><h2>Power</h2>";
    html += "<div class='big'>" + String((int)lastPower) + " W</div>";
    html += "</div>";

    // Sensor data
    html += "<div class='card'><h2>Sensor Data</h2>";
    html += "<div class='row'><span class='label'>Cadence</span><span class='value'>" +
            String(lastCadence, 1) + " RPM</span></div>";
    html += "<div class='row'><span class='label'>Battery</span><span class='value'>" +
            String(battStr) + "</span></div>";
    html += "<div class='row'><span class='label'>Energy</span><span class='value'>" +
            String((int)lastEnergy) + " kJ</span></div>";
    html += "<div class='row'><span class='label'>Crank Revs</span><span class='value'>" +
            String((int)lastCrankRev) + "</span></div>";
    html += "</div>";

    // Connections
    html += "<div class='card'><h2>Connections</h2>";
    html += "<div class='row'><span class='label'>BLE Sensor</span><span class='value " +
            String(sensorConnected ? "online'>Connected" : "offline'>Searching") + "</span></div>";
    html += "<div class='row'><span class='label'>BLE Apps</span><span class='value'>" +
            String(appConnectionCount) + "</span></div>";
    html += "<div class='row'><span class='label'>WiFi</span><span class='value online'>" +
            WiFi.localIP().toString() + "</span></div>";
    html += "<div class='row'><span class='label'>MQTT</span><span class='value " +
            String(mqttConnected ? "online'>Connected" : "offline'>Disconnected") + "</span></div>";
    if (mqttConfigured) {
        html += "<div class='row'><span class='label'>MQTT Broker</span><span class='value'>" +
                String(mqttConfig.server) + ":" + String(mqttConfig.port) + "</span></div>";
    }
    html += "</div>";

    // Device info
    html += "<div class='card'><h2>Device</h2>";
    char uptimeStr[32];
    if (days > 0) snprintf(uptimeStr, sizeof(uptimeStr), "%lud %02lu:%02lu:%02lu", days, hours, mins, secs);
    else snprintf(uptimeStr, sizeof(uptimeStr), "%02lu:%02lu:%02lu", hours, mins, secs);
    html += "<div class='row'><span class='label'>Uptime</span><span class='value'>" +
            String(uptimeStr) + "</span></div>";
    html += "<div class='row'><span class='label'>Device ID</span><span class='value'>" +
            String(deviceId) + "</span></div>";
    html += "<div class='row'><span class='label'>Free Heap</span><span class='value'>" +
            String(ESP.getFreeHeap() / 1024) + " KB</span></div>";
    html += "</div>";

    // Actions
    html += "<a href='/update' class='btn'>Firmware Update (OTA)</a>";
    html += "<a href='/reset' class='btn btn-danger' onclick=\"return confirm('Reset WiFi and MQTT settings? Device will restart in AP mode.')\">Reset WiFi &amp; MQTT Settings</a>";

    html += "</body></html>";
    webServer.send(200, "text/html", html);
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
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>MYX Bridge - Firmware Update</title>";
    html += "<style>";
    html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;";
    html += "max-width:600px;margin:0 auto;padding:20px;background:#1a1a2e;color:#e0e0e0}";
    html += "h1{color:#00d4ff;text-align:center}";
    html += ".card{background:#16213e;border-radius:12px;padding:20px;margin:16px 0;";
    html += "border:1px solid #0f3460;text-align:center}";
    html += ".card p{color:#888;margin:8px 0 16px}";
    html += "input[type=file]{color:#e0e0e0;margin:12px 0}";
    html += ".btn{display:inline-block;padding:12px 32px;margin:12px 4px 0;background:#533483;";
    html += "color:#fff;border:none;border-radius:8px;font-size:1em;cursor:pointer;text-decoration:none}";
    html += ".btn:hover{background:#6c44a0}";
    html += ".btn-upload{background:#0f3460}.btn-upload:hover{background:#1a4a7a}";
    html += "#prog{width:100%;margin:12px 0;display:none}";
    html += "#status{margin:12px 0;font-weight:600}";
    html += ".warn{color:#f39c12;font-size:0.9em}";
    html += "</style></head><body>";
    html += "<h1>Firmware Update</h1>";
    html += "<div class='card'>";
    html += "<p>Current version: <b>" + String(FW_VERSION) + "</b></p>";
    html += "<form method='POST' action='/update' enctype='multipart/form-data' id='uf'>";
    html += "<input type='file' name='firmware' accept='.bin' required><br>";
    html += "<input type='submit' value='Upload &amp; Flash' class='btn btn-upload'>";
    html += "</form>";
    html += "<progress id='prog' max='100' value='0'></progress>";
    html += "<div id='status'></div>";
    html += "<p class='warn'>Do not power off during update!</p>";
    html += "</div>";
    html += "<a href='/' class='btn'>Back</a>";
    html += "<script>";
    html += "document.getElementById('uf').addEventListener('submit',function(e){";
    html += "e.preventDefault();var f=new FormData(this);var x=new XMLHttpRequest();";
    html += "var p=document.getElementById('prog');var s=document.getElementById('status');";
    html += "p.style.display='block';s.textContent='Uploading...';";
    html += "x.upload.addEventListener('progress',function(ev){";
    html += "if(ev.lengthComputable)p.value=Math.round(ev.loaded/ev.total*100);});";
    html += "x.onreadystatechange=function(){if(x.readyState==4){";
    html += "if(x.status==200){s.textContent='Success! Rebooting...';s.style.color='#00ff88';";
    html += "setTimeout(function(){location.href='/';},8000);}";
    html += "else{s.textContent='Update failed: '+x.responseText;s.style.color='#ff4444';}}};";
    html += "x.open('POST','/update');x.send(f);});";
    html += "</script>";
    html += "</body></html>";
    webServer.send(200, "text/html", html);
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
