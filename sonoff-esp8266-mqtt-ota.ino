#include "config.h"

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SimpleTimer.h>


// -----------------------------
// Hardware (Sonoff Basic style)
// -----------------------------
namespace Pins {
  constexpr uint8_t LED   = 13; // onboard LED is often active-low
  constexpr uint8_t RELAY = 12; // relay pin
  constexpr uint8_t BTN   = 0;  // GPIO0 button (active-low)
}

constexpr uint32_t SERIAL_BAUD          = 9600;
constexpr uint16_t MQTT_PORT            = 1883;

constexpr uint32_t WIFI_RETRY_MS        = 1000;
constexpr uint8_t  WIFI_MAX_FAILS       = 150;

constexpr uint32_t MQTT_RETRY_MS        = 1000;
constexpr uint8_t  MQTT_MAX_FAILS       = 10;

constexpr uint32_t STATUS_PUBLISH_MS    = 60000;

constexpr uint32_t BTN_DEBOUNCE_MS      = 35;
constexpr uint32_t BTN_SHORT_MS         = 1000;
constexpr uint32_t BTN_MEDIUM_MS        = 5000;
// long press threshold (your old code had a huge window; keep it if you want)
constexpr uint32_t BTN_LONG_MS          = 60000;

// -----------------------------
// Topics
// -----------------------------
constexpr const char* MQTT_ALL_REQUEST  = "sonoff_all/request";
constexpr const char* MQTT_ALL_RESPONSE = "sonoff_all/response";

// -----------------------------
// Credentials (from config.h)
// -----------------------------
static const char* WIFI_SSID_C     = WIFI_SSID;
static const char* WIFI_PASS_C     = WIFI_PASSWORD;
static const char* MQTT_SERVER_C   = MQTT_ADDRESS;

// -----------------------------
// Globals
// -----------------------------
WiFiClient espClient;
PubSubClient mqttClient(espClient);
SimpleTimer timer;

enum class LedState   : uint8_t { Off, On };
enum class RelayState : uint8_t { Off, On };

static char hostName[48];
static char topicRequest[80];
static char topicResponse[80];

// connection bookkeeping
static bool mqttEverConnected = false;
static uint8_t wifiFailCount  = 0;
static uint8_t mqttFailCount  = 0;
static uint32_t lastWiFiTryMs = 0;
static uint32_t lastMqttTryMs = 0;

// button state machine
static int lastBtnRead        = HIGH;
static int stableBtnState     = HIGH;
static uint32_t lastDebounceMs = 0;
static uint32_t pressStartMs   = 0;

// -----------------------------
// Helpers
// -----------------------------
static inline bool isWiFiUp() {
  return WiFi.status() == WL_CONNECTED;
}

static inline void setLed(LedState s) {
  // active-low LED: LOW=ON, HIGH=OFF
  digitalWrite(Pins::LED, (s == LedState::On) ? LOW : HIGH);
}

static inline RelayState getRelayState() {
  return (digitalRead(Pins::RELAY) == HIGH) ? RelayState::On : RelayState::Off;
}

static inline const char* relayStateStr(RelayState s) {
  return (s == RelayState::On) ? "on" : "off";
}

static void setRelay(RelayState s, bool publish = true) {
  digitalWrite(Pins::RELAY, (s == RelayState::On) ? HIGH : LOW);
  delay(50); // small settle
  if (publish) {
    // publish only if MQTT is connected
    if (mqttClient.connected()) {
      mqttClient.publish(topicResponse, relayStateStr(getRelayState()), true /*retain*/);
    }
  }
}

static void toggleRelay(bool publish = true) {
  const RelayState next = (getRelayState() == RelayState::On) ? RelayState::Off : RelayState::On;
  setRelay(next, publish);
}

static void restartDevice() {
  Serial.println("Resetting device...");
  setLed(LedState::Off);
  delay(300);
  ESP.reset();
  delay(1000);
}

static void buildIdentityAndTopics() {
  // deviceNames[] and hostNumber come from config.h (existing behavior)
  // hostName = "esp8266-" + deviceNames[hostNumber]
  snprintf(hostName, sizeof(hostName), "esp8266-%s", deviceNames[hostNumber]);
  snprintf(topicRequest, sizeof(topicRequest), "%s/request", hostName);
  snprintf(topicResponse, sizeof(topicResponse), "%s/response", hostName);

  Serial.print("Host: ");
  Serial.println(hostName);
  Serial.print("MQTT request topic: ");
  Serial.println(topicRequest);
  Serial.print("MQTT response topic: ");
  Serial.println(topicResponse);
}

static void setupOTA() {
  ArduinoOTA.setHostname(hostName);

  ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress * 100U) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
}

static void publishRelayStatus() {
  if (!mqttClient.connected()) return;
  const RelayState s = getRelayState();
  mqttClient.publish(topicResponse, relayStateStr(s), true /*retain*/);
  Serial.print("Published status: ");
  Serial.println(relayStateStr(s));
}

// -----------------------------
// WiFi / MQTT (non-blocking)
// -----------------------------
static void ensureWiFi() {
  if (isWiFiUp()) {
    wifiFailCount = 0;
    return;
  }

  const uint32_t now = millis();
  if (now - lastWiFiTryMs < WIFI_RETRY_MS) return;
  lastWiFiTryMs = now;

  if (wifiFailCount == 0) {
    Serial.print("Connecting WiFi to ");
    Serial.println(WIFI_SSID_C);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID_C, WIFI_PASS_C);
  }

  setLed((wifiFailCount % 2 == 0) ? LedState::On : LedState::Off);

  wifiFailCount++;
  Serial.print("WiFi not connected, attempt=");
  Serial.println(wifiFailCount);

  if (wifiFailCount > WIFI_MAX_FAILS) {
    // Old behavior: if relay is ON keep trying; else restart
    if (getRelayState() == RelayState::On) {
      wifiFailCount = 0;
    } else {
      restartDevice();
    }
  }
}

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // build payload as a null-terminated string safely
  static char msg[64];
  const unsigned int n = (length >= sizeof(msg)) ? (sizeof(msg) - 1) : length;
  memcpy(msg, payload, n);
  msg[n] = '\0';

  const String t(topic);
  const String v(msg);

  Serial.print("MQTT [");
  Serial.print(t);
  Serial.print("] => ");
  Serial.println(v);

  if (t == topicRequest) {
    if (v == "on") {
      setRelay(RelayState::On, true);
    } else if (v == "off") {
      setRelay(RelayState::Off, true);
    } else if (v == "status") {
      publishRelayStatus();
    }
    return;
  }

  if (t == MQTT_ALL_REQUEST) {
    if (v == "showip") {
      static char ipMsg[96];
      snprintf(ipMsg, sizeof(ipMsg), "%s ==> %s", hostName, WiFi.localIP().toString().c_str());
      mqttClient.publish(MQTT_ALL_RESPONSE, ipMsg, false);
    }
    return;
  }
}

static void ensureMQTT() {
  if (!isWiFiUp()) return;
  if (mqttClient.connected()) return;

  const uint32_t now = millis();
  if (now - lastMqttTryMs < MQTT_RETRY_MS) return;
  lastMqttTryMs = now;

  setLed(LedState::Off);
  Serial.print("Attempting MQTT connection to ");
  Serial.println(MQTT_SERVER_C);

  // clientId must be stable memory
  const bool ok = mqttClient.connect(hostName);
  if (ok) {
    mqttFailCount = 0;
    mqttEverConnected = true;

    Serial.println("MQTT connected.");
    mqttClient.subscribe(topicRequest);
    mqttClient.subscribe(MQTT_ALL_REQUEST);

    setLed(LedState::On);
    publishRelayStatus();
    return;
  }

  mqttFailCount++;
  Serial.print("MQTT failed, rc=");
  Serial.print(mqttClient.state());
  Serial.print(" attempt=");
  Serial.println(mqttFailCount);

  // Old behavior: after N fails, if relay is ON keep trying; else restart
  if (mqttFailCount > MQTT_MAX_FAILS) {
    if (getRelayState() == RelayState::On) {
      mqttFailCount = 0;
    } else {
      restartDevice();
    }
  }

  // quick blink on failure (non-blocking alternative kept simple)
  setLed(LedState::On);
  delay(50);
  setLed(LedState::Off);
}

// -----------------------------
// Button handling (debounced)
// -----------------------------
static void handleButton() {
  const uint32_t now = millis();
  const int reading = digitalRead(Pins::BTN);

  if (reading != lastBtnRead) {
    lastDebounceMs = now;
    lastBtnRead = reading;
  }

  if (now - lastDebounceMs < BTN_DEBOUNCE_MS) return;

  if (reading != stableBtnState) {
    stableBtnState = reading;

    // pressed (active-low)
    if (stableBtnState == LOW) {
      pressStartMs = now;
      return;
    }

    // released
    const uint32_t duration = now - pressStartMs;

    if (duration < BTN_SHORT_MS) {
      Serial.println("Button: short press -> toggle relay");
      toggleRelay(true);
    } else if (duration < BTN_MEDIUM_MS) {
      Serial.println("Button: medium press -> restart");
      restartDevice();
    } else if (duration < BTN_LONG_MS) {
      Serial.println("Button: long press -> restart (settings placeholder)");
      restartDevice();
    } else {
      Serial.println("Button: very long press -> restart");
      restartDevice();
    }
  }
}

// -----------------------------
// Timer callback
// -----------------------------
static void onStatusTimer() {
  if (mqttClient.connected()) {
    publishRelayStatus();
  }
}

// -----------------------------
// Arduino entrypoints
// -----------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  pinMode(Pins::LED, OUTPUT);
  setLed(LedState::Off);

  pinMode(Pins::RELAY, OUTPUT);
  setRelay(RelayState::Off, false);

  pinMode(Pins::BTN, INPUT_PULLUP);

  buildIdentityAndTopics();

  mqttClient.setServer(MQTT_SERVER_C, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  setupOTA();

  timer.setInterval(STATUS_PUBLISH_MS, onStatusTimer);

  Serial.println("Boot complete.");
}

void loop() {
  ensureWiFi();
  ensureMQTT();

  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  ArduinoOTA.handle();
  timer.run();

  handleButton();
}
