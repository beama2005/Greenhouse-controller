#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>
#include <Firebase_ESP_Client.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <Update.h>

// --- Firebase Credentials ---
#define FIREBASE_HOST "Example: ahttps://green-house-automation-2bf5b-default-rtdb.asia-southeast1.firebasedatabase.app/" // e.g. https://your-project.firebaseio.com
#define FIREBASE_AUTH "Example: aAIzaSyCiWIAfIIgpYOv-OcwZ35sxoSP6GINblQk"
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config; // Used for token based auth

// LoRa pin definitions
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2

#define RELAY_1      12
#define RELAY_2      13
#define RELAY_3      25
#define RELAY_4      26

#define PIN_AUTO     33
#define PIN_OPEN     35
#define PIN_CLOSE    34

// --- Optocoupler input pin definitions ---
#define OPTO_OPEN_PIN 32
#define OPTO_CLOSE_PIN 27



#define CURRENT_VERSION "1.0.0"
const char* OTA_VERSION_URL = "https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO/main/version.txt";
const char* OTA_FIRMWARE_URL = "https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO/main/firmware.bin";

struct RelayTask {
  bool active = false;
  unsigned long startTime = 0;
  unsigned long duration = 0;
  uint8_t relayPin;
};

RelayTask relay1Task, relay2Task;

String currentOperation = "Idle";
unsigned long operationEndTime = 0;

// --- Network Credentials ---
// Access Point mode
const char* ap_ssid = "Green House Automation Board aXM3";
const char* ap_password = "12345678";
// WiFi client mode
const char* wifi_ssid = "1234991_Trooli.uk";
const char* wifi_password = "1dd866cb";

// Mode state
bool isAPMode = true; // Start in AP mode
unsigned long buttonPressStart = 0;
bool buttonHeld = false;
#define BOOT_BTN 0
#define MODE_LED 15


AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Preferences prefs;

// Temperature setting variables
float tempSetPoint = 25.0;
float tempHysteresis = 2.0;
float tempMaxPosition = 100.0;
float tempTime = 60.0;
float tempAlarmHigh = 35.0;
float tempAlarmLow = 10.0;
float maxWindAlarm = 40.0;
// --- Only ONE global vent runtime variable used everywhere ---
int ventRuntime = 10; // Unified for both manual and sensor modes

// --- Activation State ---
int sensorActivated = 0; // 0 = not activated, 1 = activated

// --- Vent Position Tracking ---
float ventPosition = 0.0; // 0 = fully closed, 100 = fully open
unsigned long totalOpenTime = 0;
unsigned long totalCloseTime = 0;
unsigned long lastPositionUpdate = 0;
bool sensorControlActive = false;
unsigned long sensorControlStateTime = 0;
int sensorControlState = 0; // 0=idle, 1=calibrating, 2=ready
unsigned long sensorControlStepStart = 0;
bool relayStepActive = false;

// --- VENT 2 CONTROL ---
float vent2Position = 0.0; // 0 = fully closed, 100 = fully open
unsigned long totalOpenTime2 = 0;
unsigned long totalCloseTime2 = 0;
bool relay3TaskActive = false;
bool relay4TaskActive = false;
unsigned long relay3Start = 0;
unsigned long relay3Duration = 0;
unsigned long relay4Start = 0;
unsigned long relay4Duration = 0;

// --- Wind Direction and Speed Settings ---
String vent1Direction = "N";
String vent2Direction = "N";
int leeMaxPosition = 100;
int maxSpeedSetpoint = 25;
int windMaxPosition = 100;

// --- Rain Sensor Vent Limits ---
int rainVent1Limit = 100;
int rainVent2Limit = 100;

// --- Serial Message State Trackers ---
bool vent1MaxMsgShown = false;
bool vent1MinMsgShown = false;
bool vent2MaxMsgShown = false;
bool vent2MinMsgShown = false;

// --- SYNC BUTTON STATE ---
bool syncRequested = false;
bool syncInProgress = false;
float syncVent1Target = 0.0;
float syncVent2Target = 0.0;

#define MAX_HISTORICAL_ALARMS 5
String historicalAlarms[MAX_HISTORICAL_ALARMS];

void loadTempSettings() {
  prefs.begin("tempset", true);
  tempSetPoint = prefs.getFloat("setpoint", 25.0);
  tempHysteresis = prefs.getFloat("hysteresis", 2.0);
  tempMaxPosition = prefs.getFloat("maxpos", 100.0);
  tempTime = prefs.getFloat("time", 60.0);
  ventRuntime = prefs.getInt("ventruntime", ventRuntime); // Use current value as default fallback
  vent1Direction = prefs.getString("vent1_dir", vent1Direction);
  vent2Direction = prefs.getString("vent2_dir", vent2Direction);
  leeMaxPosition = prefs.getInt("lee_maxpos", leeMaxPosition);
  maxSpeedSetpoint = prefs.getInt("maxspeed_setpoint", maxSpeedSetpoint);
  windMaxPosition = prefs.getInt("wind_maxpos", windMaxPosition);
  rainVent1Limit = prefs.getInt("rain_v1_limit", 100);
  rainVent2Limit = prefs.getInt("rain_v2_limit", 100);
  tempAlarmHigh = prefs.getFloat("alarmHigh", 35.0);
  tempAlarmLow = prefs.getFloat("alarmLow", 10.0);
  maxWindAlarm = prefs.getFloat("alarmWind", 40.0);
  prefs.end();
}

void saveAlarmThresholds() {
  prefs.begin("tempset", false);
  prefs.putFloat("alarmHigh", tempAlarmHigh);
  prefs.putFloat("alarmLow", tempAlarmLow);
  prefs.putFloat("alarmWind", maxWindAlarm); // Save wind speed
  prefs.end();
}

void saveTempSetting(const char* key, float val) {
  prefs.begin("tempset", false);
  prefs.putFloat(key, val);
  prefs.end();
}

void saveVentRuntime(int val) {
  prefs.begin("tempset", false);
  prefs.putInt("ventruntime", val);
  prefs.end();
}

void saveWindSettings() {
  prefs.begin("tempset", false);
  prefs.putString("vent1_dir", vent1Direction);
  prefs.putString("vent2_dir", vent2Direction);
  prefs.putInt("lee_maxpos", leeMaxPosition);
  prefs.putInt("maxspeed_setpoint", maxSpeedSetpoint);
  prefs.putInt("wind_maxpos", windMaxPosition);
  prefs.end();
}

void saveRainLimits() {
  prefs.begin("tempset", false);
  prefs.putInt("rain_v1_limit", rainVent1Limit);
  prefs.putInt("rain_v2_limit", rainVent2Limit);
  prefs.end();
}

String getLatestVersion() {
  HTTPClient http;
  http.begin(OTA_VERSION_URL);
  int httpCode = http.GET();
  if (httpCode == 200) {
    String latest = http.getString();
    http.end();
    latest.trim();
    return latest;
  }
  http.end();
  return "";
}

bool isNewVersion(String latest) {
  return latest != CURRENT_VERSION;
}

void performOTAUpdate() {
  HTTPClient http;
  http.begin(OTA_FIRMWARE_URL);
  int httpCode = http.GET();
  if (httpCode == 200) {
    int len = http.getSize();
    WiFiClient *stream = http.getStreamPtr();
    if (Update.begin(len)) {
      size_t written = Update.writeStream(*stream);
      if (Update.end() && Update.isFinished()) {
        Serial.println("Update complete. Rebooting.");
        delay(1000);
        ESP.restart();
      }
    } else {
      Serial.println("Failed to begin update.");
    }
  } else {
    Serial.printf("HTTP GET failed, code: %d\n", httpCode);
  }
  http.end();
}


void loadSensorActivated() {
  prefs.begin("sensoract", true);
  sensorActivated = prefs.getInt("activated", 0);
  prefs.end();
}

void saveSensorActivated(int val) {
  prefs.begin("sensoract", false);
  prefs.putInt("activated", val);
  prefs.end();
}

void sendTempSettings(AsyncWebSocketClient *client) {
  String msg = "{\"type\":\"tempsettings\",\"setpoint\":" + String(tempSetPoint,2) + ",\"hysteresis\":" + String(tempHysteresis,2) + ",\"maxpos\":" + String(tempMaxPosition,2) + ",\"time\":" + String(tempTime,2) + ",\"ventruntime\":" + String(ventRuntime) + "}";
  client->text(msg);
}

void sendAllSettings(AsyncWebSocketClient *client) {
  sendTempSettings(client);
  sendWindSettings(client);
}

void sendActivationState(AsyncWebSocketClient *client) {
  String msg = "{\"type\":\"activation\",\"activated\":" + String(sensorActivated) + "}";
  client->text(msg);
}

void sendWindSettings(AsyncWebSocketClient *client) {
  String msg = "{\"type\":\"windsettings\",\"vent1_dir\":\"" + vent1Direction + "\",\"vent2_dir\":\"" + vent2Direction + "\",\"lee_maxpos\":" + String(leeMaxPosition) + ",\"maxspeed_setpoint\":" + String(maxSpeedSetpoint) + ",\"wind_maxpos\":" + String(windMaxPosition) + ",\"rain_v1_limit\":" + String(rainVent1Limit) + ",\"rain_v2_limit\":" + String(rainVent2Limit) + "}";
  client->text(msg);
}

void handleWindSettingsMsg(const String& msg) {
  int idx6 = msg.indexOf("vent1_dir");
  int idx7 = msg.indexOf("vent2_dir");
  int idx8 = msg.indexOf("lee_maxpos");
  int idx9 = msg.indexOf("maxspeed_setpoint");
  int idx10 = msg.indexOf("wind_maxpos");
  int idx11 = msg.indexOf("rain_v1_limit");
  int idx12 = msg.indexOf("rain_v2_limit");
  if (idx6 != -1) {
    int endIdx = msg.indexOf(",", idx6);
    if (endIdx == -1) endIdx = msg.indexOf("}", idx6);
    vent1Direction = msg.substring(msg.indexOf(":", idx6)+2, endIdx-1);
  }
  if (idx7 != -1) {
    int endIdx = msg.indexOf(",", idx7);
    if (endIdx == -1) endIdx = msg.indexOf("}", idx7);
    vent2Direction = msg.substring(msg.indexOf(":", idx7)+2, endIdx-1);
  }
  if (idx8 != -1) {
    int endIdx = msg.indexOf(",", idx8);
    if (endIdx == -1) endIdx = msg.indexOf("}", idx8);
    leeMaxPosition = msg.substring(msg.indexOf(":", idx8)+1, endIdx).toInt();
  }
  if (idx9 != -1) {
    int endIdx = msg.indexOf(",", idx9);
    if (endIdx == -1) endIdx = msg.indexOf("}", idx9);
    maxSpeedSetpoint = msg.substring(msg.indexOf(":", idx9)+1, endIdx).toInt();
  }
  if (idx10 != -1) {
    int endIdx = msg.indexOf(",", idx10);
    if (endIdx == -1) endIdx = msg.indexOf("}", idx10);
    windMaxPosition = msg.substring(msg.indexOf(":", idx10)+1, endIdx).toInt();
  }
  if (idx11 != -1) {
    int endIdx = msg.indexOf(",", idx11);
    if (endIdx == -1) endIdx = msg.indexOf("}", idx11);
    rainVent1Limit = msg.substring(msg.indexOf(":", idx11)+1, endIdx).toInt();
  }
  if (idx12 != -1) {
    int endIdx = msg.indexOf(",", idx12);
    if (endIdx == -1) endIdx = msg.indexOf("}", idx12);
    rainVent2Limit = msg.substring(msg.indexOf(":", idx12)+1, endIdx).toInt();
  }
  saveWindSettings();
  saveRainLimits();
}

void sendAlarmSettings(AsyncWebSocketClient *client) {
  String msg = "{\"type\":\"alarmsettings\",\"high\":" + String(tempAlarmHigh) + ",\"low\":" + String(tempAlarmLow) + ",\"wind\":" + String(maxWindAlarm) + "}";
  client->text(msg);
}

void loadHistoricalAlarms() {
  prefs.begin("histAlarms", true);
  for (int i = 0; i < MAX_HISTORICAL_ALARMS; i++) {
    historicalAlarms[i] = prefs.getString(("a" + String(i)).c_str(), "");
  }
  prefs.end();
}

void saveHistoricalAlarms() {
  prefs.begin("histAlarms", false);
  for (int i = 0; i < MAX_HISTORICAL_ALARMS; i++) {
    prefs.putString(("a" + String(i)).c_str(), historicalAlarms[i]);
  }
  prefs.end();
}

void addHistoricalAlarm(const String& alarm) {
  // Shift older alarms
  for (int i = MAX_HISTORICAL_ALARMS - 1; i > 0; i--) {
    historicalAlarms[i] = historicalAlarms[i - 1];
  }
  historicalAlarms[0] = alarm;
  saveHistoricalAlarms();
}


void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    sendAllSettings(client);
    sendActivationState(client);
    sendAlarmSettings(client);
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
      String msg = String((char*)data);
      if (msg.startsWith("{\"type\":\"tempsettings\"")) {
        int idx1 = msg.indexOf("setpoint");
        int idx2 = msg.indexOf("hysteresis");
        int idx3 = msg.indexOf("maxpos");
        int idx4 = msg.indexOf("time");
        int idx5 = msg.indexOf("ventruntime");
        float v1 = tempSetPoint, v2 = tempHysteresis, v3 = tempMaxPosition, v4 = tempTime;
        int v5 = ventRuntime;
        if (idx1 != -1) v1 = msg.substring(msg.indexOf(":", idx1)+1, msg.indexOf(",", idx1)).toFloat();
        if (idx2 != -1) v2 = msg.substring(msg.indexOf(":", idx2)+1, msg.indexOf(",", idx2)).toFloat();
        if (idx3 != -1) v3 = msg.substring(msg.indexOf(":", idx3)+1, msg.indexOf(",", idx3)).toFloat();
        if (idx4 != -1) v4 = msg.substring(msg.indexOf(":", idx4)+1, msg.indexOf(",", idx4)).toFloat();
        if (idx5 != -1) {
          int endIdx = msg.indexOf(",", idx5);
          if (endIdx == -1) endIdx = msg.indexOf("}", idx5);
          v5 = msg.substring(msg.indexOf(":", idx5)+1, endIdx).toInt();
        }
        tempSetPoint = v1; tempHysteresis = v2; tempMaxPosition = v3; tempTime = v4; ventRuntime = v5;
        saveTempSetting("setpoint", v1);
        saveTempSetting("hysteresis", v2);
        saveTempSetting("maxpos", v3);
        saveTempSetting("time", v4);
        saveVentRuntime(v5);
        sendTempSettings(client);
      } else if (msg.startsWith("{\"type\":\"windsettings\"")) {
        handleWindSettingsMsg(msg);
        sendWindSettings(client);
      } else if (msg.startsWith("{\"type\":\"activation\"")) {
        int idx = msg.indexOf("activated");
        int val = 0;
        if (idx != -1) val = msg.substring(msg.indexOf(":", idx)+1, msg.indexOf("}", idx)).toInt();
        sensorActivated = val;
        saveSensorActivated(val);
        sendActivationState(client);
      } else if (msg.startsWith("{\"type\":\"sync\"")) {
        syncRequested = true;
        client->text("{\"type\":\"sync_ack\"}");
      } else if (msg.startsWith("{\"type\":\"sync_ack\"")) {
        Serial.println("[SYNC] Sync acknowledged by backend.");
      } else if (msg.startsWith("{\"type\":\"alarmsettings\"")) {
        int idx1 = msg.indexOf("high");
        int idx2 = msg.indexOf("low");
        if (idx1 != -1) tempAlarmHigh = msg.substring(msg.indexOf(":", idx1) + 1, msg.indexOf(",", idx1)).toFloat();
        if (idx2 != -1) {
          int endIdx = msg.indexOf("}", idx2);
          tempAlarmLow = msg.substring(msg.indexOf(":", idx2) + 1, endIdx).toFloat();
        }
        int idx3 = msg.indexOf("wind");
        if (idx3 != -1) {
          int endIdx = msg.indexOf("}", idx3);
          maxWindAlarm = msg.substring(msg.indexOf(":", idx3) + 1, endIdx).toFloat();
        }
        saveAlarmThresholds();
      }

    }
  }
}

unsigned long relay_delay = 3000; // 3 seconds
unsigned long openPressedTime = 0;
unsigned long closePressedTime = 0;

const int MAX_SENSORS = 11;
const byte localAddress = 0x01; // Receiver address (change as needed)

// Sensor data arrays
String sensorNumber[MAX_SENSORS];  // e.g., "S1"
String sensorType[MAX_SENSORS];    // e.g., "TE"
float sensorValue[MAX_SENSORS];    // e.g., 23.45

// Packet header variables
byte sender = 0;
byte recipient = 0;
byte incomingMsgId = 0;
byte incomingLength = 0;
String incoming = "";

// Track which tab is active (for optocoupler logic)
volatile int activeTab = 0; // 0 = manual, 1 = sensor

bool isAutoModeOn() {
  return digitalRead(PIN_AUTO) == LOW; // LOW = selected
}

void handleRelayTask(RelayTask &task) {
  if (sensorActivated != 0) {
    // When sensorActivated is ON, forcibly turn off relay if not in use
    if (task.active) {
      digitalWrite(task.relayPin, LOW);
      task.active = false;
      Serial.printf("Relay %d forced OFF by sensorActivated\n", task.relayPin);
    }
    return;
  }
  if (task.active && millis() - task.startTime >= task.duration) {
    digitalWrite(task.relayPin, LOW);
    task.active = false;
    Serial.printf("Relay %d OFF\n", task.relayPin);
    // If both relays are off, set operation to Idle
    if (!relay1Task.active && !relay2Task.active) {
      currentOperation = "Idle";
      operationEndTime = 0;
    }
  }
}

void setupWebServer() {
  // Main manual control page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!isAutoModeOn()) {
      // Serve a minimal page if auto mode is not selected, but with polling to reload if mode changes
      String page = "<!DOCTYPE html><html><head><title>Greenhouse Control</title><style>body { font-family: sans-serif; text-align: center; margin-top: 50px; }</style><script>setInterval(function(){fetch('/mode').then(r=>r.text()).then(m=>{if(m==='1'){location.reload();}})},2000);</script></head><body><h2>Automatic Mode is not Selected</h2><p>Please enable Automatic Mode using the switch on your device to access web controls.</p></body></html>";
      request->send(200, "text/html", page);
      return;
    }
    request->send(200, "text/html", generateManualPage(true));
  });

  // Route to return current auto mode status (used in JS)
  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", isAutoModeOn() ? "1" : "0");
  });

    server.on("/getIP", HTTP_GET, [](AsyncWebServerRequest *request){
      String json = "{";
      Serial.println("[/getIP] AP: " + WiFi.softAPIP().toString());
      Serial.println("[/getIP] STA: " + WiFi.localIP().toString());
      json += "\"ap\":\"" + WiFi.softAPIP().toString() + "\",";
      json += "\"sta\":\"" + WiFi.localIP().toString() + "\"";
      json += "}";
      request->send(200, "application/json", json);
    });
  // Control command handler
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!isAutoModeOn()) {
      request->send(403, "text/plain", "Auto Mode not active");
      return;
    }

    String cmd = request->getParam("cmd")->value();
    int runtime = request->hasParam("runtime") ? request->getParam("runtime")->value().toInt() : ventRuntime;
    handleWebCommand(cmd, runtime);
    request->send(200, "text/plain", "OK");
  });

server.on("/addHistoricalAlarm", HTTP_GET, [](AsyncWebServerRequest *request) {
  if (request->hasParam("msg")) {
    String msg = request->getParam("msg")->value();
    addHistoricalAlarm(msg);
    request->send(200, "text/plain", "Saved");
  } else {
    request->send(400, "text/plain", "Missing msg param");
  }
});

server.on("/getHistoricalAlarms", HTTP_GET, [](AsyncWebServerRequest *request) {
  String json = "[";
  for (int i = 0; i < MAX_HISTORICAL_ALARMS; i++) {
    if (i > 0) json += ",";
    json += "\"" + historicalAlarms[i] + "\"";
  }
  json += "]";
  request->send(200, "application/json", json);
});

server.on("/getTime", HTTP_GET, [](AsyncWebServerRequest *request) {
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char buffer[64];
  if (now < 946684800) {
    snprintf(buffer, sizeof(buffer), "Time not set");
    Serial.println("[getTime] Time not set, returning error.");
  } else {
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  }
  String json = "{\"datetime\":\"" + String(buffer) + "\"}";
  request->send(200, "application/json", json);
});

server.on("/setTime", HTTP_GET, [](AsyncWebServerRequest *request) {
  if (!request->hasParam("datetime")) {
    request->send(400, "text/plain", "Missing datetime param");
    return;
  }
  String datetime = request->getParam("datetime")->value(); // format: YYYY-MM-DDTHH:MM
  struct tm t = {};
  if (sscanf(datetime.c_str(), "%d-%d-%dT%d:%d", &t.tm_year, &t.tm_mon, &t.tm_mday, &t.tm_hour, &t.tm_min) == 5) {
    t.tm_year -= 1900; // struct tm starts at 1900
    t.tm_mon -= 1;     // 0-indexed months
    t.tm_sec = 0;
    time_t newTime = mktime(&t);
    if (newTime != -1) {
      struct timeval now = { newTime, 0 };
      settimeofday(&now, nullptr);
      prefs.begin("timeset", false);
      prefs.putLong("last_time", newTime);
      prefs.end();
      Serial.println("[Time] User-set time updated and saved: " + datetime);
      request->send(200, "text/plain", "Time updated and saved.");
    } else {
      request->send(500, "text/plain", "Failed to parse time.");
    }
  } else {
    request->send(400, "text/plain", "Invalid format. Use YYYY-MM-DDTHH:MM");
  }
});

server.on("/clearHistoricalAlarms", HTTP_GET, [](AsyncWebServerRequest *request) {
  prefs.begin("histAlarms", false);
  for (int i = 0; i < MAX_HISTORICAL_ALARMS; i++) {
    prefs.remove(("a" + String(i)).c_str());
    historicalAlarms[i] = "";
  }
  prefs.end();
  request->send(200, "text/plain", "OK");
});

server.on("/saveNetwork", HTTP_GET, [](AsyncWebServerRequest *request){
  if (request->hasParam("ssid") && request->hasParam("pass")) {
    String ssid = request->getParam("ssid")->value();
    String pass = request->getParam("pass")->value();

    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();

    request->send(200, "text/plain", "OK");  // Just say OK for clean response
  } else {
    request->send(400, "text/plain", "Missing SSID or password");
  }
});

server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(200, "text/plain", "Rebooting...");
  delay(1000);
  ESP.restart();
});

server.on("/wifiStatus", HTTP_GET, [](AsyncWebServerRequest *request){
  String status = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Not Connected";
  request->send(200, "application/json", "{\"status\":\"" + status + "\"}");
});

  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    unsigned long now = millis();
    unsigned long remaining = 0;
    if (operationEndTime > now && currentOperation != "Idle") {
      remaining = (operationEndTime - now) / 1000;
    } else {
      remaining = 0;
    }
    String json = "{\"operation\":\"" + currentOperation + "\",\"remaining\":" + String(remaining) + "}";
    request->send(200, "application/json", json);
  });

  // Vent status endpoint
  server.on("/ventStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"operation\":\"" + currentOperation + "\",\"vent1\":" + String(ventPosition, 1) + ",\"vent2\":" + String(vent2Position, 1) + "}";
    request->send(200, "application/json", json);
  });

  // New: Sensor data endpoint
  server.on("/sensorData", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "[";
    for (int i = 0; i < MAX_SENSORS; i++) {
      if (i > 0) json += ",";
      String fullType;
      decodeSensorType(sensorType[i], fullType);
      String valueStr = "";
      String unit = "";
      String extra = "";
      if (sensorType[i] == "TE") {
        valueStr = String(sensorValue[i], 2);
        unit = "°C";
      } else if (sensorType[i] == "HU") {
        valueStr = String(sensorValue[i], 2);
        unit = "%";
      } else if (sensorType[i] == "WS") {
        valueStr = String(sensorValue[i], 2);
        unit = "mph";
      } else if (sensorType[i] == "WD") {
        valueStr = String(sensorValue[i], 0);
        unit = "°";
        extra = " (" + degreesToCardinal(sensorValue[i]) + ")";
      } else if (sensorType[i] == "RN") {
        valueStr = (sensorValue[i] == 1.0) ? "Raining" : "Not Raining";
        unit = "";
      } else {
        valueStr = String(sensorValue[i], 2);
        unit = "";
      }
      json += "{";
      json += "\"number\":\"" + sensorNumber[i] + "\",";
      json += "\"type\":\"" + fullType + "\",";
      json += "\"value\":\"" + valueStr + (unit.length() > 0 ? (" " + unit) : "") + extra + "\"";
      json += "}";
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  // Set active tab endpoint
  server.on("/setTab", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("tab")) {
      String tab = request->getParam("tab")->value();
      if (tab == "manual") activeTab = 0;
      else if (tab == "sensor") activeTab = 1;
    }
    request->send(200, "text/plain", "OK");
  });

  // Settings endpoint for UI sync
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"ventruntime\":" + String(ventRuntime);
    json += ",\"setpoint\":" + String(tempSetPoint,2);
    json += ",\"hysteresis\":" + String(tempHysteresis,2);
    json += ",\"maxpos\":" + String(tempMaxPosition,2);
    json += ",\"time\":" + String(tempTime,2);
    json += ",\"vent1_dir\":\"" + vent1Direction + "\"";
    json += ",\"vent2_dir\":\"" + vent2Direction + "\"";
    json += ",\"lee_maxpos\":" + String(leeMaxPosition);
    json += ",\"maxspeed_setpoint\":" + String(maxSpeedSetpoint);
    json += ",\"wind_maxpos\":" + String(windMaxPosition);
    json += ",\"rain_v1_limit\":" + String(rainVent1Limit);
    json += ",\"rain_v2_limit\":" + String(rainVent2Limit);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.addHandler(&ws);
  server.begin();
  Serial.println("Web server started.");
}

void handleWebCommand(String cmd, int runtime) {
  if (sensorActivated != 0) {
    Serial.println("Relay command ignored: sensorActivated is ON");
    return;
  }
  // Always update the global ventRuntime and save it
  ventRuntime = runtime;
  saveVentRuntime(runtime);
  int totalMs = ventRuntime * 1000;

  if (cmd == "calibrate") {
    activateRelay(RELAY_2, totalMs + 5000, relay2Task);
    currentOperation = "Calibrating";
    operationEndTime = millis() + totalMs + 5000;
  } else if (cmd == "open") {
    activateRelay(RELAY_1, totalMs, relay1Task);
    currentOperation = "Opening";
    operationEndTime = millis() + totalMs;
  } else if (cmd == "close") {
    activateRelay(RELAY_2, totalMs, relay2Task);
    currentOperation = "Closing";
    operationEndTime = millis() + totalMs;
  } else if (cmd == "open75") {
    activateRelay(RELAY_1, totalMs * 0.75, relay1Task);
    currentOperation = "Opening 75%";
    operationEndTime = millis() + (unsigned long)(totalMs * 0.75);
  } else if (cmd == "close75") {
    activateRelay(RELAY_2, totalMs * 0.75, relay2Task);
    currentOperation = "Closing 75%";
    operationEndTime = millis() + (unsigned long)(totalMs * 0.75);
  } else if (cmd == "open50") {
    activateRelay(RELAY_1, totalMs * 0.5, relay1Task);
    currentOperation = "Opening 50%";
    operationEndTime = millis() + (unsigned long)(totalMs * 0.5);
  } else if (cmd == "close50") {
    activateRelay(RELAY_2, totalMs * 0.5, relay2Task);
    currentOperation = "Closing 50%";
    operationEndTime = millis() + (unsigned long)(totalMs * 0.5);
  } else if (cmd == "open25") {
    activateRelay(RELAY_1, totalMs * 0.25, relay1Task);
    currentOperation = "Opening 25%";
    operationEndTime = millis() + (unsigned long)(totalMs * 0.25);
  } else if (cmd == "close25") {
    activateRelay(RELAY_2, totalMs * 0.25, relay2Task);
    currentOperation = "Closing 25%";
    operationEndTime = millis() + (unsigned long)(totalMs * 0.25);
  }

  Serial.printf("Handled web command: %s (%d ms)\n", cmd.c_str(), totalMs);
}

void activateRelay(uint8_t pin, unsigned long duration, RelayTask &task) {
  if (sensorActivated != 0) {
    Serial.printf("Relay activation blocked: sensorActivated is ON\n");
    return;
  }
  digitalWrite(pin, HIGH);
  task.active = true;
  task.startTime = millis();
  task.duration = duration;
  task.relayPin = pin;
  Serial.printf("Relay %d ON for %lu ms\n", pin, duration);
}

void activateSensorRelay(uint8_t pin, unsigned long duration, RelayTask &task) {
  digitalWrite(pin, HIGH);
  task.active = true;
  task.startTime = millis();
  task.duration = duration;
  task.relayPin = pin;
  Serial.printf("[SensorCtrl] Relay %d ON for %lu ms\n", pin, duration);
}

void handleSensorRelayTask(RelayTask &task) {
  if (task.active && millis() - task.startTime >= task.duration) {
    digitalWrite(task.relayPin, LOW);
    task.active = false;
    Serial.printf("[SensorCtrl] Relay %d OFF\n", task.relayPin);
  }
}

void setupRelaysAndSwitch() {

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);
  digitalWrite(RELAY_1, LOW);
  digitalWrite(RELAY_2, LOW);
  digitalWrite(RELAY_3, LOW);
  digitalWrite(RELAY_4, LOW);

  pinMode(PIN_AUTO, INPUT_PULLUP);   // External pull-ups used
  pinMode(PIN_OPEN, INPUT);
  pinMode(PIN_CLOSE, INPUT);

  // Optocoupler inputs (external pulldown)
  pinMode(OPTO_OPEN_PIN, INPUT);
  pinMode(OPTO_CLOSE_PIN, INPUT);

  Serial.println("Relay and rotary switch setup complete.");
}

void handleRotarySwitch() {
  if (isAutoModeOn() || sensorActivated != 0) {
    // Don't allow manual open/close when auto mode is active or sensor control is active
    return;
  }

  // Only allow manual control if relay is not busy with a timed task
  if (!relay1Task.active) {
    bool isOpenPressed = digitalRead(PIN_OPEN) == LOW;
    if (isOpenPressed) {
      if (openPressedTime == 0) {
        openPressedTime = millis();
        Serial.println("Open switch pressed.");
      } else if (millis() - openPressedTime >= relay_delay) {
        if (digitalRead(RELAY_1) == LOW) {
          digitalWrite(RELAY_1, HIGH);
          Serial.println("RELAY_1 activated (Open).");
        }
      }
    } else {
      if (openPressedTime != 0 || digitalRead(RELAY_1) == HIGH) {
        Serial.println("Open switch released. RELAY_1 turned off.");
      }
      openPressedTime = 0;
      digitalWrite(RELAY_1, LOW);
    }
  }

  if (!relay2Task.active) {
    bool isClosePressed = digitalRead(PIN_CLOSE) == LOW;
    if (isClosePressed) {
      if (closePressedTime == 0) {
        closePressedTime = millis();
        Serial.println("Close switch pressed.");
      } else if (millis() - closePressedTime >= relay_delay) {
        if (digitalRead(RELAY_2) == LOW) {
          digitalWrite(RELAY_2, HIGH);
          Serial.println("RELAY_2 activated (Close).");
        }
      }
    } else {
      if (closePressedTime != 0 || digitalRead(RELAY_2) == HIGH) {
        Serial.println("Close switch released. RELAY_2 turned off.");
      }
      closePressedTime = 0;
      digitalWrite(RELAY_2, LOW);
    }
  }
}

void handleOptoInputs() {
  // Allow optocoupler to control relays ONLY if:
  // 1. Automatic mode is ON
  // 2. sensorActivated == 0
  if (isAutoModeOn() && sensorActivated == 0) {
    // Only control relays if not busy with a timed task
    if (!relay1Task.active) {
      if (digitalRead(OPTO_OPEN_PIN) == HIGH) {
        digitalWrite(RELAY_1, HIGH);
      } else {
        digitalWrite(RELAY_1, LOW);
      }
    }
    if (!relay2Task.active) {
      if (digitalRead(OPTO_CLOSE_PIN) == HIGH) {
        digitalWrite(RELAY_2, HIGH);
      } else {
        digitalWrite(RELAY_2, LOW);
      }
    }
  } else {
    // If either auto mode is off or sensorActivated is 1, ensure relays are not controlled by opto
    if (!relay1Task.active) digitalWrite(RELAY_1, LOW);
    if (!relay2Task.active) digitalWrite(RELAY_2, LOW);
  }
}

void receiveLoRaData() {
  // --- On every LoRa data receive, will upload to Firebase at the end ---
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  // Read packet header
  recipient = LoRa.read();
  sender = LoRa.read();
  incomingMsgId = LoRa.read();
  incomingLength = LoRa.read();

  // Read payload
  incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  // Check for intended recipient
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("Not for me. Ignoring packet.");
    return;
  }

  displayPacketInfo();       // Print header info
  parseSensorData(incoming); // Parse payload
  displaySensorData();       // Print sensor info

  // --- Upload to Firebase ---
  //sendToFirebase(); //disabled firebase
} // End receiveLoRaData


void parseSensorData(String data) {
  int start = 0;
  int sensorIndex = 0;

  while (start < data.length() && sensorIndex < MAX_SENSORS) {
    int end = data.indexOf(';', start);
    if (end == -1) end = data.length();

    String entry = data.substring(start, end);
    int firstComma = entry.indexOf(',');
    int secondComma = entry.indexOf(',', firstComma + 1);

    if (firstComma != -1 && secondComma != -1) {
      sensorNumber[sensorIndex] = entry.substring(0, firstComma);
      sensorType[sensorIndex] = entry.substring(firstComma + 1, secondComma);
      sensorValue[sensorIndex] = entry.substring(secondComma + 1).toFloat();
      sensorIndex++;
    }

    start = end + 1;
  }
}

void displaySensorData() {
//  Serial.println("=== Sensor Data ===");
//
//  for (int i = 0; i < MAX_SENSORS; i++) {
//    if (sensorNumber[i].length() > 0) {
//      String fullType;
//      decodeSensorType(sensorType[i], fullType);
//
//      Serial.print(sensorNumber[i]);
//      Serial.print(" - ");
//      Serial.print(fullType);
//      Serial.print(": ");
//      Serial.println(sensorValue[i]);
//    }
//  }
//
//  Serial.println("===================");
}

void decodeSensorType(String code, String &type) {
  if (code == "TE") type = "Temperature";
  else if (code == "HU") type = "Humidity";
  else if (code == "WS") type = "Wind Speed";
  else if (code == "WD") type = "Wind Direction";
  else if (code == "RN") type = "Rain";
  else type = "Unknown";
}

String degreesToCardinal(float deg) {
  static const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW", "N"};
  int idx = (int)((deg + 22.5) / 45.0);
  idx = idx % 8;
  return String(directions[idx]);
}

void displayPacketInfo() {
//  Serial.println("=== Packet Header Info ===");
//  Serial.print("Sender: 0x"); Serial.println(sender, HEX);
//  Serial.print("Recipient: 0x"); Serial.println(recipient, HEX);
//  Serial.print("Message ID: "); Serial.println(incomingMsgId);
//  Serial.print("Payload Length: "); Serial.println(incomingLength);
//  Serial.print("Raw Payload: "); Serial.println(incoming);
//  Serial.println("==========================");
}

const char manualControlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Greenhouse Control</title>
  <meta charset="UTF-8">
  <style>
body {
  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
  background-color: #f4f7f4;
  color: #2f4f2f;
  margin: 0;
  padding: 20px;
  text-align: center;
}

h3 {
  color: #2d5d34;
  margin-bottom: 10px;
}

button {
  background-color: #4CAF50;
  color: white;
  border: none;
  border-radius: 6px;
  padding: 10px 20px;
  font-size: 15px;
  cursor: pointer;
  transition: background-color 0.2s ease;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

button:hover {
  background-color: #3e8e41;
}

input, select {
  padding: 8px;
  font-size: 15px;
  border: 1px solid #b8d6b8;
  border-radius: 5px;
  width: 120px;
  transition: border-color 0.3s ease;
}

input:focus, select:focus {
  border-color: #4CAF50;
  outline: none;
}

.tab {
  display: inline-block;
  background-color: #e1efe1;
  padding: 10px 25px;
  border-radius: 8px 8px 0 0;
  cursor: pointer;
  margin-right: 5px;
  font-weight: 600;
  color: #345c34;
  border: 1px solid #b8d6b8;
  border-bottom: none;
  transition: background 0.3s ease;
  position: relative;
}

.tab.active {
  background-color: white;
  border-bottom: 1px solid white;
  font-weight: 700;
}

.tab#alarmsTab.active-alarms::after {
  content: '';
  position: absolute;
  top: 5px;
  right: 5px;
  width: 8px;
  height: 8px;
  background-color: red;
  border-radius: 50%;
}

.tab-content {
  border: 1px solid #b8d6b8;
  border-radius: 0 8px 8px 8px;
  background-color: white;
  padding: 30px;
  margin-top: -1px;
  display: none;
}

.tab-content.active {
  display: block;
}

.status-panel {
  margin-top: 30px;
  background-color: #ecf7ec;
  border-radius: 10px;
  padding: 20px 30px;
  box-shadow: 0 2px 6px rgba(0, 80, 0, 0.1);
  display: inline-block;
}

.sensor-grid {
  display: flex;
  flex-wrap: wrap;
  justify-content: center;
  gap: 15px;
  margin-top: 20px;
}

.sensor-box {
  background-color: #f2faf2;
  border: 1px solid #cce3cc;
  border-radius: 8px;
  width: 140px;
  padding: 15px;
  text-align: center;
  box-shadow: 0 1px 3px rgba(0, 80, 0, 0.1);
}

.sensor-pin {
  font-weight: bold;
  font-size: 16px;
  color: #2f7d32;
}

.sensor-type {
  font-size: 13px;
  color: #6a8c6a;
}

.sensor-value {
  font-size: 18px;
  font-weight: bold;
  color: #2d6e2d;
}

.settings-container {
  display: flex;
  flex-wrap: wrap;
  gap: 20px;
  justify-content: center;
  margin-top: 30px;
}

.settings-panel {
  background-color: #f9fdf9;
  border: 1px solid #cde8cd;
  border-radius: 12px;
  padding: 20px;
  box-shadow: 0 2px 8px rgba(0, 80, 0, 0.08);
  flex: 1;
  min-width: 280px;
  max-width: 400px;
}

#alarmsContent .settings-panel {
  margin: 0 auto;
  display: flex;
  flex-direction: column;
  align-items: center;
  min-width: 300px;
}

#alarmBoxContent {
  min-height: 40px;
  height: auto;
  color: red;
  font-weight: bold;
  width: 100%;
  text-align: center;
}

.settings-title {
  font-size: 17px;
  font-weight: bold;
  color: #2e5531;
  margin-bottom: 12px;
  text-align: left;
}

.settings-row-horizontal {
  display: flex;
  flex-wrap: wrap;
  gap: 20px;
  align-items: flex-end;
  margin-bottom: 15px;
}

.setting-col {
  display: flex;
  flex-direction: column;
  align-items: flex-start;
}

.setting-label {
  font-size: 14px;
  margin-bottom: 6px;
  color: #476547;
}

.centered-button {
  display: flex;
  justify-content: center;
  margin: 15px 0;
}

#activateBtn {
  width: 110px;
  background-color: #008cba;
  font-weight: bold;
}

#activateBtn:hover {
  background-color: #006f98;
}

#activateBtn[disabled] {
  background-color: #b8d6b8;
  cursor: not-allowed;
}

#syncBtn {
  background-color: #388e3c;
  font-weight: 600;
}

#syncBtn:hover:not(:disabled) {
  background-color: #2b6e2e;
}

#syncBtn:disabled {
  background-color: #b8d6b8;
  cursor: not-allowed;
}

#historicalAlarmList li {
  margin-bottom: 8px;
  font-size: 14px;
  color: #2f4f2f;
}

#alarmList li {
  margin-bottom: 8px;
  font-size: 14px;
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  background-color: #ffffff;
  padding: 10px 20px;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 80, 0, 0.1);
  margin-bottom: 10px;
}

.header-title {
  font-size: 20px;
  font-weight: 600;
  color: #2d5d34;
}

.time-display {
  font-size: 16px;
  font-weight: 500;
  color: #2f4f2f;
  background-color: #e1efe1;
  padding: 8px 15px;
  border-radius: 20px;
  box-shadow: 0 1px 3px rgba(0, 80, 0, 0.08);
  transition: all 0.3s ease;
}

@media (max-width: 600px) {
  .settings-panel {
    min-width: 100%;
  }
  .header {
    flex-direction: column;
    align-items: center;
    gap: 10px;
  }
  .time-display {
    font-size: 14px;
    padding: 6px 12px;
  }
}
  </style>
  <script>
    let activeTab = 'manual';
    let wifiDisconnected = false; // Track WiFi disconnection state

    // Poll auto mode and reload if mode changes
    setInterval(function(){fetch('/mode').then(r=>r.text()).then(m=>{if(m!=='1'){location.reload();}})},2000);

    function sendCommand(cmd) {
      const runtime = document.getElementById('runtime').value || 10;
      fetch(`/control?cmd=${cmd}&runtime=${runtime}`);
    }

    function showTab(tabName) {
      activeTab = tabName;
      fetch(`/setTab?tab=${tabName}`);
      document.querySelectorAll('.tab').forEach(tab => {
        tab.classList.remove('active');
      });
      document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.remove('active');
      });
      document.getElementById(tabName + 'Tab').classList.add('active');
      document.getElementById(tabName + 'Content').classList.add('active');
    }

    function pollStatus() {
      fetch('/status').then(res => res.json()).then(data => {
        document.getElementById('opName').textContent = data.operation;
        document.getElementById('opTime').textContent = data.remaining + ' s';
      });
    }

function pollSensorData() {
  Promise.all([
    fetch('/sensorData').then(res => res.json()),
    fetch('/getTime').then(res => res.json())
  ])
    .then(([sensorData, timeData]) => {
      const grid = document.getElementById('sensorGrid');
      grid.innerHTML = '';

      const alarmList = document.getElementById('alarmList');
      alarmList.innerHTML = '';

      // Maintain a map of active alarms with their timestamps
      if (!window.activeAlarmsMap) {
        window.activeAlarmsMap = new Map();
      }

      const highTempThresholdInput = document.getElementById('tempAlarmThreshold');
      const lowTempThresholdInput = document.getElementById('lowTempAlarmThreshold');
      const highTempThreshold = highTempThresholdInput ? parseFloat(highTempThresholdInput.value || 35) : 35;
      const lowTempThreshold = lowTempThresholdInput ? parseFloat(lowTempThresholdInput.value || 10) : 10;
      const windThreshold = parseFloat(document.getElementById('windAlarmThreshold').value || 40);
      const timestamp = timeData.datetime || 'Unknown';

      // Generate new alarms
      let newAlarms = [];

      sensorData.forEach(sensor => {
        if (sensor.type === "Unknown") return;

        if (sensor.type === "Temperature") {
          let tempValue = parseFloat(sensor.value.toString().replace(/[^\d.-]/g, ''));
          if (tempValue >= highTempThreshold) {
            let alarmKey = `HighTemp_${sensor.number}`;
            let alarmText = `[${timestamp}] High Temperature: ${tempValue}°C (Threshold: ≥${highTempThreshold}°C)`;
            if (window.activeAlarmsMap.has(alarmKey)) {
              // Reuse existing alarm text with original timestamp
              alarmText = window.activeAlarmsMap.get(alarmKey).text;
            } else {
              // Store new alarm with timestamp
              window.activeAlarmsMap.set(alarmKey, { text: alarmText, timestamp });
            }
            newAlarms.push(window.activeAlarmsMap.get(alarmKey).text);
          }
          if (tempValue <= lowTempThreshold) {
            let alarmKey = `LowTemp_${sensor.number}`;
            let alarmText = `[${timestamp}] Low Temperature: ${tempValue}°C (Threshold: ≤${lowTempThreshold}°C)`;
            if (window.activeAlarmsMap.has(alarmKey)) {
              alarmText = window.activeAlarmsMap.get(alarmKey).text;
            } else {
              window.activeAlarmsMap.set(alarmKey, { text: alarmText, timestamp });
            }
            newAlarms.push(window.activeAlarmsMap.get(alarmKey).text);
          }
        }

        if (sensor.type === "Wind Speed") {
          let windValue = parseFloat(sensor.value.toString().replace(/[^\d.-]/g, ''));
          if (windValue >= windThreshold) {
            let alarmKey = `HighWind_${sensor.number}`;
            let alarmText = `[${timestamp}] High Wind Speed: ${windValue} mph (Threshold: ≥${windThreshold} mph)`;
            if (window.activeAlarmsMap.has(alarmKey)) {
              alarmText = window.activeAlarmsMap.get(alarmKey).text;
            } else {
              window.activeAlarmsMap.set(alarmKey, { text: alarmText, timestamp });
            }
            newAlarms.push(window.activeAlarmsMap.get(alarmKey).text);
          }
        }
      });

      // WiFi disconnection alarm
      let wifiAlarmKey = `WiFi_Disconnected`;
      let wifiAlarmText = `[${timestamp}] WiFi Disconnected`;
      if (wifiDisconnected) {
        if (window.activeAlarmsMap.has(wifiAlarmKey)) {
          wifiAlarmText = window.activeAlarmsMap.get(wifiAlarmKey).text;
        } else {
          window.activeAlarmsMap.set(wifiAlarmKey, { text: wifiAlarmText, timestamp });
        }
        newAlarms.push(window.activeAlarmsMap.get(wifiAlarmKey).text);
      } else {
        window.activeAlarmsMap.delete(wifiAlarmKey);
      }

      // Update sensor grid
      sensorData.forEach(sensor => {
        if (sensor.type === "Unknown") return;
        const box = document.createElement('div');
        box.className = 'sensor-box';
        box.innerHTML = `
          <div class='sensor-pin'>${sensor.number}</div>
          <div class='sensor-type'>${sensor.type}</div>
          <div class='sensor-value'>${sensor.value}</div>
        `;
        grid.appendChild(box);
      });

      // Update alarms tab indicator
      const alarmsTab = document.getElementById('alarmsTab');
      if (newAlarms.length > 0) {
        alarmsTab.classList.add('active-alarms');
      } else {
        alarmsTab.classList.remove('active-alarms');
      }

      // Handle historical alarm recording
      if (newAlarms.length === 0 && window.prevActiveAlarms && window.prevActiveAlarms.length > 0) {
        const cleared = window.prevActiveAlarms.join(" || ");
        fetch(`/addHistoricalAlarm?msg=${encodeURIComponent(cleared)}`);
      }

      // Update active alarms list
      if (newAlarms.length === 0) {
        alarmList.innerHTML = '<li>No active alarms.</li>';
      } else {
        newAlarms.forEach(alarm => {
          const li = document.createElement('li');
          li.textContent = alarm;
          alarmList.appendChild(li);
        });
      }

      // Clean up resolved alarms
      const currentAlarmKeys = new Set();
      sensorData.forEach(sensor => {
        if (sensor.type === "Temperature") {
          let tempValue = parseFloat(sensor.value.toString().replace(/[^\d.-]/g, ''));
          if (tempValue >= highTempThreshold) {
            currentAlarmKeys.add(`HighTemp_${sensor.number}`);
          }
          if (tempValue <= lowTempThreshold) {
            currentAlarmKeys.add(`LowTemp_${sensor.number}`);
          }
        }
        if (sensor.type === "Wind Speed") {
          let windValue = parseFloat(sensor.value.toString().replace(/[^\d.-]/g, ''));
          if (windValue >= windThreshold) {
            currentAlarmKeys.add(`HighWind_${sensor.number}`);
          }
        }
      });
      if (wifiDisconnected) {
        currentAlarmKeys.add(wifiAlarmKey);
      }

      // Remove alarms that are no longer active
      for (let key of window.activeAlarmsMap.keys()) {
        if (!currentAlarmKeys.has(key)) {
          window.activeAlarmsMap.delete(key);
        }
      }

      window.prevActiveAlarms = [...newAlarms];

      // Fetch and render historical alarms
      fetch('/getHistoricalAlarms')
        .then(res => res.json())
        .then(data => {
          const list = document.getElementById('historicalAlarmList');
          list.innerHTML = '';
          if (data.length === 0 || data.every(a => a === "")) {
            list.innerHTML = '<li>No historical alarms yet.</li>';
          } else {
            data.forEach(a => {
              if (a) {
                const li = document.createElement('li');
                li.textContent = a;
                list.appendChild(li);
              }
            });
          }
        });
    })
    .catch(err => {
      console.error("Failed to poll sensor data or time:", err);
    });
}
    let ws;
    function setupWS() {
      ws = new WebSocket('ws://' + window.location.hostname + '/ws');
      ws.onmessage = function(event) {
        try {
          const msg = JSON.parse(event.data);
          if (msg.type === 'tempsettings') {
            document.getElementById('setpoint').value = msg.setpoint;
            document.getElementById('hysteresis').value = msg.hysteresis;
            document.getElementById('maxpos').value = msg.maxpos;
            document.getElementById('time').value = msg.time;
            document.getElementById('runtime').value = msg.ventruntime;
          } else if (msg.type === 'activation') {
            activationState = msg.activated;
            updateActivateBtn();
          } else if (msg.type === 'windsettings') {
            document.getElementById('vent1_dir').value = msg.vent1_dir;
            document.getElementById('vent2_dir').value = msg.vent2_dir;
            document.getElementById('lee_maxpos').value = msg.lee_maxpos;
            document.getElementById('maxspeed_setpoint').value = msg.maxspeed_setpoint;
            document.getElementById('wind_maxpos').value = msg.wind_maxpos;
            document.getElementById('rain_v1_limit').value = msg.rain_v1_limit;
            document.getElementById('rain_v2_limit').value = msg.rain_v2_limit;
          } else if (msg.type === 'sync_ack') {
            document.getElementById('syncBtn').disabled = false;
          }else if (msg.type === 'alarmsettings') {
          document.getElementById('tempAlarmThreshold').value = msg.high;
          document.getElementById('lowTempAlarmThreshold').value = msg.low;
          document.getElementById('windAlarmThreshold').value = msg.wind;
        }

        } catch (e) {}
      };
    }

    function sendTempSettings() {
      if (!ws || ws.readyState !== 1) return;
      const msg = {
        type: 'tempsettings',
        setpoint: parseFloat(document.getElementById('setpoint').value),
        hysteresis: parseFloat(document.getElementById('hysteresis').value),
        maxpos: parseFloat(document.getElementById('maxpos').value),
        time: parseFloat(document.getElementById('time').value),
        ventruntime: parseInt(document.getElementById('runtime').value),
      };
      ws.send(JSON.stringify(msg));
    }

    function sendWindSettings() {
      if (!ws || ws.readyState !== 1) return;
      const msg = {
        type: 'windsettings',
        vent1_dir: document.getElementById('vent1_dir').value,
        vent2_dir: document.getElementById('vent2_dir').value,
        lee_maxpos: parseInt(document.getElementById('lee_maxpos').value),
        maxspeed_setpoint: parseInt(document.getElementById('maxspeed_setpoint').value),
        wind_maxpos: parseInt(document.getElementById('wind_maxpos').value),
        rain_v1_limit: parseInt(document.getElementById('rain_v1_limit').value),
        rain_v2_limit: parseInt(document.getElementById('rain_v2_limit').value)
      };
      ws.send(JSON.stringify(msg));
    }

    function clearHistoricalAlarms() {
      fetch('/clearHistoricalAlarms')
        .then(res => res.text())
        .then(() => {
          const list = document.getElementById('historicalAlarmList');
          list.innerHTML = '<li>No historical alarms yet.</li>';
        })
        .catch(err => {
          alert("Failed to clear historical alarms");
          console.error(err);
        });
    }

    let activationState = 0;
    function toggleActivation() {
      if (!ws || ws.readyState !== 1) return;
      let newState = activationState ? 0 : 1;
      ws.send(JSON.stringify({type:'activation',activated:newState}));
    }

    function updateActivateBtn() {
      const btn = document.getElementById('activateBtn');
      if (!btn) return;
      btn.textContent = activationState ? 'Disable' : 'Activate';
      btn.style.background = activationState ? '#c33' : '#69c';
    }

    setInterval(function(){
      fetch('/mode').then(r=>r.text()).then(m=>{
        if(m!=='1') {
          activationState = 0;
          updateActivateBtn();
        }
      })
    },2000);

    function debounce(func, wait) {
      let timeout;
      return function(...args) {
        clearTimeout(timeout);
        timeout = setTimeout(() => func.apply(this, args), wait);
      };
    }

    function sendAlarmSettings() {
      if (!ws || ws.readyState !== 1) return;
      const msg = {
        type: 'alarmsettings',
        high: parseFloat(document.getElementById('tempAlarmThreshold').value),
        low: parseFloat(document.getElementById('lowTempAlarmThreshold').value),
        wind: parseFloat(document.getElementById('windAlarmThreshold').value)
      };
      ws.send(JSON.stringify(msg));
    }


    function saveNetworkSettings() {
      const ssid = document.getElementById('ssidInput').value;
      const pass = document.getElementById('passInput').value;

      fetch(`/saveNetwork?ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}`)
        .then(res => {
          if (!res.ok) throw new Error("Server error");
          return res.text();
        })
        .then(() => {
          alert("Settings saved. Device will now reboot.");
          return fetch('/reboot');
        })
        .catch(err => {
          alert("Failed to save settings.");
          console.error(err);
        });
    }

    function loadIPAddresses() {
      fetch('/getIP')
        .then(res => res.json())
        .then(data => {
          console.log("Fetched IPs:", data);
          document.getElementById('apIP').textContent = data.ap;
          document.getElementById('staIP').textContent = data.sta;
        })
        .catch(err => {
          console.error('Failed to fetch IP addresses', err);
        });
    }

    function updateWiFiStatus() {
      fetch('/wifiStatus')
        .then(res => res.json())
        .then(data => {
          const statusEl = document.getElementById('wifiStatus');
          statusEl.textContent = data.status;

          if (data.status === "Connected") {
            statusEl.style.color = "green";
            wifiDisconnected = false; // WiFi is connected, no alarm
          } else {
            statusEl.style.color = "red";
            wifiDisconnected = true; // WiFi is disconnected, trigger alarm
          }
        })
        .catch(err => {
          console.error("Failed to fetch WiFi status", err);
          const statusEl = document.getElementById('wifiStatus');
          statusEl.textContent = "Unknown";
          statusEl.style.color = "gray";
          wifiDisconnected = true; // Treat fetch failure as disconnected
        });
    }

    function fetchCurrentTime() {
      fetch('/getTime')
        .then(res => res.json())
        .then(data => {
          document.getElementById('currentTime').textContent = data.datetime || 'Unavailable';
          document.getElementById('headerTime').textContent = data.datetime || 'Time not set';
        })
        .catch(() => {
          document.getElementById('currentTime').textContent = 'Error';
          document.getElementById('headerTime').textContent = 'Time not set';
        });
    }

    function updateTime() {
      const date = document.getElementById('dateInput').value;
      const time = document.getElementById('timeInput').value;
      if (!date || !time) {
        alert("Please select both date and time.");
        return;
      }

      fetch(`/setTime?datetime=${encodeURIComponent(date + 'T' + time)}`)
        .then(res => res.text())
        .then(msg => {
          alert(msg);
          fetchCurrentTime(); // refresh
        })
        .catch(err => {
          alert("Failed to set time");
          console.error(err);
        });
    }

    window.onload = function() {
      showTab('manual');
      pollStatus();
      pollSensorData();
      setupWS();
      setInterval(pollStatus, 1000);
      setInterval(pollSensorData, 1000);
      loadIPAddresses();
      updateWiFiStatus();
      fetchCurrentTime();
      setInterval(fetchCurrentTime, 10000); // update every 10 sec
      setInterval(updateWiFiStatus, 5000);
      fetch('/settings').then(res => res.json()).then(data => {
        document.getElementById('runtime').value = data.ventruntime;
        document.getElementById('runtime').disabled = false;
      });
      document.getElementById('runtime').addEventListener('input', debounce(function() {
        sendTempSettings();
      }, 300));
      document.getElementById('vent1_dir').onchange = sendWindSettings;
      document.getElementById('vent2_dir').onchange = sendWindSettings;
      document.getElementById('lee_maxpos').onchange = sendWindSettings;
      document.getElementById('maxspeed_setpoint').onchange = sendWindSettings;
      document.getElementById('wind_maxpos').onchange = sendWindSettings;
      document.getElementById('rain_v1_limit').onchange = sendWindSettings;
      document.getElementById('rain_v2_limit').onchange = sendWindSettings;
      document.getElementById('tempAlarmThreshold').onchange = sendAlarmSettings;
      document.getElementById('lowTempAlarmThreshold').onchange = sendAlarmSettings;
      document.getElementById('windAlarmThreshold').onchange = sendAlarmSettings;
      document.getElementById('syncBtn').onclick = function() {
        if (!ws || ws.readyState !== 1) return;
        ws.send(JSON.stringify({type:'sync'}));
        document.getElementById('syncBtn').disabled = true;
        setTimeout(()=>{document.getElementById('syncBtn').disabled = false;}, 15000);
      };
    };
  </script>
</head>
<body>
  <div class="header">
      <div class="header-title">Greenhouse Control</div>
      <div class="time-display" id="headerTime">Loading...</div>
    </div>
  <div>
    <span id="manualTab" class="tab active" onclick="showTab('manual')">Manual Control</span>
    <span id="sensorTab" class="tab" onclick="showTab('sensor')">Sensor Data</span>
    <span id="networkTab" class="tab" onclick="showTab('network')">General Settings</span>
    <span id="alarmsTab" class="tab" onclick="showTab('alarms')">Alarms</span>
  </div>

  <div id="manualContent" class="tab-content active">
    <div style='text-align: center;'>
      <h3>Manual Control</h3>
      <label>Vent Runtime (sec): </label>
      <input type='number' id='runtime' value='' disabled><br><br>

      <button onclick="sendCommand('calibrate')">Calibration</button><br><br>

      <div>
        <button onclick="sendCommand('open')">Open</button>
        <button onclick="sendCommand('close')">Close</button>
      </div><br>

      <div>
        <button onclick="sendCommand('open75')">75% Open</button>
        <button onclick="sendCommand('close75')">75% Close</button>
      </div><br>

      <div>
        <button onclick="sendCommand('open50')">50% Open</button>
        <button onclick="sendCommand('close50')">50% Close</button>
      </div><br>

      <div>
        <button onclick="sendCommand('open25')">25% Open</button>
        <button onclick="sendCommand('close25')">25% Close</button>
      </div>

      <div class="status-panel">
        <h3>Current Operation</h3>
        <div>Operation: <span id="opName">Idle</span></div>
        <div>Time Remaining: <span id="opTime">0 s</span></div>
      </div>
    </div>
  </div>

<div id="sensorContent" class="tab-content">
  <h3>Sensor Data</h3>

  <!-- Activate Button -->
  <div class="centered-button">
    <button id="activateBtn" style="width:110px; background:#69c; color:white; font-weight:bold; border:none; cursor:pointer;" onclick="toggleActivation()">Activate</button>
  </div>

  <!-- Sensor Grid -->
  <div class="sensor-grid" id="sensorGrid"></div>

  <!-- Settings Panels Grid -->
  <div class="settings-container">
    
    <!-- Temperature Settings Panel -->
    <div class="settings-panel">
      <div class="settings-title">Temperature</div>
      <div class="settings-row-horizontal">
        <div class="setting-col">
          <div class="setting-label">Set point</div>
          <input class="setting-input" id="setpoint" type="number" step="0.1" onchange="sendTempSettings()">
        </div>
        <div class="setting-col">
          <div class="setting-label">Hysteresis</div>
          <input class="setting-input" id="hysteresis" type="number" step="0.1" onchange="sendTempSettings()">
        </div>
        <div class="setting-col">
          <div class="setting-label">Pulse %</div>
          <input class="setting-input" id="maxpos" type="number" step="0.1" onchange="sendTempSettings()">
        </div>
        <div class="setting-col">
          <div class="setting-label">Pulse Delay</div>
          <input class="setting-input" id="time" type="number" step="0.1" onchange="sendTempSettings()">
        </div>
      </div>
    </div>

    <!-- Wind Direction Settings Panel -->
    <div class="settings-panel">
      <div class="settings-title">Wind Direction</div>
      <div class="settings-row-horizontal">
        <div class="setting-col">
          <div class="setting-label">Vent 1 Direction</div>
          <select class="setting-input" id="vent1_dir" onchange="sendWindSettings()">
            <option value="N">N</option><option value="NE">NE</option><option value="E">E</option><option value="SE">SE</option>
            <option value="S">S</option><option value="SW">SW</option><option value="W">W</option><option value="NW">NW</option>
          </select>
        </div>
        <div class="setting-col">
          <div class="setting-label">Vent 2 Direction</div>
          <select class="setting-input" id="vent2_dir" onchange="sendWindSettings()">
            <option value="N">N</option><option value="NE">NE</option><option value="E">E</option><option value="SE">SE</option>
            <option value="S">S</option><option value="SW">SW</option><option value="W">W</option><option value="NW">NW</option>
          </select>
        </div>
      </div>
    </div>

    <!-- Wind Speed Settings Panel -->
    <div class="settings-panel">
      <div class="settings-title">Wind Speed</div>
      <div class="settings-row-horizontal">
        <div class="setting-col">
          <div class="setting-label">Lee Side Max Position</div>
          <input class="setting-input" id="lee_maxpos" type="number" step="1" min="0" max="100" value="100" onchange="sendWindSettings()">
        </div>
        <div class="setting-col">
          <div class="setting-label">Max Speed Setpoint</div>
          <input class="setting-input" id="maxspeed_setpoint" type="number" step="1" min="0" value="25" onchange="sendWindSettings()">
        </div>
        <div class="setting-col">
          <div class="setting-label">Wind Side Max Position</div>
          <input class="setting-input" id="wind_maxpos" type="number" step="1" min="0" max="100" value="100" onchange="sendWindSettings()">
        </div>
      </div>
    </div>

    <!-- Rain Sensor Vent Limits Panel -->
    <div class="settings-panel">
      <div class="settings-title">Rain Sensor Vent Limits</div>
      <div class="settings-row-horizontal">
        <div class="setting-col">
          <div class="setting-label">Vent 1 Limit</div>
          <input class="setting-input" id="rain_v1_limit" type="number" step="1" min="0" max="100" value="100" onchange="sendWindSettings()">
        </div>
        <div class="setting-col">
          <div class="setting-label">Vent 2 Limit</div>
          <input class="setting-input" id="rain_v2_limit" type="number" step="1" min="0" max="100" value="100" onchange="sendWindSettings()">
        </div>
      </div>
    </div>

    <!-- Alarm Settings Panel -->
    <div class="settings-panel">
      <div class="settings-title">Alarm Settings</div>
      <div class="settings-row-horizontal">
        <div class="setting-col">
          <div class="setting-label">High Temperature Threshold (≥°C)</div>
          <input class="setting-input" id="tempAlarmThreshold" type="number" step="0.1" value="35.0">
        </div>
        <div class="setting-col">
          <div class="setting-label">Low Temperature Threshold (≤°C)</div>
          <input class="setting-input" id="lowTempAlarmThreshold" type="number" step="0.1" value="10.0">
        </div>
        <!-- ADD BELOW -->
        <div class="setting-col">
          <div class="setting-label">Max Wind Speed Alarm (mph)</div>
          <input class="setting-input" id="windAlarmThreshold" type="number" step="0.1" value="40.0">
        </div>
      </div>
    </div>

  </div> <!-- End of .settings-container -->

  <!-- Vent Status Section -->
  <div style="margin-top: 16px; display: flex; flex-direction: column; align-items: center;">
    <button id="syncBtn" style="margin-bottom: 16px;">Sync</button>
    <div id="ventStatusPanel" style="padding: 12px; background: #ffffff; border: 1px solid #a4cfa1; border-radius: 8px; min-width: 280px; text-align: left;">
      <div><b>Current Operation:</b> <span id="currentOperation">Idle</span></div>
      <div><b>Vent 1 Position:</b> <span id="vent1Pos">0</span>%</div>
      <div><b>Vent 2 Position:</b> <span id="vent2Pos">0</span>%</div>
    </div>
  </div>

  <!-- Vent Polling Script -->
  <script>
    function pollVentStatus() {
      fetch('/ventStatus').then(res => res.json()).then(data => {
        document.getElementById('currentOperation').textContent = data.operation;
        document.getElementById('vent1Pos').textContent = data.vent1;
        document.getElementById('vent2Pos').textContent = data.vent2;
      });
    }
    setInterval(pollVentStatus, 1000);
    pollVentStatus();
  </script>
</div>

<div id="networkContent" class="tab-content">
  <div class="settings-container">
    <!-- WiFi Network Settings Box -->
    <div class="settings-panel">
      <h3 style="align-self: center; margin-bottom: 10px;">WiFi Network Settings</h3>
      <div style="display: flex; width: 100%;">
        <label style="width: 100px;">SSID:</label>
        <input id="ssidInput" type="text" style="flex: 1;">
      </div>
      <div style="display: flex; width: 100%; margin-top: 15px;">
        <label style="width: 100px;">Password:</label>
        <input id="passInput" type="password" style="flex: 1;">
      </div>
      <div style="width: 100%; text-align: center; margin-top: 15px;">
        <button onclick="saveNetworkSettings()">Save</button>
      </div>
      <div style="width: 100%; text-align: center; margin-top: 15px;">
        <div>
          <strong>WiFi Status:</strong>
          <span id="wifiStatus" style="font-weight: bold;">Checking...</span>
        </div>
        <div style="margin-top: 8px;">
          <h4 style="margin: 5px 0;">Current IP Addresses</h4>
          <div>Access Point (AP): <span id="apIP">Loading...</span></div>
          <div>WiFi (STA): <span id="staIP">Loading...</span></div>
        </div>
      </div>
    </div>

    <!-- Time Settings Box -->
    <div class="settings-panel">
      <h3 style="align-self: center; margin-bottom: 10px;">Time Settings</h3>
      <div style="margin-bottom: 10px;">
        <b>Current Time:</b> <span id="currentTime">Loading...</span>
      </div>
      <div style="display: flex; width: 100%;">
        <label style="width: 100px;">Date:</label>
        <input id="dateInput" type="date" style="flex: 1; padding: 6px;">
      </div>
      <div style="display: flex; width: 100%; margin-top: 15px;">
        <label style="width: 100px;">Time:</label>
        <input id="timeInput" type="time" style="flex: 1; padding: 6px;">
      </div>
      <div style="width: 100%; text-align: center; margin-top: 15px;">
        <button onclick="updateTime()" style="background-color: #388e3c; color: white; padding: 8px 16px; border: none; border-radius: 5px; font-weight: bold;">
          Set Time
        </button>
      </div>
    </div>
  </div>
</div>

<div id="alarmsContent" class="tab-content">
  <div class="settings-panel">

    <!-- Active Alarms -->
    <div class="settings-title">Active Alarms</div>
    <div id="alarmBoxContent">
      <ul id="alarmList" style="list-style: none; padding-left: 0; margin: 0;">
        <li>No active alarms.</li>
      </ul>
    </div>

    <!-- Historical Alarms (placed outside alarmBoxContent) -->
    <div class="settings-title" style="margin-top: 20px;">Historical Alarms</div>
    <div id="historicalBoxContent">
      <ul id="historicalAlarmList" style="list-style: none; padding-left: 0; margin: 0;">
        <li>No historical alarms yet.</li>
      </ul>
    </div>
      <div style="text-align: center; margin-top: 20px;">
        <button onclick="clearHistoricalAlarms()" style="background-color: #c33; color: white; padding: 10px 20px; border: none; border-radius: 6px; font-weight: bold; cursor: pointer;">
          Clear History
        </button>
      </div>
  </div>
</div>


</body>
</html>
)rawliteral";

String generateManualPage(bool autoMode) {
  // autoMode is always true here, so just return the template
  return String(manualControlPage);
}


void setup() {

  config.database_url = FIREBASE_HOST;
  auth.user.email = "";
  auth.user.password = "";
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.begin(115200);
  while (!Serial);

// Set timezone to BST (UTC+1)
  setenv("TZ", "BST-1", 1); // BST is UTC+1, negative offset for POSIX convention
  tzset();

  // Restore saved time from Preferences
  prefs.begin("timeset", true);
  time_t savedTime = prefs.getLong("last_time", 0);
  prefs.end();
  if (savedTime != 0) {
    struct timeval now = { savedTime, 0 };
    settimeofday(&now, nullptr);
    Serial.println("[Time] Restored saved time from Preferences (BST adjusted).");
  } else {
    Serial.println("[Time] No saved time found in Preferences. Waiting for user to set time.");
  }


  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password);
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.println("[WiFi] AP + STA mode enabled");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("STA IP: ");
  Serial.println(WiFi.localIP());


prefs.begin("wifi", true);
String savedSsid = prefs.getString("ssid", wifi_ssid);
String savedPass = prefs.getString("pass", wifi_password);
prefs.end();

WiFi.mode(WIFI_AP_STA);
WiFi.softAP(ap_ssid, ap_password);
WiFi.begin(savedSsid.c_str(), savedPass.c_str());

String latestVersion = getLatestVersion();
if (isNewVersion(latestVersion)) {
  Serial.printf("New version available: %s (current: %s)\n", latestVersion.c_str(), CURRENT_VERSION);
  performOTAUpdate();  // This will reboot the ESP32 after update
} else {
  Serial.println("Firmware is up-to-date.");
}


  
  if (!MDNS.begin("greenhouse")) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.println("mDNS responder started: http://greenhouse.local");
  }

  loadTempSettings();
  loadSensorActivated();
  loadHistoricalAlarms();

  ws.onEvent(onWsEvent);
  setupWebServer();

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check connections.");
    while (1);
  }
  Serial.println("LoRa initialized.");
  setupRelaysAndSwitch();
}


void sendToFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Firebase] Skipped upload: WiFi not connected");
    return;
  }

  FirebaseJson json;
  json.set("timestamp", (int)time(nullptr));
  json.set("vent1_position", ventPosition);
  json.set("vent2_position", vent2Position);

  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensorNumber[i].length() > 0) {
      String path = String("sensors/") + sensorNumber[i] + "_" + sensorType[i];
      json.set(path, sensorValue[i]);
    }
  }

  if (Firebase.RTDB.setJSON(&fbdo, "/greenhouse/data", &json)) {
    Serial.println("[Firebase] Upload success");
  } else {
    Serial.print("[Firebase] Upload failed: "); Serial.println(fbdo.errorReason());
  }

  char timeStr[32];
  time_t now = time(nullptr);
  struct tm timeinfo;
  if (localtime_r(&now, &timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H-%M-%S", &timeinfo);
  } else {
    snprintf(timeStr, sizeof(timeStr), "%d", (int)now);
  }
  String logPath = String("/greenhouse/logs/") + String(timeStr);
  if (Firebase.RTDB.setJSON(&fbdo, logPath.c_str(), &json)) {
    Serial.println("[Firebase] Backup log upload success");
  } else {
    Serial.print("[Firebase] Backup log upload failed: "); Serial.println(fbdo.errorReason());
  }
}


void loop() {

// --- Handle time synchronization, drift correction, and saving ---
  static unsigned long lastTimeSave = 0;
  const unsigned long timeSaveInterval = 120000; // Save every 2 minutes
  static unsigned long lastDriftCorrection = 0;
  const unsigned long driftCorrectionInterval = 3600000; // Correct drift every 1 hour
  const long driftCorrectionSeconds = 120; // Add 120s per hour (based on 6 min drift over 3 hrs)
  static unsigned long lastNtpCheck = 0 - 3600000; // Force initial NTP check shortly after boot
  const unsigned long ntpCheckInterval = 3600000; // Check NTP every 1 hour

  // Periodically check WiFi status for debugging
  static unsigned long lastWiFiCheck = 0;
  const unsigned long wifiCheckInterval = 60000; // Check every 1 minute
  if (millis() - lastWiFiCheck > wifiCheckInterval) {
    Serial.print("[WiFi] Debug: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    lastWiFiCheck = millis();
  }

  // Save current time to Preferences (UTC timestamp)
  if (millis() - lastTimeSave > timeSaveInterval) {
    time_t now;
    time(&now);
    if (now >= 946684800) {
      prefs.begin("timeset", false);
      prefs.putLong("last_time", now);
      prefs.end();
      Serial.println("[Time] Current time saved to Preferences (UTC).");
    }
    lastTimeSave = millis();
  }

  // Attempt NTP sync when WiFi is connected
  if (WiFi.status() == WL_CONNECTED && millis() - lastNtpCheck > ntpCheckInterval) {
    Serial.println("[Time] Debug: NTP check triggered. millis() = " + String(millis()) + ", lastNtpCheck = " + String(lastNtpCheck));
    Serial.println("[Time] Attempting NTP sync...");
    // Configure NTP with BST offset (3600s = 1 hour)
    configTime(3600, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    time_t now;
    for (int i = 0; i < 10; i++) {
      delay(1500); // Wait 1.5s between retries
      time(&now);
      if (now >= 946684800) {
        Serial.println("[Time] NTP sync successful (BST adjusted).");
        prefs.begin("timeset", false);
        prefs.putLong("last_time", now);
        prefs.end();
        break;
      }
    }
    if (now < 946684800) {
      Serial.println("[Time] NTP sync failed.");
    }
    lastNtpCheck = millis();
  }

  // Apply drift correction only when WiFi is disconnected
  if (WiFi.status() != WL_CONNECTED && millis() - lastDriftCorrection > driftCorrectionInterval) {
    time_t now;
    time(&now);
    if (now >= 946684800) {
      struct timeval tv;
      tv.tv_sec = now + driftCorrectionSeconds;
      tv.tv_usec = 0;
      settimeofday(&tv, nullptr);
      Serial.println("[Time] Applied drift correction: added " + String(driftCorrectionSeconds) + " seconds.");
      prefs.begin("timeset", false);
      prefs.putLong("last_time", tv.tv_sec);
      prefs.end();
    }
    lastDriftCorrection = millis();
  }


  // --- Handle BOOT button for mode switch ---
  static bool lastBtnState = HIGH;
  bool btnState = digitalRead(BOOT_BTN);
  static bool switching = false;
  static unsigned long ledOnTime = 0;

  if (!switching) {
    if (btnState == LOW && lastBtnState == HIGH) {
      // Button just pressed
      buttonPressStart = millis();
      buttonHeld = true;
      digitalWrite(MODE_LED, HIGH);
      ledOnTime = millis();
    } else if (btnState == LOW && buttonHeld) {
      // Button held
      if (millis() - buttonPressStart >= 10000) {
        // 10s held, switch mode
        switching = true;
        buttonHeld = false;
        // Toggle mode
        if (isAPMode) {
          // Switch to WiFi client mode
          WiFi.softAPdisconnect(true);
          WiFi.mode(WIFI_STA);
          WiFi.begin(wifi_ssid, wifi_password);
          Serial.println("[WiFi] Switching to WiFi client mode...");
          // Optional: Wait for connection in background
          isAPMode = false;
        } else {
          // Switch to AP mode
          WiFi.disconnect(true);
          WiFi.mode(WIFI_AP);
          WiFi.softAP(ap_ssid, ap_password);
          Serial.println("[WiFi] Switching to Access Point mode...");
          isAPMode = true;
        }
        // Keep LED on for 1s after switch
        ledOnTime = millis();
      }
    } else if (btnState == HIGH && lastBtnState == LOW) {
      // Button released
      buttonHeld = false;
      if (!switching) digitalWrite(MODE_LED, LOW);
    }
  }
  // After switching, turn off LED 1s later
  if (switching && millis() - ledOnTime > 1000) {
    digitalWrite(MODE_LED, LOW);
    switching = false;
  }
  lastBtnState = btnState;

  // --- Existing loop logic ---
  if (syncRequested && sensorActivated) handleSyncRequest();
  processSyncMoveToTargets();
  if (sensorActivated == 1) {
    handleSensorRelayTask(relay1Task);
    handleSensorRelayTask(relay2Task);
    handleVent2RelayTask(RELAY_3, relay3TaskActive, relay3Start, relay3Duration);
    handleVent2RelayTask(RELAY_4, relay4TaskActive, relay4Start, relay4Duration);
  } else {
    handleRelayTask(relay1Task);
    handleRelayTask(relay2Task);
    handleVent2RelayTask(RELAY_3, relay3TaskActive, relay3Start, relay3Duration);
    handleVent2RelayTask(RELAY_4, relay4TaskActive, relay4Start, relay4Duration);
  }
  receiveLoRaData();
  handleRotarySwitch();
  handleOptoInputs();
  // Reset activation state if auto mode is off
  static int prevAuto = 1;
  int nowAuto = isAutoModeOn() ? 1 : 0;
  if (prevAuto && !nowAuto && sensorActivated) {
    sensorActivated = 0;
    saveSensorActivated(0);
  }
  prevAuto = nowAuto;

  // --- Sensor-based control logic ---
  checkSensorControlActivation(ventRuntime);
  if (sensorActivated == 1) {
    float temp = getLatestTemperature();
    sensorControlLoop(temp, ventRuntime);
    vent2SensorControlLoop(temp, ventRuntime);
    enforceWindLimits(ventRuntime); // <--- Wind safety correction
  }
}

// Helper: Clamp value
float clamp(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

// --- Sensor Control Logic ---
void sensorControlLoop(float currentTemp, int ventRuntime) {
  // Only run if sensorActivated==1 and calibration done
  if (sensorActivated != 1 || sensorControlState != 2) return;
  static unsigned long lastActionTime = 0;
  static bool waiting = false;
  static int lastAction = 0; // 1=open, 2=close, 0=none
  static unsigned long stepDuration = 0;
  float minT = tempSetPoint - tempHysteresis;
  float maxT = tempSetPoint + tempHysteresis;
  unsigned long now = millis();
  unsigned long maxStep = (unsigned long)(ventRuntime * (tempMaxPosition / 100.0) * 1000.0);
  unsigned long pauseTime = (unsigned long)(tempTime * 1000.0);

  float currentPos = ventPosition;
  float fullTravelMs = ventRuntime * 1000.0;

  // Wind/Lee side logic
  String vent1Side = "", vent2Side = "";
  determineWindLeeSide(vent1Side, vent2Side);
  float windSpeed = getLatestWindSpeed();
  int windLimit = (vent1Side == "Wind" && windSpeed > maxSpeedSetpoint) ? windMaxPosition : ((vent1Side == "Lee" && windSpeed > maxSpeedSetpoint) ? leeMaxPosition : 100);

  // --- Rain Emergency Priority ---
  if (isRaining()) {
    windLimit = min(windLimit, rainVent1Limit);
    if (ventPosition > rainVent1Limit && !relay1Task.active && !relay2Task.active) {
      // Instantly close to rain limit, highest priority
      float percentToClose = ventPosition - rainVent1Limit;
      unsigned long closeMs = (unsigned long)(fullTravelMs * (percentToClose / 100.0));
      if (closeMs > 0) {
        activateSensorRelay(RELAY_2, closeMs, relay2Task);
        totalCloseTime += closeMs;
        ventPosition = rainVent1Limit;
        currentOperation = "Closing (Rain)";
        Serial.printf("[RainEmergency] Closing Vent 1 to rain limit %d%% (was %.1f%%)\n", rainVent1Limit, ventPosition + percentToClose);
      }
      // Do not process any open/close requests until at limit
      return;
    }
    // If at or below rain limit, allow normal logic but cap all openings to rain limit
    windLimit = rainVent1Limit;
  }

  if (waiting) {
    if (now - lastActionTime >= pauseTime) {
      waiting = false;
      lastAction = 0;
      if (!relay1Task.active && !relay2Task.active) currentOperation = "Idle";
    } else {
      return;
    }
  }
  // Only act if not waiting
  if (currentTemp > maxT) {
    // Open vent
    if (!relay1Task.active && !relay2Task.active) {
      if (currentPos >= windLimit) {
        if (!vent1MaxMsgShown) {
          Serial.printf("[SensorCtrl] Vent 1 at limit (%.0f%%, %s side)%s\n", (float)windLimit, vent1Side.c_str(), isRaining() ? " [Rain]" : "");
          vent1MaxMsgShown = true;
        }
        vent1MinMsgShown = false;
        ventPosition = windLimit;
        if ((vent1Side == "Wind" && windSpeed > maxSpeedSetpoint) || (vent1Side == "Lee" && windSpeed > maxSpeedSetpoint)) {
          currentOperation = "Idle";
        }
        return;
      }
      vent1MaxMsgShown = false;
      float percentToOpen = windLimit - currentPos;
      if (percentToOpen <= 0) return;
      unsigned long openMs = (unsigned long)(fullTravelMs * (percentToOpen / 100.0));
      if (openMs > maxStep) openMs = maxStep;
      activateSensorRelay(RELAY_1, openMs, relay1Task);
      totalOpenTime += openMs;
      lastActionTime = now;
      waiting = true;
      lastAction = 1;
      if ((vent1Side == "Wind" && windSpeed > maxSpeedSetpoint) || (vent1Side == "Lee" && windSpeed > maxSpeedSetpoint)) {
        currentOperation = "Opening (Wind)";
      } else {
        currentOperation = "Opening (Temperature)";
      }
      // Update position
      ventPosition = clamp((totalOpenTime - totalCloseTime) / fullTravelMs * 100.0, 0.0, 100.0);
      if (ventPosition > windLimit) ventPosition = windLimit;
      Serial.printf("[SensorCtrl] Vent 1 opened for %.2fs, position: %.1f%% (%s side, windSpeed: %.1f)%s\n", openMs/1000.0, ventPosition, vent1Side.c_str(), windSpeed, isRaining() ? " [Rain]" : "");
      if (ventPosition >= windLimit && !vent1MaxMsgShown) {
        Serial.printf("[SensorCtrl] Vent 1 reached limit (%.0f%%) due to wind/rain.\n", (float)windLimit);
        vent1MaxMsgShown = true;
      }
      vent1MinMsgShown = false;
    }
  } else if (currentTemp < minT) {
    // Close vent
    if (!relay1Task.active && !relay2Task.active) {
      if (currentPos <= 0.0) {
        if (!vent1MinMsgShown) {
          Serial.println("[SensorCtrl] Vent 1 already at minimum position (0%)");
          vent1MinMsgShown = true;
        }
        vent1MaxMsgShown = false;
        ventPosition = 0.0;
        return;
      }
      vent1MinMsgShown = false;
      float percentToClose = currentPos;
      unsigned long closeMs = (unsigned long)(fullTravelMs * (percentToClose / 100.0));
      if (closeMs > maxStep) closeMs = maxStep;
      activateSensorRelay(RELAY_2, closeMs, relay2Task);
      totalCloseTime += closeMs;
      lastActionTime = now;
      waiting = true;
      lastAction = 2;
      if ((vent1Side == "Wind" && windSpeed > maxSpeedSetpoint) || (vent1Side == "Lee" && windSpeed > maxSpeedSetpoint)) {
        currentOperation = "Closing (Wind)";
      } else {
        currentOperation = "Closing (Temperature)";
      }
      // Update position
      ventPosition = clamp((totalOpenTime - totalCloseTime) / fullTravelMs * 100.0, 0.0, 100.0);
      if (ventPosition < 0.0) ventPosition = 0.0;
      Serial.printf("[SensorCtrl] Vent 1 closed for %.2fs, position: %.1f%% (%s side, windSpeed: %.1f)%s\n", closeMs/1000.0, ventPosition, vent1Side.c_str(), windSpeed, isRaining() ? " [Rain]" : "");
      if (ventPosition <= 0.0 && !vent1MinMsgShown) {
        Serial.println("[SensorCtrl] Vent 1 is fully closed.");
        vent1MinMsgShown = true;
      }
      vent1MaxMsgShown = false;
    }
  } else {
    // If neither open nor close, reset flags
    vent1MaxMsgShown = false;
    vent1MinMsgShown = false;
  }
}

void vent2SensorControlLoop(float currentTemp, int ventRuntime) {
  // Only run if sensorActivated==1 and calibration done
  if (sensorActivated != 1 || sensorControlState != 2) return;
  static unsigned long lastActionTime2 = 0;
  static bool waiting2 = false;
  float minT = tempSetPoint - tempHysteresis;
  float maxT = tempSetPoint + tempHysteresis;
  unsigned long now = millis();
  unsigned long maxStep = (unsigned long)(ventRuntime * (tempMaxPosition / 100.0) * 1000.0);
  unsigned long pauseTime = (unsigned long)(tempTime * 1000.0);
  float currentPos = vent2Position;
  float fullTravelMs = ventRuntime * 1000.0;

  // Wind/Lee side logic
  String vent1Side = "", vent2Side = "";
  determineWindLeeSide(vent1Side, vent2Side);
  float windSpeed = getLatestWindSpeed();
  int windLimit = (vent2Side == "Wind" && windSpeed > maxSpeedSetpoint) ? windMaxPosition : ((vent2Side == "Lee" && windSpeed > maxSpeedSetpoint) ? leeMaxPosition : 100);

  // --- Rain Emergency Priority ---
  if (isRaining()) {
    windLimit = min(windLimit, rainVent2Limit);
    if (vent2Position > rainVent2Limit && !relay3TaskActive && !relay4TaskActive) {
      float percentToClose = vent2Position - rainVent2Limit;
      unsigned long closeMs = (unsigned long)(fullTravelMs * (percentToClose / 100.0));
      if (closeMs > 0) {
        activateVent2Relay(RELAY_4, closeMs, relay4TaskActive, relay4Start, relay4Duration);
        totalCloseTime2 += closeMs;
        vent2Position = rainVent2Limit;
        currentOperation = "Closing (Rain)";
        Serial.printf("[RainEmergency] Closing Vent 2 to rain limit %d%% (was %.1f%%)\n", rainVent2Limit, vent2Position + percentToClose);
      }
      return;
    }
    windLimit = rainVent2Limit;
  }

  if (waiting2) {
    if (now - lastActionTime2 >= pauseTime) {
      waiting2 = false;
      if (!relay3TaskActive && !relay4TaskActive) currentOperation = "Idle";
    } else {
      return;
    }
  }
  // Only act if not waiting
  if (currentTemp > maxT) {
    // Open vent 2
    if (!relay3TaskActive && !relay4TaskActive) {
      if (currentPos >= windLimit) {
        if (!vent2MaxMsgShown) {
          Serial.printf("[SensorCtrl] Vent 2 at limit (%.0f%%, %s side)%s\n", (float)windLimit, vent2Side.c_str(), isRaining() ? " [Rain]" : "");
          vent2MaxMsgShown = true;
        }
        vent2MinMsgShown = false;
        vent2Position = windLimit;
        if ((vent2Side == "Wind" && windSpeed > maxSpeedSetpoint) || (vent2Side == "Lee" && windSpeed > maxSpeedSetpoint)) {
          currentOperation = "Idle";
        }
        return;
      }
      vent2MaxMsgShown = false;
      float percentToOpen = windLimit - currentPos;
      if (percentToOpen <= 0) return;
      unsigned long openMs = (unsigned long)(fullTravelMs * (percentToOpen / 100.0));
      if (openMs > maxStep) openMs = maxStep;
      activateVent2Relay(RELAY_3, openMs, relay3TaskActive, relay3Start, relay3Duration);
      totalOpenTime2 += openMs;
      lastActionTime2 = now;
      waiting2 = true;
      vent2Position = clamp((totalOpenTime2 - totalCloseTime2) / fullTravelMs * 100.0, 0.0, 100.0);
      if (vent2Position > windLimit) vent2Position = windLimit;
      if ((vent2Side == "Wind" && windSpeed > maxSpeedSetpoint) || (vent2Side == "Lee" && windSpeed > maxSpeedSetpoint)) {
        currentOperation = "Opening (Wind)";
      } else {
        currentOperation = "Opening (Temperature)";
      }
      Serial.printf("[SensorCtrl] Vent 2 opened for %.2fs, position: %.1f%% (%s side, windSpeed: %.1f)%s\n", openMs/1000.0, vent2Position, vent2Side.c_str(), windSpeed, isRaining() ? " [Rain]" : "");
      if (vent2Position >= windLimit && !vent2MaxMsgShown) {
        Serial.printf("[SensorCtrl] Vent 2 reached limit (%.0f%%) due to wind/rain.\n", (float)windLimit);
        vent2MaxMsgShown = true;
      }
      vent2MinMsgShown = false;
    }
  } else if (currentTemp < minT) {
    // Close vent 2
    if (!relay3TaskActive && !relay4TaskActive) {
      if (currentPos <= 0.0) {
        if (!vent2MinMsgShown) {
          Serial.println("[SensorCtrl] Vent 2 already at minimum position (0%)");
          vent2MinMsgShown = true;
        }
        vent2MaxMsgShown = false;
        vent2Position = 0.0;
        return;
      }
      vent2MinMsgShown = false;
      float percentToClose = currentPos;
      unsigned long closeMs = (unsigned long)(fullTravelMs * (percentToClose / 100.0));
      if (closeMs > maxStep) closeMs = maxStep;
      activateVent2Relay(RELAY_4, closeMs, relay4TaskActive, relay4Start, relay4Duration);
      totalCloseTime2 += closeMs;
      lastActionTime2 = now;
      waiting2 = true;
      vent2Position = clamp((totalOpenTime2 - totalCloseTime2) / fullTravelMs * 100.0, 0.0, 100.0);
      if (vent2Position < 0.0) vent2Position = 0.0;
      if ((vent2Side == "Wind" && windSpeed > maxSpeedSetpoint) || (vent2Side == "Lee" && windSpeed > maxSpeedSetpoint)) {
        currentOperation = "Closing (Wind)";
      } else {
        currentOperation = "Closing (Temperature)";
      }
      Serial.printf("[SensorCtrl] Vent 2 closed for %.2fs, position: %.1f%% (%s side, windSpeed: %.1f)%s\n", closeMs/1000.0, vent2Position, vent2Side.c_str(), windSpeed, isRaining() ? " [Rain]" : "");
      if (vent2Position <= 0.0 && !vent2MinMsgShown) {
        Serial.println("[SensorCtrl] Vent 2 is fully closed.");
        vent2MinMsgShown = true;
      }
      vent2MaxMsgShown = false;
    }
  } else {
    vent2MaxMsgShown = false;
    vent2MinMsgShown = false;
  }
}

// --- Wind Safety Correction Logic ---
void enforceWindLimits(int ventRuntime) {
  // Get wind data
  float windSpeed = getLatestWindSpeed();
  String vent1Side = "", vent2Side = "";
  determineWindLeeSide(vent1Side, vent2Side);

  int vent1Limit = (vent1Side == "Wind" && windSpeed > maxSpeedSetpoint) ? windMaxPosition : ((vent1Side == "Lee" && windSpeed > maxSpeedSetpoint) ? leeMaxPosition : 100);
  int vent2Limit = (vent2Side == "Wind" && windSpeed > maxSpeedSetpoint) ? windMaxPosition : ((vent2Side == "Lee" && windSpeed > maxSpeedSetpoint) ? leeMaxPosition : 100);

  // --- Apply rain override ---
  if (isRaining()) {
    vent1Limit = min(vent1Limit, rainVent1Limit);
    vent2Limit = min(vent2Limit, rainVent2Limit);
  }

  float fullTravelMs = ventRuntime * 1000.0;

  // Only close if above limit
  if (ventPosition > vent1Limit && !relay1Task.active && !relay2Task.active) {
    float percentToClose = ventPosition - vent1Limit;
    unsigned long closeMs = (unsigned long)(fullTravelMs * (percentToClose / 100.0));
    if (closeMs > 0) {
      activateSensorRelay(RELAY_2, closeMs, relay2Task);
      totalCloseTime += closeMs;
      ventPosition = vent1Limit;
      Serial.printf("[WindSafety] Closing Vent 1 to limit %.0f%% (was %.1f%%, %s side, windSpeed: %.1f)%s\n", (float)vent1Limit, ventPosition + percentToClose, vent1Side.c_str(), windSpeed, isRaining() ? " [Rain]" : "");
    }
  }
  if (vent2Position > vent2Limit && !relay3TaskActive && !relay4TaskActive) {
    float percentToClose = vent2Position - vent2Limit;
    unsigned long closeMs = (unsigned long)(fullTravelMs * (percentToClose / 100.0));
    if (closeMs > 0) {
      activateVent2Relay(RELAY_4, closeMs, relay4TaskActive, relay4Start, relay4Duration);
      totalCloseTime2 += closeMs;
      vent2Position = vent2Limit;
      Serial.printf("[WindSafety] Closing Vent 2 to limit %.0f%% (was %.1f%%, %s side, windSpeed: %.1f)%s\n", (float)vent2Limit, vent2Position + percentToClose, vent2Side.c_str(), windSpeed, isRaining() ? " [Rain]" : "");
    }
  }
}

// --- Wind Direction and Speed Settings ---
String getLatestWindDirection() {
  // Use first sensor with type "WD" (Wind Direction)
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensorType[i] == "WD") {
      float deg = sensorValue[i];
      return degreesToCardinal(deg);
    }
  }
  return "N"; // fallback
}

float getLatestWindSpeed() {
  // Use first sensor with type "WS" (Wind Speed)
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensorType[i] == "WS") {
      return sensorValue[i];
    }
  }
  return 0.0; // fallback
}

// Determine which vent is wind side and which is lee side
void determineWindLeeSide(String &vent1Side, String &vent2Side) {
  String windDir = getLatestWindDirection();
  if (vent1Direction == windDir) {
    vent1Side = "Wind";
    vent2Side = "Lee";
  } else if (vent2Direction == windDir) {
    vent1Side = "Lee";
    vent2Side = "Wind";
  } else {
    // Neither matches exactly, so pick closest
    vent1Side = (vent1Direction == windDir) ? "Wind" : "Lee";
    vent2Side = (vent2Direction == windDir) ? "Wind" : "Lee";
  }
}

// --- Sensor Control Activation ---
void checkSensorControlActivation(int ventRuntime) {
  static int prevSensorActivated = 0;
  if (sensorActivated == 1 && prevSensorActivated != 1) {
    // Start calibration: close fully
    Serial.println("[SensorCtrl] Calibration: closing vent 1 fully...");
    activateSensorRelay(RELAY_2, (ventRuntime + 5) * 1000, relay2Task);
    Serial.println("[SensorCtrl] Calibration: closing vent 2 fully...");
    activateVent2Relay(RELAY_4, (ventRuntime + 5) * 1000, relay4TaskActive, relay4Start, relay4Duration);
    sensorControlState = 1;
    sensorControlStepStart = millis();
    relayStepActive = true;
    totalOpenTime = 0;
    totalCloseTime = 0;
    ventPosition = 0.0;
    totalOpenTime2 = 0;
    totalCloseTime2 = 0;
    vent2Position = 0.0;
  }
  // Calibration done?
  if (sensorControlState == 1 && !relay2Task.active && !relay4TaskActive && relayStepActive) {
    Serial.println("[SensorCtrl] Calibration complete. Vent 1 at 0% (fully closed).\n[SensorCtrl] Calibration complete. Vent 2 at 0% (fully closed).");
    ventPosition = 0.0;
    totalOpenTime = 0;
    totalCloseTime = 0;
    vent2Position = 0.0;
    totalOpenTime2 = 0;
    totalCloseTime2 = 0;
    sensorControlState = 2;
    relayStepActive = false;
  }
  if (sensorActivated == 0 && prevSensorActivated == 1) {
    // Reset state
    sensorControlState = 0;
    relayStepActive = false;
    ventPosition = 0.0;
    totalOpenTime = 0;
    totalCloseTime = 0;
    vent2Position = 0.0;
    totalOpenTime2 = 0;
    totalCloseTime2 = 0;
    Serial.println("[SensorCtrl] Sensor control deactivated, state reset.");
  }
  prevSensorActivated = sensorActivated;
}

// --- Get latest temperature from LoRa sensors ---
float getLatestTemperature() {
  // Use first sensor with type "TE"
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensorType[i] == "TE") {
      return sensorValue[i];
    }
  }
  return tempSetPoint; // fallback
}

// --- VENT 2 RELAY CONTROL FUNCTIONS ---
void activateVent2Relay(uint8_t pin, unsigned long duration, bool &taskActive, unsigned long &taskStart, unsigned long &taskDuration) {
  digitalWrite(pin, HIGH);
  taskActive = true;
  taskStart = millis();
  taskDuration = duration;
  Serial.printf("[SensorCtrl] Vent 2 Relay %d ON for %lu ms\n", pin, duration);
}

void handleVent2RelayTask(uint8_t pin, bool &taskActive, unsigned long &taskStart, unsigned long &taskDuration) {
  if (taskActive && millis() - taskStart >= taskDuration) {
    digitalWrite(pin, LOW);
    taskActive = false;
    Serial.printf("[SensorCtrl] Vent 2 Relay %d OFF\n", pin);
  }
}

// --- Helper: Get Rain State ---
bool isRaining() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensorType[i] == "RN") {
      return sensorValue[i] == 1.0;
    }
  }
  return false;
}

// --- SYNC HANDLER ---
void handleSyncRequest() {
  if (!sensorActivated || syncInProgress) return;
  // If any relay is active, wait until all are off
  if (relay1Task.active || relay2Task.active || relay3TaskActive || relay4TaskActive) {
    // Will be checked again in loop
    return;
  }
  // Save current positions
  syncVent1Target = ventPosition;
  syncVent2Target = vent2Position;
  syncInProgress = true;
  currentOperation = "Closing (Sync Calibration)";
  Serial.println("[SYNC] Starting calibration: closing both vents fully");
  // Calibrate: close both for ventRuntime + 5s
  unsigned long closeMs = (ventRuntime * 1000) + 5000;
  activateSensorRelay(RELAY_2, closeMs, relay2Task);
  activateVent2Relay(RELAY_4, closeMs, relay4TaskActive, relay4Start, relay4Duration);
}

void processSyncMoveToTargets() {
  if (!syncInProgress) return;
  // Only move if both relays are off
  if (relay2Task.active || relay4TaskActive) return;
  Serial.println("[SYNC] Calibration done, moving vents to saved positions");
  // Move vent 1
  if (syncVent1Target > 0.0) {
    float percentToOpen = syncVent1Target;
    unsigned long openMs = (unsigned long)((ventRuntime * 1000.0) * (percentToOpen / 100.0));
    if (openMs > 0) {
      activateSensorRelay(RELAY_1, openMs, relay1Task);
      currentOperation = "Opening (Sync)";
    }
  }
  // Move vent 2
  if (syncVent2Target > 0.0) {
    float percentToOpen = syncVent2Target;
    unsigned long openMs = (unsigned long)((ventRuntime * 1000.0) * (percentToOpen / 100.0));
    if (openMs > 0) {
      activateVent2Relay(RELAY_3, openMs, relay3TaskActive, relay3Start, relay3Duration);
      currentOperation = "Opening (Sync)";
    }
  }
  syncInProgress = false;
  syncRequested = false;
}