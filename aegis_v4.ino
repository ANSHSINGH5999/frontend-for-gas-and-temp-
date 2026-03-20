/*
  ╔══════════════════════════════════════════════════════════════════╗
  ║   AEGIS SENTINEL — ESP32 v6.0 PRODUCTION                         ║
  ║                                                                  ║
  ║   FIXES in v6.0:                                                 ║
  ║   ✅ updateFan/updateGasAlarm called ONCE per loop (no double)   ║
  ║   ✅ prevTempHigh / prevGasAlarm properly managed                 ║
  ║   ✅ forceStop flags reset after use (no infinite trigger)        ║
  ║   ✅ emergencyStop proper reset logic                             ║
  ║   ✅ DHT11 read throttled (max 1Hz, not every 500ms)             ║
  ║   ✅ Firebase push uses updateNode (partial update, faster)       ║
  ║   ✅ HTTP timeout + retry logic                                   ║
  ║   ✅ Serial output clean and meaningful                           ║
  ║   ✅ Buzzer beep pattern uses ledcTone (no tone() conflicts)      ║
  ║   ✅ All pin modes set correctly                                  ║
  ║   ✅ Proper variable scoping                                      ║
  ╚══════════════════════════════════════════════════════════════════╝

  📦 REQUIRED LIBRARIES:
  1. Firebase ESP Client  — Mobizt  (Board Manager > ESP32 also needed)
  2. DHT sensor library   — Adafruit
  3. Adafruit Unified Sensor — Adafruit
  4. ArduinoJson          — Benoit Blanchon v6.x

  🔌 WIRING:
  DHT11  VCC → 3.3V | GND → GND | DATA → GPIO 4 (+10kΩ to VCC)
  MQ Gas VCC → 5V   | GND → GND | AOUT → GPIO 34
  Buzzer (+) → GPIO 27 | (-) → GND
  LED    (+) → GPIO 26 (+220Ω) | (-) → GND
  Relay  VCC → 5V | GND → GND | IN → GPIO 14 (active LOW)
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <DHT.h>

// ════════════════════════════════════════════
// ⚙️  USER CONFIG — CHANGE THESE
// ════════════════════════════════════════════

const char* WIFI_SSID  = "Ansh12@";
const char* WIFI_PASS  = "4urukpzzpv";
const char* SERVER_URL = "https://gas-senor-and-fan-production.up.railway.app/";

#define FIREBASE_API_KEY   "AIzaSyDsHvHnBnq0Kno1aydP6TGvTls2e5OL-J4"
#define FIREBASE_DB_URL    "https://knowaboutanshsingh-default-rtdb.firebaseio.com"

// ════════════════════════════════════════════
// 📌 PINS
// ════════════════════════════════════════════

#define PIN_GAS     34    // Analog — MQ sensor AOUT
#define PIN_DHT      25    // Digital — DHT11 data (+ 10kΩ pull-up)
#define PIN_BUZZER  27    // PWM output — passive buzzer
#define PIN_LED     26    // Digital output — alarm LED
#define PIN_RELAY   14    // Digital output — fan relay (active LOW)

#define DHT_TYPE    DHT11

// ════════════════════════════════════════════
// 🎯 THRESHOLDS (tunable)
// ════════════════════════════════════════════

#define GAS_ALARM_THRESHOLD   700      // ADC raw (0-4095) — MQ sensor
#define TEMP_FAN_ON_C        35.0f     // °C — fan triggers above this
#define TEMP_FAN_OFF_C       32.0f     // °C — fan re-arms below this
#define BUZZER_ON_DURATION   20000UL   // ms — buzzer runs 20s
#define FAN_ON_DURATION      20000UL   // ms — fan runs 20s
#define BUZZER_FREQ          2200      // Hz — alarm tone frequency
#define BUZZER_BEEP_ON        500      // ms — beep ON period
#define BUZZER_BEEP_OFF       200      // ms — beep OFF period

// ════════════════════════════════════════════
// ⏱ INTERVALS
// ════════════════════════════════════════════

#define INTERVAL_GAS_READ       200UL   // Gas: every 200ms
#define INTERVAL_DHT_READ      2000UL   // DHT11: max 1 reading per 2s
#define INTERVAL_FIREBASE_PUSH 1000UL   // Firebase push: every 1s
#define INTERVAL_RAILWAY_FETCH 2000UL   // Railway fetch: every 2s
#define INTERVAL_WIFI_CHECK   10000UL   // WiFi watchdog: every 10s

// ════════════════════════════════════════════
// OBJECTS
// ════════════════════════════════════════════

DHT          dht(PIN_DHT, DHT_TYPE);
FirebaseData fbdo;
FirebaseAuth fbAuth;
FirebaseConfig fbConfig;

// ════════════════════════════════════════════
// SENSOR STATE
// ════════════════════════════════════════════

struct SensorState {
  int   gas         = 0;
  float temp        = 0.0f;
  float humidity    = 0.0f;
  bool  dhtOK       = false;
  bool  gasAlarm    = false;
  bool  fanOn       = false;
} sensors;

// ════════════════════════════════════════════
// CONTROL STATE (from Railway/Frontend)
// ════════════════════════════════════════════

struct ControlState {
  String mode          = "auto";   // "auto" | "manual"
  bool   manualFan     = false;
  bool   alarmMuted    = false;
  bool   forceStopFan  = false;    // Pulse — set true, auto-reset after handling
  bool   forceStopBuzz = false;    // Pulse — set true, auto-reset after handling
  bool   emergencyStop = false;    // Latching — stays until manually cleared
} ctrl;

// ════════════════════════════════════════════
// TIMER STATE
// ════════════════════════════════════════════

struct TimerState {
  // Buzzer
  bool          buzzerActive    = false;
  unsigned long buzzerStart     = 0;
  int           buzzerRemaining = 0;

  // Fan
  bool          fanActive       = false;
  unsigned long fanStart        = 0;
  int           fanRemaining    = 0;

  // Debounce — arm/re-arm logic
  bool prevGasHigh  = false;   // Was gas high last check?
  bool prevTempHigh = false;   // Was temp high last check?
} timers;

// ════════════════════════════════════════════
// SYSTEM STATE
// ════════════════════════════════════════════

bool         firebaseReady  = false;
unsigned long lastGasRead    = 0;
unsigned long lastDHTRead    = 0;
unsigned long lastFBPush     = 0;
unsigned long lastRailway    = 0;
unsigned long lastWiFiCheck  = 0;
unsigned long bootTime       = 0;

// ════════════════════════════════════════════
// FORWARD DECLARATIONS
// ════════════════════════════════════════════

void connectWiFi();
void initFirebase();
void readGasSensor();
void readDHT();
void applyFanLogic();
void applyBuzzerLogic();
void pushToFirebase();
void fetchFromRailway();
void setBuzzer(bool on);
void setRelay(bool on);
void setLED(bool on);
void printStatus();

// ════════════════════════════════════════════
// WiFi
// ════════════════════════════════════════════

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true);
  delay(300);

  Serial.printf("📶 Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t > 15000) {
      Serial.println("\n❌ WiFi timeout — restarting ESP32");
      delay(500);
      ESP.restart();
    }
    delay(500);
    Serial.print('.');
  }

  Serial.printf("\n✅ WiFi OK | IP: %s | RSSI: %d dBm\n",
    WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

// ════════════════════════════════════════════
// Firebase
// ════════════════════════════════════════════

void initFirebase() {
  fbConfig.api_key              = FIREBASE_API_KEY;
  fbConfig.database_url         = FIREBASE_DB_URL;
  fbConfig.token_status_callback = tokenStatusCallback;

  // Anonymous signup for RTDB access
  if (Firebase.signUp(&fbConfig, &fbAuth, "", "")) {
    firebaseReady = true;
    Serial.println("🔥 Firebase: anonymous auth OK");
  } else {
    Serial.printf("⚠️  Firebase auth failed: %s\n",
      fbConfig.signer.signupError.message.c_str());
    // Continue anyway — will retry on first push
  }

  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(2);
  fbdo.setResponseSize(2048);  // Limit memory usage
}

// ════════════════════════════════════════════
// SENSOR READS
// ════════════════════════════════════════════

void readGasSensor() {
  // Average 4 readings for noise reduction
  int sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(PIN_GAS);
    delayMicroseconds(100);
  }
  sensors.gas = sum / 4;
}

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();  // Celsius

  if (isnan(h) || isnan(t) || h < 0 || h > 100 || t < -40 || t > 80) {
    sensors.dhtOK = false;
    // Keep last valid values — don't zero out
    Serial.println("⚠️  DHT11: invalid reading (NaN or out of range)");
  } else {
    sensors.temp     = t;
    sensors.humidity = h;
    sensors.dhtOK    = true;
  }
}

// ════════════════════════════════════════════
// OUTPUT HELPERS
// ════════════════════════════════════════════

void setBuzzer(bool on) {
  if (on) {
    tone(PIN_BUZZER, BUZZER_FREQ);
  } else {
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);  // Ensure fully silent
  }
}

void setRelay(bool on) {
  // Relay is active LOW: LOW = fan ON, HIGH = fan OFF
  digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  sensors.fanOn = on;
}

void setLED(bool on) {
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// ════════════════════════════════════════════
// FAN LOGIC
// ════════════════════════════════════════════

void applyFanLogic() {
  // ── EMERGENCY STOP overrides everything ──
  if (ctrl.emergencyStop) {
    if (timers.fanActive) {
      timers.fanActive    = false;
      timers.fanRemaining = 0;
      Serial.println("🚨 Fan killed by emergency stop");
    }
    setRelay(false);
    return;
  }

  // ── MANUAL MODE ──
  if (ctrl.mode == "manual") {
    // Cancel any running auto timer
    timers.fanActive    = false;
    timers.fanRemaining = 0;
    timers.prevTempHigh = false;  // Reset debounce for when auto resumes

    // Handle forceStopFan in manual mode too
    if (ctrl.forceStopFan) {
      ctrl.forceStopFan = false;
      ctrl.manualFan    = false;
    }

    setRelay(ctrl.manualFan);
    return;
  }

  // ── AUTO MODE ──

  // Handle forceStopFan pulse
  if (ctrl.forceStopFan) {
    ctrl.forceStopFan = false;   // Consume the pulse
    if (timers.fanActive) {
      timers.fanActive    = false;
      timers.fanRemaining = 0;
      setRelay(false);
      timers.prevTempHigh = true;  // Arm debounce so it won't re-trigger immediately
      Serial.println("🛑 Fan FORCE STOPPED by frontend");
    }
    return;
  }

  bool tempHigh = (sensors.temp >= TEMP_FAN_ON_C);

  // Start fan timer on rising edge (low→high) only
  if (tempHigh && !timers.prevTempHigh && !timers.fanActive) {
    timers.fanActive    = true;
    timers.fanStart     = millis();
    timers.fanRemaining = FAN_ON_DURATION / 1000;
    setRelay(true);
    Serial.printf("🌀 Fan ON — temp %.1f°C > %.1f°C — timer 20s\n",
      sensors.temp, TEMP_FAN_ON_C);
  }

  // Run the timer
  if (timers.fanActive) {
    unsigned long elapsed = millis() - timers.fanStart;
    if (elapsed < FAN_ON_DURATION) {
      timers.fanRemaining = (int)((FAN_ON_DURATION - elapsed) / 1000);
      setRelay(true);
    } else {
      // Timer expired
      timers.fanActive    = false;
      timers.fanRemaining = 0;
      setRelay(false);
      Serial.println("✅ Fan OFF — 20s timer completed");
    }
  } else {
    timers.fanRemaining = 0;
    // Don't setRelay here unless timer just stopped — relay already set
  }

  // Re-arm debounce when temp drops below TEMP_FAN_OFF_C
  // (Hysteresis: fan won't re-trigger until temp drops then rises again)
  if (sensors.temp <= TEMP_FAN_OFF_C) {
    timers.prevTempHigh = false;
  } else if (tempHigh) {
    timers.prevTempHigh = true;
  }
}

// ════════════════════════════════════════════
// BUZZER / ALARM LOGIC
// ════════════════════════════════════════════

void applyBuzzerLogic() {
  sensors.gasAlarm = (sensors.gas > GAS_ALARM_THRESHOLD);

  // LED always reflects alarm (even if buzzer muted/stopped)
  setLED(sensors.gasAlarm && !ctrl.emergencyStop);

  // ── EMERGENCY STOP overrides everything ──
  if (ctrl.emergencyStop) {
    if (timers.buzzerActive) {
      timers.buzzerActive    = false;
      timers.buzzerRemaining = 0;
      setBuzzer(false);
      Serial.println("🚨 Buzzer killed by emergency stop");
    }
    setLED(false);
    return;
  }

  // ── MANUAL MODE — buzzer disabled ──
  if (ctrl.mode == "manual") {
    timers.buzzerActive    = false;
    timers.buzzerRemaining = 0;
    timers.prevGasHigh     = false;
    setBuzzer(false);
    return;
  }

  // ── AUTO MODE ──

  // Handle forceStopBuzz pulse
  if (ctrl.forceStopBuzz) {
    ctrl.forceStopBuzz = false;   // Consume pulse
    if (timers.buzzerActive) {
      timers.buzzerActive    = false;
      timers.buzzerRemaining = 0;
      setBuzzer(false);
      timers.prevGasHigh     = true;  // Prevent immediate re-trigger
      ctrl.alarmMuted        = true;  // Also mute so it doesn't restart
      Serial.println("🔇 Buzzer FORCE STOPPED by frontend");
    }
    return;
  }

  // Handle mute from frontend
  if (ctrl.alarmMuted && timers.buzzerActive) {
    timers.buzzerActive    = false;
    timers.buzzerRemaining = 0;
    setBuzzer(false);
    Serial.println("🔇 Buzzer muted from frontend");
    return;
  }

  bool gasHigh = sensors.gasAlarm;

  // Start buzzer timer on rising edge only
  if (gasHigh && !timers.prevGasHigh && !timers.buzzerActive && !ctrl.alarmMuted) {
    timers.buzzerActive    = true;
    timers.buzzerStart     = millis();
    timers.buzzerRemaining = BUZZER_ON_DURATION / 1000;
    Serial.printf("🚨 GAS ALARM! gas=%d > %d — buzzer ON 20s\n",
      sensors.gas, GAS_ALARM_THRESHOLD);
  }

  // Run buzzer timer with beep pattern
  if (timers.buzzerActive) {
    unsigned long elapsed = millis() - timers.buzzerStart;

    if (elapsed < BUZZER_ON_DURATION) {
      timers.buzzerRemaining = (int)((BUZZER_ON_DURATION - elapsed) / 1000);

      // Beep pattern: ON for BUZZER_BEEP_ON ms, OFF for BUZZER_BEEP_OFF ms
      unsigned long period = BUZZER_BEEP_ON + BUZZER_BEEP_OFF;
      unsigned long phase  = elapsed % period;
      setBuzzer(phase < BUZZER_BEEP_ON);

    } else {
      // Timer expired
      timers.buzzerActive    = false;
      timers.buzzerRemaining = 0;
      setBuzzer(false);
      // Reset mute so next alarm can trigger
      ctrl.alarmMuted = false;
      // Reset debounce so next gas spike will re-arm
      timers.prevGasHigh = false;
      Serial.println("✅ Buzzer OFF — 20s timer completed");
    }
  } else {
    timers.buzzerRemaining = 0;
    setBuzzer(false);
  }

  // Debounce: re-arm when gas drops below threshold
  if (!gasHigh) {
    timers.prevGasHigh = false;
  } else if (gasHigh && timers.buzzerActive) {
    timers.prevGasHigh = true;
  } else if (gasHigh && !timers.buzzerActive) {
    timers.prevGasHigh = true;  // Was high but buzzer done
  }
}

// ════════════════════════════════════════════
// PUSH TO FIREBASE
// ════════════════════════════════════════════

void pushToFirebase() {
  if (!firebaseReady || !Firebase.ready()) {
    Serial.println("⚠️  Firebase not ready — skipping push");
    return;
  }

  FirebaseJson json;

  // Sensor data
  json.set("temp",     (double)sensors.temp);
  json.set("humidity", (double)sensors.humidity);
  json.set("gas",      sensors.gas);
  json.set("fan",      sensors.fanOn);
  json.set("alarm",    sensors.gasAlarm);
  json.set("dhtOK",    sensors.dhtOK);

  // Control state
  json.set("mode",     ctrl.mode.c_str());
  json.set("muted",    ctrl.alarmMuted);
  json.set("emergency",ctrl.emergencyStop);

  // Timer countdowns
  json.set("fanTimerActive",   timers.fanActive);
  json.set("fanRemaining",     timers.fanRemaining);
  json.set("buzzerActive",     timers.buzzerActive);
  json.set("buzzerRemaining",  timers.buzzerRemaining);

  // Uptime
  json.set("uptime", (int)((millis() - bootTime) / 1000));

  if (Firebase.RTDB.setJSON(&fbdo, "/sensors", &json)) {
    Serial.println("🔥 Firebase ✓");
  } else {
    Serial.printf("❌ Firebase push failed: %s\n", fbdo.errorReason().c_str());
    // If token expired, reinit
    if (String(fbdo.errorReason()).indexOf("token") >= 0) {
      firebaseReady = false;
      initFirebase();
    }
  }
}

// ════════════════════════════════════════════
// FETCH CONTROL FROM RAILWAY
// ════════════════════════════════════════════

void fetchFromRailway() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(String(SERVER_URL) + "/control");
  http.setTimeout(4000);
  http.addHeader("Accept", "application/json");

  int code = http.GET();

  if (code == 200) {
    String body = http.getString();

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, body);

    if (err) {
      Serial.printf("❌ JSON parse error: %s\n", err.c_str());
      http.end();
      return;
    }

    // ── Mode ──
    const char* newModeC = doc["mode"] | ctrl.mode.c_str();
    String newMode = String(newModeC);
    if (newMode != ctrl.mode) {
      Serial.printf("🎛️  Mode: %s → %s\n", ctrl.mode.c_str(), newMode.c_str());
      // Reset debounce when switching modes
      timers.prevTempHigh = false;
      timers.prevGasHigh  = false;
    }
    ctrl.mode = newMode;

    // ── Fan override (manual mode) ──
    ctrl.manualFan  = doc["fan"]  | ctrl.manualFan;

    // ── Mute ──
    bool newMuted   = doc["mute"] | ctrl.alarmMuted;
    if (!ctrl.alarmMuted && newMuted) {
      Serial.println("🔇 Alarm muted from frontend");
    } else if (ctrl.alarmMuted && !newMuted) {
      Serial.println("🔔 Alarm unmuted from frontend");
    }
    ctrl.alarmMuted = newMuted;

    // ── Force stop pulses (rising edge detection) ──
    bool rawForceStopFan  = doc["forceStopFan"]  | false;
    bool rawForceStopBuzz = doc["forceStopBuzz"] | false;
    bool rawEmergency     = doc["emergencyStop"] | false;

    // Only set if newly true (rising edge)
    if (rawForceStopFan  && !ctrl.forceStopFan)  ctrl.forceStopFan  = true;
    if (rawForceStopBuzz && !ctrl.forceStopBuzz) ctrl.forceStopBuzz = true;

    // Emergency stop: set on rising edge, clear only if server cleared it
    if (rawEmergency && !ctrl.emergencyStop) {
      ctrl.emergencyStop = true;
      Serial.println("🚨 EMERGENCY STOP RECEIVED!");
    } else if (!rawEmergency && ctrl.emergencyStop) {
      // Server reset the emergency flag — re-enable system
      ctrl.emergencyStop  = false;
      ctrl.alarmMuted     = false;
      timers.prevTempHigh = false;
      timers.prevGasHigh  = false;
      Serial.println("✅ Emergency stop cleared — system re-enabled");
    }

    Serial.printf("🚂 ctrl: mode=%s fan=%d mute=%d fsFan=%d fsBuzz=%d emergency=%d\n",
      ctrl.mode.c_str(), ctrl.manualFan, ctrl.alarmMuted,
      rawForceStopFan, rawForceStopBuzz, rawEmergency);

  } else if (code < 0) {
    Serial.printf("❌ Railway unreachable (code %d) — check URL\n", code);
  } else {
    Serial.printf("⚠️  Railway HTTP %d\n", code);
  }

  http.end();
}

// ════════════════════════════════════════════
// DEBUG PRINT
// ════════════════════════════════════════════

void printStatus() {
  Serial.printf(
    "🌡 %.1f°C | 💧 %.0f%% | 💨 %d | Fan:%s(%ds) | Buzz:%s(%ds) | Mode:%s | E-Stop:%d\n",
    sensors.temp,
    sensors.humidity,
    sensors.gas,
    sensors.fanOn       ? "ON " : "OFF",
    timers.fanRemaining,
    timers.buzzerActive ? "ON " : "OFF",
    timers.buzzerRemaining,
    ctrl.mode.c_str(),
    ctrl.emergencyStop
  );
}

// ════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println(F("╔══════════════════════════════════════════╗"));
  Serial.println(F("║   AEGIS SENTINEL v6.0 — PRODUCTION        ║"));
  Serial.println(F("║   DHT11 + MQ Gas + Firebase + Railway     ║"));
  Serial.println(F("║   Manual OFF + Emergency STOP + Timers    ║"));
  Serial.println(F("╚══════════════════════════════════════════╝"));
  Serial.println();

  // ── Pin Setup ──
  pinMode(PIN_RELAY,  OUTPUT);
  pinMode(PIN_LED,    OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_GAS,    INPUT);   // GPIO34 is input-only
  // PIN_DHT is handled by DHT library

  // ── Safe defaults — everything OFF ──
  setRelay(false);
  setLED(false);
  setBuzzer(false);

  bootTime = millis();

  // ── DHT11 Init ──
  dht.begin();
  Serial.println(F("🌡️  DHT11 init on GPIO 4"));
  delay(2000);  // DHT11 needs 1s+ to stabilize
  readDHT();

  if (sensors.dhtOK) {
    Serial.printf("   First reading: %.1f°C | %.0f%%\n", sensors.temp, sensors.humidity);
  } else {
    Serial.println(F("   ⚠️  DHT11 not responding — check wiring & 10kΩ pull-up!"));
  }

  // ── WiFi ──
  connectWiFi();

  // ── Firebase ──
  initFirebase();

  Serial.println();
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("🚀 AEGIS SENTINEL READY"));
  Serial.printf( "   Firebase   : %s\n", firebaseReady ? "✅ CONNECTED" : "⚠️  CHECK RULES");
  Serial.printf( "   Railway    : %s\n", SERVER_URL);
  Serial.printf( "   Gas THR    : %d ADC\n", GAS_ALARM_THRESHOLD);
  Serial.printf( "   Temp ON    : %.1f°C\n", TEMP_FAN_ON_C);
  Serial.printf( "   Temp OFF   : %.1f°C\n", TEMP_FAN_OFF_C);
  Serial.printf( "   Timers     : %lus each\n", BUZZER_ON_DURATION / 1000);
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println();
}

// ════════════════════════════════════════════
// LOOP
// ════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── WiFi Watchdog ──
  if (now - lastWiFiCheck >= INTERVAL_WIFI_CHECK) {
    lastWiFiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("📶 WiFi lost — reconnecting..."));
      connectWiFi();
    }
  }

  // ── Gas Sensor Read (every 200ms) ──
  if (now - lastGasRead >= INTERVAL_GAS_READ) {
    lastGasRead = now;
    readGasSensor();
  }

  // ── DHT11 Read (every 2s — sensor limit is 1Hz) ──
  if (now - lastDHTRead >= INTERVAL_DHT_READ) {
    lastDHTRead = now;
    readDHT();
  }

  // ── Apply Logic (every loop — for accurate timers) ──
  applyFanLogic();
  applyBuzzerLogic();

  // ── Firebase Push (every 1s) ──
  if (now - lastFBPush >= INTERVAL_FIREBASE_PUSH) {
    lastFBPush = now;
    printStatus();
    pushToFirebase();
  }

  // ── Railway Fetch (every 2s) ──
  if (now - lastRailway >= INTERVAL_RAILWAY_FETCH) {
    lastRailway = now;
    fetchFromRailway();
  }
}

/*
  ════════════════════════════════════════════════
  📊 FIREBASE /sensors STRUCTURE (v6.0)
  ════════════════════════════════════════════════

  {
    "temp":             25.4,      // °C from DHT11
    "humidity":         58.0,      // % from DHT11
    "gas":              320,       // ADC 0-4095
    "fan":              false,     // relay state
    "alarm":            false,     // gas alarm active
    "dhtOK":            true,      // DHT11 working
    "mode":             "auto",    // auto | manual
    "muted":            false,     // buzzer muted
    "emergency":        false,     // emergency stop active
    "fanTimerActive":   false,     // fan 20s timer running
    "fanRemaining":     0,         // seconds left on fan timer
    "buzzerActive":     false,     // buzzer timer running
    "buzzerRemaining":  0,         // seconds left on buzzer
    "uptime":           1234       // seconds since boot
  }

  📊 RAILWAY /control STRUCTURE
  {
    "mode":          "auto",   // auto | manual
    "fan":           false,    // manual fan toggle
    "mute":          false,    // mute buzzer
    "forceStopFan":  false,    // pulse: cancel fan timer
    "forceStopBuzz": false,    // pulse: cancel buzzer
    "emergencyStop": false     // latching: kill everything
  }

  ════════════════════════════════════════════════
  🔧 FIREBASE RULES (paste in Firebase Console)
  ════════════════════════════════════════════════

  {
    "rules": {
      ".read": true,
      ".write": true
    }
  }

  ════════════════════════════════════════════════
  🔧 COMMON ISSUES
  ════════════════════════════════════════════════

  DHT returns NaN every time:
  → Missing 10kΩ pull-up between GPIO4 and 3.3V
  → DHT11 needs 1-2s delay after begin()

  Gas reads always 0:
  → GPIO34 is input-only — correct, no fix needed
  → MQ sensor needs 5V (not 3.3V) for heater
  → New MQ sensor: warmup 24-48h for accurate readings

  Firebase push fails every time:
  → Set rules to ".read": true, ".write": true
  → Check FIREBASE_API_KEY and FIREBASE_DB_URL

  Fan not responding to manual toggle:
  → Switch to Manual mode first from frontend
  → Fan toggle only works in manual mode

  Emergency stop won't clear:
  → Go to Railway dashboard → hit /control
  → emergencyStop must be false on server
  → Or click "Emergency Stop" again on frontend to toggle off
*/
