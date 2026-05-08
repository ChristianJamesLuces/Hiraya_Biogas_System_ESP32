#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <time.h>

// ==========================================
// ⚙️ CONFIGURATION
// ==========================================
const char* WIFI_SSID = "S2S_scGbPu_EXT";
const char* WIFI_PASS = "Ud45xdQ7";

const char* SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InZubXNieHJtZG5kdmJjaHZ2eXpqIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NjI5MTk5MTksImV4cCI6MjA3ODQ5NTkxOX0._vG85bXqgP7-OD8D4SFwCF_GdsTZ1a3mY-8JMHHUR1w";
const char* SUPABASE_URL = "https://vnmsbxrmdndvbchvvyzj.supabase.co";
const char* TABLE_ENDPOINT = "/rest/v1/sensorreading";
const String FULL_URL = String(SUPABASE_URL) + TABLE_ENDPOINT;

const char* DIGESTER_ID = "D-01";

// --- ⏰ TIME ---
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.google.com";
const char* ntpServer3 = "time.cloudflare.com";
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;

// --- ⏱️ SCHEDULE ---
const int PREHEAT_HOUR = 7;   const int PREHEAT_MIN = 45;
const int START_HOUR   = 8;   const int START_MIN   = 0;
const int STOP_HOUR    = 16;  const int STOP_MIN    = 0;
const int MOTOR_RUN_MINUTES = 6;
const int WAKE_HOUR = 7;  const int WAKE_MIN = 15;

// --- ⚠️ ALARM THRESHOLDS ---
const float TEMP_NOTGOOD_LOW  = 46.0;
const float TEMP_NOTGOOD_HIGH = 60.0;
const float TEMP_BAD          = 60.0;

const float PH_IDEAL_HIGH    = 7.4;
const float PH_IDEAL_LOW     = 6.7;
const float PH_NOTGOOD_LOW   = 6.4;
const float PH_NOTGOOD_HIGH  = 6.6;
const float PH_BAD           = 6.4;

const float US_NOTGOOD_LOW   = 80.0;
const float US_NOTGOOD_HIGH  = 90.0;
const float US_BAD           = 90.0;

const unsigned long NOTGOOD_DURATION = 60000;
const unsigned long BAD_DURATION     = 180000;

// --- 📊 AVERAGING ---
const unsigned long AVG_INTERVAL = 300000;
unsigned long lastAvgTime = 0;
int avgSampleCount = 0;
float sumTempC = 0, sumPH = 0;
float sumCH4Dig = 0, sumCH4Sto = 0;
float sumRsDig = 0, sumRsSto = 0;
float sumRatioDig = 0, sumRatioSto = 0;
long sumRawDig = 0, sumRawSto = 0;
float sumDistanceCm = 0;
int distanceSampleCount = 0;
int tempSampleCount = 0;
int phSampleCount = 0;

float avgTempC = 0, avgPH = 0;
float avgCH4Dig = 0, avgCH4Sto = 0;
float avgRsDig = 0, avgRsSto = 0;
float avgRatioDig = 0, avgRatioSto = 0;
int avgRawDig = 0, avgRawSto = 0;
float avgDistanceCm = 0;

// --- pH Alarm 10-Second Averaging ---
float phAlarmBuffer[10];
int phAlarmIndex = 0;
bool phAlarmBufferFull = false;
float phAlarmAvg = -99.9;
bool phAlarmReady = false;

// --- Motor ---
int currentMotorLogId = -1;
int uploadCount = 0;
unsigned long motorStartTime = 0;
bool motorScheduledStop = false;
const int MOTOR_PWM = 50;
bool motorStartedToday = false;
bool isMotorRunning = false;

// --- WiFi ---
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000;
bool wifiReconnecting = false;

// --- NTP ---
unsigned long lastNtpSync = 0;
const unsigned long NTP_RESYNC_INTERVAL = 1800000;

// --- Software Watchdog ---
unsigned long lastSuccessfulLoop = 0;
const unsigned long WATCHDOG_TIMEOUT = 300000;

// --- 🚨 ALARM STATE ---
bool isBuzzerActive = false;
unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 0;
bool buzzerBlinking = false;
unsigned long lastBlinkToggle = 0;
bool buzzerPinState = false;
String currentAlarmType = "";
String currentAlarmSensorId = "";

// ==========================================
// 📌 PIN CONFIGURATION
// ==========================================
const int RPWM_PIN = 25;
const int LPWM_PIN = 26;
const int R_EN_PIN = 27;
const int L_EN_PIN = 14;

const int PH_ANALOG_PIN = 33;
const int MQ4_DIG_ANALOG_PIN = 36;
const int MQ4_STO_ANALOG_PIN = 39;

const int BUZZER_PIN = 4;
const int ULTRASONIC_TRIG_PIN = 17;
const int ULTRASONIC_ECHO_PIN = 5;
#define TEMP_SENSOR_PIN 16

const int PWM_FREQ = 1000;
const int PWM_RES = 8;

// --- 🧪 SENSOR CONSTANTS ---
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

const float ADC_MAX = 4095.0;
const float VCC_ESP = 3.3;
const float RL_KOHM = 10.0;

#define V_REF 3.3
#define DIVIDER_RATIO (20.0 / (10.0 + 20.0))
#define V_NEUTRAL_CALIBRATION 2.560
#define PH_STEP 0.1656140351

float RO_MQ4_DIG = 217.0894;
float RO_MQ4_STO = 309.724;
const float MQ4_M = -0.4835528;
const float MQ4_B =  1.7561222;
const float TEMP_CORRECTION = 0.0;

// --- Global Variables ---
float lastTempC = 0.0, lastDistanceCm = 0.0, lastPHValue = 0.0;
float ch4_dig_ppm = 0.0, ch4_sto_ppm = 0.0;
float last_rs_dig = 0.0, last_rs_sto = 0.0;
float last_ratio_dig = 0.0, last_ratio_sto = 0.0;
int last_raw_dig = 0, last_raw_sto = 0;
float lastPH_voltage = 0.0;
long lastUltrasonicRaw = 0;
String alarmReason = "";

// ==========================================
// HELPER FUNCTIONS
// ==========================================

int getTimeInMinutes(struct tm &t) { return t.tm_hour * 60 + t.tm_min; }

bool isBeforePreheat(struct tm &t) {
    return (getTimeInMinutes(t) < PREHEAT_HOUR * 60 + PREHEAT_MIN);
}

bool isPreheating(struct tm &t) {
    int now = getTimeInMinutes(t);
    return (now >= PREHEAT_HOUR * 60 + PREHEAT_MIN && now < START_HOUR * 60 + START_MIN);
}

bool isActiveHours(struct tm &t) {
    int now = getTimeInMinutes(t);
    return (now >= START_HOUR * 60 + START_MIN && now < STOP_HOUR * 60 + STOP_MIN);
}

bool isPastShutdown(struct tm &t) {
    return (getTimeInMinutes(t) >= STOP_HOUR * 60 + STOP_MIN);
}

String getTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return "";
    char buf[30];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S+08:00", &timeinfo);
    return String(buf);
}

void checkSoftwareWatchdog() {
    if (millis() - lastSuccessfulLoop >= WATCHDOG_TIMEOUT) {
        Serial.println("\n🐕 WATCHDOG: Restarting...");
        Serial.flush(); delay(100);
        ESP.restart();
    }
}

// ==========================================
// SENSOR FUNCTIONS
// ==========================================

float getRsKOhm(int pin) {
    long adcSum = 0;
    for (int i = 0; i < 30; i++) { adcSum += analogRead(pin); delay(2); }
    float avgADC = (float)adcSum / 30.0;
    if (avgADC < 10) return -1.0;
    float vRead = (avgADC * VCC_ESP) / ADC_MAX;
    float vSensor = vRead * 2.0;
    if (vSensor == 0) return 1e6;
    if (vSensor >= 4.9) return 0.0;
    return RL_KOHM * (5.0 / vSensor - 1.0);
}

float getCH4ppm(float rs, float ro) {
    if (rs <= 0 || ro <= 0) return 0.0;
    float ratio = rs / ro;
    float ppm = pow(10.0, (log10(ratio) - MQ4_B) / MQ4_M);
    if (ppm > 10000) return 10000.0;
    if (ppm < 200) return 0.0;
    return ppm;
}

void updatePhAlarmAverage() {
    if (lastPHValue == -99.9) {
        phAlarmReady = false;
        return;
    }

    phAlarmBuffer[phAlarmIndex] = lastPHValue;
    phAlarmIndex++;

    if (phAlarmIndex >= 10) {
        // Buffer full — compute average
        float sum = 0;
        for (int i = 0; i < 10; i++) sum += phAlarmBuffer[i];
        phAlarmAvg = sum / 10.0;
        phAlarmReady = true;
        phAlarmIndex = 0; // reset for next 10-second cycle
        Serial.printf("   🧪 pH 10s avg: %.2f (from 10 samples)\n", phAlarmAvg);
    } else {
        // Not enough samples yet — don't trigger alarm
        phAlarmReady = false;
    }
}

void readSensors() {
    tempSensor.requestTemperatures();
    float rawTemp = tempSensor.getTempCByIndex(0);
    lastTempC = (rawTemp == DEVICE_DISCONNECTED_C || rawTemp == -127.0) ? -127.0 : rawTemp + TEMP_CORRECTION;

    last_raw_dig = analogRead(MQ4_DIG_ANALOG_PIN);
    last_raw_sto = analogRead(MQ4_STO_ANALOG_PIN);
    last_rs_dig  = getRsKOhm(MQ4_DIG_ANALOG_PIN);
    last_rs_sto  = getRsKOhm(MQ4_STO_ANALOG_PIN);
    last_ratio_dig = (last_rs_dig > 0 && RO_MQ4_DIG > 0) ? last_rs_dig / RO_MQ4_DIG : 0;
    last_ratio_sto = (last_rs_sto > 0 && RO_MQ4_STO > 0) ? last_rs_sto / RO_MQ4_STO : 0;
    ch4_dig_ppm = getCH4ppm(last_rs_dig, RO_MQ4_DIG);
    ch4_sto_ppm = getCH4ppm(last_rs_sto, RO_MQ4_STO);

    // pH with clamp 0-14
    lastPH_voltage = (analogRead(PH_ANALOG_PIN) / ADC_MAX * V_REF) / DIVIDER_RATIO;
    if (lastPH_voltage < 0.1 || lastPH_voltage > 4.9) {
        lastPHValue = -99.9;
    } else {
        lastPHValue = 6.86 + (V_NEUTRAL_CALIBRATION - lastPH_voltage) / PH_STEP;
        if (lastPHValue < 0.0) lastPHValue = 0.0;
        if (lastPHValue > 14.0) lastPHValue = 14.0;
    }

    // pH alarm averaging (10 samples = 10 seconds)
    updatePhAlarmAverage();

    // Ultrasonic
    lastUltrasonicRaw = 0;
    for (int i = 0; i < 3; i++) {
        digitalWrite(ULTRASONIC_TRIG_PIN, LOW); delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIG_PIN, HIGH); delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
        long reading = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 50000);
        if (reading > 0) { lastUltrasonicRaw = reading; break; }
        delay(60);
    }
    lastDistanceCm = (lastUltrasonicRaw * 0.0343) / 2;
}

// ==========================================
// WIFI + NTP
// ==========================================

void connectToWiFi() {
    WiFi.mode(WIFI_STA); WiFi.disconnect(true); delay(1000);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE,
                IPAddress(8,8,8,8), IPAddress(8,8,4,4));
    Serial.printf("Connecting to: '%s'\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) { delay(500); Serial.print("."); attempts++; }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n❌ WiFi FAILED.");
    } else {
        Serial.println("\n✅ WiFi Connected!");
        Serial.print("   IP: "); Serial.println(WiFi.localIP());
        WiFiClientSecure tc; tc.setInsecure();
        if (tc.connect("vnmsbxrmdndvbchvvyzj.supabase.co", 443)) {
            Serial.println("   ✅ Supabase reachable!"); tc.stop();
        } else { Serial.println("   ❌ Supabase NOT reachable"); }
    }
}

void checkWiFiBackground() {
    if (millis() - lastWiFiCheck < WIFI_CHECK_INTERVAL) return;
    lastWiFiCheck = millis();
    if (WiFi.status() == WL_CONNECTED) { wifiReconnecting = false; return; }
    if (!wifiReconnecting) {
        WiFi.disconnect(true); delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASS); wifiReconnecting = true;
    } else {
        if (WiFi.status() == WL_CONNECTED) wifiReconnecting = false;
    }
}

bool syncNTP() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
    struct tm ti; int a = 0;
    while (!getLocalTime(&ti) && a < 30) { delay(1000); Serial.print("."); a++; }
    if (getLocalTime(&ti)) {
        Serial.printf("\n✅ NTP: %02d:%02d:%02d PHT\n", ti.tm_hour, ti.tm_min, ti.tm_sec);
        lastNtpSync = millis(); return true;
    }
    Serial.println("\n❌ NTP failed!"); return false;
}

void checkNtpResync() {
    if (millis() - lastNtpSync >= NTP_RESYNC_INTERVAL && WiFi.status() == WL_CONNECTED) {
        Serial.println("🔄 NTP re-sync..."); syncNTP();
    }
}

// ==========================================
// MOTOR FUNCTIONS
// ==========================================

void logMotorStart() {
    if (WiFi.status() != WL_CONNECTED) return;
    WiFiClientSecure c; c.setInsecure(); c.setTimeout(30000);
    HTTPClient http;
    http.begin(c, String(SUPABASE_URL) + "/rest/v1/motorlog");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
    http.addHeader("Prefer", "return=representation");
    DynamicJsonDocument doc(256); JsonObject obj = doc.to<JsonObject>();
    obj["status"] = "running"; obj["digester_id"] = DIGESTER_ID; obj["duration_sec"] = 0;
    String p; serializeJson(doc, p);
    int rc = http.POST(p);
    if (rc > 0) {
        String b = http.getString();
        DynamicJsonDocument rd(512); deserializeJson(rd, b);
        currentMotorLogId = rd[0]["log_id"].as<int>();
        Serial.printf("   ✅ Motor log_id: %d\n", currentMotorLogId);
    }
    http.end();
}

void logMotorStop(unsigned long dur) {
    if (WiFi.status() != WL_CONNECTED || currentMotorLogId <= 0) return;
    WiFiClientSecure c; c.setInsecure(); c.setTimeout(30000);
    HTTPClient http;
    http.begin(c, String(SUPABASE_URL) + "/rest/v1/motorlog?log_id=eq." + String(currentMotorLogId));
    http.addHeader("Content-Type", "application/json");
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
    http.addHeader("Prefer", "return=minimal");
    DynamicJsonDocument doc(256); JsonObject obj = doc.to<JsonObject>();
    obj["status"] = "completed"; obj["stopped_at"] = getTimestamp(); obj["duration_sec"] = dur;
    String p; serializeJson(doc, p);
    http.PATCH(p); http.end();
    currentMotorLogId = -1;
}

void startMotor() {
    Serial.printf("\n⚙️ MOTOR STARTING — PWM: %d\n", MOTOR_PWM);
    ledcWrite(RPWM_PIN, 150); delay(200);
    ledcWrite(RPWM_PIN, MOTOR_PWM);
    isMotorRunning = true; motorStartTime = millis(); motorScheduledStop = false;
    Serial.println("   ✅ Motor running.");
    logMotorStart();
}

void stopMotor() {
    ledcWrite(RPWM_PIN, 0); isMotorRunning = false; motorScheduledStop = true;
    unsigned long dur = (millis() - motorStartTime) / 1000;
    Serial.printf("⏹ MOTOR STOPPED. %lu sec.\n", dur);
    logMotorStop(dur);
}

// ==========================================
// 🚨 ALARM SYSTEM
// ==========================================

void logAlertToSupabase(String sensorId, String alertType) {
    if (WiFi.status() != WL_CONNECTED) return;
    WiFiClientSecure c; c.setInsecure(); c.setTimeout(30000);
    HTTPClient http;
    http.begin(c, String(SUPABASE_URL) + "/rest/v1/alertlog");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
    http.addHeader("Prefer", "return=minimal");
    DynamicJsonDocument doc(256); JsonObject obj = doc.to<JsonObject>();
    obj["sensor_id"] = sensorId; obj["alert_type"] = alertType; obj["status"] = "New";
    String p; serializeJson(doc, p);
    Serial.println("🚨 Alert: " + p);
    int rc = http.POST(p);
    if (rc == 201) Serial.println("   ✅ Alert logged");
    else Serial.printf("   ❌ Alert error: %d\n", rc);
    http.end();
}

void triggerAlarm(String sensorId, String alertType, bool isBad) {
    if (isBuzzerActive) return;
    isBuzzerActive = true;
    buzzerStartTime = millis();
    currentAlarmSensorId = sensorId;
    currentAlarmType = alertType;

    if (isBad) {
        buzzerBlinking = false;
        buzzerDuration = BAD_DURATION;
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerPinState = true;
        Serial.printf("🚨 BAD: %s — %s\n", sensorId.c_str(), alertType.c_str());
    } else {
        buzzerBlinking = true;
        buzzerDuration = NOTGOOD_DURATION;
        lastBlinkToggle = millis();
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerPinState = true;
        Serial.printf("⚠️ NOT GOOD: %s — %s\n", sensorId.c_str(), alertType.c_str());
    }
    logAlertToSupabase(sensorId, alertType);
}

void updateBuzzer() {
    if (!isBuzzerActive) return;
    if (millis() - buzzerStartTime >= buzzerDuration) {
        isBuzzerActive = false;
        digitalWrite(BUZZER_PIN, LOW); buzzerPinState = false;
        Serial.println("✅ ALARM CLEAR: Timeout");
        return;
    }
    if (buzzerBlinking && millis() - lastBlinkToggle >= 300) {
        buzzerPinState = !buzzerPinState;
        digitalWrite(BUZZER_PIN, buzzerPinState ? HIGH : LOW);
        lastBlinkToggle = millis();
    }
}

bool isTempNormal() {
    if (lastTempC == -127.0) return true;
    return (lastTempC < TEMP_NOTGOOD_LOW);
}

bool isPhNormal() {
    if (lastPHValue == -99.9) return true;
    return (lastPHValue > PH_NOTGOOD_HIGH);
}

bool isUltrasonicNormal() {
    if (lastUltrasonicRaw == 0) return true;
    return (lastDistanceCm < US_NOTGOOD_LOW);
}

void checkAlarms() {
    alarmReason = "";

    // --- AUTO-CLEAR: if buzzer active, check if sensor returned to normal ---
    if (isBuzzerActive) {
        bool backToNormal = false;

        if (currentAlarmSensorId == "T-01") {
            backToNormal = isTempNormal();
        } else if (currentAlarmSensorId == "PH-01") {
            backToNormal = isPhNormal();
        } else if (currentAlarmSensorId == "US-01") {
            backToNormal = isUltrasonicNormal();
        }

        if (backToNormal) {
            isBuzzerActive = false;
            digitalWrite(BUZZER_PIN, LOW);
            buzzerPinState = false;
            Serial.printf("✅ ALARM CLEARED: %s returned to normal\n", currentAlarmSensorId.c_str());
            alarmReason = "None";
            return;
        }

        unsigned long rem = (buzzerDuration - (millis() - buzzerStartTime)) / 1000;
        alarmReason = currentAlarmType + " (" + String(rem) + "s left)";
        return;
    }

    // --- CHECK NEW ALARMS ---

    // Temperature (instant check)
    if (lastTempC != -127.0) {
        if (lastTempC > TEMP_BAD) {
            triggerAlarm("T-01", "Bad: Overheating (>" + String(TEMP_BAD, 0) + "C)", true);
            return;
        }
        if (lastTempC >= TEMP_NOTGOOD_LOW && lastTempC <= TEMP_NOTGOOD_HIGH) {
            triggerAlarm("T-01", "Not Good: High Temp (" + String(TEMP_NOTGOOD_LOW, 0) + "-" + String(TEMP_NOTGOOD_HIGH, 0) + "C)", false);
            return;
        }
    }

    // pH (ONLY triggers after 10-second average is ready)
    if (phAlarmReady) {
        if (phAlarmAvg < PH_BAD) {
            triggerAlarm("PH-01", "Bad: Critical Acidic pH (<" + String(PH_BAD, 1) + ")", true);
            phAlarmReady = false; // consumed — wait for next 10s batch
            return;
        }
        if (phAlarmAvg >= PH_NOTGOOD_LOW && phAlarmAvg <= PH_NOTGOOD_HIGH) {
            triggerAlarm("PH-01", "Not Good: Low pH (" + String(PH_NOTGOOD_LOW, 1) + "-" + String(PH_NOTGOOD_HIGH, 1) + ")", false);
            phAlarmReady = false;
            return;
        }
        phAlarmReady = false; // consumed even if normal
    }

    // Ultrasonic (instant check)
    if (lastUltrasonicRaw > 0) {
        if (lastDistanceCm > US_BAD) {
            triggerAlarm("US-01", "Bad: Level Critical (>" + String(US_BAD, 0) + "cm)", true);
            return;
        }
        if (lastDistanceCm >= US_NOTGOOD_LOW && lastDistanceCm <= US_NOTGOOD_HIGH) {
            triggerAlarm("US-01", "Not Good: Level High (" + String(US_NOTGOOD_LOW, 0) + "-" + String(US_NOTGOOD_HIGH, 0) + "cm)", false);
            return;
        }
    }

    alarmReason = "None";
    digitalWrite(BUZZER_PIN, LOW);
}

// ==========================================
// AVERAGING
// ==========================================

void addToAverage() {
    if (lastTempC != -127.0) { sumTempC += lastTempC; tempSampleCount++; }
    if (lastPHValue != -99.9) { sumPH += lastPHValue; phSampleCount++; }
    sumCH4Dig += ch4_dig_ppm; sumCH4Sto += ch4_sto_ppm;
    sumRsDig += last_rs_dig; sumRsSto += last_rs_sto;
    sumRatioDig += last_ratio_dig; sumRatioSto += last_ratio_sto;
    sumRawDig += last_raw_dig; sumRawSto += last_raw_sto;
    if (lastUltrasonicRaw > 0) { sumDistanceCm += lastDistanceCm; distanceSampleCount++; }
    avgSampleCount++;
}

void calculateAverage() {
    if (avgSampleCount == 0) return;
    avgTempC     = (tempSampleCount > 0) ? sumTempC / tempSampleCount : -127.0;
    avgPH        = (phSampleCount > 0) ? sumPH / phSampleCount : -99.9;
    avgCH4Dig    = sumCH4Dig / avgSampleCount;
    avgCH4Sto    = sumCH4Sto / avgSampleCount;
    avgRsDig     = sumRsDig / avgSampleCount;
    avgRsSto     = sumRsSto / avgSampleCount;
    avgRatioDig  = sumRatioDig / avgSampleCount;
    avgRatioSto  = sumRatioSto / avgSampleCount;
    avgRawDig    = sumRawDig / avgSampleCount;
    avgRawSto    = sumRawSto / avgSampleCount;
    avgDistanceCm = (distanceSampleCount > 0) ? sumDistanceCm / distanceSampleCount : 0;
}

void resetAverage() {
    sumTempC = 0; sumPH = 0;
    sumCH4Dig = 0; sumCH4Sto = 0;
    sumRsDig = 0; sumRsSto = 0;
    sumRatioDig = 0; sumRatioSto = 0;
    sumRawDig = 0; sumRawSto = 0;
    sumDistanceCm = 0;
    avgSampleCount = 0; tempSampleCount = 0;
    phSampleCount = 0; distanceSampleCount = 0;
}

// ==========================================
// CLOUD UPLOAD
// ==========================================

void sendDataToCloud() {
    if (WiFi.status() != WL_CONNECTED || avgSampleCount == 0) return;
    Serial.println("\n========== 📤 UPLOADING ==========");
    WiFiClientSecure c; c.setInsecure(); c.setTimeout(30000);
    HTTPClient http;
    http.begin(c, FULL_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
    http.addHeader("Prefer", "return=minimal");

    DynamicJsonDocument doc(2048); JsonArray arr = doc.to<JsonArray>();

    JsonObject r0 = arr.createNestedObject();
    r0["sensor_id"] = "T-01"; r0["value"] = avgTempC;
    r0["unit"] = "C"; r0["raw_value"] = nullptr;
    r0["rs_kohm"] = nullptr; r0["rs_ro_ratio"] = nullptr;

    JsonObject r1 = arr.createNestedObject();
    r1["sensor_id"] = "CH4-DIG"; r1["value"] = avgCH4Dig;
    r1["unit"] = "ppm"; r1["raw_value"] = avgRawDig;
    r1["rs_kohm"] = avgRsDig; r1["rs_ro_ratio"] = avgRatioDig;

    JsonObject r2 = arr.createNestedObject();
    r2["sensor_id"] = "CH4-STO"; r2["value"] = avgCH4Sto;
    r2["unit"] = "ppm"; r2["raw_value"] = avgRawSto;
    r2["rs_kohm"] = avgRsSto; r2["rs_ro_ratio"] = avgRatioSto;

    JsonObject r3 = arr.createNestedObject();
    r3["sensor_id"] = "PH-01"; r3["value"] = avgPH;
    r3["unit"] = "pH"; r3["raw_value"] = nullptr;
    r3["rs_kohm"] = nullptr; r3["rs_ro_ratio"] = nullptr;

    JsonObject r4 = arr.createNestedObject();
    r4["sensor_id"] = "US-01"; r4["value"] = avgDistanceCm;
    r4["unit"] = "cm"; r4["raw_value"] = nullptr;
    r4["rs_kohm"] = nullptr; r4["rs_ro_ratio"] = nullptr;

    String jp; serializeJson(arr, jp);
    Serial.println("   Payload: " + jp);
    int rc = http.POST(jp);
    if (rc == 201) { uploadCount++; Serial.printf("   ✅ Upload #%d\n", uploadCount); }
    else if (rc > 0) Serial.println("   ⚠️ " + http.getString());
    else Serial.printf("   ❌ %s\n", http.errorToString(rc).c_str());
    Serial.println("==================================");
    http.end();
}

// ==========================================
// DEEP SLEEP
// ==========================================

void enterDeepSleep(struct tm &ti) {
    int nowS = ti.tm_hour * 3600 + ti.tm_min * 60 + ti.tm_sec;
    int tgtS = WAKE_HOUR * 3600 + WAKE_MIN * 60;
    int slp = (nowS >= tgtS) ? (86400 - nowS) + tgtS : tgtS - nowS;
    if (slp < 60) slp = 60;
    Serial.printf("\n🌙 DEEP SLEEP at %02d:%02d:%02d (%d sec)\n", ti.tm_hour, ti.tm_min, ti.tm_sec, slp);
    if (isMotorRunning) stopMotor();
    digitalWrite(BUZZER_PIN, LOW); ledcWrite(RPWM_PIN, 0);
    Serial.flush();
    esp_sleep_enable_timer_wakeup((uint64_t)slp * 1000000ULL);
    esp_deep_sleep_start();
}

// ==========================================
// PRINT
// ==========================================

void printFullReadings(struct tm &t, String phase) {
    Serial.println("\n========== SENSOR READINGS ==========");
    Serial.printf("⏰ Time    : %02d:%02d:%02d  [%s]\n", t.tm_hour, t.tm_min, t.tm_sec, phase.c_str());
    if (lastTempC == -127.0) Serial.println("🌡️ Temp    : ❌ Disconnected");
    else { Serial.print("🌡️ Temp    : "); Serial.print(lastTempC, 2); Serial.println(" °C"); }
    Serial.print("💨 CH4-D   : "); Serial.print(ch4_dig_ppm, 2); Serial.println(" ppm");
    Serial.print("💨 CH4-S   : "); Serial.print(ch4_sto_ppm, 2); Serial.println(" ppm");
    Serial.print("🔬 RS-D    : "); Serial.print(last_rs_dig, 4); Serial.println(" kΩ");
    Serial.print("🔬 RS-S    : "); Serial.print(last_rs_sto, 4); Serial.println(" kΩ");
    Serial.print("🧪 pH      : ");
    if (lastPHValue == -99.9) Serial.println("❌ Out of range");
    else { Serial.print(lastPHValue, 2); Serial.printf(" (10s avg: %.2f)\n", phAlarmAvg); }
    Serial.print("📏 Dist    : ");
    if (lastUltrasonicRaw == 0) Serial.println("❌ No echo");
    else { Serial.print(lastDistanceCm, 2); Serial.println(" cm"); }
    Serial.print("⚙️ Motor   : "); Serial.println(isMotorRunning ? "🔄 Running" : "⏹ Stopped");
    Serial.print("🚨 Alarm   : "); Serial.println(alarmReason);
    Serial.print("📶 WiFi    : "); Serial.println(WiFi.status() == WL_CONNECTED ? "✅" : "❌");
    Serial.print("📤 Uploads : "); Serial.println(uploadCount);
    Serial.println("=====================================");
}

void printAveragedReadings(struct tm &t) {
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║  📊 5-MIN AVERAGED READINGS (Upload)  ║");
    Serial.println("╠═══════════════════════════════════════╣");
    Serial.printf("║ ⏰ Time   : %02d:%02d:%02d\n", t.tm_hour, t.tm_min, t.tm_sec);
    Serial.printf("║ 📈 Samples: %d\n", avgSampleCount);
    Serial.printf("║ 🌡️ Temp   : %.2f °C\n", avgTempC);
    Serial.printf("║ 💨 CH4-D  : %.2f ppm\n", avgCH4Dig);
    Serial.printf("║ 💨 CH4-S  : %.2f ppm\n", avgCH4Sto);
    Serial.printf("║ 🧪 pH     : %.2f\n", avgPH);
    Serial.printf("║ 📏 Dist   : %.2f cm\n", avgDistanceCm);
    Serial.printf("║ 📤 Upload#: %d\n", uploadCount + 1);
    Serial.println("╚═══════════════════════════════════════╝");
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200); delay(500);
    Serial.println("\n==========================================");
    Serial.println("   🌱 BIOFLAME SYSTEM ONLINE 🌱");
    Serial.println("==========================================");

    // Init pH buffer
    for (int i = 0; i < 10; i++) phAlarmBuffer[i] = 0;

    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(MQ4_DIG_ANALOG_PIN, INPUT); pinMode(MQ4_STO_ANALOG_PIN, INPUT);
    pinMode(PH_ANALOG_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

    pinMode(R_EN_PIN, OUTPUT); pinMode(L_EN_PIN, OUTPUT);
    pinMode(RPWM_PIN, OUTPUT); pinMode(LPWM_PIN, OUTPUT);
    ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RES);
    digitalWrite(R_EN_PIN, HIGH); digitalWrite(L_EN_PIN, HIGH); digitalWrite(LPWM_PIN, LOW);

    tempSensor.begin(); tempSensor.setResolution(12);
    int dc = tempSensor.getDeviceCount();
    if (dc == 0) Serial.println("❌ Temp sensor NOT detected!");
    else Serial.printf("✅ Temp sensor (%d)\n", dc);

    digitalWrite(BUZZER_PIN, HIGH); delay(200); digitalWrite(BUZZER_PIN, LOW);
    Serial.println("✅ Buzzer OK");

    connectToWiFi();
    if (WiFi.status() != WL_CONNECTED) { delay(3000); connectToWiFi(); }

    Serial.print("⏰ Syncing time...");
    bool timeOk = syncNTP();

    if (timeOk) {
        struct tm ti; getLocalTime(&ti);
        if (isBeforePreheat(ti)) {
            Serial.printf("⏳ Early wake %02d:%02d. Waiting...\n", ti.tm_hour, ti.tm_min);
            while (true) { getLocalTime(&ti); if (!isBeforePreheat(ti)) break; delay(10000); }
        }
        if (isPastShutdown(ti)) { enterDeepSleep(ti); }
    }

    lastSuccessfulLoop = millis();

    Serial.println("\n📋 Schedule & Thresholds:");
    Serial.printf("   🔥 Preheat : %02d:%02d\n", PREHEAT_HOUR, PREHEAT_MIN);
    Serial.printf("   ▶️ Start   : %02d:%02d (PWM %d)\n", START_HOUR, START_MIN, MOTOR_PWM);
    Serial.printf("   🌙 Sleep   : %02d:%02d\n", STOP_HOUR, STOP_MIN);
    Serial.println("   🌡️ Temp   : Ideal 25-40°C | Not Good 50-60°C | Bad >60°C");
    Serial.println("   🧪 pH     : Ideal 6.7-7.4 | Not Good 6.4-6.6 | Bad <6.4 (10s avg)");
    Serial.println("   📏 Level  : Not Good 80-90cm | Bad >90cm");
    Serial.println("   🔇 Auto-clear: Buzzer stops when reading returns to normal");
    Serial.println("==========================================\n");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    checkSoftwareWatchdog();

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        if (WiFi.status() != WL_CONNECTED) connectToWiFi();
        if (WiFi.status() == WL_CONNECTED) syncNTP();
        readSensors();
        lastSuccessfulLoop = millis();
        delay(5000);
        return;
    }

    lastSuccessfulLoop = millis();
    checkWiFiBackground();
    checkNtpResync();
    updateBuzzer();

    if (isPastShutdown(timeinfo)) {
        if (avgSampleCount > 0) { calculateAverage(); sendDataToCloud(); resetAverage(); }
        enterDeepSleep(timeinfo);
    }

    if (isPreheating(timeinfo)) {
        readSensors();
        static unsigned long lp = 0;
        if (millis() - lp >= 10000) {
            Serial.printf("[%02d:%02d:%02d] ♨️ PREHEATING — %d min left\n",
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                (START_HOUR * 60 + START_MIN) - getTimeInMinutes(timeinfo));
            lp = millis();
        }
        delay(1000); return;
    }

    if (isActiveHours(timeinfo)) {
        if (!motorStartedToday && !isMotorRunning) {
            startMotor(); motorStartedToday = true; lastAvgTime = millis();
        }
        if (isMotorRunning && !motorScheduledStop) {
            if (millis() - motorStartTime >= (unsigned long)MOTOR_RUN_MINUTES * 60 * 1000) stopMotor();
        }
        readSensors(); checkAlarms(); addToAverage();
        printFullReadings(timeinfo, "ACTIVE");
        if (millis() - lastAvgTime >= AVG_INTERVAL) {
            calculateAverage(); printAveragedReadings(timeinfo);
            sendDataToCloud(); resetAverage(); lastAvgTime = millis();
        }
        delay(1000); return;
    }
    delay(1000);
}
