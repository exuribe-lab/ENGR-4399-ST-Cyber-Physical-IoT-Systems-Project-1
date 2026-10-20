#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Credentials 
#define WIFI_SSID     ""     // 2.4 GHz network or phone hotspot
#define WIFI_PASS     ""
#define OWM_API_KEY   ""
#define LAT           "29.42"
#define LON           "-98.49"
#define AIO_USER      ""
#define AIO_KEY       ""

// 1 = fake weather values (WiFi and dashboard stay ON). 0 = real OpenWeather data.
#define USE_MOCK_API  0
#define MOCK_TOUT     18.0f
#define MOCK_POP      0.10f
#define MOCK_RAIN     false

// Pins 
#define DHT_PIN       4
#define DHT_TYPE      DHT11
#define THERM_PIN     34                   // ADC1
#define SOIL_PIN      32                   // ADC1
#define SERVO_PIN     18

// ---------- Control parameters ----------
const float T_SET        = 25.0f;   // setpoint (C)
const float DELTA_IN     = 1.0f;    // delta: deadband above setpoint
const float DELTA_OUT    = 2.0f;    // Delta: outside must be this much cooler
const float SENSOR_DIFF  = 4.0f;    // max allowed DHT vs thermistor disagreement
const float POP_LIMIT    = 0.40f;   // rain probability that blocks the vent
const float SPIKE_MARGIN = 4.0f;    // forecast above Tset by this = heat spike
const float PRECOOL_BIAS = 2.0f;    // lower setpoint during a forecast spike
const int   ANGLE_MIN    = 30;      // vent angle when just opening
const int   ANGLE_MAX    = 120;
const float BASELINE_DUTY = 0.5f;   // fixed-schedule baseline: vent open 50% of the time

const float THERM_OFFSET_C = 3.5f;  // calibration: thermistor read ~3.5 C low vs the DHT11

// Soil moisture and irrigation (calibrate on the hardware sensor)
// Read "soil_raw" in the Serial output with the probe in dry air and again in a cup of water,
// then put those two numbers here.
const int   SOIL_RAW_DRY  = 350;   // ADC value in dry air/soil
const int   SOIL_RAW_WET  = 3000;   // ADC value in water/wet soil
const int   SOIL_DRY_PCT  = 35;     // below this the plant needs water
const int   SOIL_WET_PCT  = 60;     // at or above this the soil is wet enough
const float SCHEDULE_PERIOD_S = 120.0f;  // demo value; use 21600 (6 h) for a real deployment

#if USE_MOCK_API
const unsigned long POLL_MS = 15000UL;
#else
const unsigned long POLL_MS = 600000UL;   // 10 min between real API calls
#endif
const unsigned long RETRY_MS = 30000UL;   // retry sooner if a fetch failed

const unsigned long PUB_INTERVAL_MS = 20000UL;  // 8 feeds every 20 s stays under the free 30/min limit

// ---------- Objects ----------
DHT dht(DHT_PIN, DHT_TYPE);
Servo vent;
WiFiClient netClient;
PubSubClient mqtt(netClient);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// ---------- State ----------
struct Weather {
  bool  valid    = false;
  float tout     = NAN;
  float nextTemp = NAN;
  float pop      = 0;
  bool  raining  = false;
} weather;

float tin = NAN, tDht = NAN, tTherm = NAN;
int   soilPct = 0;
int   soilRaw = 0;
float hum = NAN;
bool  sensorFault = false;
bool  ventOpen = false;
int   ventAngle = 0;

unsigned long lastPoll = 0, lastTick = 0, lastJson = 0, lastTs = 0;
unsigned long pollGap = POLL_MS;
float ventOpenSec = 0, elapsedSec = 0, degHours = 0;
bool  rainRisk = false, waterNeeded = false;
int   irrigationAvoided = 0, irrigationWatered = 0;
float nextSlotSec = SCHEDULE_PERIOD_S;
float cToF(float c) { return c * 9.0f / 5.0f + 32.0f; }

// ---------- Sensors ----------
// Hardware thermistor: 10k NTC on the 3V3 side, 10k fixed resistor to GND (Steinhart-Hart)
float readThermistorC() {
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += constrain(analogRead(THERM_PIN), 10, 4085);
    delay(3);
  }
  float reading = sum / 5.0f;
  float r = 10000.0f * ((4095.0f / reading) - 1.0f);
  if (r <= 0) return NAN;
  double lnR = log(r);
  double tempK = 1.0 / (0.001129148 + (0.000234125 + 0.0000000876741 * lnR * lnR) * lnR);
float tempC= (float)(tempK - 273.15);
return tempC+ THERM_OFFSET_C;
}

int readSoilPct() {
  long sum = 0;
  for (int i = 0; i < 10; i++) sum += analogRead(SOIL_PIN);
  soilRaw = sum / 10;
  return constrain(map(soilRaw, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100), 0, 100);
}

void readSensors() {
  tDht   = dht.readTemperature();
  tTherm = readThermistorC();
  hum = dht.readHumidity();
  soilPct  = readSoilPct();

  bool dhtOk = !isnan(tDht), thOk = !isnan(tTherm);
  sensorFault = false;
  if (dhtOk && thOk) {
    if (fabs(tDht - tTherm) > SENSOR_DIFF) {
      sensorFault = true;
      tin = tTherm;                       // thermistor treated as primary on disagreement
    } else {
      tin = (tDht + tTherm) / 2.0f;
    }
  } else if (thOk) { tin = tTherm; sensorFault = true; }
  else if (dhtOk)  { tin = tDht;   sensorFault = true; }
  else             { tin = NAN;    sensorFault = true; }
}

// ---------- Cloud ----------
bool httpGetJson(const String &url, JsonDocument &doc) {
  WiFiClientSecure client;
  client.setInsecure();                   // classroom demo; use a CA cert in production
  HTTPClient http;
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  bool ok = false;
  if (code == 200) ok = (deserializeJson(doc, http.getString()) == DeserializationError::Ok);
  else Serial.printf("OpenWeather HTTP %d\n", code);
  http.end();
  return ok;
}

bool fetchWeather() {
#if USE_MOCK_API
  weather.tout = MOCK_TOUT; weather.nextTemp = MOCK_TOUT + 1;
  weather.pop = MOCK_POP;   weather.raining = MOCK_RAIN;
  weather.valid = true;
  return true;
#else
  if (WiFi.status() != WL_CONNECTED) return false;
  String base = "https://api.openweathermap.org/data/2.5/";
  String tail = String("&lat=") + LAT + "&lon=" + LON + "&units=metric&appid=" + OWM_API_KEY;

  JsonDocument cur;
  if (!httpGetJson(base + "weather?" + tail.substring(1), cur)) return false;
  weather.tout = cur["main"]["temp"] | NAN;
  String main = cur["weather"][0]["main"] | "";
  float rain1h = cur["rain"]["1h"] | 0.0f;
  weather.raining = rain1h > 0 || main == "Rain" || main == "Drizzle" || main == "Thunderstorm";

  JsonDocument fc;
  if (httpGetJson(base + "forecast?cnt=2" + tail, fc)) {
    weather.pop = fc["list"][0]["pop"] | 0.0f;
    weather.nextTemp = fc["list"][1]["main"]["temp"] | weather.tout;
  } else {
    weather.pop = 0; weather.nextTemp = weather.tout;
  }
  weather.valid = !isnan(weather.tout);
  return weather.valid;       // last good response stays cached if a poll fails
#endif
}

// ---------- Control ----------
void updateControl() {
  rainRisk = weather.valid && (weather.raining || weather.pop >= POP_LIMIT);
  if (isnan(tin) || !weather.valid) {     // fail safe: close vent
    ventOpen = false;
  } else {
    float tset = T_SET;
    if (weather.nextTemp > T_SET + SPIKE_MARGIN) tset -= PRECOOL_BIAS;

    bool tooHot        = tin > tset + DELTA_IN;
    bool outsideCooler = weather.tout < tin - DELTA_OUT;

    if (!ventOpen && tooHot && outsideCooler && !rainRisk) ventOpen = true;
    else if (ventOpen && (rainRisk || tin <= tset || weather.tout >= tin)) ventOpen = false;

    ventAngle = ventOpen ? constrain((int)(ANGLE_MIN + (tin - tset) * 5.5f), ANGLE_MIN, ANGLE_MAX) : 0;
  }
  if (!ventOpen) ventAngle = 0;
  vent.write(ventAngle);
}

void updateMetrics(float dt) {
  elapsedSec += dt;
  if (ventOpen) ventOpenSec += dt;
  if (!isnan(tin) && tin > T_SET) degHours += (tin - T_SET) * dt / 3600.0f;

  // Plant protection: water only when soil is dry and no rain is expected
  waterNeeded = (soilPct < SOIL_DRY_PCT) && !rainRisk;

  // At each fixed-schedule slot the baseline would water; we only do so if needed
  if (elapsedSec >= nextSlotSec) {
    nextSlotSec += SCHEDULE_PERIOD_S;
    if (waterNeeded) irrigationWatered++;
    else irrigationAvoided++;
  }
}

float savedPct() {                         // actuator-off time vs fixed schedule baseline
  float base = BASELINE_DUTY * elapsedSec;
  return base > 0 ? (base - ventOpenSec) / base * 100.0f : 0;
}

// ---------- Outputs ----------
void drawOled() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);  oled.printf("Tin %.1fF Tout %.1fF", cToF(tin), cToF(weather.tout));
  oled.setCursor(0, 14); oled.printf("Rain %s Pop %.0f%%", weather.raining ? "YES" : "no", weather.pop * 100);
  oled.setCursor(0, 28); oled.printf("Vent: %s %d deg", ventOpen ? "OPEN" : "shut", ventAngle);
  oled.setCursor(0, 42); oled.printf("Soil %d%% Water:%s", soilPct, waterNeeded ? "NEED" : (rainRisk ? "skip" : "ok"));
  oled.setCursor(0, 56); oled.printf("Saved %.0f%% Sens:%s", savedPct(), sensorFault ? "FLT" : "ok");
  oled.display();
}

#define PRINT_JSON 0   // set to 1 to also print the full JSON line (for report data)

void printStatus() {
Serial.printf("Tin %.1fF | Tout %.1fF | Rain %s | Vent %s %d | Soil %d%% (raw %d) | Saved %.0f%% | Sens %s\n",
              cToF(tin), cToF(weather.tout), weather.raining ? "YES" : "no",
              ventOpen ? "OPEN" : "shut", ventAngle,
              soilPct, soilRaw, savedPct(), sensorFault ? "FLT" : "ok");
}

void printJson() {
  JsonDocument doc;
  doc["tin"] = cToF(tin);//clear adafruit old logs to make room for fahrenheit conversion
  doc["t_dht"] = tDht;
  doc["t_therm"] = tTherm;
  doc["tout"] = weather.tout;
  doc["pop"] = weather.pop;
  doc["rain"] = weather.raining;
  doc["humidity"] = hum;
  doc["vent_open"] = ventOpen;
  doc["vent_angle"] = ventAngle;
  doc["fault"] = sensorFault;
  doc["actuator_off_saved_pct"] = savedPct();
  doc["degree_hours"] = degHours;
  doc["soil_pct"] = soilPct;
  doc["soil_raw"] = soilRaw;
  doc["water_needed"] = waterNeeded;
  doc["irrigation_avoided"] = irrigationAvoided;
  doc["irrigation_watered"] = irrigationWatered;
  serializeJson(doc, Serial);
  Serial.println();
}

bool mqttConnect() {
  if (mqtt.connected()) return true;
  static unsigned long lastTry = 0;
  if (millis() - lastTry < 10000) return false;     // retry at most every 10 s
  lastTry = millis();
  bool ok = mqtt.connect("greenhouse-esp32", AIO_USER, AIO_KEY);
  Serial.println(ok ? "Adafruit IO: connected" : "Adafruit IO: connect failed");
  return ok;
}

void pubFeed(const char *feed, float value) {
  if (isnan(value)) return;
  char topic[64], payload[16];
  snprintf(topic, sizeof(topic), "%s/feeds/%s", AIO_USER, feed);
  snprintf(payload, sizeof(payload), "%.2f", value);
  mqtt.publish(topic, payload);
}
void publishDashboard() {
  if (WiFi.status() != WL_CONNECTED || !mqttConnect()) return;
  pubFeed("tin", cToF(tin));
  pubFeed("tout", cToF(weather.tout));
  pubFeed("soil", soilPct);
  pubFeed("vent-angle", ventAngle);
  pubFeed("humidity", hum);
  pubFeed("degree-hours", degHours);
  pubFeed("saved-pct", savedPct());
  pubFeed("irrigation-avoided", irrigationAvoided);
  Serial.println("Adafruit IO: published");
}

  void handleSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove newline and extra spaces

    if (command == "storm") {
      ventOpen = true;
      ventAngle = 90;
      delay(5000);
      vent.write(ventAngle);
      Serial.println("Vent opened");
    }}}
// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  dht.begin();
  vent.attach(SERVO_PIN);
  vent.write(0);
  mqtt.setServer("io.adafruit.com", 1883);
  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    while (1);
  }
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Smart Greenhouse");
  oled.println("Connecting WiFi...");
  oled.display();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi NOT connected (check name/password, use 2.4 GHz)");
  }
  pollGap = fetchWeather() ? POLL_MS : RETRY_MS;
  lastPoll = millis();
  lastTick = millis();
}
void loop() {
  unsigned long now = millis();
  handleSerialCommand();

  if (now - lastPoll >= pollGap) {          // poll the API only every pollGap, otherwise use cache
    pollGap = fetchWeather() ? POLL_MS : RETRY_MS;
    lastPoll = now;
  }

  if (now - lastTick >= 1000) {
    float dt = (now - lastTick) / 1000.0f;
    lastTick = now;
    readSensors();
    updateControl();
    updateMetrics(dt);
    drawOled();
  }

  if (now - lastJson >= 5000) {
    printStatus();
#if PRINT_JSON
    printJson();
#endif
    lastJson = now;
  }
  if (mqtt.connected()) mqtt.loop();
  if (now - lastTs >= PUB_INTERVAL_MS) {
    publishDashboard();
    lastTs = now;
  }
}