
#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <HX711.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

//  WIFI 

#define WIFI_SSID     "Avii"
#define WIFI_PASSWORD "Avishka04"

//  FIREBASE 

#define DATABASE_SECRET "bzJ1X7QqxRNoMdQY8rQmiADwZrwe40k4r6EBFYPq"
#define API_KEY         "AIzaSyAE2jjgeKgqIiHy4NhzVbcHSr-qoBrFUc4"
#define DATABASE_URL    "smartwastepro-1085e-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData   fbdo;
FirebaseAuth   auth;
FirebaseConfig config;

//  PIN DEFINITIONS 

#define TRIG1 5
#define ECHO1 18
#define TRIG2 19
#define ECHO2 23
#define TRIG3 26
#define ECHO3 27

#define LED1 12
#define LED2 13
#define LED3 25

#define BUZZER 14

#define DT1  36
#define SCK1 33
#define DT2  4
#define SCK2 16
#define DT3  17
#define SCK3 21

// GPIO 34 & 35 are INPUT-ONLY  i added external 10kΩ pullup to 3.3V

#define BTN1 34
#define BTN2 35
#define BTN3 32

//  FILL  level for bin hight 

#define FULL_LEVEL  8
#define LEVEL_75   14
#define LEVEL_50   22

// TIMING 

#define SENSOR_INTERVAL   500
#define FIREBASE_INTERVAL 3000
#define BUZZER_BEEP_ON    200
#define BUZZER_BEEP_OFF   800
#define DEBOUNCE_MS        50

//  WEIGHT FILTER 

#define WEIGHT_SAMPLES   10
#define WEIGHT_DEADBAND   5.0f

// LONG PRESS DURATION — 2000ms = 2 seconds  ( for buttons to trare rests to 0)

#define LONG_PRESS_MS 2000


//  load cell 3  separate controllers

HX711 scale1, scale2, scale3;

unsigned long prevSensorMillis   = 0;
unsigned long prevFirebaseMillis = 0;
unsigned long buzzerMillis       = 0;

bool buzzerState  = false;
bool buzzerActive = false;

String lastL1 = "INIT", lastL2 = "INIT", lastL3 = "INIT";
float  lastW1 = -999.0f, lastW2 = -999.0f, lastW3 = -999.0f;

float wBuf1[WEIGHT_SAMPLES] = {};
float wBuf2[WEIGHT_SAMPLES] = {};
float wBuf3[WEIGHT_SAMPLES] = {};
int   wIdx  = 0;
bool  wFull = false;


// Button long-press detection variables

bool          btn1Last = HIGH, btn2Last = HIGH, btn3Last = HIGH;    // last button state (HIGH = not pressed, LOW = pressed)
unsigned long btn1PressTime = 0,  btn2PressTime = 0,  btn3PressTime = 0; 
bool          btn1Handled  = false, btn2Handled = false, btn3Handled = false; //ensure long-press action runs only once




//  FORWARD DECLARATIONS

float  getDistance(int trigPin, int echoPin);
String getLevel(float d);
float  getRawWeight(HX711 &scale);
float  movingAvg(float *buf, float newVal);
void   handleButtons();
void   controlLED(int pin, const String &level);
void   handleBuzzer(const String &l1, const String &l2, const String &l3);
void   sendToFirebase(const String &l1, const String &l2, const String &l3,
                     float w1, float w2, float w3);


//  ULTRASONIC — single  reading for fill level 

float getDistance(int trigPin, int echoPin) {

  // Send a clean 10µs trigger pulse

  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Wait for echo, 30ms timeout

  long duration = pulseIn(echoPin, HIGH, 30000UL);

  // If no echo received, sensor is blocked or disconnected

  if (duration == 0) return 999.0f;

  // Convert to cm: speed of sound = 0.0343 cm/µs, divide by 2 (there and back)

  float distance = duration * 0.0343f / 2.0f;
  return distance;
}


//  FILL LEVEL STRING

String getLevel(float d) {
  if (d <= 0.0f || d > 400.0f) return "ERROR";
  if (d < FULL_LEVEL)           return "FULL";
  if (d < LEVEL_75)             return "75%";
  if (d < LEVEL_50)             return "50%";
  return "EMPTY";
}

// WEIGHT 
// Only clip clearly negative noise (e.g. < -5g) to 0,
// Tare is triggered by button long-press 

float getRawWeight(HX711 &scale) {
  if (!scale.is_ready()) return 0.0f;
  float w = scale.get_units(1);

  // Only clip clearly negative noise,  don't wipe small real weights
  return (w < -5.0f) ? 0.0f : w;
}


//  MOVING AVERAGE — smooth out weight readings, especially when bin is empty or nearly empty

float movingAvg(float *buf, float newVal) {
  buf[wIdx]  = newVal;
  int   count = wFull ? WEIGHT_SAMPLES : (wIdx + 1);
  float sum   = 0.0f;
  for (int i = 0; i < count; i++) sum += buf[i];
  return sum / static_cast<float>(count);
}


// BUTTONS — long press to tare weight to 0 for each bin
//   - Button pressed  → record the time
//   - Button held 2s  → tare once (wasHandled = true)
//   - Button released → reset tracking variables
//   - Short press     → nothing happens

void handleButtons() {
  unsigned long now = millis();

  bool b1 = static_cast<bool>(digitalRead(BTN1));
  bool b2 = static_cast<bool>(digitalRead(BTN2));
  bool b3 = static_cast<bool>(digitalRead(BTN3));

  // Plastic (BTN1)

  if (b1 == LOW && btn1Last == HIGH) {
    // Button just pressed — record time
    btn1PressTime = now;
    btn1Handled   = false;
  }

  if (b1 == LOW && !btn1Handled) {
    // Button is being held — check if 2 seconds have passed

    if ((now - btn1PressTime) >= LONG_PRESS_MS) {
      Serial.println("[BTN] Long press: Tare Plastic");
      scale1.tare();
      memset(wBuf1, 0, sizeof(wBuf1));
      lastW1 = 0.0f;
      Firebase.RTDB.setFloat(&fbdo,
        "/bins/bin001/compartments/plastic/weight", 0.0f);
      btn1Handled = true;  // prevent repeated tare while still held
    }
  }
  if (b1 == HIGH && btn1Last == LOW) {
    // Button released — reset
    btn1Handled = false;
  }
  btn1Last = b1;

  // Food (BTN2)

  if (b2 == LOW && btn2Last == HIGH) {
    btn2PressTime = now;
    btn2Handled   = false;
  }
  if (b2 == LOW && !btn2Handled) {
    if ((now - btn2PressTime) >= LONG_PRESS_MS) {
      Serial.println("[BTN] Long press: Tare Food");
      scale2.tare();
      memset(wBuf2, 0, sizeof(wBuf2));
      lastW2 = 0.0f;
      Firebase.RTDB.setFloat(&fbdo,
        "/bins/bin001/compartments/food/weight", 0.0f);
      btn2Handled = true;
    }
  }
  if (b2 == HIGH && btn2Last == LOW) {
    btn2Handled = false;
  }
  btn2Last = b2;

  // Metal (BTN3)

  if (b3 == LOW && btn3Last == HIGH) {
    btn3PressTime = now;
    btn3Handled   = false;
  }
  if (b3 == LOW && !btn3Handled) {
    if ((now - btn3PressTime) >= LONG_PRESS_MS) {
      Serial.println("[BTN] Long press: Tare Metal");
      scale3.tare();
      memset(wBuf3, 0, sizeof(wBuf3));
      lastW3 = 0.0f;
      Firebase.RTDB.setFloat(&fbdo,
        "/bins/bin001/compartments/metal/weight", 0.0f);
      btn3Handled = true;
    }
  }
  if (b3 == HIGH && btn3Last == LOW) {
    btn3Handled = false;
  }
  btn3Last = b3;
}


//LED — ON only when FULL

void controlLED(int pin, const String &level) { 
  digitalWrite(pin, (level == "FULL") ? HIGH : LOW);
}


//  BUZZER — turns ON immediately when FULL


void handleBuzzer(const String &l1, const String &l2, const String &l3) {
  unsigned long now     = millis();
  bool          anyFull = (l1 == "FULL" || l2 == "FULL" || l3 == "FULL");

  if (!anyFull) {
    // No bin is full — silence buzzer and reset state
    if (buzzerActive) {
      ledcWriteTone(0, 0);
      buzzerActive = false;
      buzzerState  = false;
    }
    return;
  }

  // start beeping immediately
  if (!buzzerActive) {
    ledcWriteTone(0, 2000);   // start tone RIGHT NOW
    buzzerState  = true;      //  now in the ON phase
    buzzerMillis = now;       // record when ON phase started
    buzzerActive = true;
    return;
  }

  // Buzzer is already active — run the normal beep pattern
  if (buzzerState) {
    // check if ON duration is done
    if (now - buzzerMillis >= BUZZER_BEEP_ON) {
      ledcWriteTone(0, 0);      // turn off
      buzzerState  = false;
      buzzerMillis = now;
    }
  } else {
    //  check if OFF duration is done
    if (now - buzzerMillis >= BUZZER_BEEP_OFF) {
      ledcWriteTone(0, 2000);   // turn on
      buzzerState  = true;
      buzzerMillis = now;
    }
  }
}


//  FIREBASE

void sendToFirebase(const String &l1, const String &l2, const String &l3,
                    float w1, float w2, float w3) {
  unsigned long now = millis();

  bool levelChanged  = (l1 != lastL1 || l2 != lastL2 || l3 != lastL3);
  bool weightChanged = (fabsf(w1 - lastW1) > WEIGHT_DEADBAND ||
                        fabsf(w2 - lastW2) > WEIGHT_DEADBAND ||
                        fabsf(w3 - lastW3) > WEIGHT_DEADBAND);
  bool forceUpdate   = (now - prevFirebaseMillis >= FIREBASE_INTERVAL);

  if (!levelChanged && !weightChanged && !forceUpdate) return;

  prevFirebaseMillis = now;
  lastL1 = l1; lastL2 = l2; lastL3 = l3;
  lastW1 = w1; lastW2 = w2; lastW3 = w3;

  Serial.println("[FB] Sending update...");

  FirebaseJson json;
  json.set("compartments/plastic/level",  l1);
  json.set("compartments/plastic/weight", w1);
  json.set("compartments/food/level",     l2);
  json.set("compartments/food/weight",    w2);
  json.set("compartments/metal/level",    l3);
  json.set("compartments/metal/weight",   w3);
  json.set("lastUpdated/.sv",             "timestamp");

  if (!Firebase.RTDB.updateNode(&fbdo, "/bins/bin001", &json)) {
    Serial.print("[FB] Error: ");
    Serial.println(fbdo.errorReason());
  }
}


//  SETUP

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);
  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);
  digitalWrite(TRIG3, LOW);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);
  pinMode(BTN3, INPUT_PULLUP);

  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER, 0);
  ledcWriteTone(0, 0);

  scale1.begin(DT1, SCK1);
  scale2.begin(DT2, SCK2);
  scale3.begin(DT3, SCK3);

  scale1.set_scale(2280.0f);
  scale2.set_scale(2280.0f);
  scale3.set_scale(2280.0f);

  Serial.println("[SCALE] Taring — keep bins empty...");
  delay(200);
  scale1.tare();
  scale2.tare();
  scale3.tare();
  Serial.println("[SCALE] Tare done.");

  Serial.print("[WiFi] Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WiFi] Connected — IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] Failed — running offline.");
  }

  config.api_key                    = API_KEY;
  config.database_url               = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("[FB] Firebase ready.");
}


//  LOOP 

void loop() {
  unsigned long now = millis();

  handleButtons();

  if (now - prevSensorMillis < SENSOR_INTERVAL) return;
  prevSensorMillis = now;

  float d1 = getDistance(TRIG1, ECHO1);
  float d2 = getDistance(TRIG2, ECHO2);
  float d3 = getDistance(TRIG3, ECHO3);

  String l1 = getLevel(d1);
  String l2 = getLevel(d2);
  String l3 = getLevel(d3);

  float w1 = movingAvg(wBuf1, getRawWeight(scale1));
  float w2 = movingAvg(wBuf2, getRawWeight(scale2));
  float w3 = movingAvg(wBuf3, getRawWeight(scale3));

  wIdx = (wIdx + 1) % WEIGHT_SAMPLES;
  if (wIdx == 0) wFull = true;

  Serial.println("\n===== SMART BIN =====");
  Serial.printf("Plastic : %-6s | %6.1f g | %.1f cm\n", l1.c_str(), w1, d1);
  Serial.printf("Food    : %-6s | %6.1f g | %.1f cm\n", l2.c_str(), w2, d2);
  Serial.printf("Metal   : %-6s | %6.1f g | %.1f cm\n", l3.c_str(), w3, d3);

  controlLED(LED1, l1);
  controlLED(LED2, l2);
  controlLED(LED3, l3);

  handleBuzzer(l1, l2, l3);

  if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
    sendToFirebase(l1, l2, l3, w1, w2, w3);
  }
}