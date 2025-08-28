// ====== Dual-MPU Posture Monitor with Built-in Motion Detection ======
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>
#include <SoftwareSerial.h>

// ---------- Pins ----------
const uint8_t BTN_PIN   = 2;  // to GND
const uint8_t SERVO_UP  = 3;  // upper back servo
const uint8_t SERVO_LOW = 5;  // lower back servo

// ESP8266 on SoftwareSerial
SoftwareSerial esp(8, 9);  // RX = 8, TX = 9

// ---------- Dual IMU ----------
const uint8_t MPU_ADDR_UPPER = 0x68;  // Upper back
const uint8_t MPU_ADDR_LOWER = 0x69;  // Lower back
MPU6050 mpuUpper(MPU_ADDR_UPPER);
MPU6050 mpuLower(MPU_ADDR_LOWER);

// Flip if forward/back feels inverted after calibration
// Upper MPU: pins outgoing, Lower MPU: pins ingoing (opposite orientations)
int UPPER_PITCH_SIGN = +1;  // Upper MPU with pins outward
int LOWER_PITCH_SIGN = -1;  // Lower MPU with pins inward (opposite orientation)

// ---------- Tuning (snappy, responsive) ----------
float ALPHA        = 0.45f;   // faster filter (0.3..0.6)
float DEAD_DEG     = 0.8f;    // smaller deadband -> more frequent moves
float GAIN         = 3.5f;    // larger gain -> bigger jumps
int   MAX_DELTA_DEG = 80;     // cap from center (0..90 typical)

// Snap when deviation is large or changes abruptly:
float SNAP_DEG        = 8.0f;   // if |deviation| >= SNAP_DEG -> snap to max
float JERK_SNAP_DEG   = 6.0f;   // if |delta deviation between loops| >= this -> snap

// ---------- Servos ----------
Servo upperServo, lowerServo;
const int SERVO_CENTER_UP  = 90;
const int SERVO_CENTER_LOW = 90;
int UP_DIR  = +1;  // flip to -1 if upper rotates wrong way
int LOW_DIR = +1;  // flip to -1 if lower rotates wrong way

// ===== Servo Vibration System =====
bool upperServoVibrating = false;
bool lowerServoVibrating = false;
unsigned long lastVibeMs = 0;
bool vibePhase = false;
const unsigned long VIBE_PERIOD_MS = 250;  // Vibration speed (ms) - made faster
const int VIBE_AMPLITUDE = 25;              // Vibration strength (degrees) - made stronger

// ===== Wi-Fi / ThingSpeak Configuration =====
const char* WIFI_SSID = "Naveed";       
const char* WIFI_PASS = "nav200211";        
const char* TS_HOST   = "api.thingspeak.com";
const char* TS_APIKEY = "5ILUVXSMC5IL0SDR"; 
const int   TS_PORT   = 80;
const unsigned long THINGSPEAK_INTERVAL_MS = 20000;  // 20 seconds
unsigned long lastThingSpeakMs = 0;

// ===== Motion Detection Parameters =====
const unsigned long MOTION_CHECK_MS = 300;  // Check every 300ms
unsigned long lastMotionCheckMs = 0;

// ===== System State =====
enum SystemMode {
  MODE_SITTING = 0,
  MODE_WALKING = 1, 
  MODE_CALIBRATING = 2
};

SystemMode currentMode = MODE_SITTING;
SystemMode previousMode = MODE_SITTING;

// ===== Analytics Data =====
struct PostureStats {
  unsigned long sessionStartMs = 0;
  int badPostureSitting = 0;
  int badPostureWalking = 0;
  int totalReadings = 0;
  int goodPostureReadings = 0;
  bool lastPostureWasBad = false;
  float totalDeviationSum = 0.0f;
  int severeBadPostureCount = 0;
  unsigned long walkingTimeMs = 0;
  unsigned long sittingTimeMs = 0;
  unsigned long modeStartMs = 0;
};

PostureStats stats;

// ---------- State ----------
struct PostureState {
  float neutralPitchUpper = 0.0f;
  float neutralPitchLower = 0.0f;
  float filtUpperDev = 0.0f;      // Filtered upper deviation
  float filtLowerDev = 0.0f;      // Filtered lower deviation  
  float filtLordosis = 0.0f;      // Filtered lordosis (lower - upper)
  bool isCalibrated = false;
};

PostureState posture;

float prevUpperRel = 0.0f;
float prevLowerRel = 0.0f;
float prevLordosis = 0.0f;

unsigned long lastCalibMs = 0;
const unsigned long CAL_HOLD_MS = 600; // brief freeze after calibration

// Debug print throttle
unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 120;

//////////////////// Continuous Servo Movement Function ////////////////////

void updateServoVibration() {
  // Update vibration timing
  if (millis() - lastVibeMs > VIBE_PERIOD_MS) {
    vibePhase = !vibePhase;
    lastVibeMs = millis();
  }
  
  // Upper servo - vibrate if flag is set
  if (upperServoVibrating) {
    // Scale amplitude based on severity for better feedback
    int amplitude = constrain(int(abs(posture.filtUpperDev) * 3), VIBE_AMPLITUDE/2, VIBE_AMPLITUDE*2);
    int vibeOffset = vibePhase ? amplitude : -amplitude;
    int target = SERVO_CENTER_UP + UP_DIR * vibeOffset;
    target = constrain(target, 0, 180);
    upperServo.write(target);
  } else {
    upperServo.write(SERVO_CENTER_UP);  // Center when not vibrating
  }
  
  // Lower servo - vibrate if flag is set  
  if (lowerServoVibrating) {
    // Scale amplitude based on severity for better feedback
    int amplitude = constrain(int(abs(posture.filtLordosis) * 3), VIBE_AMPLITUDE/2, VIBE_AMPLITUDE*2);
    int vibeOffset = vibePhase ? amplitude : -amplitude;
    int target = SERVO_CENTER_LOW + LOW_DIR * vibeOffset;
    target = constrain(target, 0, 180);
    lowerServo.write(target);
  } else {
    lowerServo.write(SERVO_CENTER_LOW);  // Center when not vibrating
  }
}

//////////////////// ESP Helper Functions ////////////////////

void espFlushRx(unsigned long ms = 50) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (esp.available()) { Serial.write(esp.read()); }
  }
}

bool espWait(const char* token, unsigned long ms = 8000, bool echoToUSB = true) {
  unsigned long t0 = millis();
  size_t n = strlen(token), m = 0;
  while (millis() - t0 < ms) {
    while (esp.available()) {
      char c = esp.read();
      if (echoToUSB) Serial.write(c);
      if (c == token[m]) { m++; if (m == n) return true; }
      else m = (c == token[0]) ? 1 : 0;
    }
  }
  return false;
}

bool espCmd(const String& cmd, const char* expect = "OK", unsigned long tmo = 8000, int postDelay = 50) {
  Serial.print(F("\n>> ")); Serial.println(cmd);
  esp.println(cmd);
  bool ok = espWait(expect, tmo, true);
  delay(postDelay);
  return ok;
}

bool espConnectWiFi() {
  if (!espCmd(F("AT"))) {
    Serial.println(F("ESP not responding to AT"));
    return false;
  }

  if (!espCmd(F("AT+CWMODE=1"))) {
    Serial.println(F("Failed to set CWMODE=1"));
    return false;
  }

  espCmd(F("AT+CIPMUX=0"));

  String join = String(F("AT+CWJAP=\"")) + WIFI_SSID + F("\",\"") + WIFI_PASS + F("\"");
  Serial.println(F("Joining Wi-Fi..."));
  esp.println(join);

  bool gotIP = espWait("WIFI GOT IP", 20000, true);
  bool ok    = espWait("OK",          5000,  true);
  if (!(gotIP || ok)) {
    Serial.println(F("\nWi-Fi join failed"));
    return false;
  }

  espCmd(F("AT+CIFSR"));
  Serial.println(F("\nWi-Fi connected."));
  return true;
}

//////////////////// Motion Detection using Simple Acceleration Magnitude ////////////////////

void detectWalkingMode() {
  if (millis() - lastMotionCheckMs < MOTION_CHECK_MS) return;
  lastMotionCheckMs = millis();
  
  // Get raw acceleration values from upper MPU
  int16_t ax, ay, az, gx, gy, gz;
  mpuUpper.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate total acceleration magnitude
  float accelMagnitude = sqrt(float(ax)*ax + float(ay)*ay + float(az)*az);
  
  // Simple moving average filter
  static float accelHistory[5] = {16384, 16384, 16384, 16384, 16384}; // Initialize to 1g
  static int historyIndex = 0;
  
  // Update history
  accelHistory[historyIndex] = accelMagnitude;
  historyIndex = (historyIndex + 1) % 5;
  
  // Calculate average
  float avgAccel = 0;
  for (int i = 0; i < 5; i++) {
    avgAccel += accelHistory[i];
  }
  avgAccel /= 5;
  
  // Calculate variance (difference from average)
  float variance = 0;
  for (int i = 0; i < 5; i++) {
    float diff = accelHistory[i] - avgAccel;
    variance += diff * diff;
  }
  variance = sqrt(variance / 5);
  
  // Debug output every 10th check
  static int debugCounter = 0;
  if (++debugCounter >= 10) {
    debugCounter = 0;
    Serial.print(F("Accel: avg=")); Serial.print(avgAccel, 0);
    Serial.print(F(" var=")); Serial.print(variance, 0);
    Serial.print(F(" raw=")); Serial.print(accelMagnitude, 0);
    Serial.println();
  }
  
  // Walking: High variance in acceleration (rhythmic stepping)
  // Sitting: Low variance (steady orientation)
  if (variance > 3000.0f && avgAccel > 14000.0f) {  // High variance + reasonable acceleration
    if (currentMode != MODE_WALKING && currentMode != MODE_CALIBRATING) {
      Serial.print(F("WALKING MODE - Variance: ")); Serial.print(variance, 0);
      Serial.print(F(" Avg: ")); Serial.println(avgAccel, 0);
      currentMode = MODE_WALKING;
    }
  } else {
    if (currentMode == MODE_WALKING) {
      Serial.print(F("SITTING MODE - Variance: ")); Serial.print(variance, 0);
      Serial.print(F(" Avg: ")); Serial.println(avgAccel, 0);
      currentMode = MODE_SITTING;
    }
  }
}

//////////////////// Analytics Functions ////////////////////

void updatePostureStats(float upperDev, float lordosis) {
  unsigned long currentMs = millis();
  stats.totalReadings++;
  
  // Track mode durations
  if (currentMode != previousMode) {
    if (previousMode == MODE_WALKING) {
      stats.walkingTimeMs += currentMs - stats.modeStartMs;
    } else if (previousMode == MODE_SITTING) {
      stats.sittingTimeMs += currentMs - stats.modeStartMs;
    }
    stats.modeStartMs = currentMs;
    previousMode = currentMode;
  }
  
  // Determine if posture is bad
  bool upperBad = (upperDev > DEAD_DEG);
  bool lordosisBad = (abs(lordosis) > DEAD_DEG);
  bool currentPostureIsBad = upperBad || lordosisBad;
  bool currentPostureIsSevere = (upperDev > DEAD_DEG * 3.0f) || (abs(lordosis) > DEAD_DEG * 3.0f);
  
  stats.totalDeviationSum += abs(upperDev) + abs(lordosis);
  
  if (currentPostureIsBad) {
    if (!stats.lastPostureWasBad) {
      if (currentMode == MODE_WALKING) {
        stats.badPostureWalking++;
      } else {
        stats.badPostureSitting++;
      }
    }
    
    if (currentPostureIsSevere) {
      stats.severeBadPostureCount++;
    }
  } else {
    stats.goodPostureReadings++;
  }
  
  stats.lastPostureWasBad = currentPostureIsBad;
}

int calculateQualityScore() {
  if (stats.totalReadings == 0) return 100;
  
  float goodRatio = float(stats.goodPostureReadings) / float(stats.totalReadings);
  float baseScore = goodRatio * 70;
  
  float avgDeviation = stats.totalDeviationSum / float(stats.totalReadings);
  float deviationPenalty = constrain(avgDeviation * 3, 0, 20);
  
  float severePenalty = constrain(stats.severeBadPostureCount * 2, 0, 10);
  
  return constrain(int(baseScore - deviationPenalty - severePenalty + 30), 0, 100);
}

//////////////////// ThingSpeak Update (All 8 Fields) ////////////////////

bool thingspeakUpdate() {
  if (millis() - lastThingSpeakMs < THINGSPEAK_INTERVAL_MS) {
    return false;
  }
  
  if (!posture.isCalibrated) {
    Serial.println(F("TS: Not calibrated, skipping"));
    return false;
  }
  
  lastThingSpeakMs = millis();
  
  // Calculate all metrics
  float sessionMinutes = (millis() - stats.sessionStartMs) / 60000.0f;
  int qualityScore = calculateQualityScore();
  int currentStatus = stats.lastPostureWasBad ? 1 : 0;
  
  Serial.println(F("=== SENDING TO THINGSPEAK ==="));
  Serial.print(F("Upper: ")); Serial.print(posture.filtUpperDev, 1);
  Serial.print(F(" Lordosis: ")); Serial.print(posture.filtLordosis, 1);
  Serial.print(F(" Mode: ")); Serial.print(currentMode == MODE_WALKING ? "WALK" : "SIT");
  Serial.print(F(" Quality: ")); Serial.print(qualityScore);
  Serial.println(F("%"));
  
  // TCP connection
  String cipstart = String(F("AT+CIPSTART=\"TCP\",\"")) + TS_HOST + F("\",") + TS_PORT;
  if (!espCmd(cipstart.c_str(), "OK", 8000, 100)) {
    if (!espWait("ALREADY", 500, true)) {
      Serial.println(F("CIPSTART failed"));
      return false;
    }
  }
  
  // HTTP request - send all 8 fields matching your channel configuration
  String line = String("GET /update?api_key=") + TS_APIKEY +
                "&field1=" + String(posture.filtUpperDev, 2) +        // Upper Back Angle
                "&field2=" + String(posture.filtLordosis, 2) +        // Lower Back Angle (Lordosis)
                "&field3=" + String(stats.badPostureSitting) +        // Bad Posture Count (Sitting)
                "&field4=" + String(stats.badPostureWalking) +        // Bad Posture Count (Walking)
                "&field5=" + String(int(currentMode)) +               // System Mode (0=Sitting, 1=Walking)
                "&field6=" + String(qualityScore) +                  // Posture Quality Score
                "&field7=" + String(sessionMinutes, 1) +             // Session Duration
                "&field8=" + String(currentStatus) +                 // Current Status (0=Good, 1=Bad)
                " HTTP/1.1\r\n";
  String hdrs = String("Host: ") + TS_HOST + "\r\nConnection: close\r\n\r\n";
  String req = line + hdrs;
  
  String cipsend = String(F("AT+CIPSEND=")) + req.length();
  if (!espCmd(cipsend.c_str(), ">", 3000, 0)) {
    Serial.println(F("CIPSEND prompt '>' not received"));
    espCmd(F("AT+CIPCLOSE"));
    return false;
  }
  
  Serial.println(F(">> (HTTP request)"));
  esp.print(req);
  
  bool sent = espWait("SEND OK", 5000, true);
  espCmd(F("AT+CIPCLOSE"));
  
  if (sent) {
    Serial.println(F("ThingSpeak: SEND OK"));
    printSessionStats();
  } else {
    Serial.println(F("ThingSpeak: SEND failed"));
  }
  return sent;
}

void printSessionStats() {
  Serial.println(F("\n=== SESSION ANALYTICS ==="));
  Serial.print(F("Duration: ")); Serial.print((millis() - stats.sessionStartMs)/60000.0f, 1); Serial.println(F(" min"));
  Serial.print(F("Sitting incidents: ")); Serial.println(stats.badPostureSitting);
  Serial.print(F("Walking incidents: ")); Serial.println(stats.badPostureWalking);
  Serial.print(F("Quality: ")); Serial.print(calculateQualityScore()); Serial.println(F("%"));
  Serial.println(F("==========================\n"));
}

// ---------- Helpers ----------
static inline float computePitchDeg(MPU6050 &mpu, int pitchSign) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // X toward feet; minus sign makes forward positive (after pitchSign)
  float pitch = atan2f(-float(ax), sqrtf(float(ay)*ay + float(az)*az)) * 180.0f / PI;
  return pitchSign * pitch;
}

void centerServos() {
  upperServo.write(SERVO_CENTER_UP);
  lowerServo.write(SERVO_CENTER_LOW);
}

void calibrateNeutral() {
  const int N = 180;
  double sumUpper = 0, sumLower = 0;
  Serial.println(F("CALIBRATING... sit in perfect posture and stay still"));
  
  currentMode = MODE_CALIBRATING;
  
  // Stop vibrations during calibration
  upperServoVibrating = false;
  lowerServoVibrating = false;
  centerServos();
  
  for (int i = 0; i < N; i++) {
    sumUpper += computePitchDeg(mpuUpper, UPPER_PITCH_SIGN);
    sumLower += computePitchDeg(mpuLower, LOWER_PITCH_SIGN);
    if (i % 30 == 0) Serial.print(F("."));
    delay(20);  // 3.6 seconds total
  }
  Serial.println();
  
  posture.neutralPitchUpper = sumUpper / N;
  posture.neutralPitchLower = sumLower / N;
  posture.filtUpperDev = 0.0f;
  posture.filtLowerDev = 0.0f;
  posture.filtLordosis = 0.0f;
  posture.isCalibrated = true;
  
  prevUpperRel = 0.0f;
  prevLowerRel = 0.0f;
  prevLordosis = 0.0f;
  
  // Reset statistics
  memset(&stats, 0, sizeof(stats));
  stats.sessionStartMs = millis();
  stats.modeStartMs = millis();
  
  currentMode = MODE_SITTING;
  
  centerServos();
  lastCalibMs = millis();

  Serial.print(F("CALIBRATED Upper="));
  Serial.print(posture.neutralPitchUpper, 2);
  Serial.print(F(" Lower="));
  Serial.print(posture.neutralPitchLower, 2);
  Serial.println(F(" deg"));
}

void waitBtnRelease() {
  while (digitalRead(BTN_PIN) == LOW) { delay(3); }
  delay(30);
}

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  esp.begin(9600);
  delay(300);
  
  Serial.println(F("\nDual-MPU Posture Monitor with Built-in Motion Detection"));
  Serial.println(F("Upper forward -> Upper servo | Lordosis -> Lower servo | Both -> Both servos"));

  Wire.begin();
  Wire.setClock(400000);
  
  mpuUpper.initialize();
  mpuLower.initialize();
  
  if (!mpuUpper.testConnection()) {
    Serial.println(F("ERROR: Upper MPU6050 not detected"));
    while (true) { delay(1000); }
  }
  
  if (!mpuLower.testConnection()) {
    Serial.println(F("ERROR: Lower MPU6050 not detected"));
    while (true) { delay(1000); }
  }
  
  Serial.println(F("Dual MPU6050 sensors connected"));

  upperServo.attach(SERVO_UP);
  lowerServo.attach(SERVO_LOW);
  centerServos();
  delay(150);

  // Connect Wi-Fi
  espFlushRx();
  if (!espConnectWiFi()) {
    Serial.println(F("Wi-Fi failed, continuing offline"));
  }

  calibrateNeutral();
  Serial.println(F("Press button to recalibrate"));
}

void loop() {
  // Recalibrate
  if (digitalRead(BTN_PIN) == LOW) {
    delay(12);
    if (digitalRead(BTN_PIN) == LOW) {
      calibrateNeutral();
      waitBtnRelease();
    }
  }

  if (!posture.isCalibrated) {
    delay(10);
    return;
  }

  // Walking detection using built-in MPU6050 motion detection
  detectWalkingMode();

  // Read both IMUs
  float upperPitch = computePitchDeg(mpuUpper, UPPER_PITCH_SIGN);
  float lowerPitch = computePitchDeg(mpuLower, LOWER_PITCH_SIGN);
  
  float upperRel = upperPitch - posture.neutralPitchUpper;
  float lowerRel = lowerPitch - posture.neutralPitchLower;
  float lordosis = lowerRel - upperRel;  // Lordosis = lower back more curved than upper

  // Low-pass filter all three signals
  posture.filtUpperDev = ALPHA * upperRel + (1.0f - ALPHA) * posture.filtUpperDev;
  posture.filtLowerDev = ALPHA * lowerRel + (1.0f - ALPHA) * posture.filtLowerDev;
  posture.filtLordosis = ALPHA * lordosis + (1.0f - ALPHA) * posture.filtLordosis;

  // Update analytics
  updatePostureStats(posture.filtUpperDev, posture.filtLordosis);

  // FIXED SERVO CONTROL LOGIC - Now uses vibration system properly
  if (millis() - lastCalibMs < CAL_HOLD_MS) {
    // Stop all vibrations during calibration hold
    upperServoVibrating = false;
    lowerServoVibrating = false;
  } else {
    // Determine if posture is bad for each area
    bool upperBad = (posture.filtUpperDev > DEAD_DEG);
    bool lordosisBad = (fabs(posture.filtLordosis) > DEAD_DEG);
    
    // SET VIBRATION FLAGS - This is the key fix!
    upperServoVibrating = upperBad;
    lowerServoVibrating = lordosisBad;
    
    // Debug output to see what's happening
    static unsigned long lastDebugMs = 0;
    if (millis() - lastDebugMs > 1000) {  // Every 1000ms
      lastDebugMs = millis();
      Serial.print(F("POSTURE CHECK: upperBad=")); Serial.print(upperBad);
      Serial.print(F(" lordosisBad=")); Serial.print(lordosisBad);
      Serial.print(F(" upperDev=")); Serial.print(posture.filtUpperDev, 1);
      Serial.print(F(" lordosis=")); Serial.print(posture.filtLordosis, 1);
      
      if (upperBad || lordosisBad) {
        Serial.print(F(" -> VIBRATING: "));
        if (upperBad) Serial.print(F("UPPER "));
        if (lordosisBad) Serial.print(F("LOWER "));
      } else {
        Serial.print(F(" -> SERVOS OFF"));
      }
      Serial.println();
    }
  }

  // ALWAYS call the vibration update function - this handles continuous movement
  updateServoVibration();

  // Store previous values for jerk calculation
  prevUpperRel = upperRel;
  prevLowerRel = lowerRel;
  prevLordosis = lordosis;

  // Send to ThingSpeak (all 8 fields)
  thingspeakUpdate();

  // Debug output with walking mode
  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    Serial.print(F("Upper=")); Serial.print(posture.filtUpperDev,1);
    Serial.print(F(" Lordosis="));  Serial.print(posture.filtLordosis,1);
    Serial.print(F("  Mode: "));
    Serial.print(currentMode == MODE_WALKING ? F("WALKING") : F("SITTING"));
    Serial.print(F("  Servos: "));
    
    if (millis() - lastCalibMs < CAL_HOLD_MS) {
      Serial.print(F("HOLD"));
    } else {
      bool upperBad = (posture.filtUpperDev > DEAD_DEG);
      bool lordosisBad = (fabs(posture.filtLordosis) > DEAD_DEG);
      
      if (upperBad && lordosisBad) Serial.print(F("BOTH-VIBRATING"));
      else if (upperBad) Serial.print(F("UPPER-VIBRATING"));
      else if (lordosisBad) Serial.print(F("LOWER-VIBRATING"));
      else Serial.print(F("OFF"));
    }
    Serial.println();
  }

  delay(4); // ~200–250 Hz loop for snappy response
}