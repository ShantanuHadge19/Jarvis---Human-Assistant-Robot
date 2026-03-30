/******************** BLYNK CONFIG ********************/
#define BLYNK_TEMPLATE_ID   "TMPL3jfYOcF0d"
#define BLYNK_TEMPLATE_NAME "Jarvis2"
#define BLYNK_AUTH_TOKEN    "7oZtLVbsRGxDiFOijmFzbYRlZcc5wnwP"
/*****************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

bool humanMotionActive = false;

bool shoulderUp = false, shoulderDown = false;
bool elbowUp = false, elbowDown = false;
bool bendUp = false, bendDown = false;


unsigned long lastManualMove = 0;
const int MANUAL_INTERVAL = 40; // ms (adjust speed here)
/* ===================== WIFI ===================== */
char ssid[] = "Jarvis";
char pass[] = "Shantanu";

/* ===================== VOICE MODULE ===================== */
#define VOICE_SERIAL Serial2
#define VOICE_RX 16
#define VOICE_TX 17

/* ===================== PCA9685 ===================== */
Adafruit_PWMServoDriver pca(0x40);
#define SERVO_FREQ 50

#define ELBOW_CH     0
#define BEND_CH      1
#define SHOULDER_CH  2
#define GRIPPER_CH   4
#define WRIST_CH 6

#define SERVO_MIN 100
#define SERVO_MAX 500

/* ===================== SERVO LIMITS ===================== */
#define ELBOW_MIN     0
#define ELBOW_MAX     60 //30

#define BEND_MIN      20
#define BEND_MAX      100  //80

#define SHOULDER_MIN  30
#define SHOULDER_MAX  145  //120

#define GRIP_OPEN    105
#define GRIP_CLOSE   30

#define WRIST_MIN  35
#define WRIST_MAX  140



/* ===================== ARM STATE TRACKING ===================== */
int currentElbow     = 30;
int currentBend      = 80;
int currentShoulder  = 60;
bool currentGripOpen = true;
int currentWrist = 55;


/* ===== NEW: TARGET VALUES FOR SMOOTHING ===== */
int targetElbow     = 30;
int targetBend      = 80;
int targetShoulder  = 60;
int targetWrist  = 55;

const int SMOOTH_STEP = 1;   // smaller = smoother
unsigned long lastServoUpdate = 0;
//const unsigned long SERVO_UPDATE_INTERVAL = 40; // milliseconds

/* ===================== MOTORS ===================== */
const int AIN1 = 26, AIN2 = 25, PWMA = 33;
const int BIN1 = 14, BIN2 = 12, PWMB = 13;
const int CIN1 = 5,  CIN2 = 18, PWMC = 19;
const int DIN1 = 32, DIN2 = 4,  PWMD = 23;
const int STBY = 27;

#define SPEED 180

/* ===================== BLYNK ===================== */
#define VPIN_FOLLOW      V0
#define VPIN_STOP        V1
#define VPIN_GIVE_CTRL   V2
#define VPIN_TAKE_CTRL   V3
#define VPIN_FIND_BOTTLE V4
#define VPIN_STATUS      V9

/* ===================== MODES ===================== */
enum RobotMode {
  MODE_IDLE = 0,
  MODE_HUMAN_TRACK,
  MODE_GESTURE,
  MODE_REMOTE
};

RobotMode currentMode = MODE_IDLE;

/* ===================== STATE ===================== */
String piBuffer = "";
String activeGesture = "STOP";

unsigned long lastFaceUpdate = 0;
unsigned long lastGestureUpdate = 0;

const unsigned long FACE_TIMEOUT = 500;
const unsigned long GESTURE_FAILSAFE = 5000;

/* ===================== SERVO ===================== */
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setServo(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  pca.setPWM(ch, 0, angleToPulse(angle));
}

void moveGripperSmooth(int target) {
  static int current = 90;

  while (current != target) {
    if (current < target) current++;
    else current--;

    setServo(GRIPPER_CH, current);
    delay(10);
  }
}

/* ===================== SMOOTH ARM UPDATE ===================== */
void updateArmSmooth() {
  unsigned long interval;
  int step;
  
  if (currentMode == MODE_REMOTE) {

    if (millis() - lastManualMove < MANUAL_INTERVAL)
      return;

    lastManualMove = millis();

    if (shoulderUp)   currentShoulder += 1;
    if (shoulderDown) currentShoulder -= 1;

    if (elbowUp)      currentElbow += 1;
    if (elbowDown)    currentElbow -= 1;

    if (bendUp)       currentBend += 1;
    if (bendDown)     currentBend -= 1;

    currentShoulder = constrain(currentShoulder, SHOULDER_MIN, SHOULDER_MAX);
    currentElbow    = constrain(currentElbow, ELBOW_MIN, ELBOW_MAX);
    currentBend     = constrain(currentBend, BEND_MIN, BEND_MAX);

    setServo(SHOULDER_CH, currentShoulder);
    setServo(ELBOW_CH, currentElbow);
    setServo(BEND_CH, currentBend);

    return;
  }
  

  // Fast response for gesture control
  if (currentMode == MODE_GESTURE) {
      interval = 15;   // faster updates
      step = 1;       // larger step movement
  } 
  else {
      interval = 30;  // smooth movement
      step = 1;
  }


  if (millis() - lastServoUpdate < interval)
      return;

  lastServoUpdate = millis();

  if (currentElbow < targetElbow)
    currentElbow = min(currentElbow + step, targetElbow);
  else if (currentElbow > targetElbow)
    currentElbow = max(currentElbow - step, targetElbow);

  if (currentBend < targetBend)
    currentBend = min(currentBend + step, targetBend);
  else if (currentBend > targetBend)
    currentBend = max(currentBend - step, targetBend);

  if (currentShoulder < targetShoulder)
    currentShoulder = min(currentShoulder + step, targetShoulder);
  else if (currentShoulder > targetShoulder)
    currentShoulder = max(currentShoulder - step, targetShoulder);

  //if (currentBend < targetBend) currentBend += step;
  //else if (currentBend > targetBend) currentBend -= step;

  //if (currentShoulder < targetShoulder) currentShoulder += step;
  //else if (currentShoulder > targetShoulder) currentShoulder -= step;

  // WRIST SMOOTH
  if (currentWrist < targetWrist)
    currentWrist = min(currentWrist + step, targetWrist);
  else if (currentWrist > targetWrist)
    currentWrist = max(currentWrist - step, targetWrist);

  setServo(WRIST_CH, currentWrist);
  setServo(ELBOW_CH, currentElbow);
  setServo(BEND_CH, currentBend);
  setServo(SHOULDER_CH, currentShoulder);
}

/* ===================== MOTOR CORE ===================== */
void driveMotor(int in1, int in2, int pwm, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  analogWrite(pwm, SPEED);
}

void stopAllMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  analogWrite(PWMC, 0);
  analogWrite(PWMD, 0);
}

/* ===================== MOVEMENT ===================== */
void moveForward() {
  driveMotor(AIN1, AIN2, PWMA, true);
  driveMotor(BIN1, BIN2, PWMB, true);
  driveMotor(CIN1, CIN2, PWMC, true);
  driveMotor(DIN1, DIN2, PWMD, true);
}

void moveBackward() {
  driveMotor(AIN1, AIN2, PWMA, false);
  driveMotor(BIN1, BIN2, PWMB, false);
  driveMotor(CIN1, CIN2, PWMC, false);
  driveMotor(DIN1, DIN2, PWMD, false);
}

void rotateCW() {
  driveMotor(AIN1, AIN2, PWMA, false);
  driveMotor(BIN1, BIN2, PWMB, false);
  driveMotor(CIN1, CIN2, PWMC, true);
  driveMotor(DIN1, DIN2, PWMD, true);
}

void rotateCCW() {
  driveMotor(AIN1, AIN2, PWMA, true);
  driveMotor(BIN1, BIN2, PWMB, true);
  driveMotor(CIN1, CIN2, PWMC, false);
  driveMotor(DIN1, DIN2, PWMD, false);
}

void Right() {
  driveMotor(AIN1, AIN2, PWMA, true);
  driveMotor(BIN1, BIN2, PWMB, false);
  driveMotor(CIN1, CIN2, PWMC, false);
  driveMotor(DIN1, DIN2, PWMD, true);
}

void Left() {
  driveMotor(AIN1, AIN2, PWMA, false);
  driveMotor(BIN1, BIN2, PWMB, true);
  driveMotor(CIN1, CIN2, PWMC, true);
  driveMotor(DIN1, DIN2, PWMD, false);
}

/* ===================== APPLY GESTURE ===================== */
void applyGesture() {
  if (currentMode == MODE_REMOTE) return;
  if      (activeGesture == "FORWARD")  moveForward();
  else if (activeGesture == "BACKWARD") moveBackward();
  else if (activeGesture == "CW")       rotateCW();
  else if (activeGesture == "CCW")      rotateCCW();
  else                                  stopAllMotors();
}

/* ===================== SERIAL FROM PI ===================== */
void readSerialFromPi() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      String line = piBuffer;
      piBuffer = "";
      line.trim();

      //Blynk.virtualWrite(VPIN_STATUS, line + "\n");

      if      (line == "MODE:HUMAN_TRACK") currentMode = MODE_HUMAN_TRACK;
      else if (line == "MODE:GESTURE")     currentMode = MODE_GESTURE;
      else if (line == "MODE:IDLE"){   
        currentMode = MODE_IDLE;
        stopAllMotors(); 
        activeGesture = "STOP";
      }
      else if (line == "MODE:REMOTE"){     
        currentMode = MODE_REMOTE;
        stopAllMotors();        // ✅ VERY IMPORTANT
        activeGesture = "STOP"; // ✅ VERY IMPORTANT
        shoulderUp = shoulderDown = false;
        elbowUp = elbowDown = false;
        bendUp = bendDown = false;
      }
      else if (line.startsWith("GESTURE:") && currentMode != MODE_REMOTE) {
        activeGesture = line.substring(8);
        lastGestureUpdate = millis();
        lastFaceUpdate = millis();

        if (activeGesture == "STOP") {
          stopAllMotors();
          humanMotionActive = false;
        } 
        else if (currentMode == MODE_HUMAN_TRACK) {
          humanMotionActive = true;
        }
      }

      // ===== ARM TARGET UPDATE (NO DIRECT SERVO CALL) =====
      else if (line.startsWith("ARM:ELBOW:")) {

        int newVal = constrain(line.substring(10).toInt(), ELBOW_MIN, ELBOW_MAX);

        
        targetElbow = newVal;
        
      }

      else if (line.startsWith("ARM:BEND:")) {

        int newVal = constrain(line.substring(9).toInt(), BEND_MIN, BEND_MAX);

        
        targetBend = newVal;
        
      }

      else if (line.startsWith("ARM:SHOULDER:")) {

        int newVal = constrain(line.substring(13).toInt(), SHOULDER_MIN, SHOULDER_MAX);

       
        targetShoulder = newVal;
        
      }

      else if (line.startsWith("ARM:WRIST:")) {

        int newVal = constrain(line.substring(11).toInt(), WRIST_MIN, WRIST_MAX);

        targetWrist = newVal;
      }

      else if (line == "ARM:GRIP:OPEN") {
        currentGripOpen = true;
        moveGripperSmooth(GRIP_OPEN);
      }

      else if (line == "ARM:GRIP:CLOSE") {
        currentGripOpen = false;
        moveGripperSmooth(GRIP_CLOSE);
      }
    }
    else {
      piBuffer += c;
      if (piBuffer.length() > 300) piBuffer = "";
    }
  }
}

/* ===================== VOICE HANDLER ===================== */
void handleVoiceCommand(uint8_t cmd) {
  switch (cmd) {
    case 0x04: Serial.println("CMD:FOLLOW_SHANTANU"); break;
    case 0x08: Serial.println("CMD:STOP_FOLLOW");     break;
    case 0x09: Serial.println("CMD:GESTURE_ON");      break;
    case 0x10: Serial.println("CMD:GESTURE_OFF");     break;
    case 0x17: Serial.println("CMD:FIND:BOTTLE");     break; 
    case 0x18: Serial.println("CMD:STOP_FIND");        break;
    case 0x13: Serial.println("CMD:REMOTE_ON");       break;
    case 0x14: Serial.println("CMD:REMOTE_OFF");      break;
    case 0x11: Serial.println("CMD:PICK_THE_CUP");    break;
    case 0x27: Serial.println("CMD:SLEEP");     break;
    
  }
}

void readVoiceModule() {
  static bool waitAA = false;
  while (VOICE_SERIAL.available()) {
    uint8_t b = VOICE_SERIAL.read();
    if (!waitAA) {
      if (b == 0xAA) waitAA = true;
    } else {
      handleVoiceCommand(b);
      waitAA = false;
    }
  }
}

void sendVoice(uint8_t cmd) {
  delay(50);
  VOICE_SERIAL.write(0xAA);
  delay(10);
  VOICE_SERIAL.write(cmd);
}

/* ===================== BLYNK ===================== */
BLYNK_WRITE(VPIN_FOLLOW)    { 
  if (param.asInt()) {
    Serial.println("CMD:FOLLOW_SHANTANU"); 
    delay(50);
    sendVoice(0x04);
  }
}

BLYNK_WRITE(VPIN_STOP)      { 
  if (param.asInt()) {
    Serial.println("CMD:STOP_FOLLOW"); 
    delay(50);
    sendVoice(0x08);
  }
}

BLYNK_WRITE(VPIN_GIVE_CTRL) { 
  if (param.asInt()) {
    Serial.println("CMD:GESTURE_ON"); 
    delay(50);
    sendVoice(0x09);
  }
}

BLYNK_WRITE(VPIN_TAKE_CTRL) { 
  if (param.asInt()) {
    Serial.println("CMD:GESTURE_OFF"); 
    delay(50);
    sendVoice(0x10);
  }
}

BLYNK_WRITE(VPIN_FIND_BOTTLE) {
  if (param.asInt()) {
    Serial.println("CMD:FIND:BOTTLE");
    delay(50);
    sendVoice(0x17);
  }
}
// ===================== MODE + TASK CONTROL =====================

BLYNK_WRITE(V5) { // PICK THE CUP
  if (param.asInt()) {
    Serial.println("CMD:PICK_THE_CUP");
    delay(50);
    sendVoice(0x11);
  }
}

BLYNK_WRITE(V6) { // PUT THE CUP
  if (param.asInt()) Serial.println("CMD:STOP_FIND");
  delay(50);
  sendVoice(0x18);
}

BLYNK_WRITE(V7) { // SLEEP (future use)
  if (param.asInt()) {
    Serial.println("CMD:SLEEP");
  }
}

BLYNK_WRITE(V8) { // REMOTE ON
  if (param.asInt()) {
    Serial.println("CMD:REMOTE_ON");
    sendVoice(0x13);
  }
}

BLYNK_WRITE(V25) { // REMOTE OFF
  if (param.asInt()) {
    Serial.println("CMD:REMOTE_OFF");
    sendVoice(0x14);
  }
}

BLYNK_WRITE(V10) { // FORWARD
  if (currentMode != MODE_REMOTE) return;
  if (param.asInt()) moveForward();
  else stopAllMotors();
}

BLYNK_WRITE(V11) { // BACKWARD
  if (currentMode != MODE_REMOTE) return;
  if (param.asInt()) moveBackward();
  else stopAllMotors();
}

BLYNK_WRITE(V12) { // LEFT (strafe left or rotate CCW if needed)
  if (currentMode != MODE_REMOTE) return;
  if (param.asInt()) Left();
  else stopAllMotors();
}

BLYNK_WRITE(V13) { // RIGHT`
  if (currentMode != MODE_REMOTE) return;
  if (param.asInt()) Right();
  else stopAllMotors();
}

BLYNK_WRITE(V15) { // CW
  if (currentMode != MODE_REMOTE) return;
  if (param.asInt()) rotateCW();
  else stopAllMotors();
}

BLYNK_WRITE(V16) { // CCW
  if (currentMode != MODE_REMOTE) return;
  if (param.asInt()) rotateCCW();
  else stopAllMotors();
}

BLYNK_WRITE(V17) { shoulderUp = param.asInt(); }
BLYNK_WRITE(V18) { shoulderDown = param.asInt(); }

BLYNK_WRITE(V19) { elbowUp = param.asInt(); }
BLYNK_WRITE(V20) { elbowDown = param.asInt(); }

BLYNK_WRITE(V21) { bendUp = param.asInt(); }
BLYNK_WRITE(V22) { bendDown = param.asInt(); }

BLYNK_WRITE(V23) {
  if (currentMode != MODE_REMOTE) return;
  if (param.asInt()) {
    currentGripOpen = false;
    moveGripperSmooth(GRIP_CLOSE);
  }
}

BLYNK_WRITE(V24) {
  if (currentMode != MODE_REMOTE) return;
  if (param.asInt()) {
    currentGripOpen = true;
    moveGripperSmooth(GRIP_OPEN);
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  VOICE_SERIAL.begin(9600, SERIAL_8N1, VOICE_RX, VOICE_TX);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(CIN1, OUTPUT); pinMode(CIN2, OUTPUT);
  pinMode(DIN1, OUTPUT); pinMode(DIN2, OUTPUT);

  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);

  setServo(ELBOW_CH, currentElbow);
  setServo(BEND_CH, currentBend);
  setServo(SHOULDER_CH, currentShoulder);
  setServo(GRIPPER_CH, GRIP_OPEN);
  setServo(WRIST_CH, currentWrist);  // 35 degrees

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(300);

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();
}

/* ===================== LOOP ===================== */
void loop() {

  //Blynk.run();
  readSerialFromPi();
  updateArmSmooth();
  Blynk.run();
  readVoiceModule();

  if (currentMode == MODE_IDLE ||
      currentMode == MODE_GESTURE ||
      currentMode == MODE_HUMAN_TRACK) {

    applyGesture();

    if (millis() - lastGestureUpdate > GESTURE_FAILSAFE)
        activeGesture = "STOP";
  }
  else if (currentMode == MODE_REMOTE) {
    activeGesture = "STOP";
  // Do nothing (Blynk will control)
  }

  // ===== SMOOTH ARM UPDATE CALLED EVERY LOOP =====
  //updateArmSmooth();
}
