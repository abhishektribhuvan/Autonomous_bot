#include "BluetoothSerial.h"
#include "SPI.h"
#include "MFRC522.h"

// Constants
const int IR_SENSOR_PIN_START = 12;
const int NUM_IR_SENSORS = 7;
const int WEIGHT_ARRAY_SIZE = 8;
const int LEFT_PWM_PIN = 11;
const int RIGHT_PWM_PIN = 23;
const int LEFT_MOTOR_IN1_PIN = 12;
const int LEFT_MOTOR_IN2_PIN = 13;
const int RIGHT_MOTOR_IN3_PIN = 18;
const int RIGHT_MOTOR_IN4_PIN = 19;
const int RFID_SS_PIN = 21;
const int RFID_RST_PIN = 22;
const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;
const int BLUETOOTH_BAUDRATE = 115200;
const char* BLUETOOTH_DEVICE_NAME = "PID_TUNING";

// Table UID 
const String TABLE1_CORNER_UID = "A1B2C3D4";  // Table 1 Corner 
const String TABLE1_DELIVERY_UID = "E5F6G7H8"; // Table 1 Delivery 
const String TABLE2_CORNER_UID = "X1Y2Z3W4";  // Table 2 Corner 
const String TABLE2_DELIVERY_UID = "P5Q6R7S8"; // Table 2 Delivery 

int sensorpins[] = {IR_SENSOR_PIN_START, IR_SENSOR_PIN_START + 1, IR_SENSOR_PIN_START + 2, IR_SENSOR_PIN_START + 3, IR_SENSOR_PIN_START + 4, IR_SENSOR_PIN_START + 5, IR_SENSOR_PIN_START + 6};
int weight[] = {-4, -3, -2, -1, 1, 2, 3, 4};

float Kp = 20, Ki = 0.02, Kd = 5;
float error = 0, preerror = 0, integral = 0;
int motorbasespeed = 150;
int maxmotorspeed = 255;

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);

BluetoothSerial SerialBT;
enum RobotState {
  IDLE,
  GOING_TO_TABLE1,
  CORRECTING_LEFT_TABLE1_CORNER, 
  CORRECTING_RIGHT_TABLE1_CORNER, 
  GOING_TO_TABLE1_DELIVERY,
  TURNING_LEFT_180_TABLE1_DELIVERY, 
  AT_TABLE1_DELIVERY,
  GOING_TO_TABLE2,
  CORRECTING_LEFT_TABLE2_CORNER, 
  CORRECTING_RIGHT_TABLE2_CORNER, 
  TURNING_LEFT_TABLE2,  
  GOING_TO_TABLE2_DELIVERY,
  TURNING_RIGHT_180_TABLE2_DELIVERY, 
  AT_TABLE2_DELIVERY,
  STOPPED
};

RobotState currentState = IDLE;
int targetTable = 0; 

unsigned long delayStartTime = 0;
unsigned long delayDuration = 0;
const unsigned long CORNER_TURN_DELAY = 1000;
const unsigned long SMALL_CORRECTION_DELAY = 1500; 
const unsigned long TURN_180_DELAY = 2000;    

int leftTurns = 0;
int rightTurns = 0;

void setup() {
  Serial.begin(BLUETOOTH_BAUDRATE);
  SerialBT.begin(BLUETOOTH_DEVICE_NAME);
  Serial.println("Bot ready to pair");
  SPI.begin();
  rfid.PCD_Init();

  pinMode(LEFT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_IN2_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4_PIN, OUTPUT);

  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION); 
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION); 
  ledcAttachPin(LEFT_PWM_PIN, 0);
  ledcAttachPin(RIGHT_PWM_PIN, 1);

  currentState = IDLE;
}

void loop() {
  tuning(); 
  processRobotState();
}

void tuning() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    if (cmd == 'q') Kp += 1;
    if (cmd == 'a') Kp -= 1;
    if (cmd == 'w') Kd += 1;
    if (cmd == 's') Kd -= 1;
    if (cmd == 'e') Ki += 0.1;
    if (cmd == 'd') Ki -= 0.1;

    SerialBT.print("Kp: "); SerialBT.print(Kp);
    SerialBT.print(" | Ki: "); SerialBT.print(Ki);
    SerialBT.print(" | Kd: "); SerialBT.println(Kd);
  }
  delay(50); // Small delay for Bluetooth reading
}

void processRobotState() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    if (cmd == '1') {
      targetTable = 1;
      currentState = GOING_TO_TABLE1;
      SerialBT.println("Going to Table 1");
    } else if (cmd == '2') {
      targetTable = 2;
      currentState = GOING_TO_TABLE2;
      SerialBT.println("Going to Table 2");
    } else if (cmd == '3') {
      currentState = STOPPED;
      targetTable = 0;
      SerialBT.println("Stop Bot");
    }
  }

  String currentUID = getUID(); // RFID tag

  switch (currentState) {
    case GOING_TO_TABLE1:
      if (currentUID == TABLE1_CORNER_UID) {
        stop();
        SerialBT.println("Reached Table 1 Corner - Correcting Line");
        int sensorsvalues[NUM_IR_SENSORS];
        for (int i = 0; i < NUM_IR_SENSORS; i++) {
          sensorsvalues[i] = digitalRead(sensorpins[i]);
        }
        if (sensorsvalues[0] == 1 && sensorsvalues[1] == 1) {
          currentState = CORRECTING_RIGHT_TABLE1_CORNER;
          startNonBlockingDelay(SMALL_CORRECTION_DELAY);
          turnRight(100); // Small right turn
        } else if (sensorsvalues[NUM_IR_SENSORS - 2] == 1 && sensorsvalues[NUM_IR_SENSORS - 1] == 1) {
          currentState = CORRECTING_LEFT_TABLE1_CORNER;
          startNonBlockingDelay(SMALL_CORRECTION_DELAY);
          turnLeft(100); // Small left turn
        } else {
          currentState = TURNING_RIGHT_TABLE1; 
          startNonBlockingDelay(CORNER_TURN_DELAY);
          turnRight();
        }
      } else if (currentUID == TABLE1_DELIVERY_UID) {
        stop();
        SerialBT.println("Reached Table 1 Delivery - Turning 180 degrees Left");
        currentState = TURNING_LEFT_180_TABLE1_DELIVERY;
        startNonBlockingDelay(TURN_180_DELAY);
        turnLeft(100); 
      } else {
        followline();
      }
      break;

    case CORRECTING_LEFT_TABLE1_CORNER:
      if (isNonBlockingDelayFinished()) {
        stop();
        Serial.println("Finished Left Correction at Table 1 Corner - Proceeding to Turn");
        currentState = TURNING_RIGHT_TABLE1;
        startNonBlockingDelay(CORNER_TURN_DELAY);
        turnRight();
      }
      break;

    case CORRECTING_RIGHT_TABLE1_CORNER:
      if (isNonBlockingDelayFinished()) {
        stop();
        Serial.println("Finished Right Correction at Table 1 Corner - Proceeding to Turn");
        currentState = TURNING_RIGHT_TABLE1;
        startNonBlockingDelay(CORNER_TURN_DELAY);
        turnRight();
      }
      break;

    case TURNING_RIGHT_TABLE1:
      if (isNonBlockingDelayFinished()) {
        stop();
        Serial.println("Finished Turning Right at Table 1 Corner - Proceeding to Delivery");
        currentState = GOING_TO_TABLE1_DELIVERY;
      }
      break;

    case GOING_TO_TABLE1_DELIVERY:
      if (currentUID == TABLE1_DELIVERY_UID) {
        stop();
        SerialBT.println("Reached Table 1 Delivery - Initiating 180 Left Turn");
        currentState = TURNING_LEFT_180_TABLE1_DELIVERY;
        startNonBlockingDelay(TURN_180_DELAY);
        turnLeft(100); 
      } else {
        followline();
      }
      break;

    case TURNING_LEFT_180_TABLE1_DELIVERY:
      if (isNonBlockingDelayFinished()) {
        stop();
        Serial.println("Finished 180 Left Turn at Table 1 Delivery - Path Retrace (Not Implemented)");
        currentState = AT_TABLE1_DELIVERY; 
        startNonBlockingDelay(10000); 
      }
      break;

    case AT_TABLE1_DELIVERY:
      if (isNonBlockingDelayFinished()) {
        currentState = IDLE; 
      }
      break;

    case GOING_TO_TABLE2:
      if (currentUID == TABLE2_CORNER_UID) {
        stop();
        SerialBT.println("Reached Table 2 Corner - Correcting Line");
        int sensorsvalues[NUM_IR_SENSORS];
        for (int i = 0; i < NUM_IR_SENSORS; i++) {
          sensorsvalues[i] = digitalRead(sensorpins[i]);
        }
        if (sensorsvalues[0] == 1 && sensorsvalues[1] == 1) {
          currentState = CORRECTING_RIGHT_TABLE2_CORNER;
          startNonBlockingDelay(SMALL_CORRECTION_DELAY);
          turnRight(100); // Small right turn
        } else if (sensorsvalues[NUM_IR_SENSORS - 2] == 1 && sensorsvalues[NUM_IR_SENSORS - 1] == 1) {
          currentState = CORRECTING_LEFT_TABLE2_CORNER;
          startNonBlockingDelay(SMALL_CORRECTION_DELAY);
          turnLeft(100); // Small left turn
        } else {
          currentState = TURNING_LEFT_TABLE2; 
          startNonBlockingDelay(CORNER_TURN_DELAY);
          turnLeft();
        }
      } else if (currentUID == TABLE2_DELIVERY_UID) {
        stop();
        SerialBT.println("Reached Table 2 Delivery - Turning 180 degrees Right");
        currentState = TURNING_RIGHT_180_TABLE2_DELIVERY;
        startNonBlockingDelay(TURN_180_DELAY);
        turnRight(100);
      } else {
        followline();
      }
      break;

    case CORRECTING_LEFT_TABLE2_CORNER:
      if (isNonBlockingDelayFinished()) {
        stop();
        Serial.println("Finished Left Correction at Table 2 Corner - Proceeding to Turn");
        currentState = TURNING_LEFT_TABLE2;
        startNonBlockingDelay(CORNER_TURN_DELAY);
        turnLeft();
      }
      break;

    case CORRECTING_RIGHT_TABLE2_CORNER:
      if (isNonBlockingDelayFinished()) {
        stop();
        Serial.println("Finished Right Correction at Table 2 Corner - Proceeding to Turn");
        currentState = TURNING_LEFT_TABLE2;
        startNonBlockingDelay(CORNER_TURN_DELAY);
        turnLeft();
      }
      break;

    case TURNING_LEFT_TABLE2:
      if (isNonBlockingDelayFinished()) {
        stop();
        Serial.println("Finished Turning Left at Table 2 Corner - Proceeding to Delivery");
        currentState = GOING_TO_TABLE2_DELIVERY;
      }
      break;

    case GOING_TO_TABLE2_DELIVERY:
      if (currentUID == TABLE2_DELIVERY_UID) {
        stop();
        SerialBT.println("Reached Table 2 Delivery - Initiating 180 Right Turn");
        currentState = TURNING_RIGHT_180_TABLE2_DELIVERY;
        startNonBlockingDelay(TURN_180_DELAY);
        turnRight(100); 
      } else {
        followline();
      }
      break;

    case TURNING_RIGHT_180_TABLE2_DELIVERY:
      if (isNonBlockingDelayFinished()) {
        stop();
        Serial.println("Finished 180 Right Turn at Table 2 Delivery - Path Retrace (Not Implemented)");
        currentState = AT_TABLE2_DELIVERY; 
        startNonBlockingDelay(10000); 
      }
      break;

    case AT_TABLE2_DELIVERY:
      if (isNonBlockingDelayFinished()) {
        currentState = IDLE; 
      }
      break;

    case STOPPED:
    default:
      stop();
      break;
  }

  delay(10);
}

String getUID() {
  if (!rfid.PICC_IsNewCardPresent()) return "";
  if (!rfid.PICC_ReadCardSerial()) return "";

  String uidString = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    uidString += (rfid.uid.uidByte[i] < 0x10 ? "0" : "") + String(rfid.uid.uidByte[i], HEX);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  uidString.toUpperCase();
  return uidString;
}

// PID control
void followline() {
  int sensorsvalues[NUM_IR_SENSORS];
  int sum = 0, totalweight = 0;
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    sensorsvalues[i] = digitalRead(sensorpins[i]);
    sum += sensorsvalues[i] * weight[i];
    totalweight += sensorsvalues[i];
  }

  error = (totalweight > 0) ? (float)sum / totalweight : preerror;
  float correction = (Kp * error) + (Ki * integral) + (Kd * (error - preerror));
  integral += error;
  preerror = error;

  int leftspeed = constrain(motorbasespeed + correction, 0, maxmotorspeed);
  int rightspeed = constrain(motorbasespeed - correction, 0, maxmotorspeed);

  MLeft(leftspeed);
  MRight(rightspeed);
}

void turnRight(int speed = -1) {
  int actualSpeed = (speed == -1) ? motorbasespeed : speed;
  MLeft(actualSpeed);  // Forward on left
  MRight(0);             // Stop right
}

void turnLeft(int speed = -1) {
  int actualSpeed = (speed == -1) ? motorbasespeed : speed;
  MLeft(0);              // Stop left
  MRight(actualSpeed); // Forward on right
}

void MLeft(int speed) {
  digitalWrite(LEFT_MOTOR_IN1_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_IN2_PIN, LOW);
  ledcWrite(0, constrain(speed, 0, maxmotorspeed));
}

void MRight(int speed) {
  digitalWrite(RIGHT_MOTOR_IN3_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4_PIN, LOW);
  ledcWrite(1, constrain(speed, 0, maxmotorspeed));
}


void stop() {
  digitalWrite(LEFT_MOTOR_IN1_PIN, LOW);
  digitalWrite(LEFT_MOTOR_IN2_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_IN3_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_IN4_PIN, LOW);
}

void startNonBlockingDelay(unsigned long duration) {
  delayDuration = duration;
  delayStartTime = millis();
}

bool isNonBlockingDelayFinished() {
  return (millis() - delayStartTime >= delayDuration);
}