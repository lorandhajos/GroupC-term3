#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <limits.h>
#include <string.h>

// Bluetooth module
#define bluetoothRX 10
#define bluetoothTX 11

SoftwareSerial bluetooth(bluetoothRX, bluetoothTX);

// Neopixels
#define LED_COUNT 4
#define LED_PIN   9

Adafruit_NeoPixel neoPixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Left motor
#define motorA 5
#define modeA 12

// Right motor
#define motorB 6
#define modeB 13

// Encoders
#define encoderA 2
#define encoderB 3

volatile int countA = 0;
volatile int countB = 0;

// Servo
#define servoPin 4

Servo servo;
int angle = 0;

// USS
#define trigger 8
#define echo 7

// Serial buffer
const int BUFFER_SIZE = 50;
char buf[BUFFER_SIZE] = { 0 };

// Light sensor
#define sensorSensitivity 900

QTRSensors qtr;

static const uint8_t analog_pins[] = {A7,A6,A5,A4,A3,A2,A1,A0};//{A0,A1,A2,A3,A4,A5,A6,A7}; //{A7,A6,A5,A4,A3,A2,A1,A0};
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t digitalSensorValues[SensorCount];

bool start = false;
bool solve = false;
bool end = false;
uint16_t position = 0;

// Headers
void move(int value = 0);
void turnMotor(int motor, int mode, int speed = 0);
void moveDistance(int distance, int speed = 255);
void stop();

void setup() {
  // Start serial for PC <--> arduino connection
  Serial.begin(9600);

  // Start serial for arduino <--> HC-05 connection
  bluetooth.begin(38400);
  
  // Initialize servo
  servo.attach(servoPin);
  servo.write(120);

  // Initialize neoPixels
  neoPixels.begin();
  neoPixels.setBrightness(50);
  neoPixels.show();

  // Initialize motors
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);

  pinMode(modeA, OUTPUT);
  pinMode(modeB, OUTPUT);

  // Initialize encoders
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);

  // encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encoderA), updateA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), updateB, CHANGE);

  // USS
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  // lightSensor
  qtr.setTypeAnalog();
  qtr.setSensorPins(analog_pins, SensorCount);

  moveDistance(18);

  // Start sensor calibration
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    if (i % 80 == 0) {
      moveDistance(15);
    } else {
      stop();
    }
  }
}

void updateA() {
  noInterrupts();
  countA++;
  interrupts();
}

void updateB() {
  noInterrupts();
  countB++;
  interrupts();
}

int degreeToCounter2(int degree = 0) {
  return abs(degree) * 0.25; //* 3; // / 6.1;
}

int degreeToCounter(int degree = 0) {
  return abs(degree) * 0.35; //* 3; // / 6.1;
}

int distanceToCounter(int distance = 0) {
  return abs(distance) / 5.5; //* 3; // / 5.5;
}

void moveDistance(int distance, int speed = 255) {
  int distA = countA + distanceToCounter(distance);
  int distB = countB + distanceToCounter(distance); 
  volatile int * counterA = &countA;
  volatile int * counterB = &countB;

  stop();

  if (distance > 0) {
    while (*counterA < distA && *counterB < distB) {
      turnMotor(motorA, modeA, speed);
      turnMotor(motorB, modeB, speed);
    }
  } else {
    while (*counterA < distA && *counterB < distB) {
      turnMotor(motorA, modeA, -speed);
      turnMotor(motorB, modeB, -speed);
    }
  }

  turnMotor(motorA, modeA);
  turnMotor(motorB, modeB);
}

void uTurn(int degree = 0, int speedA = 255, int speedB = 255) {
  int distA = countA + degreeToCounter2(degree);
  int distB = countB + degreeToCounter2(degree); 
  volatile int * counterA = &countA;
  volatile int * counterB = &countB;

  stop();

  if (degree > 0) {
    while (*counterA < distA) {
      turnMotor(motorB, modeB, -speedB);
      turnMotor(motorA, modeA,  speedA);
    }
    turnMotor(motorB, modeB);
    turnMotor(motorA, modeA);
  } else {
    while (*counterB < distB) {
      turnMotor(motorA, modeA, -speedA);
      turnMotor(motorB, modeB,  speedB);
    }
    turnMotor(motorA, modeA);
    turnMotor(motorB, modeB);
  }
}

void turnDegrees(int degree = 0, int speedA = 255, int speedB = 255) {
  int distA = countA + degreeToCounter(degree);
  int distB = countB + degreeToCounter(degree); 
  volatile int * counterA = &countA;
  volatile int * counterB = &countB;

  stop();

  if (degree > 0) {
    while (*counterA < distA) {
      turnMotor(motorA, modeA,  speedA);
      turnMotor(motorB, modeB, -speedB);
    }
    turnMotor(motorA, modeA);
    turnMotor(motorB, modeB);
  } else {
    while (*counterB < distB) {
      turnMotor(motorA, modeA, -speedA);
      turnMotor(motorB, modeB,  speedB);
    }
    turnMotor(motorA, modeA);
    turnMotor(motorB, modeB);
  }
}

void turnMotor(int motor, int mode, int speed = 0) {
  if (speed > 0 && speed <= 255) {
    analogWrite(motor, speed);
  } else if (speed < 0 && speed >= -255) {
    digitalWrite(mode, HIGH);
    analogWrite(motor, 255 - abs(speed));
  } else {
    digitalWrite(mode, LOW);
    analogWrite(motor, 0);
  }
}

void move(int value = 0) {
  if (value > 0 && value <= 255) {
    analogWrite(motorA, value);
    analogWrite(motorB, value);
  } else if (value < 0 && value >= -255) {
    digitalWrite(modeA, HIGH);
    digitalWrite(modeB, HIGH);
    analogWrite(motorA, 255 + value);
    analogWrite(motorB, 255 + value);
  } else {
    stop();
  }
}

void stop() {
  digitalWrite(modeA, LOW);
  digitalWrite(modeB, LOW);
  analogWrite(motorA, 0);
  analogWrite(motorB, 0);
}

void dumpSensorValues(uint16_t * values) {
  char sensorStr[50];

  snprintf(sensorStr, sizeof(sensorStr), "sensorValues: %d %d %d %d %d %d %d %d",
      values[0], values[1], values[2], values[3],
      values[4], values[5], values[6], values[7]);

  Serial.println(sensorStr);
}

int getSumOfSensorValues(uint16_t * values) {
  int sum = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    sum += values[i];
  }

  return sum;
}

void getDigitalValues(int sensitivity) {
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > sensitivity) {
      digitalSensorValues[i] = 1;
    } else {
      digitalSensorValues[i] = 0;
    }
  }  
}

void startMaze() {
  moveDistance(60);
  delay(500);
  servo.write(70);
  delay(500);
  moveDistance(70);
  delay(500);
  turnDegrees(-90, 0, 255);
  delay(1000);

  start = true;
}

void solveMaze() {
  position = qtr.readLineBlack(sensorValues);

  int16_t error = position - 3500;
  int16_t leftMotorSpeed = 255;
  int16_t rightMotorSpeed = 255;

  if (error < -500) {
    leftMotorSpeed = constrain(255 - abs(error) / 5, 0, 255);
  }

  if (error > 500) {
    rightMotorSpeed = constrain(255 - abs(error) / 5, 0, 255);
  }

  getDigitalValues(sensorSensitivity);

  if (digitalSensorValues[4] == 1 && digitalSensorValues[5] == 1 && digitalSensorValues[6] == 1 && digitalSensorValues[7] == 1) { // right and center
    stop();
    delay(500);
    moveDistance(15);
    delay(500);

    qtr.readLineBlack(sensorValues);
    getDigitalValues(sensorSensitivity);

    if (getSumOfSensorValues(digitalSensorValues) == 8) {
      stop();
      solve = true;
    } else {
      turnDegrees(90, 255, 0);
    }
  } else if (digitalSensorValues[0] == 1 && digitalSensorValues[1] == 1 && digitalSensorValues[2] == 1 && digitalSensorValues[3] == 1) {
    stop();
    delay(500);
    moveDistance(15);
    delay(500);

    qtr.readLineBlack(sensorValues);
    getDigitalValues(sensorSensitivity);

    if (getSumOfSensorValues(digitalSensorValues) == 0) {
      turnDegrees(-90, 0, 255);
    }
  } else if (digitalSensorValues[5] == 1 || digitalSensorValues[6] == 1 || // left
              digitalSensorValues[3] == 1 || digitalSensorValues[4] == 1 || // center 
              digitalSensorValues[1] == 1 || digitalSensorValues[2] == 1) { // right
    turnMotor(motorA, modeA, leftMotorSpeed);
    turnMotor(motorB, modeB, rightMotorSpeed);
    return;
  } else if (getSumOfSensorValues(digitalSensorValues) == 8) { // intersection
    stop();
    delay(500);
    moveDistance(45);
    delay(500);

    qtr.readLineBlack(sensorValues);
    getDigitalValues(sensorSensitivity);

    if (getSumOfSensorValues(digitalSensorValues) == 0) {
      uTurn(180);
    } else {
      turnDegrees(90, 255, 0);
    }
  } else if (getSumOfSensorValues(digitalSensorValues) == 0) { // dead end
    stop();
    delay(500);
    moveDistance(40); // 45

    qtr.readLineBlack(sensorValues);
    getDigitalValues(sensorSensitivity);

    if (getSumOfSensorValues(digitalSensorValues) == 0) {
      uTurn(180);
      delay(500);
    } else {
      turnDegrees(90, 255, 0);
    }
  } else {
    turnMotor(motorA, modeA, leftMotorSpeed);
    turnMotor(motorB, modeB, rightMotorSpeed);
  }
}

void endMaze() {
  stop();
  delay(500);
  servo.write(120);
  delay(500);
  moveDistance(-100);

  end = true;
}

void loop() {
  /* Serial */

  // If data is available from HC-05
  if (bluetooth.available() > 0) {
    // HC-05 -> Arduino -> PC
    // Write the output from HC-05
    bluetooth.readBytesUntil('\n', buf, BUFFER_SIZE);
    angle = atoi(buf);

    Serial.print("HC-05::> ");
    Serial.print(buf);
    Serial.println();
    Serial.flush();
  }

  // If we input a command
  if (Serial.available() > 0) {
    // PC -> Arduino -> HC-05
    // Read from serial and write to HC-05
    bluetooth.write(Serial.read());
  }

  if (!start) {
    startMaze();
  } else if (!solve) {
    solveMaze();
  } else if (!end) {
    endMaze();
  }
}
