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

int countA = 0;
int countB = 0;

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
QTRSensors qtr;

static const uint8_t analog_pins[] = {A7,A6,A5,A4,A3,A2,A1,A0};
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

bool isHoldingObject = false;
int endBlockChance = 0;
bool followLine = false;

// Headers
void move(int value = 0);
void moveDelay(int delay);
void turnMotor(int motor, int mode, int speed = 0);

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

  moveDelay(140);

  // Start sensor calibration
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    if (i % 80 == 0) {
      move(255);
      delay(35);
    } else {
      move(0);
    }
  }

  moveDelay(50);
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

int degreeToCounter(int degree = 0) {
  return abs(degree) / 5.5;
}

void moveDelay(int numDelay) {
  turnMotor(motorA, modeA, 255);
  turnMotor(motorB, modeB, 255);
  delay(numDelay);
  turnMotor(motorA, modeA);
  turnMotor(motorB, modeB);
}

// not tested
void moveDistance(int distance) {
  int distA = countA + degreeToCounter(distance);
  int distB = countB + degreeToCounter(distance); 
  int * counterA = &countA;
  int * counterB = &countB;

  if (distance > 0) {
    while (*counterA < distA || *counterB < distB) {
      // Load bearing serial print
      Serial.print("");
      turnMotor(motorA, modeA, 255);
      turnMotor(motorB, modeB, 255);
    }
  } else {
    while (*counterA < distA || *counterB < distB) {
      // Load bearing serial print
      Serial.print("");
      turnMotor(motorA, modeA, -255);
      turnMotor(motorB, modeB, -255);
    }
  }

  turnMotor(motorA, modeA);
  turnMotor(motorB, modeB);
}

void turnDegrees(int degree = 0) {
  int distA = countA + degreeToCounter(degree);
  int distB = countB + degreeToCounter(degree); 
  int * counterA = &countA;
  int * counterB = &countB;

  if (degree > 0) {
    while (*counterA < distA || *counterB < distB) {
      // Load bearing serial print
      Serial.print("");
      turnMotor(motorA, modeA,  255);
      turnMotor(motorB, modeB, -255);
    }
    turnMotor(motorA, modeA);
    turnMotor(motorB, modeB);
  } else {
    while (*counterA < distA || *counterB < distB) {
      // Load bearing serial print
      Serial.print("");
      turnMotor(motorA, modeA, -255);
      turnMotor(motorB, modeB,  255);
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
    analogWrite(motor, 255 + speed);
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
    digitalWrite(modeA, LOW);
    digitalWrite(modeB, LOW);
    analogWrite(motorA, 0);
    analogWrite(motorB, 0);
  }
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

  /* Motor */
  uint16_t position = qtr.readLineBlack(sensorValues);

  int16_t error = position - 3500;
  int16_t leftMotorSpeed = 255;
  int16_t rightMotorSpeed = 255;

  // turn left
  if (error < -500) {
    leftMotorSpeed = 0;
  }

  // turn right
  if (error > 500) {
    rightMotorSpeed = 0;
  }

  int sumOfSensors = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    sumOfSensors += sensorValues[i];
  }

  // detect the end/start block
  if (sumOfSensors >= 7984) {
    endBlockChance++;

    if (endBlockChance >= 5 && !isHoldingObject) {
      move();
      servo.write(50);
      delay(3000);
      isHoldingObject = true;
    }
  } else if (endBlockChance >= 40) {
    move();
    moveDelay(90);
    turnDegrees(-95);
    move();
    delay(3000);
    Serial.println(endBlockChance);
    endBlockChance = 0;
    followLine = true;
  }

  // follow the line
  if (followLine) {
    turnMotor(motorA, modeA, leftMotorSpeed);
    turnMotor(motorB, modeB, rightMotorSpeed);
  } else {
    turnMotor(motorA, modeA, leftMotorSpeed);
    turnMotor(motorB, modeB, rightMotorSpeed);
  }
}
