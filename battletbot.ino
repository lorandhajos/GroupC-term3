#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <limits.h>
#include <string.h>

// Bluetooth module
#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11

SoftwareSerial bluetooth(BLUETOOTH_RX, BLUETOOTH_TX);

// Neopixels
#define LED_COUNT 4
#define LED_PIN   9

Adafruit_NeoPixel neopixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Left motor
#define MOTOR_A 5
#define MODE_A 12

// Right motor
#define MOTOR_B 6
#define MODE_B 13

// Encoders
#define ENCODER_A 2
#define ENCODER_B 3

volatile int counterA = 0;
volatile int counterB = 0;

// Servo
#define SERVO 4

Servo servo;

// USS
#define TRIGGER 8
#define ECHO 7

// Serial buffer
const int BUFFER_SIZE = 50;
char buf[BUFFER_SIZE] = { 0 };

// Light sensor
#define SENSOR_SENSITIVITY 900

QTRSensors qtr;

static const uint8_t ANALOG_PINS[] = {A7,A6,A5,A4,A3,A2,A1,A0};
const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT];
uint16_t digitalSensorValues[SENSOR_COUNT];

bool start = false;
bool solve = false;
bool end = false;
uint16_t position = 0;

// Headers
void stop();
void moveDistance(int distance, int speed);
void fillNeopixels(int r = 255, int g = 255, int b = 255);

void setup()
{
    // Start serial for PC <--> arduino connection
    Serial.begin(9600);

    // Start serial for arduino <--> HC-05 connection
    bluetooth.begin(38400);
    
    // Initialize servo
    servo.attach(SERVO);
    servo.write(120);

    // Initialize neopixels
    neopixels.begin();
    neopixels.setBrightness(20);
    neopixels.show();

    // Initialize motors
    pinMode(MOTOR_A, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);

    pinMode(MODE_A, OUTPUT);
    pinMode(MODE_B, OUTPUT);

    // Initialize encoders
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);

    // encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateB, CHANGE);

    // USS
    pinMode(TRIGGER, OUTPUT);
    pinMode(ECHO, INPUT);
    int ussDistance = 1000;
    int duration = 0;

    // lightSensor
    qtr.setTypeAnalog();
    qtr.setSensorPins(ANALOG_PINS, SENSOR_COUNT);
    
    // read the uss sensor
    while (ussDistance >= 20)
    {
        digitalWrite(TRIGGER, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER, LOW);
        duration = pulseIn(ECHO, HIGH);
        ussDistance = duration * 0.034 / 2;
    }
    
    // wait for the other robot to get out the way
    wait(3000);
    
    moveDistance(18, 255);

    // start neopxiels
    fillNeopixels();

    // Start sensor calibration
    for (uint16_t i = 0; i < 250; i++)
    {
        qtr.calibrate();

        if (i % 80 == 0)
        {
            moveDistance(15, 255);
        }
        else
        {
            stop();
        }
    }
}

void updateA()
{
    noInterrupts();
    counterA++;
    interrupts();
}

void updateB()
{
    noInterrupts();
    counterB++;
    interrupts();
}

void fillNeopixels(int r = 255, int g = 255, int b = 255) {
    for (int i = 0; i < LED_COUNT; i++) {
        neopixels.setPixelColor(i, r, g, b);
        neopixels.show();
    }
}

void wait(int interval)
{
    unsigned long desiredMillis = millis() + interval;
    unsigned long currentMillis = millis();

    while (currentMillis < desiredMillis)
    {
        currentMillis = millis();
    }
}

int degreeToCounter(int degree = 0)
{
    return abs(degree) * 0.36;
}

int distanceToCounter(int distance = 0)
{
    return abs(distance) / 5.5;
}

void turnMotor(int motor, int mode, int speed = 0)
{
    if (speed > 0 && speed <= 255)
    {
        analogWrite(motor, speed);
    }
    else if (speed < 0 && speed >= -255)
    {
        digitalWrite(mode, HIGH);
        analogWrite(motor, 255 - abs(speed));
    }
    else
    {
        digitalWrite(mode, LOW);
        analogWrite(motor, 0);
    }
}

void moveDistance(int distance, int speed = 255)
{
    int distA = counterA + distanceToCounter(distance);
    int distB = counterB + distanceToCounter(distance); 
    volatile int * counterPointerA = &counterA;
    volatile int * counterPointerB = &counterB;

    stop();

    if (distance > 0)
    {
        while (*counterPointerA < distA && *counterPointerB < distB)
        {
            turnMotor(MOTOR_A, MODE_A, speed);
            turnMotor(MOTOR_B, MODE_B, speed);
        }
    }
    else
    {
        while (*counterPointerA < distA && *counterPointerB < distB)
        {   
            turnMotor(MOTOR_A, MODE_A, -speed);
            turnMotor(MOTOR_B, MODE_B, -speed);
        }
    }

    turnMotor(MOTOR_A, MODE_A);
    turnMotor(MOTOR_B, MODE_B);
}

void uTurn()
{
    int distB = counterB + abs(-95) * 0.24; 
    volatile int * counterPointerB = &counterB;

    stop();
    fillNeopixels(0, 0, 255);

    while (*counterPointerB < distB)
    {
        turnMotor(MOTOR_A, MODE_A, -255);
        turnMotor(MOTOR_B, MODE_B,  255);
    }

    turnMotor(MOTOR_A, MODE_A);
    turnMotor(MOTOR_B, MODE_B);
}

void turnDegrees(int degree = 0, int speedA = 255, int speedB = 255)
{
    int distA = counterA + degreeToCounter(degree);
    int distB = counterB + degreeToCounter(degree); 
    volatile int * counterPointerA = &counterA;
    volatile int * counterPointerB = &counterB;

    stop();

    if (degree > 0)
    {
        while (*counterPointerA < distA)
        {
            turnMotor(MOTOR_A, MODE_A,  speedA);
            turnMotor(MOTOR_B, MODE_B, -speedB);
            neopixels.setPixelColor(1, 0, 255, 0);
            neopixels.setPixelColor(2, 0, 255, 0);
            neopixels.show();
        }
    }
    else
    {
        while (*counterPointerB < distB)
        {
            turnMotor(MOTOR_A, MODE_A, -speedA);
            turnMotor(MOTOR_B, MODE_B,  speedB);
            neopixels.setPixelColor(0, 0, 255, 0);
            neopixels.setPixelColor(3, 0, 255, 0);
            neopixels.show();
        }   
    }

    turnMotor(MOTOR_A, MODE_A);
    turnMotor(MOTOR_B, MODE_B);

    wait(100);
}

void stop()
{
    digitalWrite(MODE_A, LOW);
    digitalWrite(MODE_B, LOW);
    analogWrite(MOTOR_A, 0);
    analogWrite(MOTOR_B, 0);
}

void rainbow() {
    for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256)
    {
        for(int i = 0; i < neopixels.numPixels(); i++)
        { 
            int pixelHue = firstPixelHue + (i * 65536L / neopixels.numPixels());
            neopixels.setPixelColor(i, neopixels.gamma32(neopixels.ColorHSV(pixelHue)));
        }
        neopixels.show();
        wait(5);
    }
}

void dumpSensorValues(uint16_t * values)
{
    char sensorStr[50];

    snprintf(sensorStr, sizeof(sensorStr), "sensorValues: %d %d %d %d %d %d %d %d",
        values[0], values[1], values[2], values[3],
        values[4], values[5], values[6], values[7]);

    Serial.println(sensorStr);
}

int getSumOfSensorValues(uint16_t * values)
{
    int sum = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        sum += values[i];
    }

    return sum;
}

void getDigitalValues(int sensitivity = SENSOR_SENSITIVITY)
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        if (sensorValues[i] > sensitivity)
        {
            digitalSensorValues[i] = 1;
        }
        else
        {
            digitalSensorValues[i] = 0;
        }
    }  
}

void startMaze()
{
    moveDistance(60);
    servo.write(70);
    wait(100);
    moveDistance(70);
    wait(100);
    turnDegrees(-90, 0, 255);

    start = true;
}

void solveMaze()
{
    position = qtr.readLineBlack(sensorValues);

    int16_t error = position - 3500;
    int16_t leftMotorSpeed = 255;
    int16_t rightMotorSpeed = 255;

    // calculate motor speeds
    if (error < -500)
    {
        leftMotorSpeed = constrain(255 - abs(error) / 5, 0, 255);
    }

    if (error > 500)
    {
        rightMotorSpeed = constrain(255 - abs(error) / 5, 0, 255);
    }

    getDigitalValues();

    // right and center
    if (digitalSensorValues[4] == 1 && digitalSensorValues[5] == 1 && digitalSensorValues[6] == 1 && digitalSensorValues[7] == 1)
    {
        stop();
        moveDistance(15);
        wait(100);

        qtr.readLineBlack(sensorValues);
        getDigitalValues();

        if (getSumOfSensorValues(digitalSensorValues) == 8)
        {
            stop();
            solve = true;
        }
        else
        {
            turnDegrees(90, 255, 0);
        }
    }
    else if (digitalSensorValues[5] == 1 || digitalSensorValues[6] == 1 || // left
            digitalSensorValues[3] == 1 || digitalSensorValues[4] == 1 || // center 
            digitalSensorValues[1] == 1 || digitalSensorValues[2] == 1) // right
    {
        // adjust motor speed to follow line
        turnMotor(MOTOR_A, MODE_A, leftMotorSpeed);
        turnMotor(MOTOR_B, MODE_B, rightMotorSpeed);
        fillNeopixels();
    }
    // intersection
    else if (getSumOfSensorValues(digitalSensorValues) == 8)
    {
        stop();
        moveDistance(40);
        wait(100);

        qtr.readLineBlack(sensorValues);
        getDigitalValues();

        if (getSumOfSensorValues(digitalSensorValues) == 0)
        {
            uTurn();
            wait(500);
        }
        else
        {
            turnDegrees(90, 255, 0);
        }
    }
    // dead end
    else if (getSumOfSensorValues(digitalSensorValues) == 0)
    {
        stop();
        moveDistance(40);
        wait(100);
        turnDegrees(-90, 0, 255);
        wait(100);

        qtr.readLineBlack(sensorValues);
        getDigitalValues();

        if (getSumOfSensorValues(digitalSensorValues) == 0)
        {
            uTurn();
            wait(500);
        }
    }
    else
    {
        turnMotor(MOTOR_A, MODE_A, leftMotorSpeed);
        turnMotor(MOTOR_B, MODE_B, rightMotorSpeed);
    }
}

void endMaze()
{
    stop();
    wait(100);
    servo.write(120);
    wait(100);
    moveDistance(-200);

    bluetooth.write("1337\n");

    // do end sequence
    uTurn();
    neopixels.setBrightness(100);
    for (int i = 0; i < 1000; i++)
    {
        rainbow();
    }

    // turn off neopixels
    neopixels.setBrightness(0);
    neopixels.show();

    end = true;
}

void loop()
{
    // Solve the maze
    if (!start)
    {
        startMaze();
    }
    else if (!solve)
    {
        solveMaze();
    }
    else if (!end)
    {
        endMaze();
    }
}
