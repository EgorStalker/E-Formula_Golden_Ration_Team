// Update:
// Зупиняється через 500 мс після другого круга. 
// Затримка старту після кнопки — 500 мс.
// Автостоп прибрано (пошук на швидкості 70).
// Перші 500 мс після старту швидкість 150, далі 255.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

#define RIGHT_PWM 6
#define RIGHT_MOTOR_PIN1 A2
#define RIGHT_MOTOR_PIN2 A1

#define LEFT_PWM 5
#define LEFT_MOTOR_PIN1 A4
#define LEFT_MOTOR_PIN2 A3

#define START_BUTTON 2

QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 7, 8, 9, 10, 11, A0}, 8); 

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int A0V = 0;
int A3V = 0;
int A4V = 0;
int A7V = 0;

const int maxSpeedValue = 150;
int maxSpeed = maxSpeedValue;

const int speedForCalibration = 120;

bool isRunning = false;
bool beginRunning = true;

int runForward = 1000000;
int beginRunningCount = 0;

int lastError = 0;
int integral = 0;

float Kp = 0.16; 
float Ki = 0; 
float Kd = 0.75; 

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

const int blackLineValue = 250;

int lapCounter = 0;                 
bool onFinishLine = false;          
unsigned long finishLineTimer = 0; 
unsigned long startRunTimer = 0; 

// Таймер для додаткового часу після фінішу
unsigned long finishExtraTime = 0;
bool isFinishing = false;

bool flagA0 = false;
bool flagA7 = false;

bool isCalibrated = false;
int blackSensors = 0;

uint16_t position = 0;
int error = 0;

int P = 0;
int I = 0;
int D = 0;
int pidValue = 0;

const int finishLinesValue = 1;
int countFinishLines = finishLinesValue;
int countAfterFinish = 300;

bool leftLight = false;
bool rightLight = false;

const int countToLightValue = 75;
int countToLight = countToLightValue;

const int errorToTurnLight = 500;
int errorCopy = error;
const int errorToLowerSpeed = 200;

bool flagToLower = false;
char buffer[9];
float s1, s2;
int kl = 0;

void setup() {
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  Serial.begin(9600);
  lapCounter = 0;                 
  finishLineTimer = millis();     
  onFinishLine = false;           
}

void loop() {
  if (!isCalibrated) {
    Serial.println("Натисніть кнопку для калібрування сенсорів...");
    while (digitalRead(START_BUTTON) == HIGH) {
      delay(10);
    }
    calibrateSensorsManually();
  }
  
  // Старт по кнопці
  if (digitalRead(START_BUTTON) == LOW && !isRunning) {
    delay(50); 
    if (digitalRead(START_BUTTON) == LOW) {
      Serial.println("Старт!");
      delay(500); // ЗМЕНШЕНО: тепер болід рушає через 500 мс після натискання

      countFinishLines = finishLinesValue;
      isRunning = true;
      isFinishing = false; 
      startRunTimer = millis(); 
    }
  } else if (digitalRead(START_BUTTON) == LOW && isRunning) {
    delay(50);
    if (digitalRead(START_BUTTON) == LOW) {
      driveMotors(0, 0); 
      Serial.println("Зупинка користувачем!");
      delay(1000);
      isRunning = false;
      isFinishing = false;
    }
  }

  if (isRunning) {
    
    // Динамічне керування швидкістю (перші 500 мс швидкість 150, далі 255)
    if (millis() - startRunTimer < 500) {
      maxSpeed = 150;
    } else {
      maxSpeed = 255;
    }

    position = qtrrc.readLine(sensorValues);

    int blackSensors = 0;
    for (int i = 0; i < SensorCount; i++) {
      if (sensorValues[i] > 650) { 
        blackSensors++;
      }
    }

    if (blackSensors >= 6) {
      if (!onFinishLine && (millis() - finishLineTimer > 1500)) {
        lapCounter++;
        finishLineTimer = millis(); 
        onFinishLine = true;        
        
        Serial.print("Лінія виявлена! Коло №: ");
        Serial.println(lapCounter);

        if (lapCounter >= 2) {
          finishExtraTime = millis();
          isFinishing = true;
        }
      }
    } else if (blackSensors <= 3) {
      onFinishLine = false;
    }

    // ЗУПИНКА ПІСЛЯ 500 мс НА ФІНІШІ
    if (isFinishing && (millis() - finishExtraTime >= 500)) { 
      driveMotors(0, 0); 
      isRunning = false;
      isFinishing = false;
      lapCounter = 0; 
      Serial.println("Фініш! Робот проїхав додаткові 500 мс і зупинився.");
      delay(2000); 
    }

    if (isRunning) {
      
      if (blackSensors == 0) {
        // Пошук лінії на швидкості 70 (автостоп вимкнено)
        if (lastError > 0) {
          driveMotors(70, 35); 
        } else if (lastError < 0) {
          driveMotors(35, 70); 
        } else {
          driveMotors(70, 70); 
        }
        
      } else {
        // ЗВИЧАЙНИЙ РУХ ПО PID
        error = position - 3500; 

        P = error;

        integral = integral + error;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;
        I = integral;

        D = error - lastError;
        lastError = error;

        pidValue = Kp * P + Ki * I + Kd * D;
        
        leftMotorSpeed = maxSpeed - pidValue;
        rightMotorSpeed = maxSpeed + pidValue;

        if (leftMotorSpeed > 255) leftMotorSpeed = 255;
        if (leftMotorSpeed < 0) leftMotorSpeed = 0;
        if (rightMotorSpeed > 255) rightMotorSpeed = 255;
        if (rightMotorSpeed < 0) rightMotorSpeed = 0;
        
        driveMotors(leftMotorSpeed, rightMotorSpeed);
      }
    }
    
  } else {
    driveMotors(0, 0); 
  }
}

void setSensorValues() {
  A0V = analogRead(A0);
  A3V = analogRead(A3);
  A4V = analogRead(A4);
  A7V = analogRead(A7);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > -1) {
    directLeftMotorForward();
  } else {
    directLeftMotorBackward();
    leftSpeed = -leftSpeed;
  }
  if (rightSpeed > -1) {
    directRightMotorForward();
  } else {
    directRightMotorBackward();
    rightSpeed = -rightSpeed;
  }
  analogWrite(LEFT_PWM, leftSpeed);
  analogWrite(RIGHT_PWM, rightSpeed);
}

void calibrateSensorsManually() {
  Serial.println("Калібрування сенсорів...");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);

  for (uint16_t i = 0; i < 60; i++) {
    qtrrc.calibrate();
    delay(25);
  }
  Serial.println("Сенсори відкалібровані...");
  digitalWrite(LED_BUILTIN, LOW);
  isCalibrated = true;
}

void directRightMotorForward() {
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void directLeftMotorForward() {
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
}

void directRightMotorBackward() {
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void directLeftMotorBackward() {
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
}
