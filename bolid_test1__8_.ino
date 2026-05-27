// Update:
// Зупиняється після другого круга.

// Включення необхідних бібліотек
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

// Створення об'єкта датчика лінії
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 7, 8, 9, 10, 11, A0}, 8); 

// Кількість датчиків, які використовуються
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Змінні для роботи
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
unsigned long startRunTimer = 0; // Переменная для отсчета времени разгона после старта

bool flagA0 = false;
bool flagA7 = false;

bool isCalibrated = false;
int blackSensors = 0;

uint16_t position = qtrrc.readLine(sensorValues);
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
  lapCounter = 0;                 // Обнуляем круги для нового заезда
  finishLineTimer = millis();     // Запоминаем время старта
  onFinishLine = false;           // Робот начинает движение вне финишной линии
}

void loop() {
  float sp1 = 0;
  float sp2 = 0;  

  if (!isCalibrated) {
    Serial.println("Натисніть кнопку для калібрування сенсорів...");
    while (digitalRead(START_BUTTON) == HIGH) {
      delay(10);
    }
    calibrateSensorsManually();
  }
  
  if (digitalRead(START_BUTTON) == LOW && !isRunning) {
    delay(50); // Для усунення дребезгу контактів
    if (digitalRead(START_BUTTON) == LOW) {
      Serial.println("Старт!");
      delay(1000); // Затримка перед початком руху

      countFinishLines = finishLinesValue;
      isRunning = true;
      startRunTimer = millis(); // Запоминаем точное время старта робота
    }
  } else if (digitalRead(START_BUTTON) == LOW && isRunning) {
    delay(50);
    if (digitalRead(START_BUTTON) == LOW) {
      driveMotors(0, 0); // Зупинка моторів
      Serial.println("Зупинка!");
      delay(1000);
      isRunning = false;
    }
  }

  // Якщо болід запущено, виконуємо алгоритм руху
  if (isRunning) {
    
    // Динамическое управление скоростью: первые 500 мс скорость 150, затем максимум 255
    if (millis() - startRunTimer < 100) {
      maxSpeed = 150;
    } else {
      maxSpeed = 255;
    }

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
        
        Serial.print("Линия обнаружена! Круг №: ");
        Serial.println(lapCounter);
      }
    } else if (blackSensors <= 3) {
      onFinishLine = false;
    }

    if (lapCounter >= 2) { 
      driveMotors(0, 0); 
      isRunning = false;
      Serial.println("Финиш! Робот успешно проехал дистанцию.");
      delay(2000); 
    }

    // Зчитування позиції (0-7000)
    position = qtrrc.readLine(sensorValues);

    // Обчислення помилки
    error = position - 3500; // 3500 - це позиція центру

    // Пропорційна складова
    P = error;

    // Інтегральна складова
    integral = integral + error;

    // Обмеження інтегралу для запобігання перенасичення
    if (integral > 1000) integral = 1000;
    if (integral < -1000) integral = -1000;
    I = integral;

    // Диференціальна складова
    D = error - lastError;
    lastError = error;

    // Обчислення PID-значення
    pidValue = Kp * P + Ki * I + Kd * D;
    
    // Застосування PID до швидкості моторів
    leftMotorSpeed = maxSpeed - pidValue;
    rightMotorSpeed = maxSpeed + pidValue;

    // Обмеження швидкості
    if (leftMotorSpeed > 255) leftMotorSpeed = 255;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    if (rightMotorSpeed > 255) rightMotorSpeed = 255;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    
    // Керування моторами
    driveMotors(leftMotorSpeed, rightMotorSpeed);
    
  } else {
    driveMotors(0, 0); // Зупинка моторів
  }
}

void setSensorValues() {
  A0V = analogRead(A0);
  A3V = analogRead(A3);
  A4V = analogRead(A4);
  A7V = analogRead(A7);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Керування лівим мотором
  if (leftSpeed > -1) {
    directLeftMotorForward();
  } else {
    directLeftMotorBackward();
    leftSpeed = -leftSpeed;
  }
  // Керування правим мотором
  if (rightSpeed > -1) {
    directRightMotorForward();
  } else {
    directRightMotorBackward();
    rightSpeed = -rightSpeed;
  }
  // Встановлення швидкості через ШІМ
  analogWrite(LEFT_PWM, leftSpeed);
  analogWrite(RIGHT_PWM, rightSpeed);
}

void calibrateSensorsManually() {
  Serial.println("Калібрування сенсорів...");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);

  // Калібрування протягом рухів вліво-вправо
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
