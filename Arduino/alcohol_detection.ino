#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define pins
#define MQ3_SENSOR A0
#define SERVO_PIN 9
#define ENA 6
#define IN1 5
#define IN2 7
#define BUTTON_PIN 8

// Thresholds (adjust based on calibration)
#define AUTHORIZED_BAC 0.9     // Lock vehicle
#define WARNING_BAC 0.8        // Reduced speed
#define FULL_SPEED 255         // Max PWM
#define REDUCED_SPEED 160      // Reduced PWM
#define MAX_VEHICLE_SPEED 1250 // For display only

Servo myServo;
bool ignitionOn = false;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.backlight();

  pinMode(MQ3_SENSOR, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Optional button

  myServo.attach(SERVO_PIN);
  myServo.write(0);  // Lock initially

  ignitionOn = false;
  Serial.println("Warming up MQ3 sensor...");
  lcd.setCursor(0, 0);
  lcd.print("Warming Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Please Wait...");
  delay(30000); // MQ3 stabilization time
  lcd.clear();
  Serial.println("✅ Ready for Alcohol Test");
}

void loop() {
  int sensorValue = analogRead(MQ3_SENSOR);          // 0 - 1023
  float alcoholLevel = sensorValue / 1023.0;         // Normalize to 0.0 - 1.0

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Alcohol: ");
  lcd.print(alcoholLevel, 2); // e.g., 0.76

  // Serial diagnostics
  Serial.print("MQ3 Raw: ");
  Serial.print(sensorValue);
  Serial.print(" | Level: ");
  Serial.println(alcoholLevel, 3);

  //  Decision Logic
  if (alcoholLevel > AUTHORIZED_BAC) {
    ignitionOn = false;
    myServo.write(0);  // Locked
    stopVehicle();
    lcd.setCursor(0, 1);
    lcd.print("Access Denied 🚫");
    Serial.println("🚨 BAC > 0.9 → VEHICLE LOCKED");
  }
  else if (alcoholLevel > WARNING_BAC) {
    ignitionOn = true;
    myServo.write(45);  // Partial Unlock
    driveVehicle(REDUCED_SPEED);
    lcd.setCursor(0, 1);
    lcd.print("Drive Slow ⚠");
    Serial.println("⚠ BAC 0.8–0.9 → REDUCED SPEED");
  }
  else {
    ignitionOn = true;
    myServo.write(90);  // Full Unlock
    driveVehicle(FULL_SPEED);
    lcd.setCursor(0, 1);
    lcd.print("Drive Safe ✅");
    Serial.println("✅ BAC < 0.8 → FULL SPEED");
  }

  delay(2000);
}

//  Motor Driving Logic
void driveVehicle(int speed) {
  float displaySpeed = (speed / 255.0) * MAX_VEHICLE_SPEED;
  Serial.print("🔧 Driving Motor @ PWM: ");
  Serial.print(speed);
  Serial.print(" → Approx: ");
  Serial.print(displaySpeed, 1);
  Serial.println(" km/h");

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

//  Stop Motor
void stopVehicle() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  Serial.println("🛑 Motor STOPPED");
} 
