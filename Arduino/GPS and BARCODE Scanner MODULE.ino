#include <SoftwareSerial.h>
#include <Servo.h>
#include <TinyGPS++.h>

// Serial definitions
SoftwareSerial gpsSerial(4, 5);       // GPS module
SoftwareSerial barcodeSerial(10, 12); // Barcode scanner

// Servo
Servo myServo;
const int servoPin = 9;
const int initialAngle = 0;
const int targetAngle = 90;

// Buzzer and Scanner power
const int buzzerPin = 7;
const int scannerPowerPin = 8;

// Barcode match
const String predefinedBarcode = "14519127204891";

// GPS tracking
double initialLat = 0.0, initialLon = 0.0;
bool initialPositionSet = false;
bool servoActivated = false;
bool buzzerTriggered = false; // 🔥 Buzzer flag added

// Barcode buffer
String barcodeBuffer = "";

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  barcodeSerial.begin(9600);

  myServo.attach(servoPin);
  myServo.write(initialAngle);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(scannerPowerPin, OUTPUT);
  digitalWrite(scannerPowerPin, HIGH); // Turn on scanner

  Serial.println("System ready");
}

void loop() {
  checkBarcode();

  if (servoActivated) {
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read()) && gps.location.isValid()) {
        double currentLat = gps.location.lat();
        double currentLon = gps.location.lng();

        if (!initialPositionSet) {
          initialLat = currentLat;
          initialLon = currentLon;
          initialPositionSet = true;
          Serial.println("Initial GPS position set");
        }

        double distance = calculateDistance(initialLat, initialLon, currentLat, currentLon);
        Serial.print("Distance: ");
        Serial.println(distance);

        // Buzzer at ~1.8m, with buffer
        if (!buzzerTriggered && distance >= 1.7 && distance < 2.1) {
          triggerBuzzer();
          buzzerTriggered = true;
        }

        // Reset at ~2.1m, with buffer
        if (distance >= 2.1) {
          returnServoToInitial();
          initialPositionSet = false;
          servoActivated = false;
          buzzerTriggered = false;
        }
      }
    }
  }
}

void checkBarcode() {
  while (barcodeSerial.available()) {
    char incomingChar = barcodeSerial.read();

    if (incomingChar == '\n' || incomingChar == '\r') {
      if (barcodeBuffer.length() > 0) {
        barcodeBuffer.trim();

        Serial.print("Scanned barcode: '");
        Serial.print(barcodeBuffer);
        Serial.println("'");

        if (barcodeBuffer == predefinedBarcode) {
          Serial.println("✅ Barcode matched!");
          activateServoOnce(); // 🔁 Rotate once only
          servoActivated = true;
        } else {
          Serial.println("❌ Barcode mismatch.");
        }

        barcodeBuffer = "";
      }
    } else {
      barcodeBuffer += incomingChar;
    }
  }
}

void activateServoOnce() {
  Serial.println("🔁 Rotating servo once from 0° to 90°");

  myServo.write(targetAngle);  // rotate to 90 degrees
  delay(1000);                 // wait to settle

  Serial.println("✅ Servo rotated to 90°");
}

void returnServoToInitial() {
  Serial.println("Returning servo to 0 degrees");
  myServo.write(initialAngle);
  delay(1000);
}

void triggerBuzzer() {
  Serial.println("Buzzing at ~1.8 meters");
  digitalWrite(buzzerPin, HIGH);
  delay(500);
  digitalWrite(buzzerPin, LOW);
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}
