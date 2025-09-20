// =================================================================
// ==         SIMPLE SERIAL ROBOT - ARDUINO FIRMWARE            ==
// =================================================================
// این کد وظایف زیر را بر عهده دارد:
// 1. گوش دادن به دستورات تک-کاراکتری از پورت سریال برای کنترل موتورها
// 2. خواندن GPS و ارسال اطلاعات به صورت یک رشته ساده روی پورت سریال

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// --- تعریف پین‌ها ---
const int IN1 = 8, IN2 = 9, IN3 = 10, IN4 = 11;
const int GPS_RX_PIN = 2, GPS_TX_PIN = 3;

// --- آبجکت‌های GPS ---
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// متغیر برای نگهداری زمان آخرین ارسال GPS
unsigned long lastGpsSendTime = 0;

void setup() {
  // راه‌اندازی پین‌های موتور
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // راه‌اندازی پورت سریال اصلی برای ارتباط با رزبری پای
  Serial.begin(115200); // از بادریت بالاتر برای ارتباط سریع‌تر استفاده می‌کنیم
  
  // راه‌اندازی پورت سریال مجازی برای GPS
  gpsSerial.begin(9600);
}

void loop() {
  // 1. بررسی دستورات ورودی از رزبری پای
  if (Serial.available() > 0) {
    char command = Serial.read();
    controlMotors(command);
  }

  // 2. خواندن داده‌های GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // 3. ارسال دوره‌ای داده‌های GPS (هر ۱ ثانیه)
  if (millis() - lastGpsSendTime > 1000) {
    lastGpsSendTime = millis();
    if (gps.location.isValid()) {
      // فرمت سفارشی ما: G,latitude,longitude\n
      Serial.print("G,");
      Serial.print(gps.location.lat(), 6); // دقت تا ۶ رقم اعشار
      Serial.print(",");
      Serial.println(gps.location.lng(), 6);
    }
  }
}

// تابع کنترل موتورها بر اساس کاراکتر دریافتی
void controlMotors(char cmd) {
  switch (cmd) {
    case 'F': // Forward
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      break;
    case 'B': // Backward
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
      break;
    case 'L': // Left
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      break;
    case 'R': // Right
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
      break;
    case 'S': // Stop
    default:
      digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
      break;
  }
}