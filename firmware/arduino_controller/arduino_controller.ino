// =================================================================
// ==           ROS 2 ROBOT - ARDUINO CONTROLLER CODE           ==
// =================================================================
// این کد وظایف زیر را بر عهده دارد:
// 1. کنترل موتورها از طریق تاپیک /cmd_vel
// 2. خواندن GPS و ارسال اطلاعات روی تاپیک /gps/fix

// --- کتابخانه‌های مورد نیاز ---
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>      // برای پیام‌های کنترلی
#include <sensor_msgs/msg/nav_sat_fix.h> // برای پیام‌های GPS

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// --- تعریف پین‌ها ---
// پین‌های درایور موتور L298N
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;
// پین‌های SoftwareSerial برای ماژول GPS
const int GPS_RX_PIN = 2; // TX ماژول GPS به این پین وصل می‌شود
const int GPS_TX_PIN = 3; // RX ماژول GPS به این پین وصل می‌شود

// --- آبجکت‌های ROS 2 ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t gps_publisher;
rcl_timer_t gps_timer;

// آبجکت‌های پیام‌ها
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__NavSatFix gps_msg;

// --- آبجکت‌های GPS ---
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// تابع callback برای دریافت دستورات حرکتی
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // با توجه به مقادیر دریافتی، موتورها را کنترل کن
  if (msg->linear.x > 0.1) {      // جلو
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (msg->linear.x < -0.1) { // عقب
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else if (msg->angular.z > 0.1) { // چرخش به چپ
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (msg->angular.z < -0.1) { // چرخش به راست
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else {                         // توقف
    digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
  }
}

// تابع callback تایمر برای ارسال داده GPS
void gps_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  // داده‌های سریال از GPS را بخوان
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // فقط در صورتی که موقعیت معتبر باشد، پیام را منتشر کن
  if (gps.location.isValid() && gps.location.isUpdated()) {
    gps_msg.header.stamp.sec = (int32_t)(micros() / 1000000);
    gps_msg.header.frame_id.data = (char*)"gps_link";
    
    gps_msg.latitude = gps.location.lat();
    gps_msg.longitude = gps.location.lng();
    gps_msg.altitude = gps.altitude.meters();
    
    gps_msg.status.status = gps.satellites.value() > 0 ? sensor_msgs__msg__NavSatFix__STATUS_FIX : sensor_msgs__msg__NavSatFix__STATUS_NO_FIX;
    gps_msg.status.service = sensor_msgs__msg__NavSatFix__SERVICE_GPS;
    
    rcl_publish(&gps_publisher, &gps_msg, NULL);
  }
}

void setup() {
  // راه‌اندازی پین‌های موتور
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // راه‌اندازی سریال مجازی برای GPS
  gpsSerial.begin(9600);
  
  // راه‌اندازی Micro-ROS
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "arduino_robot_node", "", &support);

  // ایجاد Subscriber برای دستورات حرکتی
  rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  // ایجاد Publisher برای داده‌های GPS
  rclc_publisher_init_default(&gps_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "gps/fix");

  // ایجاد Timer برای ارسال دوره‌ای داده‌های GPS (هر 1000 میلی‌ثانیه)
  rclc_timer_init_default(&gps_timer, &support, RCL_MS_TO_NS(1000), gps_timer_callback);

  // راه‌اندازی Executor برای مدیریت callbackها
  rclc_executor_init(&executor, &support.context, 2, &allocator); // 2 آیتم: 1 سابسکریپشن و 1 تایمر
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &gps_timer);
}

void loop() {
  // Executor را به طور مداوم اجرا کن تا پیام‌ها و تایمرها را مدیریت کند
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}