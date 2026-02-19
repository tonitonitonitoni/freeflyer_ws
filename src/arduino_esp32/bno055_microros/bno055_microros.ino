#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>  
// Run with micro-ros-agent udp4 --port 8888

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ROS objects
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;

float gyro_bias_z = 0.0;

// ----------- micro-ROS error handling -----------
#define RCCHECK(fn) do { \
  rcl_ret_t rc = (fn); \
  if (rc != RCL_RET_OK) { \
    Serial.print("RCL error at "); Serial.print(#fn); \
    Serial.print(" rc="); Serial.println((int)rc); \
    while (true) { delay(1000); } \
  } \
} while (0)

void calibrate_gyro(int samples = 1000) {
  Serial.println("Calibrating gyro... keep still");
  delay(2000);

  float sum = 0;
  for (int i = 0; i < samples; i++) {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    sum += gyro.z();  // deg/s
    delay(2);
  }
  gyro_bias_z = sum / samples;
  Serial.println("Gyro bias calibrated.");
}

void imu_timer_callback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/)
{

  // Don’t publish until agent time sync exists
  // if (!rmw_uros_epoch_synchronized()) return;

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion q = bno.getQuat();
  imu_msg.orientation.w = q.w();
  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();

  imu_msg.linear_acceleration.x = accel.x();
  imu_msg.linear_acceleration.y = accel.y();
  imu_msg.linear_acceleration.z = accel.z();

  float mag = sqrt(accel.x()*accel.x() + accel.y()*accel.y() + accel.z()*accel.z());
  if (mag > 30.0) return;  

  imu_msg.angular_velocity.x = gyro.x() * DEG_TO_RAD;
  imu_msg.angular_velocity.y = gyro.y() * DEG_TO_RAD;
  imu_msg.angular_velocity.z = (gyro.z() - gyro_bias_z) * DEG_TO_RAD;

  int64_t now = rmw_uros_epoch_nanos();
  imu_msg.header.stamp.sec = (int32_t)(now / 1000000000ULL);
  imu_msg.header.stamp.nanosec = (uint32_t)(now % 1000000000ULL);

  rcl_publish(&imu_pub, &imu_msg, NULL);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Starting BNO055 + micro-ROS...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1) { delay(1000); }
  }
  delay(1000);

  // IMU-only fusion (no magnetometer)
  bno.setMode(OPERATION_MODE_IMUPLUS);

  calibrate_gyro();

  // ---- micro-ROS transport ----
  Serial.println("Setting micro-ROS WiFi transport...");
  set_microros_wifi_transports("BELL500", "7D5CFC657424", "192.168.2.82", 8888);
  delay(200);

  // ---- init message safely ----
  Serial.println("Init IMU message...");
  sensor_msgs__msg__Imu__init(&imu_msg);

  // frame_id: MUST use assign (don’t point at string literal)
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

  // Covariances (starter guesses)
  imu_msg.angular_velocity_covariance[0] = 0.0005;
  imu_msg.angular_velocity_covariance[4] = 0.0005;
  imu_msg.angular_velocity_covariance[8] = 1e-5;

  imu_msg.linear_acceleration_covariance[0] = 0.25;
  imu_msg.linear_acceleration_covariance[4] = 0.25;
  imu_msg.linear_acceleration_covariance[8] = 0.25;

  imu_msg.orientation_covariance[0] = 0.02;
  imu_msg.orientation_covariance[4] = 0.02;
  imu_msg.orientation_covariance[8] = 0.05;

  // ---- micro-ROS node ----
  Serial.println("Init rclc support...");
  rcl_allocator_t allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  Serial.println("Init node...");
  RCCHECK(rclc_node_init_default(&node, "esp32_imu", "", &support));

  Serial.println("Init publisher...");
  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"
  ));

  Serial.println("Init timer...");
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(10),  // 100 Hz
    imu_timer_callback
  ));

  Serial.println("Init executor...");
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.println("micro-ROS BNO055 IMU publisher running.");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
