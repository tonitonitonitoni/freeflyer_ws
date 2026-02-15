#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>  // <-- IMPORTANT for frame_id assign

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ROS objects
rcl_publisher_t imu_accel_pub;
rcl_publisher_t imu_linear_pub;

sensor_msgs__msg__Imu imu_accel_msg;
sensor_msgs__msg__Imu imu_linear_msg;

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

void imu_timer_callback(rcl_timer_t *, int64_t)
{
  imu::Vector<3> accel_raw = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> accel_lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  float mag = sqrt(accel_raw.x()*accel_raw.x() +
                 accel_raw.y()*accel_raw.y() +
                 accel_raw.z()*accel_raw.z());

  if (mag > 40.0) return;  // discard insane spikes


  int64_t now = rmw_uros_epoch_nanos();
  int32_t sec = (int32_t)(now / 1000000000ULL);
  uint32_t nsec = (uint32_t)(now % 1000000000ULL);

  // -------- RAW ACCEL (gravity included) --------
  imu_accel_msg.header.stamp.sec = sec;
  imu_accel_msg.header.stamp.nanosec = nsec;

  imu_accel_msg.linear_acceleration.x = accel_raw.x();
  imu_accel_msg.linear_acceleration.y = accel_raw.y();
  imu_accel_msg.linear_acceleration.z = accel_raw.z();

  imu_accel_msg.angular_velocity.z =
      (gyro.z() - gyro_bias_z) * DEG_TO_RAD;

  rcl_publish(&imu_accel_pub, &imu_accel_msg, NULL);

  // -------- LINEAR ACCEL (gravity removed) --------
  imu_linear_msg.header.stamp.sec = sec;
  imu_linear_msg.header.stamp.nanosec = nsec;

  imu_linear_msg.linear_acceleration.x = accel_lin.x();
  imu_linear_msg.linear_acceleration.y = accel_lin.y();
  imu_linear_msg.linear_acceleration.z = accel_lin.z();

  imu_linear_msg.angular_velocity.z =
      (gyro.z() - gyro_bias_z) * DEG_TO_RAD;

  rcl_publish(&imu_linear_pub, &imu_linear_msg, NULL);
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
  sensor_msgs__msg__Imu__init(&imu_accel_msg);
  sensor_msgs__msg__Imu__init(&imu_linear_msg);

  rosidl_runtime_c__String__assign(&imu_accel_msg.header.frame_id, "imu_link");
  rosidl_runtime_c__String__assign(&imu_linear_msg.header.frame_id, "imu_link");

  // --- RAW ACCEL COVARIANCES ---
  imu_accel_msg.angular_velocity_covariance[0] = 0.0005;
  imu_accel_msg.angular_velocity_covariance[4] = 0.0005;
  imu_accel_msg.angular_velocity_covariance[8] = 1e-5;

  imu_accel_msg.linear_acceleration_covariance[0] = 0.25;
  imu_accel_msg.linear_acceleration_covariance[4] = 0.25;
  imu_accel_msg.linear_acceleration_covariance[8] = 0.25;

  // --- LINEAR ACCEL COVARIANCES ---
  imu_linear_msg.angular_velocity_covariance[0] = 0.0005;
  imu_linear_msg.angular_velocity_covariance[4] = 0.0005;
  imu_linear_msg.angular_velocity_covariance[8] = 1e-5;

  imu_linear_msg.linear_acceleration_covariance[0] = 0.25;
  imu_linear_msg.linear_acceleration_covariance[4] = 0.25;
  imu_linear_msg.linear_acceleration_covariance[8] = 0.25;


  // ---- micro-ROS node ----
  Serial.println("Init rclc support...");
  rcl_allocator_t allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  Serial.println("Init node...");
  RCCHECK(rclc_node_init_default(&node, "esp32_imu", "", &support));

  Serial.println("Init publisher...");
  RCCHECK(rclc_publisher_init_default(
    &imu_accel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_accel"
  ));

  RCCHECK(rclc_publisher_init_default(
    &imu_linear_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_linear"
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
