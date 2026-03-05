/*
 * ============================================================
 *  ROVER MICRO-ROS FIRMWARE — Stage 2
 *  ESP32 DevKit V1
 *
 *  Publishes:
 *    /imu/data        sensor_msgs/Imu
 *    /odom            nav_msgs/Odometry
 *    /tof/left        sensor_msgs/Range
 *    /tof/right       sensor_msgs/Range
 *
 *  Subscribes:
 *    /cmd_vel         geometry_msgs/Twist
 *
 *  Start agent on laptop:
 *    docker run -it --rm -v /dev:/dev --privileged --net=host \
 *      microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <DFRobot_BMI160.h>
#include "fast_functions.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#define MUX_ADDR      0x70
#define AS5600_ADDR   0x36
#define VL53L0X_ADDR  0x29
#define BMI160_ADDR   0x68

#define CH_ENC_M1    0
#define CH_ENC_M2    1
#define CH_ENC_M3    2
#define CH_ENC_M4    3
#define CH_TOF_LEFT  4
#define CH_TOF_RIGHT 5

#define WHEEL_RADIUS_M       0.05f
#define WHEEL_BASE_LR_M      0.20f
#define WHEEL_BASE_FB_M      0.20f
#define AS5600_TICKS_PER_REV 4096.0f

Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofRight;
DFRobot_BMI160   bmi160;

bool tofLeftOk  = false;
bool tofRightOk = false;
bool imuOk      = false;
bool encOk[4]   = {false, false, false, false};

uint16_t encPrevRaw[4] = {0, 0, 0, 0};
int32_t  encTicks[4]   = {0, 0, 0, 0};
bool     encInit[4]    = {false, false, false, false};

float    odom_x            = 0.0f;
float    odom_y            = 0.0f;
float    odom_theta        = 0.0f;
int32_t  odom_prevTicks[4] = {0, 0, 0, 0};
uint32_t odom_prevTimeUs   = 0;

#define CMD_VEL_TIMEOUT_MS 500
uint32_t lastCmdVelMs  = 0;
bool     motorsRunning = false;
uint32_t lastPingMs    = 0;

enum RoverState { WAITING_AGENT, AGENT_CONNECTED, AGENT_DISCONNECTED };
RoverState roverState = WAITING_AGENT;

rclc_support_t     support;
rcl_allocator_t    allocator;
rcl_node_t         node;
rclc_executor_t    executor;
rcl_publisher_t    imu_pub;
rcl_publisher_t    odom_pub;
rcl_publisher_t    tof_left_pub;
rcl_publisher_t    tof_right_pub;
rcl_subscription_t cmd_vel_sub;
rcl_timer_t        timer_main;
rcl_timer_t        timer_tof;

sensor_msgs__msg__Imu     imu_msg;
nav_msgs__msg__Odometry   odom_msg;
sensor_msgs__msg__Range   tof_left_msg;
sensor_msgs__msg__Range   tof_right_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

char frame_odom[]      = "odom";
char frame_base[]      = "base_link";
char frame_imu[]       = "imu_link";
char frame_tof_left[]  = "tof_left_link";
char frame_tof_right[] = "tof_right_link";

bool muxSelect(uint8_t ch) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << ch);
    return Wire.endTransmission() == 0;
}
void muxDeselect() {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
}
bool i2cPing(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

void encodersUpdate() {
    for (uint8_t i = 0; i < 4; i++) {
        if (!encOk[i]) continue;
        if (!muxSelect(i)) continue;
        Wire.beginTransmission(AS5600_ADDR);
        Wire.write(0x0E);
        if (Wire.endTransmission(false) != 0) continue;
        Wire.requestFrom(AS5600_ADDR, 2);
        if (Wire.available() < 2) continue;
        uint16_t raw = (((uint16_t)Wire.read() << 8) | Wire.read()) & 0x0FFF;
        if (!encInit[i]) { encPrevRaw[i] = raw; encInit[i] = true; continue; }
        int16_t delta = (int16_t)raw - (int16_t)encPrevRaw[i];
        if (delta >  2048) delta -= 4096;
        if (delta < -2048) delta += 4096;
        encTicks[i]  += delta;
        encPrevRaw[i] = raw;
    }
    muxDeselect();
}

void stampNow(builtin_interfaces__msg__Time& t) {
    int64_t ns = (int64_t)micros() * 1000LL;
    t.sec     = (int32_t)(ns / 1000000000LL);
    t.nanosec = (uint32_t)(ns % 1000000000LL);
}

void imuRead() {
    if (!imuOk) return;
    int16_t buf[6];
    bmi160.getAccelGyroData(buf);
    imu_msg.angular_velocity.x    =  (buf[0] / 65.6f)   * (3.14159265f / 180.0f);
    imu_msg.angular_velocity.y    =  (buf[1] / 65.6f)   * (3.14159265f / 180.0f);
    imu_msg.angular_velocity.z    = -(buf[2] / 65.6f)   * (3.14159265f / 180.0f);
    imu_msg.linear_acceleration.x =  (buf[3] / 8192.0f) * 9.80665f;
    imu_msg.linear_acceleration.y =  (buf[4] / 8192.0f) * 9.80665f;
    imu_msg.linear_acceleration.z = -(buf[5] / 8192.0f) * 9.80665f;
    imu_msg.orientation.x = 0; imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0; imu_msg.orientation.w = 1;
    stampNow(imu_msg.header.stamp);
}

void odomCompute() {
    uint32_t now = micros();
    float dt = (now - odom_prevTimeUs) / 1e6f;
    if (dt <= 0.0f || dt > 1.0f) {
        odom_prevTimeUs = now;
        for (uint8_t i = 0; i < 4; i++) odom_prevTicks[i] = encTicks[i];
        return;
    }
    odom_prevTimeUs = now;
    float d[4];
    for (uint8_t i = 0; i < 4; i++) {
        d[i] = ((encTicks[i] - odom_prevTicks[i]) / AS5600_TICKS_PER_REV)
               * 2.0f * 3.14159265f * WHEEL_RADIUS_M;
        odom_prevTicks[i] = encTicks[i];
    }
    float Lxy    = (WHEEL_BASE_LR_M + WHEEL_BASE_FB_M) / 2.0f;
    float dx     = ( d[0] + d[1] + d[2] + d[3]) / 4.0f;
    float dy     = (-d[0] + d[1] + d[2] - d[3]) / 4.0f;
    float dtheta = (-d[0] + d[1] - d[2] + d[3]) / (4.0f * Lxy);
    float mid    = odom_theta + dtheta / 2.0f;
    odom_x      += dx * cosf(mid) - dy * sinf(mid);
    odom_y      += dx * sinf(mid) + dy * cosf(mid);
    odom_theta  += dtheta;
    while (odom_theta >  3.14159265f) odom_theta -= 2.0f * 3.14159265f;
    while (odom_theta < -3.14159265f) odom_theta += 2.0f * 3.14159265f;
    odom_msg.pose.pose.position.x    = odom_x;
    odom_msg.pose.pose.position.y    = odom_y;
    odom_msg.pose.pose.position.z    = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sinf(odom_theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(odom_theta / 2.0f);
    odom_msg.twist.twist.linear.x    = dx / dt;
    odom_msg.twist.twist.linear.y    = dy / dt;
    odom_msg.twist.twist.angular.z   = dtheta / dt;
    stampNow(odom_msg.header.stamp);
}

void tofRead() {
    if (tofLeftOk) {
        muxSelect(CH_TOF_LEFT);
        if (tofLeft.isRangeComplete()) {
            uint16_t mm = tofLeft.readRangeResult();
            tof_left_msg.range = (mm < 8190) ? constrain(mm / 1000.0f, 0.03f, 2.0f) : 2.0f;
            stampNow(tof_left_msg.header.stamp);
        }
    }
    if (tofRightOk) {
        muxSelect(CH_TOF_RIGHT);
        if (tofRight.isRangeComplete()) {
            uint16_t mm = tofRight.readRangeResult();
            tof_right_msg.range = (mm < 8190) ? constrain(mm / 1000.0f, 0.03f, 2.0f) : 2.0f;
            stampNow(tof_right_msg.header.stamp);
        }
    }
    muxDeselect();
}

void cmdVelCallback(const void* msg_in) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
    lastCmdVelMs  = millis();
    motorsRunning = true;

    float vx = constrain(msg->linear.x  / 0.5f, -1.0f, 1.0f);
    float vy = constrain(msg->linear.y  / 0.5f, -1.0f, 1.0f);
    float wz = constrain(msg->angular.z / 2.0f, -1.0f, 1.0f);

    float m1 = vx - vy - wz;  // FL
    float m2 = vx + vy + wz;  // FR
    float m3 = vx + vy - wz;  // BL
    float m4 = vx - vy + wz;  // BR

    float mx = max(max(fabsf(m1), fabsf(m2)), max(fabsf(m3), fabsf(m4)));
    if (mx > 1.0f) { m1/=mx; m2/=mx; m3/=mx; m4/=mx; }

    motorsSetWheelPwm(
        (int)(m1 * 255.0f),
        (int)(m2 * 255.0f),
        (int)(m3 * 255.0f),
        (int)(m4 * 255.0f)
    );
}

void timerMainCallback(rcl_timer_t* timer, int64_t) {
    if (!timer) return;
    encodersUpdate();
    imuRead();
    odomCompute();
    rcl_publish(&imu_pub,  &imu_msg,  NULL);
    rcl_publish(&odom_pub, &odom_msg, NULL);
}

void timerTofCallback(rcl_timer_t* timer, int64_t) {
    if (!timer) return;
    tofRead();
    rcl_publish(&tof_left_pub,  &tof_left_msg,  NULL);
    rcl_publish(&tof_right_pub, &tof_right_msg, NULL);
}

bool createEntities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "rover_node", "", &support) != RCL_RET_OK) return false;

    rclc_publisher_init_default(&imu_pub,       &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,  msg, Imu),      "/imu/data");
    rclc_publisher_init_default(&odom_pub,      &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,     msg, Odometry), "/odom");
    rclc_publisher_init_default(&tof_left_pub,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,  msg, Range),    "/tof/left");
    rclc_publisher_init_default(&tof_right_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,  msg, Range),    "/tof/right");
    rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),  "/cmd_vel");

    rclc_timer_init_default2(&timer_main, &support, 100000000ULL, timerMainCallback, true);
    rclc_timer_init_default2(&timer_tof,  &support, 200000000ULL, timerTofCallback,  true);

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_timer(&executor, &timer_main);
    rclc_executor_add_timer(&executor, &timer_tof);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);
    return true;
}

void destroyEntities() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rcl_publisher_fini(&imu_pub,       &node);
    rcl_publisher_fini(&odom_pub,      &node);
    rcl_publisher_fini(&tof_left_pub,  &node);
    rcl_publisher_fini(&tof_right_pub, &node);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_timer_fini(&timer_main);
    rcl_timer_fini(&timer_tof);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void initHardware() {
    Wire.begin(21, 22);
    Wire.setClock(1000000);
    delay(100);

    motorsInit();
    motorsOFF();

    for (uint8_t i = 0; i < 4; i++) {
        muxSelect(i); delay(2);
        encOk[i] = i2cPing(AS5600_ADDR);
    }
    muxDeselect();

    muxSelect(CH_TOF_LEFT);  delay(2);
    tofLeftOk = tofLeft.begin(VL53L0X_ADDR, &Wire);
    if (tofLeftOk) tofLeft.startRangeContinuous(50);

    muxSelect(CH_TOF_RIGHT); delay(2);
    tofRightOk = tofRight.begin(VL53L0X_ADDR, &Wire);
    if (tofRightOk) tofRight.startRangeContinuous(50);

    muxDeselect();

    Wire.beginTransmission(BMI160_ADDR);
    if (Wire.endTransmission() == 0) {
        bmi160.softReset(); delay(10);
        imuOk = (bmi160.I2cInit(BMI160_ADDR) == BMI160_OK);
    }

    tof_left_msg.radiation_type  = sensor_msgs__msg__Range__INFRARED;
    tof_left_msg.field_of_view   = 0.44f;
    tof_left_msg.min_range       = 0.03f;
    tof_left_msg.max_range       = 2.0f;
    tof_right_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    tof_right_msg.field_of_view  = 0.44f;
    tof_right_msg.min_range      = 0.03f;
    tof_right_msg.max_range      = 2.0f;

    imu_msg.header.frame_id.data       = frame_imu;
    imu_msg.header.frame_id.size       = strlen(frame_imu);
    odom_msg.header.frame_id.data      = frame_odom;
    odom_msg.header.frame_id.size      = strlen(frame_odom);
    odom_msg.child_frame_id.data       = frame_base;
    odom_msg.child_frame_id.size       = strlen(frame_base);
    tof_left_msg.header.frame_id.data  = frame_tof_left;
    tof_left_msg.header.frame_id.size  = strlen(frame_tof_left);
    tof_right_msg.header.frame_id.data = frame_tof_right;
    tof_right_msg.header.frame_id.size = strlen(frame_tof_right);

    imu_msg.orientation_covariance[0]         = -1.0;
    imu_msg.angular_velocity_covariance[0]    = 0.001;
    imu_msg.angular_velocity_covariance[4]    = 0.001;
    imu_msg.angular_velocity_covariance[8]    = 0.001;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    odom_msg.pose.covariance[0]   = 0.01;
    odom_msg.pose.covariance[7]   = 0.01;
    odom_msg.pose.covariance[35]  = 0.05;
    odom_msg.twist.covariance[0]  = 0.01;
    odom_msg.twist.covariance[7]  = 0.01;
    odom_msg.twist.covariance[35] = 0.05;
}

void motorTest() {
    Serial.println("=== MOTOR TEST ===");
    Serial.println("Watch which physical wheel spins each time.");
    delay(2000);

    Serial.println("M1 FWD (expected: Front-Left, forward)");
    ledcWrite(CH_M1_FWD, 150); delay(1500); motorsOFF(); delay(500);

    Serial.println("M1 BKWD (expected: Front-Left, backward)");
    ledcWrite(CH_M1_BKWD, 150); delay(1500); motorsOFF(); delay(500);

    Serial.println("M2 FWD (expected: Front-Right, forward)");
    ledcWrite(CH_M2_FWD, 150); delay(1500); motorsOFF(); delay(500);

    Serial.println("M2 BKWD (expected: Front-Right, backward)");
    ledcWrite(CH_M2_BKWD, 150); delay(1500); motorsOFF(); delay(500);

    Serial.println("M3 FWD (expected: Back-Left, forward)");
    ledcWrite(CH_M3_FWD, 150); delay(1500); motorsOFF(); delay(500);

    Serial.println("M3 BKWD (expected: Back-Left, backward)");
    ledcWrite(CH_M3_BKWD, 150); delay(1500); motorsOFF(); delay(500);

    Serial.println("M4 FWD (expected: Back-Right, forward)");
    ledcWrite(CH_M4_FWD, 150); delay(1500); motorsOFF(); delay(500);

    Serial.println("M4 BKWD (expected: Back-Right, backward)");
    ledcWrite(CH_M4_BKWD, 150); delay(1500); motorsOFF(); delay(500);

    Serial.println("=== TEST COMPLETE ===");
    delay(1000);
}

void setup() {
    Serial.begin(115200);
    initHardware();
    set_microros_transports();
}

void loop() {
    switch (roverState) {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(500, 1) == RCL_RET_OK) {
                if (createEntities()) {
                    lastPingMs = millis();
                    roverState = AGENT_CONNECTED;
                }
            }
            break;

        case AGENT_CONNECTED:
            rclc_executor_spin_some(&executor, 1000000ULL);
            if (motorsRunning && (millis() - lastCmdVelMs > CMD_VEL_TIMEOUT_MS)) {
                motorsOFF();
                motorsRunning = false;
            }
            if (millis() - lastPingMs > 2000) {
                lastPingMs = millis();
                if (rmw_uros_ping_agent(100, 1) != RCL_RET_OK)
                    roverState = AGENT_DISCONNECTED;
            }
            break;

        case AGENT_DISCONNECTED:
            motorsOFF();
            destroyEntities();
            roverState = WAITING_AGENT;
            break;
    }
}