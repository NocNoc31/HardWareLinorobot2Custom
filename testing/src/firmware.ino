#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "firmware/lib/kinematics.h"
#include "firmware/lib/imu.h"

#define LED_PIN LED_BUILTIN
#define LINO_BASE MECANUM
#define USE_GY85_IMU
#define SDA_PIN 11 // specify I2C pins
#define SCL_PIN 10
#define NODE_NAME "esp32s3"
// #define TOPIC_PREFIX "esp32s3/"

#define BOARD_INIT                \
  {                               \
    Wire.begin(SDA_PIN, SCL_PIN); \
    Wire.setClock(400000);        \
  }

#ifdef USE_SYSLOG
#define RCCHECK(fn)                                                   \
  {                                                                   \
    rcl_ret_t temp_rc = fn;                                           \
    if ((temp_rc != RCL_RET_OK))                                      \
    {                                                                 \
      syslog(LOG_ERR, "%s RCCHECK failed %d", __FUNCTION__, temp_rc); \
    }                                                                 \
  }
#else
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      flashLED(3);               \
    }                            \
  } // do not block
#endif

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t imu_publisher;

sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

GY85IMU imu(SDA_PIN, SCL_PIN);

void controlCallback(rcl_timer_t * timer, int64_t last_call_time);
bool createEntities();
bool destroyEntities();
void publishData();
void syncTime();
struct timespec getTime();
void rclErrorLoop();
void flashLED(int n_times);

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(BAUDRATE);
#ifdef BOARD_INIT // board specific setup
    BOARD_INIT
#endif

    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        while(1)
        {
            flashLED(3);
        }
    }

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
#else
    set_microros_serial_transports(Serial);
#endif
#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE
#endif
}

void loop() {
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
#ifdef BOARD_LOOP // board specific loop
    BOARD_LOOP
#endif
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
       publishData();
    }
}

void twistCallback(const void * msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, & allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&imu_publisher, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void publishData()
{
    imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    while(true)
    {
        flashLED(2);
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

//-----------------------------------------------------------------------------------//

// #include <Arduino.h>
// #include <stdio.h>

// // #include "config.h"
// #include "motor.h"
// #include "kinematics.h"
// #include "pid.h"
// #include "odometry.h"
// #define ENCODER_USE_INTERRUPTS
// #define ENCODER_OPTIMIZE_INTERRUPTS
// #include "encoder.h"

// #define MOTOR_MAX_RPM 150               // motor's max RPM
// #define MAX_RPM_RATIO 0.85              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
// #define MOTOR_OPERATING_VOLTAGE 12      // motor's operating voltage (used to calculate max RPM)
// #define MOTOR_POWER_MAX_VOLTAGE 12      // max voltage of the motor's power source (used to calculate max RPM)
// #define MOTOR_POWER_MEASURED_VOLTAGE 12 // current voltage reading of the power connected to the motor (used for calibration)
// #define COUNTS_PER_REV1 450             // wheel1 encoder's no of ticks per rev
// #define WHEEL_DIAMETER 0.0560           // wheel's diameter in meters
// #define LR_WHEELS_DISTANCE 0.224        // distance between left and right wheels
// #define PWM_BITS 10                     // PWM Resolution of the microcontroller
// #define PWM_FREQUENCY 20000             // PWM Frequency

// #define PWM_MAX pow(2, PWM_BITS) - 1
// #define PWM_MIN -PWM_MAX

// #define MOTOR1_ENCODER_A 1
// #define MOTOR1_ENCODER_B 2

// #define MOTOR1_PWM 39
// #define MOTOR1_IN_A 37
// #define MOTOR1_IN_B 35
// #define LED_PIN LED_BUILTIN

// #define K_P 0 // P constant
// #define K_I 0 // I constant
// #define K_D 0 // D constant

// Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
// Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
// PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// Kinematics kinematics(
//     Kinematics::LINO_BASE,
//     MOTOR_MAX_RPM,
//     MAX_RPM_RATIO,
//     MOTOR_OPERATING_VOLTAGE,
//     MOTOR_POWER_MAX_VOLTAGE,
//     WHEEL_DIAMETER,
//     LR_WHEELS_DISTANCE);

// unsigned long prev_cmd_time = 0;

// void setup()
// {
//   pinMode(LED_PIN, OUTPUT);
//   Serial.begin(115200);
//   // giả mô phỏng
//   motor1_controller.brake();
// }

// void loop()
// {
//   moveBase();
//   delay(20); // Control loop at 50 Hz
// }

// void moveBase()
// {
//   // Example command for testing PID
//   float desired_rpm = 100.0; // Example desired RPM

//   // Get the current RPM from encoder
//   float current_rpm = motor1_encoder.getRPM();

//   // Calculate the PWM using PID controller
//   float pwm_value = motor1_pid.compute(desired_rpm, current_rpm);

//   // Apply the PWM to the motor
//   motor1_controller.spin(pwm_value);

//   // Output the values for debugging
//   Serial.print(">Desired RPM: ");
//   Serial.print(desired_rpm);
//   Serial.print(">Current RPM: ");
//   Serial.print(current_rpm);
//   Serial.print("PWM value: ");
//   Serial.println(pwm_value);
// }

// void rclErrorLoop()
// {
//   while (true)
//   {
//     flashLED(2);
//   }
// }

// void flashLED(int n_times)
// {
//   for (int i = 0; i < n_times; i++)
//   {
//     digitalWrite(LED_PIN, HIGH);
//     delay(150);
//     digitalWrite(LED_PIN, LOW);
//     delay(150);
//   }
//   delay(1000);
// }
