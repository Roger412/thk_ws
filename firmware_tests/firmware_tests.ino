// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
// #include <Wire.h>
// #include <LiquidCrystal_PCF8574.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
//#include "encoder.h"
#include "encoderhw.h"
#include "interruptservice.h"


// LiquidCrystal_PCF8574 lcd(0x27);


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t pwm_soll_publisher;
rcl_publisher_t pwm_ist_publisher;
rcl_subscription_t twist_subscriber;
rcl_publisher_t encoder_publisher;

std_msgs__msg__Int32MultiArray encoder_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32MultiArray pwm_soll_msg;
std_msgs__msg__Float32MultiArray pwm_ist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t rpm_pub_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Motor motor1, motor2, motor3, motor4;

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

Odometry odometry;
//IMU imu;

// static long alt= 0;

void setup() 
{

    pinMode(LED_PIN, OUTPUT);
    analogWriteResolution(12); 
    /*bool imu_ok = imu.init();
    if(!imu_ok)
    {
        while(1)
        {
            flashLED(3);
        }
    }*/
    
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    motor1.direction = CW;
    motor2.direction = CCW;
    motor3.direction = CW;
    motor4.direction = CCW;

    motor1.distance = 0; 
    motor2.distance = 0;
    motor3.distance = 0;
    motor4.distance = 0;

    encoderSetup(motor1_encoder);
    encoderSetup(motor2_encoder);
    encoderSetup(motor3_encoder);
    encoderSetup(motor4_encoder);

    pidtimer.begin(update_pid, (1000000 / hz));
    rpm_calc_timer.begin(update_rpm, (1000000 / samplerate_RPM));

    // int error;
    // Wire2.begin();
    // Wire2.beginTransmission(0x27); 
    // error = Wire2.endTransmission();

    // if (error == 0){
    //     lcd.begin(20, 4, Wire2);
    //     lcd.setBacklight(255);
    // } else {
    //     Serial.println(": LCD not found.");
    // }  
    pinMode(16, OUTPUT); 
    digitalWrite(16, HIGH);
    pinMode(20, OUTPUT); 
    digitalWrite(20, HIGH);
    pinMode(26, OUTPUT); 
    digitalWrite(26, HIGH);
    pinMode(31, OUTPUT); 
    digitalWrite(31, HIGH);

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
    
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase();
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
//    allocator = rcl_get_default_allocator();
    //create init_options
//    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
//    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    // create odometry publisher

 allocator = rcl_get_default_allocator();

// create init_options
auto init_options = rcl_get_zero_initialized_init_options();
RCCHECK(rcl_init_options_init(&init_options, allocator)); // <--- This was missing on ur side

// Set ROS domain id
const int domain_id = 20;
RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

// Setup support structure.
RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));

    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    // Zweiter Timer: RPM Publisher
    const unsigned int rpm_publish_interval_ms = 10;
    RCCHECK(rclc_timer_init_default(
        &rpm_pub_timer,
        &support,
        RCL_MS_TO_NS(rpm_publish_interval_ms),
        rpmPublisherCallback
    ));

    //create PWM Soll publisher for PID
    RCCHECK(rclc_publisher_init_default(
        &pwm_soll_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "pwm_soll_values"
    ));
    pwm_soll_msg.data.data = (int32_t*)malloc(sizeof(int32_t) * 4);

    // create PWM Ist publisher for PID
    RCCHECK(rclc_publisher_init_default(
        &pwm_ist_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "pwm_ist_values"
    ));
    pwm_ist_msg.data.data = (float*)malloc(sizeof(float) * 4);

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
    
    RCCHECK(rclc_executor_add_timer(&executor, &rpm_pub_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    RCCHECK(rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "enc_delta"
    ));
    encoder_msg.data.data = (int32_t*)malloc(sizeof(int32_t) * 4);

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&rpm_pub_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    rcl_publisher_fini(&pwm_soll_publisher, &node);
    rcl_publisher_fini(&pwm_ist_publisher, &node);
    rcl_publisher_fini(&encoder_publisher, &node);

    digitalWrite(LED_PIN, HIGH);
    
    return true;
}

void fullStop()
{
    
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        float BRAKE_FACTOR = 0.95;
        twist_msg.linear.x *= BRAKE_FACTOR;
        twist_msg.linear.y *= BRAKE_FACTOR;
        twist_msg.angular.z *= BRAKE_FACTOR;
        
        if (abs(twist_msg.linear.x) < 0.02) twist_msg.linear.x = 0;
        if (abs(twist_msg.linear.y) < 0.02) twist_msg.linear.y = 0;
        if (abs(twist_msg.angular.z) < 0.02) twist_msg.angular.z = 0;

        digitalWrite(LED_PIN, HIGH);
    }
    
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );
    motor1.setpoint = req_rpm.motor1;
    motor2.setpoint = req_rpm.motor2;
    motor3.setpoint = req_rpm.motor3;
    motor4.setpoint = req_rpm.motor4;

    //get the current speed of each motor
    float current_rpm1 = motor1.rpm;
    float current_rpm2 = motor2.rpm;
    float current_rpm3 = motor3.rpm;
    float current_rpm4 = motor4.rpm;


    
    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    if(twist_msg.linear.x == 0 && twist_msg.linear.y == 0 && twist_msg.angular.z == 0){
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        motor4_controller.spin(0);
    }else{
        motor1_controller.spin(motor1.stellgroesse);
        motor2_controller.spin(motor2.stellgroesse);
        motor3_controller.spin(motor3.stellgroesse);
        motor4_controller.spin(motor4.stellgroesse); 
        //motor1.setpoint = 62.5;
        //motor2.setpoint = 62.5;
        //motor3.setpoint = 62.5;
        //motor4.setpoint = 62.5;
        //motor1_controller.spin(0); //motor 3
        //motor2_controller.spin(0); //motor 1
        //motor3_controller.spin(1024); //motor 4
        //motor4_controller.spin(0); //motor 2
    }

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, 
        current_rpm2, 
        current_rpm3, 
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );

    // if(millis()-alt >= 250){
    //     lcd.clear();
    //     lcd.setCursor(0, 0);
    //     lcd.print("1: ");
    //     lcd.print(motor1.stellgroesse);
    //     // lcd.setCursor(0, 1);
    //     // lcd.print("Y: ");
    //     // lcd.print(twist_msg.linear.y);
    //     // lcd.setCursor(0, 2);
    //     // lcd.print("Z: ");
    //     // lcd.print(twist_msg.angular.z);
    //     // lcd.clear();
    //     // lcd.setCursor(0, 0);
    //     // lcd.print("I1: ");
    //     // lcd.print(current_rpm1);
    //     // lcd.setCursor(11, 0);
    //     // lcd.print("S1: ");
    //     // lcd.print(motor1.setpoint);

    //     // lcd.setCursor(0, 1);
    //     // lcd.print("I2: ");
    //     // lcd.print(current_rpm2);
    //     // lcd.setCursor(11, 1);
    //     // lcd.print("S2: ");
    //     // lcd.print(motor2.setpoint);

    //     // lcd.setCursor(0, 2);
    //     // lcd.print("I3: ");
    //     // lcd.print(current_rpm3);
    //     // lcd.setCursor(11, 2);
    //     // lcd.print("S3: ");
    //     // lcd.print(motor3.setpoint);

    //     // lcd.setCursor(0, 3);
    //     // lcd.print("I4: ");
    //     // lcd.print(current_rpm4);
    //     // lcd.setCursor(11, 3);
    //     // lcd.print("S4: ");
    //     // lcd.print(motor4.setpoint);
    //     // alt = millis();
    // }
}

void rpmPublisherCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    pwm_soll_msg.data.capacity = 4;
    pwm_soll_msg.data.size = 4;
    pwm_soll_msg.data.data[0] = motor1.setpoint;
    pwm_soll_msg.data.data[1] = motor2.setpoint;
    pwm_soll_msg.data.data[2] = motor3.setpoint;
    pwm_soll_msg.data.data[3] = motor4.setpoint;

    pwm_ist_msg.data.capacity = 4;
    pwm_ist_msg.data.size = 4;
    pwm_ist_msg.data.data[0] = motor1.rpm;
    pwm_ist_msg.data.data[1] = motor2.rpm;
    pwm_ist_msg.data.data[2] = motor3.rpm;
    pwm_ist_msg.data.data[3] = motor4.rpm;

    encoder_msg.data.capacity = 4;
    encoder_msg.data.size = 4;
    // Placed in order from written notes in base
    encoder_msg.data.data[3] = motor1_encoder.read(); // motor 1
    encoder_msg.data.data[2] = motor2_encoder.read(); // motor 2
    encoder_msg.data.data[1] = motor3_encoder.read(); // motor 3
    encoder_msg.data.data[0] = motor4_encoder.read(); // motor 4

    RCSOFTCHECK(rcl_publish(&pwm_soll_publisher, &pwm_soll_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pwm_ist_publisher, &pwm_ist_msg, NULL));
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
}

void publishData()
{
    odom_msg = odometry.getData();
    //imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    //imu_msg.header.stamp.sec = time_stamp.tv_sec;
    //imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;


    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
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