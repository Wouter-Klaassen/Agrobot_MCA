#include <ESP32Servo.h>
#include <AccelStepper.h>
//#include <Wifi.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>

#include <geometry_msgs/msg/point32.h>

rcl_subscription_t servo_subscriber;
std_msgs__msg__Int32 servo_msg;

rcl_subscription_t xyz_subscriber;
geometry_msgs__msg__Point32 xyz_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define MAXSPEED 1000
#define SPEED 50
#define ACCELERATION 500

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Stepper x
#define X_STEP_PIN 19 // step pin
#define X_DIR_PIN 18 // Direction pin

AccelStepper x_stepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);

// Stepper y
#define Y_STEP_PIN 23 // step pin
#define Y_DIR_PIN 22 // dir pin

AccelStepper y_stepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

//Stepper x
#define Z_STEP_PIN 12
#define Z_DIR_PIN 14

AccelStepper z_stepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// init servo
Servo gripperservo;
int servoPin = 4;
int servoMaxAngle = 360;

// init multihreading
TaskHandle_t Task1;
TaskHandle_t Task2;

int limitToMaxValue(int value, int maxLimit){
  if(value > maxLimit){
    return maxLimit;
  } else {
    return value;
  }
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Callback for servo
void servo_callback(const void * msgin)
{
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
  int32_t angle = msg->data;
  int servo_position;
  servo_position = limitToMaxValue(angle, servoMaxAngle);
  gripperservo.write(servo_position);
}

void xyz_callback(const void * msgin)
{
  const geometry_msgs__msg__Point32* msg = (const geometry_msgs__msg__Point32*)msgin;
  float x = msg->x;
  float y = msg->y;
  float z = msg->z;
  x_stepper.moveTo(x);
  y_stepper.moveTo(y);
  z_stepper.moveTo(z);
  
}

void setup() {
//  Serial.begin(115200);
  
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);


  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gripperservo.setPeriodHertz(50);    // standard 50 hz servo
  gripperservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object


  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));


  // create servo subscriber
  RCCHECK(rclc_subscription_init_default(
    &servo_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/servo"));

  // create xyz subscriber
  RCCHECK(rclc_subscription_init_default(
    &xyz_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
      "/xyz"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &xyz_subscriber, &xyz_msg, &xyz_callback, ON_NEW_DATA));

  // x stepper setup
  x_stepper.setMaxSpeed(MAXSPEED);
  x_stepper.setSpeed(SPEED);
  x_stepper.setAcceleration(ACCELERATION);

  // y stepper setup
  y_stepper.setMaxSpeed(MAXSPEED);
  y_stepper.setSpeed(SPEED);
  y_stepper.setAcceleration(ACCELERATION);

  // z stepper setup
  z_stepper.setMaxSpeed(MAXSPEED);
  z_stepper.setSpeed(SPEED);
  z_stepper.setAcceleration(ACCELERATION);
//
  x_stepper.moveTo(0);
  y_stepper.moveTo(0);
  z_stepper.moveTo(0);

  // multithread setup
  
  xTaskCreatePinnedToCore(Task1code,"Task1", 10000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  delay(500);
}

// micro ros task
void Task1code( void * pvParameters){
  Serial.print("Task1 Running on Core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
}

// motor task
void Task2code( void * pvParameters){
  Serial.print("Task2 Running on Core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    x_stepper.run();
    y_stepper.run();
    z_stepper.run();
  }
}

void loop() {
  // empty loop, task one uses same core as loop
  // i.e. core 0
}
