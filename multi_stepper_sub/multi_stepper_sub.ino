#include <ESP32Servo.h>
//#include <AccelStepper.h>
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


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define LED_PIN 13

// x 1
// Motorpinnen
const int x1_motorStepPin = 25;
const int x1_motorDirPin = 26;

// Encoderpinnen
const int x1_encoderA = 32;
const int x1_encoderB = 35;
const int x1_encoderZ = 34;

// Encoderpositie
volatile int x1_encoderPosition = 0;
volatile bool x1_zPulseDetected = false;

// Motorparameters
unsigned long x1_lastStepTime = 0;
const unsigned long x1_stepInterval = 1000; // 1 ms tussen stappen
int x1_targetPosition = 0; // Doelwaarde voor de encoder
bool x1_motorMoving = false; // Houd bij of de motor actief is

// x 2
// Motorpinnen
const int x2_motorStepPin = 25;
const int x2_motorDirPin = 26;

// Encoderpinnen
const int x2_encoderA = 32;
const int x2_encoderB = 35;
const int x2_encoderZ = 34;

// Encoderpositie
volatile int x2_encoderPosition = 0;
volatile bool x2_zPulseDetected = false;

// Motorparameters
unsigned long x2_lastStepTime = 0;
const unsigned long x2_stepInterval = 1000; // 1 ms tussen stappen
int x2_targetPosition = 0; // Doelwaarde voor de encoder
bool x2_motorMoving = false; // Houd bij of de motor actief is

// init multihreading
TaskHandle_t Task1;
TaskHandle_t Task2;


// Encoder interrupt handlers
void IRAM_ATTR x1_handleEncoderA() {
  if (digitalRead(x1_encoderA) == digitalRead(x1_encoderB)) {
    x1_encoderPosition++;
  } else {
    x1_encoderPosition--;
  }
}

void IRAM_ATTR x1_handleEncoderZ() {
  x1_zPulseDetected = true;
}


// Encoder interrupt handlers
void IRAM_ATTR x2_handleEncoderA() {
  if (digitalRead(x2_encoderA) == digitalRead(x2_encoderB)) {
    x2_encoderPosition++;
  } else {
    x2_encoderPosition--;
  }
}

void IRAM_ATTR x2_handleEncoderZ() {
  x2_zPulseDetected = true;
}

void xyz_callback(const void * msgin)
{
  const geometry_msgs__msg__Point32* msg = (const geometry_msgs__msg__Point32*)msgin;
  float x = msg-> x;
  x1_targetPosition = x; // Stel doelwaarde in
  x2_targetPosition = x;
  x1_motorMoving = true;
  x2_motorMoving = true;
  Serial.print("Doelwaarde ingesteld: ");
  Serial.println(x1_targetPosition);
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setup() {
  // Seriële monitor
  Serial.begin(115200);

  // Microros setup
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));


  // create servo subscriber
//  RCCHECK(rclc_subscription_init_default(
//    &servo_subscriber,
//    &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//    "/servo"));

  // create xyz subscriber
  RCCHECK(rclc_subscription_init_default(
    &xyz_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
      "/xyz"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
//  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &xyz_subscriber, &xyz_msg, &xyz_callback, ON_NEW_DATA));
  // x 1
  // Motor setup
  pinMode(x1_motorStepPin, OUTPUT);
  pinMode(x1_motorDirPin, OUTPUT);
  digitalWrite(x1_motorDirPin, LOW);

  // Encoder setup
  pinMode(x1_encoderA, INPUT_PULLUP);
  pinMode(x1_encoderB, INPUT_PULLUP);
  pinMode(x1_encoderZ, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(x1_encoderA), x1_handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(x1_encoderZ), x1_handleEncoderZ, RISING);

  // x 2
  // Motor setup
  pinMode(x2_motorStepPin, OUTPUT);
  pinMode(x2_motorDirPin, OUTPUT);
  digitalWrite(x2_motorDirPin, LOW);

  // Encoder setup
  pinMode(x2_encoderA, INPUT_PULLUP);
  pinMode(x2_encoderB, INPUT_PULLUP);
  pinMode(x2_encoderZ, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(x2_encoderA), x2_handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(x2_encoderZ), x2_handleEncoderZ, RISING);

  // Debugbericht
  Serial.println("Motor en encoder gestart. Geef doelwaarde in via seriële monitor.");
  Serial.println("Gebruik 'reset' om de encoderwaarde te resetten naar 0.");

    
  xTaskCreatePinnedToCore(Task1code,"Task1", 10000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  delay(500);
}

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
  // x 1
  // Motorbesturing op basis van encoder
  if (x1_motorMoving) {
    if (x1_encoderPosition != x1_targetPosition) {
      // Stel richting in
      bool direction = x1_encoderPosition < x1_targetPosition;
      digitalWrite(x1_motorDirPin, direction);

      // Stap maken
      unsigned long currentTime = micros();
      if (currentTime - x1_lastStepTime >= x1_stepInterval) {
        x1_lastStepTime = currentTime;
        digitalWrite(x1_motorStepPin, HIGH);
        delayMicroseconds(5); // Korte puls
        digitalWrite(x1_motorStepPin, LOW);
      }
    } else {
      // Doelwaarde bereikt, stop de motor
      x1_motorMoving = false;
      Serial.println("Doelwaarde bereikt!");
    }
  }

  // Encoder debuginformatie
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) { // Elke 500 ms debug
    lastPrintTime = millis();
    Serial.print("Huidige encoderpositie: ");
    Serial.print(x1_encoderPosition);
    Serial.print(" - Doelwaarde: ");
    Serial.println(x1_targetPosition);

    if (x1_zPulseDetected) {
      Serial.println("Z-pulse gedetecteerd!");
      x1_zPulseDetected = false;
    }
  }

  // x 2
  if (x2_motorMoving) {
    if (x2_encoderPosition != x2_targetPosition) {
      // Stel richting in
      bool direction = x2_encoderPosition < x2_targetPosition;
      digitalWrite(x2_motorDirPin, direction);

      // Stap maken
      unsigned long currentTime = micros();
      if (currentTime - x2_lastStepTime >= x2_stepInterval) {
        x2_lastStepTime = currentTime;
        digitalWrite(x2_motorStepPin, HIGH);
        delayMicroseconds(5); // Korte puls
        digitalWrite(x2_motorStepPin, LOW);
      }
    } else {
      // Doelwaarde bereikt, stop de motor
      x2_motorMoving = false;
      Serial.println("Doelwaarde bereikt!");
    }
  }

  // Encoder debuginformatie
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) { // Elke 500 ms debug
    lastPrintTime = millis();
    Serial.print("Huidige encoderpositie: ");
    Serial.print(x2_encoderPosition);
    Serial.print(" - Doelwaarde: ");
    Serial.println(x2_targetPosition);

    if (x2_zPulseDetected) {
      Serial.println("Z-pulse gedetecteerd!");
      x2_zPulseDetected = false;
    }
  }

  

  }
}

void loop() {
  // Controleer seriële invoer
//  if (Serial.available()) {
//    String input = Serial.readStringUntil('\n');
//    input.trim();
//
//    // Controleer of de invoer 'reset' is om de encoderwaarde te resetten
//    if (input == "reset") {
//      encoderPosition = 0; // Reset de encoderpositie naar 0
//      Serial.println("Encoderwaarde gereset naar 0.");
//    }
//    // Controleer of het een geldig getal is voor doelwaarde
//    else if (input.toInt() != 0 || input == "0") {
//      targetPosition = input.toInt(); // Stel doelwaarde in
//      motorMoving = true;
//      Serial.print("Doelwaarde ingesteld: ");
//      Serial.println(targetPosition);
//    } else {
//      Serial.println("Ongeldige invoer. Geef een getal in.");
//    }
//  }


}
