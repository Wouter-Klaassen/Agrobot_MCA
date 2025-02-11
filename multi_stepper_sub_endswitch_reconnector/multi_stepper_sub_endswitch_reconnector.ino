#include <ESP32Servo.h>
#include <AccelStepper.h>
//#include <Wifi.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>

#include <geometry_msgs/msg/point32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rcl_subscription_t servo_subscriber;
std_msgs__msg__Bool servo_msg;
std_msgs__msg__Int32 msg;


rcl_subscription_t xyz_subscriber;
geometry_msgs__msg__Point32 xyz_msg;

rcl_subscription_t cmd_subscriber;
std_msgs__msg__String cmd_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_publisher_t publisher;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define MAXSPEED 1000
#define SPEED 50
#define ACCELERATION 500

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;
//
//#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
//#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define LED_PIN 13

// Motorpinnen
const int x1_motorStepPin = 32;
const int x1_motorDirPin = 33;
const int x1_encoderA = 34;
const int x1_encoderB = 35;

const int x2_motorStepPin = 14;
const int x2_motorDirPin = 12;
const int x2_encoderA = 26;
const int x2_encoderB = 27;

const int y_motorStepPin = 4;
const int y_motorDirPin = 2;
const int y_encoderA = 17;
const int y_encoderB = 16;

const int x1_knopPin = 25;
const int x2_knopPin = 13;
const int y_knopPin = 15;

// Servo pin
Servo gripperservo;
const int servo_pin = 23;
int servoMaxAngle = 360;

// Actuator pins
const int z_actuatorStepPin = 18;
const int z_actuatorDirPin = 19;

const int z_knopPin = 21;

AccelStepper z_stepper(AccelStepper::DRIVER, z_actuatorStepPin, z_actuatorDirPin);


// Encoderpositie
volatile int x1_encoderPosition = 0;

volatile int x2_encoderPosition = 0;

volatile int y_encoderPosition = 0;

//volatile bool zPulseDetected = false;

// Motorparameters
unsigned long x1_lastStepTime = 0;
const unsigned long x1_stepInterval = 1000; // 1 ms tussen stappen
int x1_targetPosition = 0; // Doelwaarde voor de encoder
bool x1_motorMoving = false; // Houd bij of de motor actief is

unsigned long x2_lastStepTime = 0;
const unsigned long x2_stepInterval = 1000; // 1 ms tussen stappen
int x2_targetPosition = 0; // Doelwaarde voor de encoder
bool x2_motorMoving = false; // Houd bij of de motor actief is

unsigned long y_lastStepTime = 0;
const unsigned long y_stepInterval = 1000; // 1 ms tussen stappen
int y_targetPosition = 0; // Doelwaarde voor de encoder
bool y_motorMoving = false; // Houd bij of de motor actief is

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

void IRAM_ATTR x2_handleEncoderA() {
  if (digitalRead(x2_encoderA) == digitalRead(x2_encoderB)) {
    x2_encoderPosition++;
  } else {
    x2_encoderPosition--;
  }
}

void IRAM_ATTR y_handleEncoderA() {
  if (digitalRead(y_encoderA) == digitalRead(y_encoderB)) {
    y_encoderPosition++;
  } else {
    y_encoderPosition--;
  }
}

//void IRAM_ATTR handleEncoderZ() {
//  zPulseDetected = true;
//}

int limitToMaxValue(int value, int maxLimit){
  if(value > maxLimit){
    return maxLimit;
  } else {
    return value;
  }
}

void servo_callback(const void * msgin)
{
  const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
  bool isOpen = msg -> data;
  int servo_position;
  if(isOpen){
    servo_position = 0;
  }else{
    servo_position = 100;
  }
  servo_position = limitToMaxValue(servo_position, servoMaxAngle);
  gripperservo.write(servo_position);

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    msg.data++;
  }
}

void xyz_callback(const void * msgin)
{
  const geometry_msgs__msg__Point32* msg = (const geometry_msgs__msg__Point32*)msgin;
  float x = msg-> x;
  float y = msg -> y;
  float z = msg -> z;
  x1_targetPosition = x; // Stel doelwaarde in
  x1_motorMoving = true;
  x2_targetPosition = x;
  x2_motorMoving = true;
  y_targetPosition = y;
  y_motorMoving = true;
  z_stepper.moveTo(z);
  Serial.print("Doelwaarde ingesteld: ");
  Serial.println(x1_targetPosition + x2_targetPosition + y_targetPosition);
}

void x1_initiate() {
  Serial.println("Initiate gestart: motor beweegt in de richting van de afnemende encoderwaarde.");

  // Beweeg motor in de richting waarbij de encoderwaarde afneemt (met de klok mee)
  digitalWrite(x1_motorDirPin, HIGH); // Richting instellen
  
  // Beweeg de motor totdat de knop wordt ingedrukt
  while (digitalRead(x1_knopPin) == HIGH) {  // Wacht totdat de knop wordt ingedrukt (HIGH = niet ingedrukt, LOW = ingedrukt)
    unsigned long currentTime = micros();
    if (currentTime - x1_lastStepTime >= x1_stepInterval) {
      x1_lastStepTime = currentTime;

      // Stap voor de motor
      digitalWrite(x1_motorStepPin, HIGH);
      delayMicroseconds(5); // Korte puls
      digitalWrite(x1_motorStepPin, LOW);
    }
  }

  // Stop de motor zodra de knop wordt ingedrukt
  Serial.println("Knop ingedrukt. x1 Motor stopt.");
  delay(500); // Korte pauze om trillingen te dempen

  // Zet de motor 100 stappen in de negatieve richting
  Serial.println("x1 Motor maakt 100 stappen in de negatieve richting.");
  digitalWrite(x1_motorDirPin, LOW); // Richting veranderen naar negatief
  int stepsTaken = 0; // Tel het aantal stappen
  while (stepsTaken < 500) {
    unsigned long currentTime = micros();
    if (currentTime - x1_lastStepTime >= x1_stepInterval) {
      x1_lastStepTime = currentTime;

      // Stap voor de motor
      digitalWrite(x1_motorStepPin, HIGH);
      delayMicroseconds(5); // Korte puls
      digitalWrite(x1_motorStepPin, LOW);

      stepsTaken++; // Verhoog de teller
    }
  }

  // Reset de encoderwaarde naar 0
  x1_encoderPosition = 0;
  Serial.println("x1 Encoderwaarde gereset naar 0.");
}


void x2_initiate() {
  Serial.println("Initiate gestart: motor beweegt in de richting van de afnemende encoderwaarde.");

  // Beweeg motor in de richting waarbij de encoderwaarde afneemt (met de klok mee)
  digitalWrite(x2_motorDirPin, HIGH); // Richting instellen
  
  // Beweeg de motor totdat de knop wordt ingedrukt
  while (digitalRead(x2_knopPin) == HIGH) {  // Wacht totdat de knop wordt ingedrukt (HIGH = niet ingedrukt, LOW = ingedrukt)
    unsigned long currentTime = micros();
    if (currentTime - x2_lastStepTime >= x2_stepInterval) {
      x2_lastStepTime = currentTime;

      // Stap voor de motor
      digitalWrite(x2_motorStepPin, HIGH);
      delayMicroseconds(5); // Korte puls
      digitalWrite(x2_motorStepPin, LOW);
    }
  }

  // Stop de motor zodra de knop wordt ingedrukt
  Serial.println("Knop ingedrukt. x2 Motor stopt.");
  delay(500); // Korte pauze om trillingen te dempen

  // Zet de motor 100 stappen in de negatieve richting
  Serial.println("x2 Motor maakt 100 stappen in de negatieve richting.");
  digitalWrite(x2_motorDirPin, LOW); // Richting veranderen naar negatief
  int stepsTaken = 0; // Tel het aantal stappen
  while (stepsTaken < 500) {
    unsigned long currentTime = micros();
    if (currentTime - x2_lastStepTime >= x2_stepInterval) {
      x2_lastStepTime = currentTime;

      // Stap voor de motor
      digitalWrite(x2_motorStepPin, HIGH);
      delayMicroseconds(5); // Korte puls
      digitalWrite(x2_motorStepPin, LOW);

      stepsTaken++; // Verhoog de teller
    }
  }

  // Reset de encoderwaarde naar 0
  x2_encoderPosition = 0;
  Serial.println("x2 Encoderwaarde gereset naar 0.");
}


void y_initiate() {
  Serial.println("Initiate gestart: motor beweegt in de richting van de afnemende encoderwaarde.");

  // Beweeg motor in de richting waarbij de encoderwaarde afneemt (met de klok mee)
  digitalWrite(y_motorDirPin, HIGH); // Richting instellen
  
  // Beweeg de motor totdat de knop wordt ingedrukt
  while (digitalRead(y_knopPin) == HIGH) {  // Wacht totdat de knop wordt ingedrukt (HIGH = niet ingedrukt, LOW = ingedrukt)
    unsigned long currentTime = micros();
    if (currentTime - y_lastStepTime >= x1_stepInterval) {
      y_lastStepTime = currentTime;

      // Stap voor de motor
      digitalWrite(y_motorStepPin, HIGH);
      delayMicroseconds(5); // Korte puls
      digitalWrite(y_motorStepPin, LOW);
    }
  }

  // Stop de motor zodra de knop wordt ingedrukt
  Serial.println("Knop ingedrukt. y Motor stopt.");
  delay(500); // Korte pauze om trillingen te dempen

  // Zet de y motor 100 stappen in de negatieve richting
  Serial.println("y Motor maakt 100 stappen in de negatieve richting.");
  digitalWrite(y_motorDirPin, LOW); // Richting veranderen naar negatief
  int stepsTaken = 0; // Tel het aantal stappen
  while (stepsTaken < 500) {
    unsigned long currentTime = micros();
    if (currentTime - y_lastStepTime >= y_stepInterval) {
      y_lastStepTime = currentTime;

      // Stap voor de motor
      digitalWrite(y_motorStepPin, HIGH);
      delayMicroseconds(5); // Korte puls
      digitalWrite(y_motorStepPin, LOW);

      stepsTaken++; // Verhoog de teller
    }
  }

  // Reset de encoderwaarde naar 0
  y_encoderPosition = 0;
  Serial.println("y Encoderwaarde gereset naar 0.");
}

void z_initiate(){
  z_stepper.moveTo(0);
}

void cmd_callback(const void * msgin){
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  String input = msg->data.data;
  Serial.print(input);

  if ( input == "reset") {
    x1_encoderPosition = 0; // Reset de encoderpositie naar 0
    x2_encoderPosition = 0;
    y_encoderPosition = 0;
    Serial.println("Encoderwaarde gereset naar 0.");
  }
  else if (input == "initiate") {
    x1_initiate();
    x2_initiate();
    y_initiate();
    z_initiate();
  }
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));


  // create servo subscriber
  RCCHECK(rclc_subscription_init_default(
    &servo_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/servo"));

  // create xyz subscriber
  RCCHECK(rclc_subscription_init_default(
    &xyz_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
      "/xyz"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/cmd"));

  // create pub 
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "std_msgs_msg_Int32"));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));


  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &xyz_subscriber, &xyz_msg, &xyz_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_msg, &cmd_callback, ON_NEW_DATA));



  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&servo_subscriber, &node);
  rcl_subscription_fini(&xyz_subscriber, &node);
  rcl_subscription_fini(&servo_subscriber, &node);
  rcl_subscription_fini(&cmd_subscriber, &node);
  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  // Seriële monitor
  Serial.begin(115200);

  // Microros setup
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;

  msg.data = 0;

  // Motor setup
  motorSetup();

  // Encoder setup
  encoderSetup();

  // servo setup
  gripperservo.setPeriodHertz(50);    // standard 50 hz servo
  gripperservo.attach(servo_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object

  // z stepper setup
  z_stepper.setMaxSpeed(MAXSPEED);
  z_stepper.setSpeed(SPEED);
  z_stepper.setAcceleration(ACCELERATION);
  z_stepper.moveTo(0);
  
  attachInterrupt(digitalPinToInterrupt(x1_encoderA), x1_handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(x2_encoderA), x2_handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(y_encoderA), y_handleEncoderA, CHANGE);

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
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
      default:
        break;
    }

    if (state == AGENT_CONNECTED) {
      Serial.println("AGENT CONNECTED");
    } else {
      Serial.println("NO AGENT CONNECTED");
    }
  }
}

// motor task
void Task2code( void * pvParameters){
  Serial.print("Task2 Running on Core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    moveMotorX1();
    moveMotorX2();
    moveMotorY();
    z_stepper.run();
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

void motorSetup(){
  pinMode(x1_motorStepPin, OUTPUT);
  pinMode(x1_motorDirPin, OUTPUT);
  digitalWrite(x1_motorDirPin, LOW);

  pinMode(x2_motorStepPin, OUTPUT);
  pinMode(x2_motorDirPin, OUTPUT);
  digitalWrite(x2_motorDirPin, LOW);

  pinMode(y_motorStepPin, OUTPUT);
  pinMode(y_motorDirPin, OUTPUT);
  digitalWrite(y_motorDirPin, LOW);
}

void encoderSetup(){
  pinMode(x1_encoderA, INPUT_PULLUP);
  pinMode(x1_encoderB, INPUT_PULLUP);

  pinMode(x2_encoderA, INPUT_PULLUP);
  pinMode(x2_encoderB, INPUT_PULLUP);

  pinMode(y_encoderA, INPUT_PULLUP);
  pinMode(y_encoderB, INPUT_PULLUP);
}

void moveMotorX1(){
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

//    if (zPulseDetected) {
//      Serial.println("Z-pulse gedetecteerd!");
//      zPulseDetected = false;
//    }
  }
}

void moveMotorX2(){
// Motorbesturing op basis van encoder
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

//    if (zPulseDetected) {
//      Serial.println("Z-pulse gedetecteerd!");
//      zPulseDetected = false;
//    }
  }
}

void moveMotorY(){
// Motorbesturing op basis van encoder
  if (y_motorMoving) {
    if (y_encoderPosition != y_targetPosition) {
      // Stel richting in
      bool direction = y_encoderPosition < y_targetPosition;
      digitalWrite(y_motorDirPin, direction);

      // Stap maken
      unsigned long currentTime = micros();
      if (currentTime - y_lastStepTime >= y_stepInterval) {
        y_lastStepTime = currentTime;
        digitalWrite(y_motorStepPin, HIGH);
        delayMicroseconds(5); // Korte puls
        digitalWrite(y_motorStepPin, LOW);
      }
    } else {
      // Doelwaarde bereikt, stop de motor
      y_motorMoving = false;
      Serial.println("Doelwaarde bereikt!");
    }
  }

  // Encoder debuginformatie
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) { // Elke 500 ms debug
    lastPrintTime = millis();
    Serial.print("Huidige encoderpositie: ");
    Serial.print(y_encoderPosition);
    Serial.print(" - Doelwaarde: ");
    Serial.println(y_targetPosition);

//    if (zPulseDetected) {
//      Serial.println("Z-pulse gedetecteerd!");
//      zPulseDetected = false;
//    }
  }
}
