#include <ESP32Servo.h>

Servo myservo;

int pos;

int servoPin = 26;

bool ismoved = false;

void setup() {
  
  Serial.begin(115200);
  Serial.print("adruino online!");


  
  // put your setup code here, to run once:
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 500, 2500); // attaches the servo on pin 18 to the servo object
 }

void loop() {
    // put your main code here, to run repeatedly:
    if (Serial.available() > 0){
      pos = Serial.parseInt();
      if(pos != 0){
        Serial.print("Position: ");
        Serial.println(pos);
        myservo.write(pos);
      }
    }


    
}
