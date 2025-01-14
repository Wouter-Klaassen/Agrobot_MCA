#include <AccelStepper.h>

#define STEP_PIN 16
#define DIR_PIN 32

AccelStepper stepper(::DRIVER, )

void setup() {
  // put your setup code here, to run once:
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(50);
}

void loop() {
  // put your main code here, to run repeatedly:
  stepper.runSpeed();
}
