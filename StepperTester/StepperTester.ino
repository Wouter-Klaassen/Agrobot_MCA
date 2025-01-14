// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Stepper x
#define X_STEP_PIN 23 // step pin
#define X_DIR_PIN 22// Direction pin

AccelStepper stepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);// Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{  
   stepper.setMaxSpeed(1000);
   stepper.setSpeed(50);
   stepper.setAcceleration(500);  
   stepper.moveTo(200);
}

void loop()
{  
   stepper.run();
}
