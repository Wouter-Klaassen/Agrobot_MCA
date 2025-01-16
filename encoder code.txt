#include <AccelStepper.h>

// Motorpinnen
const int motorStepPin = 25;
const int motorDirPin = 26;

// Encoderpinnen
const int encoderA = 32;
const int encoderB = 35;
const int encoderZ = 34;

// Encoderpositie
volatile int encoderPosition = 0;
volatile bool zPulseDetected = false;

// Doelwaarde
int targetPosition = 0;

// AccelStepper-instantie
AccelStepper stepper(AccelStepper::DRIVER, motorStepPin, motorDirPin);

// Encoder interrupt handlers
void IRAM_ATTR handleEncoderA() {
  if (digitalRead(encoderA) == digitalRead(encoderB)) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

void IRAM_ATTR handleEncoderZ() {
  zPulseDetected = true;
}

void setup() {
  // Seriële monitor
  Serial.begin(115200);

  // Motor setup
  stepper.setMaxSpeed(1000); // Maximale snelheid
  stepper.setAcceleration(500); // Versnelling

  // Encoder setup
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  pinMode(encoderZ, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderZ), handleEncoderZ, RISING);

  // Debugbericht
  Serial.println("Motor en encoder gestart. Geef doelwaarde in via seriële monitor.");
  Serial.println("Gebruik 'reset' om de encoderwaarde te resetten naar 0.");
}

void loop() {
  // Controleer seriële invoer
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Controleer of de invoer 'reset' is om de encoderwaarde te resetten
    if (input == "reset") {
      encoderPosition = 0; // Reset de encoderpositie naar 0
      Serial.println("Encoderwaarde gereset naar 0.");
    }
    // Controleer of het een geldig getal is voor doelwaarde
    else if (input.toInt() != 0 || input == "0") {
      targetPosition = input.toInt(); // Stel doelwaarde in
      stepper.moveTo(targetPosition); // Beweeg naar doelwaarde
      Serial.print("Doelwaarde ingesteld: ");
      Serial.println(targetPosition);
    } else {
      Serial.println("Ongeldige invoer. Geef een getal in.");
    }
  }

  // Beweeg de motor naar de doelpositie
  stepper.run();

  // Encoder debuginformatie
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) { // Elke 500 ms debug
    lastPrintTime = millis();
    Serial.print("Huidige encoderpositie: ");
    Serial.print(encoderPosition);
    Serial.print(" - Doelwaarde: ");
    Serial.println(targetPosition);

    if (zPulseDetected) {
      Serial.println("Z-pulse gedetecteerd!");
      zPulseDetected = false;
    }
  }
}
