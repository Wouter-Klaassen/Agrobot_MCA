// Motorpinnen
const int motorStepPin1 = 32; 
const int motorDirPin1 = 33; 
const int motorStepPin2 = 25;  // Tweede motor step pin
const int motorDirPin2 = 26;   // Tweede motor dir pin

// Encoderpinnen
const int encoderA1 = 36;  // Encoder 1 A pin
const int encoderB1 = 39;  // Encoder 1 B pin
const int encoderA2 = 12;  // Encoder 2 A pin
const int encoderB2 = 13;  // Encoder 2 B pin

// Encoderpositie
volatile int encoderPosition1 = 0;
volatile int encoderPosition2 = 0;

// Motorparameters
unsigned long lastStepTime = 0;
const unsigned long stepInterval = 1000; // 1 ms tussen stappen
int targetPosition = 0; // Doelwaarde voor de encoder
bool motorMoving = false; // Houd bij of de motor actief is

// Encoder interrupt handlers voor motor 1
void IRAM_ATTR handleEncoderA1() {
  if (digitalRead(encoderA1) == digitalRead(encoderB1)) {
    encoderPosition1++;
  } else {
    encoderPosition1--;
  }
}

// Encoder interrupt handlers voor motor 2
void IRAM_ATTR handleEncoderA2() {
  if (digitalRead(encoderA2) == digitalRead(encoderB2)) {
    encoderPosition2--;
  } else {
    encoderPosition2++;
  }
}

void setup() {
  // Seriële monitor
  Serial.begin(115200);

  // Motor setup
  pinMode(motorStepPin1, OUTPUT);
  pinMode(motorDirPin1, OUTPUT);
  pinMode(motorStepPin2, OUTPUT);
  pinMode(motorDirPin2, OUTPUT);
  
  // Beide motoren beginnen in tegenovergestelde richting
  digitalWrite(motorDirPin1, HIGH);  // Motor 1 draait met de klok mee
  digitalWrite(motorDirPin2, LOW);   // Motor 2 draait tegen de klok in

  // Encoder setup
  pinMode(encoderA1, INPUT_PULLUP);
  pinMode(encoderB1, INPUT_PULLUP);
  pinMode(encoderA2, INPUT_PULLUP);
  pinMode(encoderB2, INPUT_PULLUP);
  
  // Zet interrupt voor de encoders
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA2), handleEncoderA2, CHANGE);

  // Debugbericht
  Serial.println("Motoren en encoders gestart. Geef doelwaarde in via seriële monitor.");
  Serial.println("Gebruik 'reset' om de encoderwaarde te resetten naar 0.");
}

void loop() {
  // Controleer seriële invoer
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Controleer of de invoer 'reset' is om de encoderwaarde te resetten
    if (input == "reset") {
      encoderPosition1 = 0; // Reset de encoderpositie naar 0 voor motor 1
      encoderPosition2 = 0; // Reset de encoderpositie naar 0 voor motor 2
      Serial.println("Encoderwaarden gereset naar 0.");
    }
    // Controleer of het een geldig getal is voor doelwaarde
    else if (input.toInt() != 0 || input == "0") {
      targetPosition = input.toInt(); // Stel doelwaarde in
      motorMoving = true;
      Serial.print("Doelwaarde ingesteld: ");
      Serial.println(targetPosition);
    } else {
      Serial.println("Ongeldige invoer. Geef een getal in.");
    }
  }

  // Motorbesturing op basis van encoder
  if (motorMoving) {
    if (encoderPosition1 != targetPosition || encoderPosition2 != targetPosition) {
      // Stel richting in voor beide motoren (tegenovergestelde richting)
      bool directionMotor1 = encoderPosition1 < targetPosition;
      bool directionMotor2 = encoderPosition2 > targetPosition;  // Tegengestelde richting

      // Stel richting in voor beide motoren
      digitalWrite(motorDirPin1, directionMotor1);
      digitalWrite(motorDirPin2, directionMotor2);

      // Stap maken voor beide motoren
      unsigned long currentTime = micros();
      if (currentTime - lastStepTime >= stepInterval) {
        lastStepTime = currentTime;
        
        // Stap voor de eerste motor
        digitalWrite(motorStepPin1, HIGH);
        delayMicroseconds(5); // Korte puls
        digitalWrite(motorStepPin1, LOW);

        // Stap voor de tweede motor
        digitalWrite(motorStepPin2, HIGH);
        delayMicroseconds(5); // Korte puls
        digitalWrite(motorStepPin2, LOW);
      }
    } else {
      // Doelwaarde bereikt, stop de motoren
      motorMoving = false;
      Serial.println("Doelwaarde bereikt!");
    }
  }

  // Encoder debuginformatie
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) { // Elke 500 ms debug
    lastPrintTime = millis();
    Serial.print("Encoder 1 positie: ");
    Serial.print(encoderPosition1);
    Serial.print(" - Encoder 2 positie: ");
    Serial.print(encoderPosition2);
    Serial.print(" - Doelwaarde: ");
    Serial.println(targetPosition);
  }
}