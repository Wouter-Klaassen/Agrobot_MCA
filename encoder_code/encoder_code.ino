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

// Motorparameters
unsigned long lastStepTime = 0;
const unsigned long stepInterval = 1000; // 1 ms tussen stappen
int targetPosition = 0; // Doelwaarde voor de encoder
bool motorMoving = false; // Houd bij of de motor actief is

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
  pinMode(motorStepPin, OUTPUT);
  pinMode(motorDirPin, OUTPUT);
  digitalWrite(motorDirPin, LOW);

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
      motorMoving = true;
      Serial.print("Doelwaarde ingesteld: ");
      Serial.println(targetPosition);
    } else {
      Serial.println("Ongeldige invoer. Geef een getal in.");
    }
  }

  // Motorbesturing op basis van encoder
  if (motorMoving) {
    if (encoderPosition != targetPosition) {
      // Stel richting in
      bool direction = encoderPosition < targetPosition;
      digitalWrite(motorDirPin, direction);

      // Stap maken
      unsigned long currentTime = micros();
      if (currentTime - lastStepTime >= stepInterval) {
        lastStepTime = currentTime;
        digitalWrite(motorStepPin, HIGH);
        delayMicroseconds(5); // Korte puls
        digitalWrite(motorStepPin, LOW);
      }
    } else {
      // Doelwaarde bereikt, stop de motor
      motorMoving = false;
      Serial.println("Doelwaarde bereikt!");
    }
  }

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
