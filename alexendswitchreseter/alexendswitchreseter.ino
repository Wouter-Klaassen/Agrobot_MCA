// Motorpinnen
const int motorStepPin1 = 4; //x:32, x2:14
const int motorDirPin1 = 2; //x:33, x2:12

// Encoderpinnen
const int encoderA1 = 17;  // Encoder 1 A pin x:34, x2:26
const int encoderB1 = 16;  // Encoder 1 B pin x:35, x2:27

const int knopPin = 15; //x: 25, x2:13

// Encoderpositie
volatile int encoderPosition1 = 0;

// Motorparameters
unsigned long lastStepTime = 0;
const unsigned long stepInterval = 1000; // 1 ms tussen stappen
int targetPosition = 0; // Doelwaarde voor de encoder
bool motorMoving = false; // Houd bij of de motor actief is

// Encoder interrupt handler voor motor 1
void IRAM_ATTR handleEncoderA1() { //IRAM_ATTR is nodig om interrupts juist te verwerken
  if (digitalRead(encoderA1) == digitalRead(encoderB1)) {
    encoderPosition1--;
  } else {
    encoderPosition1++;
  }
}

void setup() {
  // Seriële monitor
  Serial.begin(115200);

  // Motor setup
  pinMode(motorStepPin1, OUTPUT);
  pinMode(motorDirPin1, OUTPUT);
  
  // Motor begint tegen de klok in te draaien
  digitalWrite(motorDirPin1, LOW);

  // Encoder setup
  pinMode(encoderA1, INPUT_PULLUP);
  pinMode(encoderB1, INPUT_PULLUP);

  pinMode(knopPin, INPUT_PULLUP);

  // Zet interrupt voor de encoder
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderA1, CHANGE);

  // Debugbericht
  Serial.println("Motor en encoder gestart. Geef doelwaarde in via seriële monitor.");
  Serial.println("Gebruik 'reset' om de encoderwaarde te resetten naar 0.");
}

void initiate() {
  Serial.println("Initiate gestart: motor beweegt in de richting van de afnemende encoderwaarde.");

  // Beweeg motor in de richting waarbij de encoderwaarde afneemt (met de klok mee)
  digitalWrite(motorDirPin1, HIGH); // Richting instellen
  
  // Beweeg de motor totdat de knop wordt ingedrukt
  while (digitalRead(knopPin) == HIGH) {  // Wacht totdat de knop wordt ingedrukt (HIGH = niet ingedrukt, LOW = ingedrukt)
    unsigned long currentTime = micros();
    if (currentTime - lastStepTime >= stepInterval) {
      lastStepTime = currentTime;

      // Stap voor de motor
      digitalWrite(motorStepPin1, HIGH);
      delayMicroseconds(5); // Korte puls
      digitalWrite(motorStepPin1, LOW);
    }
  }

  // Stop de motor zodra de knop wordt ingedrukt
  Serial.println("Knop ingedrukt. Motor stopt.");
  delay(500); // Korte pauze om trillingen te dempen

  // Zet de motor 100 stappen in de negatieve richting
  Serial.println("Motor maakt 100 stappen in de negatieve richting.");
  digitalWrite(motorDirPin1, LOW); // Richting veranderen naar negatief
  int stepsTaken = 0; // Tel het aantal stappen
  while (stepsTaken < 500) {
    unsigned long currentTime = micros();
    if (currentTime - lastStepTime >= stepInterval) {
      lastStepTime = currentTime;

      // Stap voor de motor
      digitalWrite(motorStepPin1, HIGH);
      delayMicroseconds(5); // Korte puls
      digitalWrite(motorStepPin1, LOW);

      stepsTaken++; // Verhoog de teller
    }
  }

  // Reset de encoderwaarde naar 0
  encoderPosition1 = 0;
  Serial.println("Encoderwaarde gereset naar 0.");
}



void loop() {
  // Controleer seriële invoer
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Controleer of de invoer 'reset' is om de encoderwaarde te resetten
    if (input == "reset") {
      encoderPosition1 = 0; // Reset de encoderpositie naar 0
      Serial.println("Encoderwaarde gereset naar 0.");
    }
    else if (input == "initiate") {
      initiate();
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
    if (encoderPosition1 != targetPosition) {
      // Stel richting in voor de motor
      bool directionMotor1 = encoderPosition1 < targetPosition;
      digitalWrite(motorDirPin1, !directionMotor1);

      // Stap maken voor de motor
      unsigned long currentTime = micros();
      if (currentTime - lastStepTime >= stepInterval) {
        lastStepTime = currentTime;
        
        // Stap voor de motor
        digitalWrite(motorStepPin1, HIGH);
        delayMicroseconds(5); // Korte puls
        digitalWrite(motorStepPin1, LOW);
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
    Serial.print("Encoder 1 positie: ");
    Serial.print(encoderPosition1);
    Serial.print(" - Doelwaarde: ");
    Serial.println(targetPosition);
  }
}
