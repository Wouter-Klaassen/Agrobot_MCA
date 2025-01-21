#define end 15


void setup() {
  // put your setup code here, to run once:
  pinMode(end, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(end)==HIGH){
    Serial.println("ENDSTOP HIGH");  
  }
  if(digitalRead(end)==LOW){
    Serial.println("ENDSTOP LOW");
  }
}
