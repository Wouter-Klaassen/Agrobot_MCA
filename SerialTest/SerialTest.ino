int pos;







void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("arduino ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
      pos = Serial.parseInt();
      if(pos != 0){
        Serial.print("Position: ");
        Serial.println(pos);    
      }
  }
}
