TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  xTaskCreatePinnedToCore(Task1code,"Task1", 10000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  delay(500);
  
}

void Task1code( void * pvParameters){
  Serial.print("Task1 Running on Core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    Serial.println("Hello from core 1");
    delay(2000);
  }
}

void Task2code( void * pvParameters){
  Serial.print("Task2 Running on Core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    Serial.println("Hello from core 2");
    delay(500);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
