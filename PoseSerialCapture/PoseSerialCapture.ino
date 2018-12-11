void setup() {
  // put your setup code here, to run once:
  Serial.begin(500000);
  int i=0;
  pinMode(13, OUTPUT);
}

void loop() {
  
  // put your main code here, to run repeatedly:
    Serial.print("0 1 1 1 1 1 2\n");
    delay(10);
    Serial.print("1 0 1 0 1 0 1\n");

}
