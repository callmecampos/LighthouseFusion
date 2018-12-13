#define HWSERIAL Serial1
#define SampleRate (120)
const int ledPin=13;
void setup() {
  // put your setup code here, to run once:
  HWSERIAL.begin(38400);
  int i=0;
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);  
}

void loop() {
  
  // put your main code here, to run repeatedly:
    HWSERIAL.println("0 1 1 1 1 1 2");
    delay(SampleRate/2);
    HWSERIAL.println("1 0 1 0 1 0 1");
    delay(SampleRate/2);
}
