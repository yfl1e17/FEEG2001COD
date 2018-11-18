//int Green_LEDPin = 4
//int Red_LEDPin = 5
int Switch_Pin = 11
int Buzzer_Pin = 12
void setup() {
  // put your setup code here, to run once:
  //pinmode(Green_LEDPin, OUTPUT);
  //pinmode(Red_LEDPin, OUTPUT);
  pinmode(Switch_Pin, INPUT);
  pinmode(Buzzer_Pin, OUTPUT);
}
void buzz() {
  tone(Buzzer_Pin, 1000, 200)
}
void loop() {
  SwitchState = digitalRead(Switch_Pin);
  if (SwitchState==HIGH) {
    //run operation
    break
  }
  else {
    break
  }

}
