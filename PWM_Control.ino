#include <PWM.h>

int incomingByte;
int pin = 10;
int level = 0;

void setup() {
  InitTimersSafe(); 

  bool success = SetPinFrequencySafe(pin, 25000);

  Serial.begin(9600);
  pinMode(pin, OUTPUT);
}

void loop() {
  while(Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.write(incomingByte);
    level = map(incomingByte, 0, 100, 0, 255);
    Serial.write(level);
  }
  pwmWrite(pin, level);
}
