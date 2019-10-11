#include "motor_driver.h"

motorDriver md;

uint32_t timerQueue[3];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial.println("openCR initiating...");

  timerQueue[0] = 0; //50ms
  timerQueue[1] = 0; //100ms
  timerQueue[2] = 0; //1000ms

  md.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t t = millis();

  if((t-timerQueue[0]) > 1000)
  {
    timerQueue[0] = t;
    md.movetoPosition(0xFE,2048);
  }

  if((t-timerQueue[1] > 2000))
  {
    timerQueue[1] = t;
    md.movetoPosition(0xFE,2200);
  }
}
