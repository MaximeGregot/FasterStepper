#include <Arduino.h>
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;

int i = 0;
OneShotTimer t(GPT1);

void fonction()
{
  if(i == 1)
  {
    digitalWriteFast(11, LOW);
    i = 0;
    t.trigger(100.0);
  }
  else
  {
    digitalWriteFast(11, HIGH);
    i = 1;
    t.trigger(3.0);
  }
}

void setup() {
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWriteFast(12, LOW);
  t.begin(fonction);
  t.trigger(500);
}

void loop() {
  delay(500);
}