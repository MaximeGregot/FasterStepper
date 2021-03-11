#include <Arduino.h>
#include <WS2812Serial.h>
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;

#define S0_STEP 11
#define S0_DIR  12



int dir;
long ctrl;
long aim;
long pos;
long n;
double speed;
bool jobdone;


PeriodicTimer tPer(GPT2);
OneShotTimer  tOne(GPT1);

void periodic()
{
  if(dir * (ctrl - pos) < 0)
  {
    if(aim - pos > n)
    {
      aim = pos + n;
    }
  }
  else
  {
    if(ctrl > n)
    {
      aim = ctrl;
    }
  }
}

void oneShot()
{


}

void setup()
{
  tPer.begin(periodic, 1000, true);
  tOne.begin(oneShot);

}

void loop() {
  // put your main code here, to run repeatedly:
}