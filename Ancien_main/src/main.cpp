#include <Arduino.h>

#define DIR_PIN          12
#define STEP_PIN         11
#define ENABLE_PIN       15

#define STEP_HIGH       digitalWriteFast(STEP_PIN, HIGH);
#define STEP_LOW        digitalWriteFast(STEP_PIN, LOW);
#define STEP_DELAY      delayMicroseconds(3);
#define DIR_1           digitalWriteFast(DIR_PIN, HIGH);
#define DIR_0           digitalWriteFast(DIR_PIN, LOW);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(DIR_PIN,     HIGH);
  digitalWrite(ENABLE_PIN,  LOW);
  
}

void step(long nbrPas, long vitesse)
{
  bool jobDone = false;
  long pauseInit = 1000000;
  long pause = pauseInit;
  long pas = 0;
  bool montee = true;
  bool descente = false;
  bool plateau = false;
  bool noStep = true; //vrai tant que le pas n'a pas ete effectue
  long n = 0;


  while (!jobDone)
  {
    noStep = true;
    if (pas < nbrPas) //effectue le pas
    {
      delayMicroseconds(pause / 1000);
      STEP_HIGH
      STEP_DELAY
      STEP_LOW

      pas++;
    }
    else //termine la manoeuvre
    {
      jobDone = true;
    }

    if (montee) //acceration
    {
      n++;
      noStep = false;

      if (n == 1) //calcul de la pause en montee
      {
        pause = (long)pause * (1 - ((double)2 / (4 * n + 1)))*0.676;
      }
      else
      {
        pause = (long)pause * (1 - ((double)2 / (4 * n + 1)));
      }

      if (pause < vitesse) //fin "naturelle" de la montee
      {
        pause = vitesse;
        montee = false;
        plateau = true;
      }

      if (n >= (nbrPas / 2) - 1) //fin "forcee" de la montee
      {
        montee = false;
        descente = true;
      }
    }

    if (descente && noStep) //deceleration
    {
      n--;
      noStep = false;

      if (n == 1) //calcul de la pause en descente
      {
        pause = (long)pause * (1 + (double)2 / (4 * n - 1))/0.676;
      }
      else
      {
        pause = (long)pause * (1 + (double)2 / (4 * n - 1));
      }

      if (pause >= pauseInit) //fin de la descente
      {
        pause = pauseInit;
        descente = false;
        plateau = true; //on repasse a un pas constant
      }
    }
    if (plateau && noStep) //vitesse constante
    {
      noStep = false;
      if ((nbrPas - pas) <= n)
      {
        plateau = false;
        descente = true;
      }
    }
  }
}

void loop()
{
  delay(1000);
  DIR_1
  digitalWrite(LED_BUILTIN, HIGH);
  step(6400,    800000);
  delay(1000);
  DIR_0
  digitalWrite(LED_BUILTIN, LOW);
  step(6400,   500000);
}