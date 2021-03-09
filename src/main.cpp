#include <Arduino.h>
#include <WS2812Serial.h>
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;

#define SPIN_STEP 200
#define NUM_STEPPER 7
#define DELAY_HIGH 3.0

#define PIN_SPIN_1  1


OneShotTimer    t1(GPT1);



struct controller
{
    volatile bool       button1;    // etat du bouton 1
    volatile bool       button2;    // etat du bouton 2
    volatile long       position;   // position du bouton
    volatile long       lowerBoundary;  // limite inf de la position
    volatile long       upperBoundary;  // limite sup de la position
};


struct stepper
{
    volatile char           state;              // etat du moteur (a : acceleration ; c : constant ; b : brake)
    volatile long           motPos;             // position reelle du moteur
    volatile long           aim;                // position finale visee par le moteur
    volatile float          speed;              // vitesse (periode) la plus rapide que le moteur va atteindre
    volatile bool           pinState;           // etat du moteur (HIGH ou LOW)
    volatile char           dir;                // sens de deplacement (+1 vers la droite, -1 vers la gauche)
    volatile unsigned long  brakeZone;          // distance de freinage du moteur (en nombre de pas)
    volatile unsigned int   n;                  // nombre de pas depuis le debut de l'acceleration (pour le calcul du temps de pause durant l'acceleration/freinage)
    volatile unsigned long  t;                  // temps restant avant le prochain pas
    volatile unsigned long  stepTime;           // temps total du pas actuel
    bool                    jobDone;            // vrai si le moteur n'a pas a avancer ; faux si le moteur est en train d'avancer/demarrer

    volatile void           (*stepHigh)();      // fait avancer le moteur d'un pas (etat de la roche STEP a "HIGH")
    volatile void           (*stepLow)();       // ramene la broche STEP du moteur a l'etat "LOW"
    volatile void           (*setDir)(char);    // definit la direction du moteur
};


volatile stepper stepperList[7];
volatile byte stepperFlag;
volatile unsigned long timer;
volatile unsigned int i;    
volatile bool canMove;   // vrai si les moteurs sont autorises a bouger
volatile bool emergency;    // vrai si un moteur touche l'interrupteur de fin de course
volatile byte emergencyFlag;    // numerote les moteurs qui touchent les interrupteurs de fin de course

volatile controller controllerList[4];  // liste des commandes

void initStepper(stepper mot)
{
    mot.state = 's';
    mot.motPos = 0;
    mot.aim = 0;
    mot.speed = 0;
    mot.pinState = false;
    mot.dir = 1;
    mot.brakeZone = 0;
    mot.n = 1;
    mot.t = 0;
    mot.stepTime = 0;
    mot.jobDone = true;
}

void stepAndSetStepTime(volatile unsigned int i) // fait un pas ; fait varier 't' en fonction de la phase d'acceleration/freinage ; rajoute la prochaine valeur de t
{
    if (canMove)
    {
        if (!stepperList[i].pinState) // la broche "STEP" du moteur est LOW
        {
            stepperList[i].stepHigh;
            stepperList[i].t = DELAY_HIGH;
            stepperList[i].motPos += stepperList[i].dir;
            stepperList[i].pinState = true;
        }
        else // la broche "STEP" du moteur est HIGH
        {
            stepperList[i].stepLow;

            if (stepperList[i].state == 'a') // le moteur accelere ; calcul de t avec n
            {
            }

            if (stepperList[i].state == 'c') // le moteur va a la vitesse max, sauf s'il depasse la zone de freinage : 'b' [fait]
            {
                if (abs(stepperList[i].motPos - stepperList[i].aim) < stepperList[i].brakeZone)
                {
                    stepperList[i].state = 'b';
                }

                else
                {
                    stepperList[i].t = stepperList[i].speed - DELAY_HIGH;
                }
            }

            if (stepperList[i].state == 'b') // s'arrete si le moteur est arrive sur sa cible ; sinon : calcule le nouveau t
            {
                if (stepperList[i].motPos == stepperList[i].aim)
                {
                    stepperList[i].jobDone = true;
                    stepperList[i].state = 'a';
                }
            }
        }
    }
}

void setTimer() // regle le prochain intervalle de pause ; reactualise les flags
{
    timer = 1000000;
    stepperFlag = 0;
    for (i = 0; i < NUM_STEPPER; i++)
    {
        if ((stepperList[i].t <= timer) && (!stepperList[i].jobDone))
        {
            if (stepperList[i].t == timer)
            {
                stepperFlag |= (1 << i);
            }
            else
            {
                stepperFlag = (1 << i);
                timer = stepperList[i].t;
            }
        }
    }

    for (i = 0; i < NUM_STEPPER; i++)
    {
        stepperList[i].t -= timer;
    }

    t1.trigger(timer);
}

//Fonctions d'avance et de direction propre a chaque moteur :

//

void timerInterrupt()
{
    if(stepperFlag == 0)
    {
        // Cas ou aucun moteur ne doit avancer : que faire ?
        t1.trigger(2);
    }
    else
    {
        for(i=0; i<NUM_STEPPER; i++)
        {
            if(stepperFlag & (1<<i))
            {
                stepAndSetStepTime(i);
            }
        }
        setTimer();
    }
}

void spinInterrupt1()
{
    if(digitalReadFast(PIN_SPIN_1) == HIGH)                             {controllerList[1].position += SPIN_STEP;}
    else                                                                {controllerList[1].position -= SPIN_STEP;}
    if(controllerList[1].position < controllerList[1].lowerBoundary)    {controllerList[1].position = controllerList[1].lowerBoundary;}
    if(controllerList[1].position > controllerList[1].upperBoundary)    {controllerList[1].position = controllerList[1].upperBoundary;}
}



void setup()
{
    pinMode(PIN_SPIN_1, INPUT);
    attachInterrupt(PIN_SPIN_1, spinInterrupt1, CHANGE);


    t1.begin(timerInterrupt);
    t1.trigger(20);
    delay(300);
}

void loop()
{
    delay(300);
}