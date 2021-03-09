#pragma region  // INCLUDE et DEFINE

#include <Arduino.h>
#include <WS2812Serial.h>
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;

#define SPIN_STEP   200
#define DELAY_HIGH  3.0
#define STALL_SPD   1000    // 1 ms


#define PIN_SPIN_0  0
#define PIN_SPIN_1  1
#define PIN_SPIN_2  2
#define PIN_SPIN_3  3

#pragma endregion

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
    volatile long           motPos;             // position reelle du moteur

    volatile unsigned long  brakeZone;          // distance de freinage du moteur (en nombre de pas)
    volatile unsigned int   n;                  // nombre de pas depuis le debut de l'acceleration (pour le calcul du temps de pause durant l'acceleration/freinage)
    volatile double         t;                  // temps restant avant le prochain pas
    volatile double         stepTime;           // temps total du pas actuel
    volatile char           state;              // etat du moteur (a : acceleration ; c : constant ; b : brake)
    volatile long           aim;                // position finale visee par le moteur

    volatile double         speed;              // vitesse (periode) la plus rapide que le moteur va atteindre
    volatile bool           pinState;           // etat du moteur (HIGH ou LOW)
    volatile char           dir;                // sens de deplacement (+1 vers la droite, -1 vers la gauche)
    bool                    jobDone;            // vrai si le moteur n'a pas a avancer ; faux si le moteur est en train d'avancer/demarrer

    volatile void           (*stepHigh)();      // fait avancer le moteur d'un pas (etat de la roche STEP a "HIGH")
    volatile void           (*stepLow)();       // ramene la broche STEP du moteur a l'etat "LOW"
    volatile void           (*dirHigh)();       // definit la direction du moteur (droite)
    volatile void           (*dirLow)();        // (gauche)
};


volatile stepper stepperList[7];
volatile byte stepperFlag;
volatile double timer;
unsigned int i;    
volatile bool canMove;   // vrai si les moteurs sont autorises a bouger
volatile bool emergency = false;    // vrai si un moteur touche l'interrupteur de fin de course
volatile byte emergencyFlag = 0;    // numerote les moteurs qui touchent les interrupteurs de fin de course

volatile controller controllerList[4];  // liste des commandes


void initStepper(stepper mot)
{
    mot.state = 'a';
    mot.motPos = 0;
    mot.aim = 0;
    mot.speed = 0;
    mot.pinState = false;
    mot.dir = 1;
    mot.brakeZone = 0;
    mot.n = 0;
    mot.t = (double) 0;
    mot.stepTime = (double) 0;
    mot.jobDone = true;
}

void stepAndSetStepTime(volatile unsigned int i)    // fait un pas
                                                    // fait varier 't' en fonction de la phase d'acceleration/freinage
                                                    // rajoute la prochaine valeur de t
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

            if (abs(stepperList[i].motPos - stepperList[i].aim) < stepperList[i].n + 1)  // freinage en cas de depassement, d'approche de la cible ou d'interruption de la montee
            {                                                                            // A CHANGER en "min(stepperList[i].brakeZone, stepperList[i].n + 1)" si le calcul de brakeZone a un interet
                stepperList[i].state = 'b';
            }

            if (stepperList[i].state == 'a' && stepperList[i].stepTime <= 1.005 * stepperList[i].speed)
            {
                stepperList[i].stepTime = stepperList[i].speed;
                stepperList[i].state = 'c';
            }

            if (stepperList[i].state == 'a') // le moteur accelere ; calcul de t avec n
            {
                stepperList[i].n++;
                if (stepperList[i].n == 1) // calcul de la pause en montee
                {
                    stepperList[i].stepTime = stepperList[i].stepTime * (1 - (2 / (double)(4 * stepperList[i].n + 1))) * 0.676;
                }
                else
                {
                    stepperList[i].stepTime = stepperList[i].stepTime * (1 - (2 / (double)(4 * stepperList[i].n + 1)));
                }
            }

            if (stepperList[i].state == 'c') // le moteur va a la vitesse max, sauf s'il depasse la zone de freinage : 'b' [fait]
            {
                stepperList[i].stepTime = stepperList[i].speed;
            }

            if (stepperList[i].state == 'b') // s'arrete si le moteur est arrive sur sa cible ; sinon : calcule le nouveau t
            {
                if (stepperList[i].motPos == stepperList[i].aim)
                {
                    stepperList[i].jobDone = true;
                    stepperList[i].state = 'a';
                }
                else
                {
                    if(stepperList[i].speed < STALL_SPD)
                    {
                        stepperList[i].n--;
                        if (stepperList[i].n == 1) // calcul de la pause en descente
                        {
                            stepperList[i].stepTime = stepperList[i].stepTime * (1 + 2 / (double)(4 * stepperList[i].n - 1)) / 0.676;
                        }
                        else
                        {
                            stepperList[i].stepTime = stepperList[i].stepTime * (1 + 2 / (double)(4 * stepperList[i].n - 1));
                        }
                    }
                }
            }

            stepperList[i].t = stepperList[i].stepTime - DELAY_HIGH;
        }
    }
}

void setTimer() // regle le prochain intervalle de pause ; reactualise les flags
{
    timer = 5000000; // 5000 ms
    stepperFlag = 0;
    for (i = 0; i < 7; i++)
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

    for (i = 0; i < 7; i++)
    {
        if(!stepperList[i].jobDone) {stepperList[i].t -= timer;}
    }

    t1.trigger(timer);
}

//Fonctions d'avance et de direction propre a chaque moteur :

//

double getSpeed(int numStepper)
{
    return((double)100000);
}


// INITIATION - ACTUALISATION DU MOUVEMENT

void moveTo(int numStepper, long newAim, double newSpeed)
{
    stepperList[numStepper].n = 0;
    if(stepperList[numStepper].jobDone)
    {
        stepperList[numStepper].stepTime = max(newSpeed, STALL_SPD);    // la vitesse initiale vaut STALL_SPD, sauf si le moteur va plus lentement (vittesse constante)
    }
    if((stepperList[numStepper].aim - stepperList[numStepper].motPos) * (newAim - stepperList[numStepper].motPos) > (double) 0)  // Si la nouvelle cible est à l'oppose de l'ancienne, elle reste la meme
    {
        if(abs(stepperList[numStepper].aim - stepperList[numStepper].motPos) < abs(newAim - stepperList[numStepper].motPos))    // Si la nouvelle cible est plus loin que l'ancienne, elle remplace l'ancienne
        {
            stepperList[numStepper].aim = newAim;
            stepperList[numStepper].state = 'a';
        }
    }

    stepperList[numStepper].speed = newSpeed;

    if(stepperList[numStepper].aim - stepperList[numStepper].motPos >= (double) 0)  // regle le nouveau sens du moteur (reste le meme si en train de decelerer)
    {
        stepperList[numStepper].dir = 1;
        stepperList[numStepper].dirHigh;
    }
    else
    {
        stepperList[numStepper].dir = -1;
        stepperList[numStepper].dirLow;
    }
    
    if(stepperList[numStepper].aim != stepperList[numStepper].motPos)
    {
        stepperList[numStepper].jobDone = false;
    }
}



void timerInterrupt()
{
    if(stepperFlag == 0)
    {
        t1.trigger(20);
    }
    else
    {
        for(i=0; i<7; i++)
        {
            if(stepperFlag & (1<<i))
            {
                stepAndSetStepTime(i);
            }
        }
        setTimer();
    }

    for(i=0; i<4; i++)
    {
        if(stepperList[i].jobDone)
        {
            moveTo(i, controllerList[i].position, getSpeed(i)); // si le moteur s'est arrete pour faire demi-tour, il verifie s'il doit repartir
        }
    }
}

void spinInterrupt0()   // INTERRUPT COMMANDES
{
    if(digitalReadFast(PIN_SPIN_0) == HIGH)                             {controllerList[0].position += SPIN_STEP;}
    else                                                                {controllerList[0].position -= SPIN_STEP;}
    if(controllerList[0].position < controllerList[0].lowerBoundary)    {controllerList[0].position = controllerList[0].lowerBoundary;}
    if(controllerList[0].position > controllerList[0].upperBoundary)    {controllerList[0].position = controllerList[0].upperBoundary;}
}
void spinInterrupt1()   
{
    if(digitalReadFast(PIN_SPIN_1) == HIGH)                             {controllerList[1].position += SPIN_STEP;}
    else                                                                {controllerList[1].position -= SPIN_STEP;}
    if(controllerList[1].position < controllerList[1].lowerBoundary)    {controllerList[1].position = controllerList[1].lowerBoundary;}
    if(controllerList[1].position > controllerList[1].upperBoundary)    {controllerList[1].position = controllerList[1].upperBoundary;}
}
void spinInterrupt2()
{
    if(digitalReadFast(PIN_SPIN_2) == HIGH)                             {controllerList[2].position += SPIN_STEP;}
    else                                                                {controllerList[2].position -= SPIN_STEP;}
    if(controllerList[2].position < controllerList[2].lowerBoundary)    {controllerList[2].position = controllerList[2].lowerBoundary;}
    if(controllerList[2].position > controllerList[2].upperBoundary)    {controllerList[2].position = controllerList[2].upperBoundary;}
}
void spinInterrupt3()
{
    if(digitalReadFast(PIN_SPIN_3) == HIGH)                             {controllerList[3].position += SPIN_STEP;}
    else                                                                {controllerList[3].position -= SPIN_STEP;}
    if(controllerList[3].position < controllerList[3].lowerBoundary)    {controllerList[3].position = controllerList[3].lowerBoundary;}
    if(controllerList[3].position > controllerList[3].upperBoundary)    {controllerList[3].position = controllerList[3].upperBoundary;}


}




// FONCTIONS DE SECURITE

void clearEmgFlags(int numStepper)
{
    emergencyFlag |= ~(1 << numStepper);
}


void setup()
{
    // COMMANDES
    pinMode(PIN_SPIN_0, INPUT);
    attachInterrupt(PIN_SPIN_0, spinInterrupt0, CHANGE);
    pinMode(PIN_SPIN_1, INPUT);
    attachInterrupt(PIN_SPIN_1, spinInterrupt1, CHANGE);
    pinMode(PIN_SPIN_2, INPUT);
    attachInterrupt(PIN_SPIN_2, spinInterrupt2, CHANGE);
    pinMode(PIN_SPIN_3, INPUT);
    attachInterrupt(PIN_SPIN_3, spinInterrupt3, CHANGE);
    


    t1.begin(timerInterrupt);
    t1.trigger(20);
    delay(300);
}

void loop()
{
    delay(300);
}