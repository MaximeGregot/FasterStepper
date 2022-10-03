#include <Arduino.h>
#include <WS2812Serial.h>
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;
#include <i2c_device.h>
#include <SimpleFOC.h>

#define DELTA_HIGH 3.0
#define OFFSCREEN 1000000
#define MARGIN 150

// I/O PINOUT

#define TX_1 1
#define TX_2 8
#define TX_3 14
#define TX_4 17
#define TX_5 20

#define SDA 18
#define SCL 19

#define STEP_0 0  // Player 1
#define STEP_1 6  // Player 2
#define STEP_2 7  // Player 3
#define STEP_3 9  // Player 4
#define STEP_4 10 // Ball Y axis
#define STEP_5 11 // Ball X axis motor 1
#define STEP_6 12 // Ball X axis motor 2
#define DIR_0 24
#define DIR_1 25
#define DIR_2 26
#define DIR_3 27
#define DIR_4 28
#define DIR_5 29
#define DIR_6 30

#define MIN_SWITCH_0 31
#define MIN_SWITCH_1 32
#define MIN_SWITCH_2 33
#define MIN_SWITCH_3 34
#define MIN_SWITCH_4 35
#define MIN_SWITCH_5 36
#define MIN_SWITCH_6 37
#define MAX_SWITCH_0 38
#define MAX_SWITCH_1 39
#define MAX_SWITCH_2 40
#define MAX_SWITCH_3 41
#define MAX_SWITCH_4 15
#define MAX_SWITCH_5 16
#define MAX_SWITCH_6 21

#define DO4 3822
#define RE4 3405
#define MI4 3034
#define FA4 2863
#define SOL4 2551

#define DO5 1910
#define RE5 1703
#define MI5 1517
#define FA5 1432
#define SOL5 1276

#define DO6 955.6
#define RE6 851.3
#define MI6 758.4
#define FA6 715.9
#define SOL6 637.8
#define LA6 568.2
#define SI6 506.2

#define DO7 477.8
#define RE7 425.7
#define MI7 379.2
#define FA7 358.0
#define SOL7 318.9
#define LA7 284.1
#define SI7 253.1
double notes[] = {DO5, RE5, MI5, FA5, SOL5, DO6, RE6, MI6, FA6, SOL6, LA6, SI6, DO7, RE7, MI7, FA7, SOL7, LA7, SI7};

double ref[] = {2000, 828.427, 635.674, 535.898, 472.136, 426.844, 392.523, 365.352, 343.146, 324.555, 308.694, 294.954, 282.899, 272.212, 262.652, 254.033, 246.211, 239.07, 232.517, 226.474, 220.879, 215.68, 210.832, 206.296, 202.041, 198.039, 194.266, 190.7, 187.324, 184.122,
				181.078, 178.18, 175.417, 172.778, 170.256, 167.84, 165.525, 163.303, 161.168, 159.115, 157.138, 155.233, 153.396, 151.622, 149.909, 148.252, 146.649, 145.097, 143.594, 142.136, 140.721, 139.348, 138.015, 136.719, 135.459, 134.233, 133.039, 131.877, 130.745, 129.642,
				128.566, 127.516, 126.492, 125.492, 124.515, 123.561, 122.629, 121.717, 120.825, 119.953, 119.099, 118.263, 117.445, 116.643, 115.858, 115.088, 114.333, 113.593, 112.867, 112.155, 111.456, 110.77, 110.097, 109.436, 108.786, 108.148, 107.521, 106.905, 106.299, 105.704,
				105.118, 104.542, 103.975, 103.418, 102.869, 102.329, 101.798, 101.274, 100.759, 100.251, 99.751, 99.259, 98.773, 98.295, 97.823, 97.359, 96.901, 96.449, 96.003, 95.564, 95.131, 94.703, 94.281, 93.865, 93.454, 93.049, 92.648, 92.253, 91.863, 91.478,
				91.098, 90.722, 90.351, 89.984, 89.622, 89.265, 88.911, 88.562, 88.216, 87.875, 87.538, 87.204, 86.875, 86.549, 86.226, 85.908, 85.592, 85.28, 84.972, 84.667, 84.365, 84.066, 83.771, 83.479, 83.189, 82.903, 82.619, 82.339, 82.061, 81.786,
				81.514, 81.245, 80.978, 80.714, 80.452, 80.193, 79.936, 79.682, 79.43, 79.181, 78.934, 78.689, 78.447, 78.206, 77.968, 77.732, 77.499, 77.267, 77.037, 76.81, 76.584, 76.36, 76.139, 75.919, 75.701, 75.485, 75.271, 75.059, 74.848, 74.639,
				74.432, 74.227, 74.023, 73.821, 73.621, 73.422, 73.225, 73.03, 72.836, 72.643, 72.452, 72.263, 72.075, 71.889, 71.704, 71.52, 71.338, 71.157, 70.977, 70.799, 70.623, 70.447, 70.273, 70.1, 69.928, 69.758, 69.589, 69.421, 69.254, 69.089,
				68.925, 68.761, 68.599, 68.439, 68.279, 68.12, 67.963, 67.806, 67.651, 67.497, 67.344, 67.191, 67.04, 66.89, 66.741, 66.593, 66.446, 66.299, 66.154, 66.01, 65.867, 65.724, 65.583, 65.442, 65.302, 65.164, 65.026, 64.889, 64.752, 64.617,
				64.483, 64.349, 64.216, 64.084, 63.953, 63.823, 63.693, 63.564, 63.436, 63.309, 63.182, 63.057, 62.932, 62.807, 62.684, 62.561, 62.439, 62.318, 62.197, 62.077, 61.958, 61.839, 61.721, 61.604, 61.488, 61.372, 61.256, 61.142, 61.028, 60.915,
				60.802, 60.69, 60.578, 60.467, 60.357, 60.248, 60.139, 60.03, 59.922, 59.815, 59.708, 59.602, 59.496, 59.391, 59.287, 59.183, 59.08, 58.977, 58.875, 58.773, 58.671, 58.571, 58.471, 58.371, 58.272, 58.173, 58.075, 57.977, 57.88, 57.783,
				57.687, 57.591, 57.496, 57.401, 57.307, 57.213, 57.12, 57.027, 56.934, 56.842, 56.75, 56.659, 56.569, 56.478, 56.388, 56.299, 56.21, 56.121, 56.033, 55.945, 55.858, 55.771, 55.685, 55.598, 55.513, 55.427, 55.342, 55.258, 55.174, 55.09,
				55.007, 54.924, 54.841, 54.759, 54.677, 54.595, 54.514, 54.433, 54.353, 54.273, 54.193, 54.113, 54.034, 53.956, 53.877, 53.799, 53.722, 53.644, 53.567, 53.49, 53.414, 53.338, 53.262, 53.187, 53.112, 53.037, 52.963, 52.889, 52.815, 52.741,
				52.668, 52.595, 52.523, 52.45, 52.378, 52.307, 52.235, 52.164, 52.093, 52.023, 51.952, 51.882, 51.813, 51.743, 51.674, 51.605, 51.537, 51.469, 51.4, 51.333, 51.265, 51.198, 51.131, 51.064, 50.998, 50.932, 50.866, 50.8, 50.735, 50.669,
				50.605, 50.54, 50.475, 50.411, 50.347, 50.284, 50.22, 50.157, 50.094, 50.031, 49.969, 49.907, 49.844, 49.783, 49.721, 49.66, 49.599, 49.538, 49.477, 49.417, 49.356, 49.296, 49.237, 49.177, 49.118, 49.059, 49.0, 48.941, 48.882, 48.824,
				48.766, 48.708, 48.65, 48.593, 48.536, 48.479, 48.422, 48.365, 48.309, 48.252, 48.196, 48.14, 48.085, 48.029, 47.974, 47.919, 47.864, 47.809, 47.755, 47.7, 47.646, 47.592, 47.538, 47.485, 47.431, 47.378, 47.325, 47.272, 47.219, 47.167,
				47.114, 47.062, 47.01, 46.958, 46.907, 46.855, 46.804, 46.752, 46.701, 46.651, 46.6, 46.549, 46.499, 46.449, 46.399, 46.349, 46.299, 46.25, 46.2, 46.151, 46.102, 46.053, 46.004, 45.956, 45.907, 45.859, 45.811, 45.763, 45.715, 45.667,
				45.62, 45.572, 45.525, 45.478, 45.431, 45.384, 45.338, 45.291, 45.245, 45.198, 45.152, 45.106, 45.061, 45.015, 44.969, 44.924, 44.879, 44.834, 44.789, 44.744, 44.699, 44.654, 44.61, 44.566, 44.521, 44.477, 44.433, 44.39, 44.346, 44.302,
				44.259, 44.216, 44.173, 44.13, 44.087, 44.044, 44.001, 43.959, 43.916, 43.874, 43.832, 43.79, 43.748, 43.706, 43.664, 43.623, 43.581, 43.54, 43.499, 43.458, 43.417, 43.376, 43.335, 43.295, 43.254, 43.214, 43.173, 43.133, 43.093, 43.053,
				43.013, 42.974, 42.934, 42.894, 42.855, 42.816, 42.776, 42.737, 42.698, 42.66, 42.621, 42.582, 42.544, 42.505, 42.467, 42.429, 42.39, 42.352, 42.314, 42.277, 42.239, 42.201, 42.164, 42.126, 42.089, 42.052, 42.015, 41.978, 41.941, 41.904,
				41.867, 41.83, 41.794, 41.757, 41.721, 41.685, 41.649, 41.613, 41.577, 41.541, 41.505, 41.469, 41.434, 41.398, 41.363, 41.327, 41.292, 41.257, 41.222, 41.187, 41.152, 41.117, 41.082, 41.048, 41.013, 40.979, 40.944, 40.91, 40.876, 40.842,
				40.808, 40.774, 40.74, 40.706, 40.673, 40.639, 40.605, 40.572, 40.539, 40.505, 40.472, 40.439, 40.406, 40.373, 40.34, 40.308, 40.275, 40.242, 40.21, 40.177, 40.145, 40.112, 40.08, 40.048, 40.016, 39.984, 39.952, 39.92, 39.888, 39.857,
				39.825, 39.794, 39.762, 39.731, 39.699, 39.668, 39.637, 39.606, 39.575, 39.544, 39.513, 39.482, 39.451, 39.421, 39.39, 39.36, 39.329, 39.299, 39.269, 39.238, 39.208, 39.178, 39.148, 39.118, 39.088, 39.058, 39.029, 38.999, 38.969, 38.94,
				38.91, 38.881, 38.851, 38.822, 38.793, 38.764, 38.735, 38.706, 38.677, 38.648, 38.619, 38.59, 38.561, 38.533, 38.504, 38.476, 38.447, 38.419, 38.391, 38.362, 38.334, 38.306, 38.278, 38.25, 38.222, 38.194, 38.166, 38.139, 38.111, 38.083,
				38.056, 38.028, 38.001, 37.973, 37.946, 37.919, 37.891, 37.864, 37.837, 37.81, 37.783, 37.756, 37.729, 37.702, 37.676, 37.649, 37.622, 37.596, 37.569, 37.543, 37.516, 37.49, 37.463, 37.437, 37.411, 37.385, 37.359, 37.333, 37.307, 37.281,
				37.255, 37.229, 37.203, 37.178, 37.152, 37.126, 37.101, 37.075, 37.05, 37.024, 36.999, 36.974, 36.948, 36.923, 36.898, 36.873, 36.848, 36.823, 36.798, 36.773, 36.748, 36.724, 36.699, 36.674, 36.649, 36.625, 36.6, 36.576, 36.551, 36.527,
				36.503, 36.478, 36.454, 36.43, 36.406, 36.382, 36.358, 36.334, 36.31, 36.286, 36.262, 36.238, 36.214, 36.191, 36.167, 36.143, 36.12, 36.096, 36.073, 36.049, 36.026, 36.002, 35.979, 35.956, 35.933, 35.909, 35.886, 35.863, 35.84, 35.817,
				35.794, 35.771, 35.749, 35.726, 35.703, 35.68, 35.657, 35.635, 35.612, 35.59, 35.567, 35.545, 35.522, 35.5, 35.478, 35.455, 35.433, 35.411, 35.389, 35.366, 35.344, 35.322, 35.3, 35.278, 35.256, 35.234, 35.213, 35.191, 35.169, 35.147,
				35.126, 35.104, 35.082, 35.061, 35.039, 35.018, 34.996, 34.975, 34.954, 34.932, 34.911, 34.89, 34.868, 34.847, 34.826, 34.805, 34.784, 34.763, 34.742, 34.721, 34.7, 34.679, 34.658, 34.638, 34.617, 34.596, 34.575, 34.555, 34.534, 34.514,
				34.493, 34.473, 34.452, 34.432, 34.411, 34.391, 34.371, 34.35, 34.33, 34.31, 34.29, 34.269, 34.249, 34.229, 34.209, 34.189, 34.169, 34.149, 34.129, 34.11, 34.09, 34.07, 34.05, 34.031, 34.011, 33.991, 33.972, 33.952, 33.932, 33.913,
				33.893, 33.874, 33.855, 33.835, 33.816, 33.797, 33.777, 33.758, 33.739, 33.72, 33.7, 33.681, 33.662, 33.643, 33.624, 33.605, 33.586, 33.567, 33.548, 33.529, 33.511, 33.492, 33.473, 33.454, 33.436, 33.417, 33.398, 33.38, 33.361, 33.343,
				33.324, 33.306, 33.287, 33.269, 33.25, 33.232, 33.214, 33.195, 33.177, 33.159, 33.141, 33.122, 33.104, 33.086, 33.068, 33.05, 33.032, 33.014, 32.996, 32.978, 32.96, 32.942, 32.924, 32.906, 32.889, 32.871, 32.853, 32.835, 32.818, 32.8,
				32.782, 32.765, 32.747, 32.73, 32.712, 32.695, 32.677, 32.66, 32.642, 32.625, 32.608, 32.59, 32.573, 32.556, 32.539, 32.521, 32.504, 32.487, 32.47, 32.453, 32.436, 32.419, 32.402, 32.385, 32.368, 32.351, 32.334, 32.317, 32.3, 32.283,
				32.266, 32.25, 32.233, 32.216, 32.199, 32.183, 32.166, 32.15, 32.133, 32.116, 32.1, 32.083, 32.067, 32.05, 32.034, 32.017, 32.001, 31.985, 31.968, 31.952, 31.936, 31.919, 31.903, 31.887, 31.871, 31.855, 31.838, 31.822, 31.806, 31.79,
				31.774, 31.758, 31.742, 31.726, 31.71, 31.694, 31.678, 31.662, 31.647, 31.631, 31.615, 31.599, 31.583, 31.568, 31.552, 31.536, 31.521, 31.505, 31.489, 31.474, 31.458, 31.443, 31.427, 31.411, 31.396, 31.381, 31.365, 31.35, 31.334, 31.319,
				31.304, 31.288, 31.273, 31.258, 31.242, 31.227, 31.212, 31.197, 31.182, 31.166, 31.151, 31.136, 31.121, 31.106, 31.091, 31.076, 31.061, 31.046, 31.031, 31.016, 31.001, 30.986, 30.971, 30.957, 30.942, 30.927, 30.912, 30.897, 30.883, 30.868,
				30.853, 30.839, 30.824, 30.809, 30.795, 30.78, 30.766, 30.751, 30.737, 30.722, 30.708, 30.693, 30.679, 30.664, 30.65, 30.635, 30.621, 30.607, 30.592, 30.578, 30.564, 30.549, 30.535, 30.521, 30.507, 30.493, 30.478, 30.464, 30.45, 30.436,
				30.422, 30.408, 30.394, 30.38, 30.366, 30.352, 30.338, 30.324, 30.31, 30.296, 30.282, 30.268, 30.254, 30.241, 30.227, 30.213, 30.199, 30.185, 30.172, 30.158, 30.144, 30.131, 30.117, 30.103, 30.09, 30.076, 30.062, 30.049, 30.035, 30.022,
				30.008, 29.995};

struct stepper
{
	long n;					// position dans la liste ref[]
	int dir;				// direction du mouvement (+/-1)
	long pos;				// position du moteur
	long aim;				// cible du moteur
	double stepT;			// prochain temps de pause
	double speed;			// temps de pause minimum pour le mouvement actuel
	double delta;			// temps de pause restant avant le prochain pas
	bool move;				// GET true if the motor is moving
	bool brake;				// GET true if the motor is braking
	long min;				// minimum absolute position (determined by the limit switch)
	long max;				// maximum ---
	bool minSwitch;			// left limit switch is triggered
	bool maxSwitch;			// right limit switch is triggered
	unsigned char initStep; // step of the motor's initialization
};

struct controller
{
	long pos;	// position de la commande
	bool input; // bouton du joueur
};

int compteur = 0;
int variable = 0; // used for testing
stepper s[7];
controller cmd[7];
byte nextMoveFlag = 0;
byte emergency = 0;
double timer;
bool gameOn = false;

PeriodicTimer ptimer(GPT1);	  // adjusts the aim of each motor, based on the controller's position
PeriodicTimer ticktimer(TCK); // initializes the position of every motor
OneShotTimer ostimer(GPT2);	  // calls the "step" function when a motor needs to move
PeriodicTimer serial(TCK);	  // variable monitoring

void initStepper(int i)
{
	s[i].n = 0;
	s[i].dir = 1;
	s[i].pos = 0;
	s[i].aim = 0;
	s[i].stepT = 0;
	s[i].speed = 200;
	s[i].delta = 0;
	s[i].move = false;
	s[i].brake = false;
	s[i].min = -OFFSCREEN;
	s[i].max = OFFSCREEN;
	s[i].minSwitch = false;
	s[i].maxSwitch = false;
	s[i].initStep = 1;
}

void initCommand(int i) // initializes the "cmd" entities
{
	cmd[i].pos = 0;
	cmd[i].input = false;
}

void declarePinout() // initializes the pinout
{
	pinMode(STEP_0, OUTPUT);
	pinMode(STEP_1, OUTPUT);
	pinMode(STEP_2, OUTPUT);
	pinMode(STEP_3, OUTPUT);
	pinMode(STEP_4, OUTPUT);
	pinMode(STEP_5, OUTPUT);
	pinMode(STEP_6, OUTPUT);

	pinMode(DIR_0, OUTPUT);
	pinMode(DIR_1, OUTPUT);
	pinMode(DIR_2, OUTPUT);
	pinMode(DIR_3, OUTPUT);
	pinMode(DIR_4, OUTPUT);
	pinMode(DIR_5, OUTPUT);
	pinMode(DIR_6, OUTPUT);

	pinMode(MIN_SWITCH_0, INPUT_PULLUP);
	pinMode(MIN_SWITCH_1, INPUT_PULLUP);
	pinMode(MIN_SWITCH_2, INPUT_PULLUP);
	pinMode(MIN_SWITCH_3, INPUT_PULLUP);
	pinMode(MIN_SWITCH_4, INPUT_PULLUP);
	pinMode(MIN_SWITCH_5, INPUT_PULLUP);
	pinMode(MIN_SWITCH_6, INPUT_PULLUP);

	pinMode(MAX_SWITCH_0, INPUT_PULLUP);
	pinMode(MAX_SWITCH_1, INPUT_PULLUP);
	pinMode(MAX_SWITCH_2, INPUT_PULLUP);
	pinMode(MAX_SWITCH_3, INPUT_PULLUP);
	pinMode(MAX_SWITCH_4, INPUT_PULLUP);
	pinMode(MAX_SWITCH_5, INPUT_PULLUP);
	pinMode(MAX_SWITCH_6, INPUT_PULLUP);

	pinMode(TX_1, OUTPUT);
	pinMode(TX_2, OUTPUT);
	pinMode(TX_3, OUTPUT);
	pinMode(TX_4, OUTPUT);
	pinMode(TX_5, OUTPUT);
}

double setTimer() // sets the new timer before the next "step()" function
{
	timer = 200;
	nextMoveFlag = 0;

	for (int i = 0; i < 7; i++)
	{
		if (s[i].move && (s[i].delta < timer))
		{
			if (s[i].delta == timer)
			{
				nextMoveFlag |= (1 << i);
			}
			else
			{
				nextMoveFlag = (1 << i);
				timer = s[i].delta;
			}
		}
	}
	timer = max(timer, 0.1);
	for (int i = 0; i < 7; i++)
	{
		if (s[i].move)
		{
			s[i].delta -= timer;
		}
	}
	return (timer);
}

void dwfDir(int i, int state)
{
	switch (i)
	{
	case 0:
		digitalWriteFast(DIR_0, state); // replace "state" by "1 - state" if the motor turns the wrong way round
		break;
	case 1:
		digitalWriteFast(DIR_1, state);
		break;
	case 2:
		digitalWriteFast(DIR_2, state);
		break;
	case 3:
		digitalWriteFast(DIR_3, state);
		break;
	case 4:
		digitalWriteFast(DIR_4, state);
		break;
	case 5:
		digitalWriteFast(DIR_5, state);
		break;
	case 6:
		digitalWriteFast(DIR_6, state);
		break;
	default:
		break;
	}
}
void dwfStep(int i, int state)
{
	switch (i)
	{
	case 0:
		digitalWriteFast(STEP_0, state);
		break;
	case 1:
		digitalWriteFast(STEP_1, state);
		break;
	case 2:
		digitalWriteFast(STEP_2, state);
		break;
	case 3:
		digitalWriteFast(STEP_3, state);
		break;
	case 4:
		digitalWriteFast(STEP_4, state);
		break;
	case 5:
		digitalWriteFast(STEP_5, state);
		break;
	case 6:
		digitalWriteFast(STEP_6, state);
		break;
	default:
		break;
	}
}
int drfStep(int i)
{
	switch (i)
	{
	case 0:
		return (digitalReadFast(STEP_0));
		break;
	case 1:
		return (digitalReadFast(STEP_1));
		break;
	case 2:
		return (digitalReadFast(STEP_2));
		break;
	case 3:
		return (digitalReadFast(STEP_3));
		break;
	case 4:
		return (digitalReadFast(STEP_4));
		break;
	case 5:
		return (digitalReadFast(STEP_5));
		break;
	case 6:
		return (digitalReadFast(STEP_6));
		break;
	default:
		return (LOW);
		break;
	}
}

void setDir(int i)
{
	if (s[i].aim - s[i].pos < 0)
	{
		s[i].dir = -1;
		dwfDir(i, LOW);
	}
	else
	{
		s[i].dir = 1;
		dwfDir(i, HIGH);
	}
}

void pTimer() // adjusts the aim of each motor, based on the controller's position
{
	for (int i = 0; i < 7; i++)
	{
		if (s[i].move)
		{
			if (((cmd[i].pos - s[i].pos) * (s[i].aim - s[i].pos) < 0.0) || (abs(cmd[i].pos - s[i].pos) <= s[i].n))
			{
				s[i].aim = s[i].pos + (s[i].dir * s[i].n); // le (n-1) a ete enleve
			}
			else
			{
				s[i].aim = cmd[i].pos;
				s[i].brake = false;
			}
		}
		else
		{
			if (s[i].aim != cmd[i].pos)
			{
				s[i].aim = cmd[i].pos;
				setDir(i);
				s[i].n = 0;
				s[i].move = true;
			}
		}
	}
	for (int i = 0; i < 7; i++)
	{
		if (emergency & (1 << i))
		{
			s[i].move = false;
		}
	}
}

void step(int i) // Makes the motor move one step ahead and sets the speed according to the max speed
{
	if (drfStep(i) == LOW && s[i].move)
	{
		s[i].delta = DELTA_HIGH;
		dwfStep(i, HIGH);
		s[i].pos += s[i].dir;
	}
	else if (s[i].pos != s[i].aim)
	{
		if (s[i].n >= abs(s[i].aim - s[i].pos))
		{
			s[i].brake = true;
		}
		if (s[i].brake)
		{
			s[i].n--;
			s[i].stepT = ref[s[i].n - 1];
		}
		else
		{
			if (ref[s[i].n] > s[i].speed)
			{
				s[i].stepT = ref[s[i].n];
				s[i].n++;
			}
			else
			{
				s[i].stepT = s[i].speed;
			}
		}
		s[i].delta = s[i].stepT - DELTA_HIGH;
		dwfStep(i, LOW);
	}
	if (s[i].pos == s[i].aim)
	{
		s[i].move = false;
		s[i].brake = false;
		s[i].n = 0;
	}
}

void setEmergency(int i) // Stops the motor immediately
{
	emergency &= (1 << i);
}

void clearEmergency(int i) // Acquits the emergency
{
	emergency &= ~(1 << i);
}

void intMinSwitch0()
{
	s[0].minSwitch = !(bool)digitalReadFast(MIN_SWITCH_0);
	if (s[0].minSwitch)
	{
		setEmergency(0);
	}
}
void intMinSwitch1()
{
	s[1].minSwitch = !(bool)digitalReadFast(MIN_SWITCH_1);
	if (s[1].minSwitch)
	{
		setEmergency(1);
	}
}
void intMinSwitch2()
{
	s[2].minSwitch = !(bool)digitalReadFast(MIN_SWITCH_2);
	if (s[2].minSwitch)
	{
		setEmergency(2);
	}
}
void intMinSwitch3()
{
	s[3].minSwitch = !(bool)digitalReadFast(MIN_SWITCH_3);
	if (s[3].minSwitch)
	{
		setEmergency(3);
	}
}
void intMinSwitch4()
{
	s[4].minSwitch = !(bool)digitalReadFast(MIN_SWITCH_4);
	if (s[4].minSwitch)
	{
		setEmergency(4);
	}
}
void intMinSwitch5()
{
	s[5].minSwitch = !(bool)digitalReadFast(MIN_SWITCH_5);
	if (s[5].minSwitch)
	{
		setEmergency(5);
	}
}
void intMinSwitch6()
{
	s[6].minSwitch = !(bool)digitalReadFast(MIN_SWITCH_6);
	if (s[6].minSwitch)
	{
		setEmergency(6);
	}
}

void intMaxSwitch0()
{
	s[0].maxSwitch = !(bool)digitalReadFast(MAX_SWITCH_0);
	if (s[0].maxSwitch)
	{
		setEmergency(0);
	}
}
void intMaxSwitch1()
{
	s[1].maxSwitch = !(bool)digitalReadFast(MAX_SWITCH_1);
	if (s[1].maxSwitch)
	{
		setEmergency(1);
	}
}
void intMaxSwitch2()
{
	s[2].maxSwitch = !(bool)digitalReadFast(MAX_SWITCH_2);
	if (s[2].maxSwitch)
	{
		setEmergency(2);
	}
}
void intMaxSwitch3()
{
	s[3].maxSwitch = !(bool)digitalReadFast(MAX_SWITCH_3);
	if (s[3].maxSwitch)
	{
		setEmergency(3);
	}
}
void intMaxSwitch4()
{
	s[4].maxSwitch = !(bool)digitalReadFast(MAX_SWITCH_4);
	if (s[4].maxSwitch)
	{
		setEmergency(4);
	}
}
void intMaxSwitch5()
{
	s[5].maxSwitch = !(bool)digitalReadFast(MAX_SWITCH_5);
	if (s[5].maxSwitch)
	{
		setEmergency(5);
	}
}
void intMaxSwitch6()
{
	s[6].maxSwitch = !(bool)digitalReadFast(MAX_SWITCH_6);
	if (s[6].maxSwitch)
	{
		setEmergency(6);
	}
}

void initAllSwitches() // initializes all limit switches
{
	intMinSwitch0();
	intMinSwitch1();
	intMinSwitch2();
	intMinSwitch3();
	intMinSwitch4();
	intMinSwitch5();
	intMinSwitch6();
	intMaxSwitch0();
	intMaxSwitch1();
	intMaxSwitch2();
	intMaxSwitch3();
	intMaxSwitch4();
	intMaxSwitch5();
	intMaxSwitch6();
}

void interruptsInit()
{
	attachInterrupt(MIN_SWITCH_0, intMinSwitch0, RISING); // A CHANGER
	attachInterrupt(MIN_SWITCH_1, intMinSwitch1, CHANGE);
	attachInterrupt(MIN_SWITCH_2, intMinSwitch2, CHANGE);
	attachInterrupt(MIN_SWITCH_3, intMinSwitch3, CHANGE);
	attachInterrupt(MIN_SWITCH_4, intMinSwitch4, CHANGE);
	attachInterrupt(MIN_SWITCH_5, intMinSwitch5, CHANGE);
	attachInterrupt(MIN_SWITCH_6, intMinSwitch6, CHANGE);

	attachInterrupt(MAX_SWITCH_0, intMaxSwitch0, CHANGE);
	attachInterrupt(MAX_SWITCH_1, intMaxSwitch1, CHANGE);
	attachInterrupt(MAX_SWITCH_2, intMaxSwitch2, CHANGE);
	attachInterrupt(MAX_SWITCH_3, intMaxSwitch3, CHANGE);
	attachInterrupt(MAX_SWITCH_4, intMaxSwitch4, CHANGE);
	attachInterrupt(MAX_SWITCH_5, intMaxSwitch5, CHANGE);
	attachInterrupt(MAX_SWITCH_6, intMaxSwitch6, CHANGE);
}

void osTimer()
{
	for (int i = 0; i < 7; i++)
	{
		if (nextMoveFlag & (1 << i))
		{
			step(i);
		}
	}
	ostimer.trigger(setTimer());
}

void setupLimits()
{
	for (int i = 2; i < 7; i++)
	{
		long middle = (s[i].min + s[i].max) / 2;
		long delta = s[i].max - s[i].min - 2 * MARGIN;
		s[i].min = middle - delta;
		s[i].max = middle + delta;
		s[i].pos -= middle;
		cmd[i].pos -= middle;
	}
	for (int i = 0; i < 2; i++)
	{
		long middle = (s[i].min + s[i].max) / 2;
		long delta = s[i].max - s[i].min - 2 * MARGIN;
	}
}

void tickTimer()
{
	for (int i = 0; i < 7; i++)
	{
		switch (s[i].initStep)
		{
		case 1: // goes to the minSwitch
			s[i].speed = 40;
			cmd[i].pos = -OFFSCREEN;
			s[i].initStep++;
			break;

		case 2: // waits for minSwitch to be triggered
			if (s[i].minSwitch)
			{
				clearEmergency(i);
				s[i].initStep++;
			}
			break;
			
		case 3: // X-axis : wait for second motor
			if (i <= 4)
			{
				s[i].initStep++;
			}
			else if (s[5].initStep >= 3 && s[6].initStep >= 3)
			{
				s[i].initStep++;
			}
			break;

		case 4: // backs away from minSwitch
			cmd[i].pos = s[i].pos + MARGIN;
			s[i].initStep++;
			break;

		case 5: // if arrived, goes back towards minSwitch, but slowly
			if (s[i].pos == cmd[i].pos)
			{
				s[i].speed = 400;
				cmd[i].pos = -OFFSCREEN;
				s[i].initStep++;
			}
			break;

		case 6:
			if (s[i].minSwitch)
			{
				clearEmergency(i);
				s[i].initStep++;
			}
			break;

		case 7: // X-axis : wait for second motor
			if (i <= 4)
			{
				s[i].min = s[i].pos;
				s[i].initStep++;
			}
			else if (s[5].initStep >= 7 && s[6].initStep >= 7)
			{
				s[i].min = s[i].pos;
				s[i].initStep++;
			}
			break;
	// FIN DE LA VERIF
		case 8:
				s[i].speed = 40;
				s[i].aim = OFFSCREEN;
				s[i].initStep++;
		break;

		case 9: // goes to the maxSwitch
			if (s[i].maxSwitch)
			{
				clearEmergency(i);
				s[i].initStep++;
			}
			break;

		case 7: // X-axis : wait for second motor
			if (i <= 4)
			{
				s[i].initStep++;
			}
			else if (s[5].initStep >= 7 && s[6].initStep >= 7)
			{
				s[i].initStep++;
			}
			break;
		case 8: // backs away from maxSwitch
			cmd[i].pos = s[i].pos - MARGIN;
			s[i].initStep++;
			break;
		case 9: // if arrived, goes back towards maxSwitch, but slowly
			if (s[i].pos == cmd[i].pos)
			{
				s[i].speed = 400;
				cmd[i].pos = OFFSCREEN;
				s[i].initStep++;
			}
			break;
		case 10: // X-axis : wait for second motor
			if (i <= 4)
			{
				s[i].initStep++;
				s[i].min = s[i].pos;
			}
			else if (s[5].initStep >= 10 && s[6].initStep >= 10)
			{
				s[i].initStep++;
				s[i].max = s[i].pos;
			}
			break;
		case 11: // end of the setup process
			if (i <= 4)
			{
				cmd[i].pos = s[i].pos;
			}
			else if (s[5].initStep == 11 && s[6].initStep == 11)
			{
				;
			}
		case 0: // the setup id done
			break;
		default:
			s[i].initStep = 1;
			break;
		}
	}
}

// void tickTimer()
// {
//   for (int i = 0; i < 7; i++)
//   {
//     if (!s[i].lTouch)
//     {
//       if (emergency & (1 << i)) // clears emergency ; moves out of the switch
//       {
//         if ((i < 2) && ((emergency & 0b11) == 0b11))
//         {
//           cmd[0].pos = s[0].pos + MARGIN;
//           cmd[1].pos = s[1].pos + MARGIN;
//         }
//         else if (i >= 2)
//         {
//           clearEmergency(i);
//           s[i].lTouch = true;
//           cmd[i].pos = s[i].pos + MARGIN;
//         }
//       }
//       else // moves to the left quickly
//       {
//         s[i].speed = 40;
//         cmd[i].pos = -OFFSCREEN;
//       }
//     }
//     else if (!s[i].lOk)
//     {
//       if (emergency & (1 << i)) // clears emergency ; moves out of the switch
//       {
//         if ((i < 2) && ((emergency & 0b11) == 0b11))
//         {
//           emergency &= ~0b11;
//           s[0].min = s[0].pos;
//           s[1].min = s[1].pos;
//           s[0].lTouch = true;
//           s[1].lTouch = true;
//           cmd[0].pos = s[0].min + MARGIN;
//           cmd[1].pos = s[1].min + MARGIN;
//         }
//         else if (i >= 2)
//         {
//           s[i].min = s[i].pos;
//           clearEmergency(i);
//           s[i].lOk = true;
//           cmd[i].pos = s[i].min + MARGIN;
//         }
//       }
//       else if (s[i].pos == cmd[i].pos) // moves left slowly
//       {
//         s[i].speed = 400;
//         cmd[i].pos = -OFFSCREEN;
//       }
//     }
//     if (!s[i].rTouch)
//     {
//       if (emergency & (1 << i)) // clears emergency ; moves out of the switch
//       {
//         if ((i < 2) && ((emergency & 0b11) == 0b11))
//         {
//           emergency &= ~0b11;
//           s[0].max = s[0].pos;
//           s[1].max = s[1].pos;
//           s[0].rTouch = true;
//           s[1].rTouch = true;
//           cmd[0].pos = s[1].pos - MARGIN;
//           cmd[1].pos = s[0].pos - MARGIN;
//         }
//         else if (i >= 2)
//         {
//           clearEmergency(i);
//           s[i].rTouch = true;
//           cmd[i].pos = s[i].pos - MARGIN;
//         }
//       }
//       else // moves to the right quickly
//       {
//         s[i].speed = 40;
//         cmd[i].pos = OFFSCREEN;
//       }
//     }
//     else if (!s[i].rOk)
//     {
//       if (emergency & (1 << i)) // clears emergency ; moves out of the switch
//       {
//         if ((i < 2) && ((emergency & 0b11) == 0b11))
//         {
//           emergency &= ~0b11;
//           s[0].max = s[0].pos;
//           s[1].max = s[1].pos;
//           s[0].rTouch = true;
//           s[1].rTouch = true;
//           cmd[0].pos = s[1].pos - MARGIN;
//           cmd[1].pos = s[0].pos - MARGIN;
//         }
//         else if (i >= 2)
//         {
//         s[i].max = s[i].pos;
//         clearEmergency(i);
//         s[i].lOk = true;
//         cmd[i].pos = s[i].max - MARGIN;
//         }
//       }
//       else if (s[i].pos == cmd[i].pos) // moves right slowly
//       {
//         s[i].speed = 400;
//         cmd[i].pos = OFFSCREEN;
//       }
//     }
//     else
//     {
//       if (i >= 2)
//       {
//         long middle = (s[i].min + s[i].max) / 2;
//         long delta = s[i].max - s[i].min - 2 * MARGIN;
//         s[i].min = middle - delta;
//         s[i].max = middle + delta;
//         s[i].pos -= middle;
//         cmd[i].pos -= middle;
//       }
//       else
//       {
//         ;
//       }
//     }
//   }
// }

/*
void tickTimer()
{
  for (int i = 0; i < 7; i++)
  {
	if ((i < 2) || (s[0].maxSwitch || s[1].maxSwitch)) // CHANGE : use "rOk" and "lOk" + "lTouch" and "rTouch"
	{
	  cmd[i].pos = s[i].pos;
	  s[0].ok = true;
	  s[1].ok = true;
	}

	if ((s[i].minSwitch == true) && (s[i].min == -OFFSCREEN))
	{
	  s[i].min = s[i].pos;
	  cmd[i].pos = s[i].min;
	}
	else if ((s[i].maxSwitch == true) && (s[i].max == OFFSCREEN))
	{
	  s[i].max = s[i].pos;
	  cmd[i].pos = s[i].max;
	  s[i].ok = true;
	}
	else if (!s[i].move)
	{
	  if ((i > 1) || (!s[0].move && !s[1].move))
	  {
		if (!s[i].ok)
		{
		  cmd[i].pos = OFFSCREEN;
		  if (i < 2)
		  {
			cmd[0].pos = OFFSCREEN;
			cmd[1].pos = OFFSCREEN;
		  }
		}
		else
		{
		  if (i > 1)
		  {
			s[i].pos -= (s[i].min + s[i].max) / 2;
			cmd[i].pos = 0;
		  }
		  else
		  {
			if (s[0].max < s[1].max)
			{
			  s[0].pos -= (s[0].max + s[0].min) / 2;
			  s[1].pos -= s[1].min - s[0].min;
			}
			else
			{
			  s[1].pos -= (s[1].max + s[1].min) / 2;
			  s[0].pos -= s[0].min - s[1].min;
			}
		  }
		}
	  }
	}
  }
}
*/

void initialize()
{
	int i;
	declarePinout();

	for (i = 0; i < 7; i++)
	{
		initStepper(i);
		initCommand(i);
	}

	initAllSwitches();
	interruptsInit();

	ticktimer.begin(tickTimer, 10.0);
	bool isFinished = false;
	while (!isFinished)
	{
		isFinished = true;
		for (i = 0; i < 7; i++)
		{
			if (!(s[i].initStep == 0))
			{
				isFinished = false;
			}
		}
	}
	ticktimer.stop();

	for (i = 0; i < 7; i++)
	{
		long length = s[i].max - s[i].min - MARGIN;
		s[i].min = -length / 2;
		s[i].max = length / 2;
	}
}

void suivi()
{
	Serial.println(String(cmd[0].pos) + "\t aim0 : " + String(s[0].aim) + "\t pos0 : " + String(s[0].pos) + "\t n0 : " + String(s[0].n) + "\t compteur : " + String(compteur));
	// Serial.println(String(cmd[1].pos) + "\t aim1 : " + String(s[1].aim) + "\t pos1 : " + String(s[1].pos) + "\t n1 : " + String(s[1].n));
}

void setup()
{
	delay(2000);
	declarePinout();
	interruptsInit();
	// Serial.begin(9600);
	// initialize();
	initStepper(0);
	initCommand(0);

	// TEST 7 STEPPERS
	for (int i = 0; i < 7; i++)
	{
		initStepper(i);
		initCommand(i);
	}

	ptimer.begin(pTimer, 20.0);
	delay(2000);
	ostimer.begin(osTimer);
	ostimer.trigger(300.0);
	// serial.begin(suivi, 400000);
}

void loop()
{

	// TEST 7 STEPPERS
	for (int i = 0; i < 7; i++)
	{
		s[i].speed = random(60, 500);
		cmd[i].pos = 0;
	}
	delay(1000);

	for (int i = 0; i < 7; i++)
	{
		cmd[i].pos = random(8000, 32000);
	}
	delay(2000);

	// for (variable = 0; variable < (signed)(sizeof(notes) / sizeof(notes[0])); variable++)
	// {
	// 	s[0].speed = notes[variable];
	// 	compteur = 0;
	// 	cmd[0].pos = 0;
	// 	delay(2000);
	// 	compteur = 0;
	// 	cmd[0].pos = 1600;
	// 	delay(2000);
	// }
}