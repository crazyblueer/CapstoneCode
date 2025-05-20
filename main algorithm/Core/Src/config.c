/*
 * config.c
 *
 *  Created on: May 4, 2025
 *      Author: macos
 */

#include "config.h"
#include <math.h>
const char* mic_labels[NUM_MICS] = {"Mic1", "Mic2", "Mic3", "Mic4"};
int16_t data_mic1[BUFFER_SIZE];
int16_t data_mic2[BUFFER_SIZE];
int16_t data_mic3[BUFFER_SIZE];
int16_t data_mic4[BUFFER_SIZE];


//x = r * cos (theta); y = r * sin(theta)
const float mic_positions[NUM_MICS][2] = {
		{0.000f, 0.000f}, //Mic 1 (origin)
		{MIC_DISTANCE, 0.000f}, //Mic 2
		{MIC_DISTANCE, MIC_DISTANCE}, //Mic 3
		{0.000f, MIC_DISTANCE}, //Mic 4
};

void config_init(){

}
