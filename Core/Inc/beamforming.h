/*
 * beamforming.h
 *
 *  Created on: May 4, 2025
 *      Author: macos
 */

#ifndef INC_BEAMFORMING_H_
#define INC_BEAMFORMING_H_

#include "arm_math.h"
#include "config.h"
#include "preprocessing.h"
#include "vad.h"

//Functions
void beamforming_init(void);

//void delay_and_sum_beamforming(MicFFT* mics,  VADFrameInfo* infos, int* out_angle, float* out_energy);
void delay_and_sum_beamforming(MicFFT* mics, VADFrameInfo* infos, int* out_angle, float* out_energy);
void update_angle_buffer(float new_angle);
float smooth_angle(void);
float update_smoothed_angle_exp(float new_angle);

#endif /* INC_BEAMFORMING_H_ */
