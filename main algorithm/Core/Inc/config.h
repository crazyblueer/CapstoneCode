/*
 * config.h
 *
 *  Created on: May 4, 2025
 *      Author: macos
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "arm_math.h"

//system parameters
#define NUM_MICS 4
#define FFT_LENGTH 1024
#define SAMPLE_RATE 32000
#define MIC_DISTANCE 0.06f
#define SPEED_OF_SOUND 343.0f
#define BUFFER_SIZE 2048


extern const char* mic_labels[NUM_MICS];

extern int16_t data_mic1[BUFFER_SIZE];
extern int16_t data_mic2[BUFFER_SIZE];
extern int16_t data_mic3[BUFFER_SIZE];
extern int16_t data_mic4[BUFFER_SIZE];


//Preprocessing params
//#define BANDPASS_LOW_CUT 150.0f
//#define BANDPASS_HIGH_CUT 8000.0f
#define VAD_ZCR_LOW 0.25f
#define VAD_ZCR_HIGH 0.55f
#define VAD_FREQ_MIN 200.0f
#define VAD_FREQ_MAX 3000.0f
#define VAD_FREQ_TOLERANCE 100.0f
#define NOISE_CALIBERATION_TIME_SEC 5
#define VAD_ENERGY_MULTIPLIER 0.06f
//#define VAD_ENERGY_MULTIPLIER 0.05f


typedef struct {
    float real;
    float imag;
} arm_cmplx_float_t;

typedef struct {
	int16_t* raw_buffer;
	float32_t fft_input[FFT_LENGTH];
	float32_t fft_output[FFT_LENGTH];
	float32_t fft_magnitude[FFT_LENGTH/2];
	arm_cmplx_float_t fft_complex[FFT_LENGTH / 2];
}MicFFT;


//Algorithm Selection
typedef enum {
	ALGO_BEAMFORMING =0,
	ALGO_GCCPHAT,
	ALGO_MUSIC
}SSLAlgorithm;

//Mic Array Layout
extern const float mic_positions[NUM_MICS][2];


//Functions
void config_init();


#endif /* INC_CONFIG_H_ */
