/*
 * preprocessing.c
 *
 *  Created on: May 4, 2025
 *      Author: macos
 */
#include "preprocessing.h"
#include "arm_math.h"
#include <math.h>
#include "config.h"

//static variables
static arm_fir_instance_f32 bandpass_filter;
#define NUM_TAPS  64
static float32_t fir_state[PREPROCESSING_FRAME_SIZE + NUM_TAPS -1];
static float32_t hamming_window[PREPROCESSING_FRAME_SIZE];


 const float32_t fir_coeffs[NUM_TAPS] = { //200-2000hz
		 -0.00099502, -0.00145850, -0.00183171, -0.00199172,
		 -0.00181273, -0.00124511, -0.00041347, 0.00033168,
		 0.00047146, -0.00049482, -0.00278311, -0.00609076,
		 -0.00952972, -0.01182020, -0.01173840, -0.00868879,
		 -0.00317708, 0.00305551, 0.00738885, 0.00715200,
		 0.00070374, -0.01160256, -0.02696683, -0.04047902,
		 -0.04627426, -0.03924314, -0.01679796, 0.01988837,
		 0.06564057, 0.11221916, 0.15024852, 0.17161158, 0.17161158,
		 0.15024852, 0.11221916, 0.06564057, 0.01988837, -0.01679796,
		 -0.03924314, -0.04627426, -0.04047902, -0.02696683, -0.01160256,
		 0.00070374, 0.00715200, 0.00738885, 0.00305551, -0.00317708,
		 -0.00868879, -0.01173840, -0.01182020, -0.00952972, -0.00609076,
		 -0.00278311, -0.00049482, 0.00047146, 0.00033168, -0.00041347,
		 -0.00124511, -0.00181273, -0.00199172, -0.00183171, -0.00145850,
		 -0.00099502
};

 //Init FIR filter and precomputes hamming window
 void preprocessing_init(void){
 	//init fir filter
 	arm_fir_init_f32(&bandpass_filter, NUM_TAPS, (float32_t*)fir_coeffs, fir_state, PREPROCESSING_FRAME_SIZE);

 	//generate hAMMING WINDOW
 	for (int i = 0; i < PREPROCESSING_FRAME_SIZE; i++){
 		hamming_window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i/(PREPROCESSING_FRAME_SIZE -1));

 	}

 }


 //Apply Hamming window
 void apply_hamming_window(float32_t* buffer){
 	for (int i =0; i < PREPROCESSING_FRAME_SIZE; i++){
 		buffer[i] *= hamming_window[i];
 	}
 }

 //apply bandpass FIR filter
 void apply_bandpass_filter(float32_t* input, float32_t* output){
 	arm_fir_f32(&bandpass_filter, input,output, PREPROCESSING_FRAME_SIZE);
 }

 //mic_samples: raw input
 //output_frame: result after bandpass and window
 void prepare_frame(float32_t* mic_samples, float32_t* output_frame){
 	float32_t filtered[PREPROCESSING_FRAME_SIZE];
 	//bandpass filter
 	apply_bandpass_filter(mic_samples, filtered);
 	//apply hamming window
 	for (int i = 0; i < PREPROCESSING_FRAME_SIZE;i++){
 		output_frame[i] = filtered[i] * hamming_window[i];
 	}

 }


 void process_fft_for_mic(MicFFT* mic,
                          arm_rfft_fast_instance_f32* fft_instance,
                          int offset_samples,
                          float* out_freq,
                          float* out_mag) {
     int16_t* samples = mic->raw_buffer;
     int base = offset_samples * 2;

     // Extract left channel from interleaved I2S (L, R, L, R...)
     for (int j = 0; j < FFT_LENGTH; j++) {
         mic->fft_input[j] = (float32_t)samples[base + j * 2];
     }

     // Bandpass filter + Hamming window
     prepare_frame(mic->fft_input, mic->fft_input);

     // FFT and magnitude spectrum
     arm_rfft_fast_f32(fft_instance, mic->fft_input, mic->fft_output, 0);
     arm_cmplx_mag_f32(mic->fft_output, mic->fft_magnitude, FFT_LENGTH / 2);
     mic->fft_magnitude[0] = 0;  // Remove DC

     // Find dominant frequency
     float32_t max_val = 0;
     uint32_t max_bin = 0;
     arm_max_f32(mic->fft_magnitude, FFT_LENGTH / 2, &max_val, &max_bin);

     *out_freq = SAMPLE_RATE * max_bin / FFT_LENGTH;
     *out_mag = max_val;
 }


