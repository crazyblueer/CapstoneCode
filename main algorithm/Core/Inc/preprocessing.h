/*
 * preprocessing.h
 *
 *  Created on: May 4, 2025
 *      Author: macos
 */

#ifndef INC_PREPROCESSING_H_
#define INC_PREPROCESSING_H_

#include "arm_math.h"
#include "config.h"

//Preprocessing buffer size (match FFT_LENGTH)
#define PREPROCESSING_FRAME_SIZE FFT_LENGTH


//Function prototype
void preprocessing_init(void);
void apply_hamming_window(float32_t* buffer);
void apply_bandpass_filter(float32_t* input, float32_t* output);
void prepare_frame(float32_t* mic_samples, float32_t* output_frame);
void process_fft_for_mic(MicFFT* mic,
                         arm_rfft_fast_instance_f32* fft_instance,
                         int offset_samples,
                         float* out_freq,
                         float* out_mag);

#endif /* INC_PREPROCESSING_H_ */
