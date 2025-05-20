/*
 * vad.h
 *
 *  Created on: May 4, 2025
 *      Author: macos
 */

#ifndef INC_VAD_H_
#define INC_VAD_H_

#include "arm_math.h"
#include "config.h"
#include "preprocessing.h"

//VAD Frame info
typedef struct {
	uint8_t is_voice;
	float32_t energy;
	float32_t zcr;
	float32_t dominant_freq;
}VADFrameInfo;
extern float noise_spectrum[PREPROCESSING_FRAME_SIZE / 2];
void vad_reset_noise_estimation(void);

//functions
void vad_init(void);
uint8_t vad_is_noise_estimation_done(void);
float32_t vad_get_noise_energy(void);
VADFrameInfo vad_process_mic_fft(MicFFT* mic,
                                 arm_rfft_fast_instance_f32* fft_instance,
                                 int offset_samples);
void vad_update_noise_estimation(float32_t* frame);
uint8_t vad_all_mics_agree(VADFrameInfo* mic_infos);

#endif /* INC_VAD_H_ */
