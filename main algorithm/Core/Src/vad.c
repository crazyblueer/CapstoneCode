/*
 * vad.c
 *
 *  Created on: May 4, 2025
 *      Author: macos
 */

#include "vad.h"
#include "preprocessing.h"
#include "arm_math.h"
#include "stdio.h"
#include "config.h"


static float32_t noise_energy_accum = 0.0f;
float noise_spectrum[PREPROCESSING_FRAME_SIZE / 2] = {0};

static uint32_t noise_frame_count = 0;
static uint8_t noise_estimation_done = 0;
static float32_t noise_energy_average = 0.0f;

extern arm_rfft_fast_instance_f32 fft_audio_instance;

void vad_init(void){
	noise_energy_accum = 0.0f;
	noise_frame_count  = 0;
	noise_estimation_done = 0;
	noise_energy_average = 0.0f;
}
void vad_reset_noise_estimation(void){
    noise_energy_accum = 0.0f;
    noise_frame_count  = 0;
    noise_estimation_done = 0;
    noise_energy_average = 0.0f;
}


uint8_t vad_is_noise_estimation_done(void) {
    return noise_estimation_done;
}

float32_t vad_get_noise_energy(void) {
    return noise_energy_average;
}

void vad_update_noise_estimation(float32_t* frame){
	float32_t fft_output[FFT_LENGTH];
	float32_t magnitude[FFT_LENGTH/2];

	//fft
	arm_rfft_fast_f32(&fft_audio_instance, frame,fft_output,0);
	arm_cmplx_mag_f32(fft_output, magnitude, FFT_LENGTH/2);

	//compute energy
	float32_t energy = 0.0f;
	for (int i = 0; i < FFT_LENGTH/2; i++){
		energy += magnitude[i] *magnitude[i];
	}

	noise_energy_accum += energy;
	noise_frame_count++;

	if (noise_frame_count >= (SAMPLE_RATE/(FFT_LENGTH/2)) * NOISE_CALIBERATION_TIME_SEC){
		noise_energy_average = noise_energy_accum / noise_frame_count;
		noise_estimation_done = 1;
	}
}

VADFrameInfo vad_process_mic_fft(MicFFT* mic,arm_rfft_fast_instance_f32* fft_instance, int offset_samples) {
    VADFrameInfo info = {0};

    int16_t* samples = mic->raw_buffer;
    int base = offset_samples * 2;

    // Convert int16 → float
    for (int j = 0; j < PREPROCESSING_FRAME_SIZE; j++) {
        mic->fft_input[j] =(float32_t)samples[base + j * 2];
    }

    // Preprocessing (bandpass + window)
    prepare_frame(mic->fft_input, mic->fft_input);
//    float gain = 2.0f;
//    for (int i = 0; i < PREPROCESSING_FRAME_SIZE; i++) {
//        mic->fft_input[i] *= gain;
//    }

    // FFT + magnitude
    arm_rfft_fast_f32(fft_instance, mic->fft_input, mic->fft_output, 0);
    for (int i = 0; i < FFT_LENGTH / 2; i++) {
        mic->fft_complex[i].real = mic->fft_output[2 * i];
        mic->fft_complex[i].imag = mic->fft_output[2 * i + 1];
    }


    arm_cmplx_mag_f32(mic->fft_output, mic->fft_magnitude, PREPROCESSING_FRAME_SIZE / 2);
    mic->fft_magnitude[0] = 0;

    // Dominant freq
    float32_t max_val = 0;
    uint32_t max_bin = 0;
    arm_max_f32(mic->fft_magnitude, PREPROCESSING_FRAME_SIZE / 2, &max_val, &max_bin);

    info.dominant_freq = SAMPLE_RATE * max_bin / PREPROCESSING_FRAME_SIZE;
    *(&info.energy) = 0;

    // Energy
    for (int i = 0; i < PREPROCESSING_FRAME_SIZE / 2; i++) {
        info.energy += mic->fft_magnitude[i] * mic->fft_magnitude[i];
    }

    // ZCR
    int zero_crossings = 0;
    for (int i = 1; i < PREPROCESSING_FRAME_SIZE; i++) {
        if ((mic->fft_input[i - 1] >= 0 && mic->fft_input[i] < 0) ||
            (mic->fft_input[i - 1] < 0 && mic->fft_input[i] >= 0)) {
            zero_crossings++;
        }
    }
    info.zcr = (float)zero_crossings / PREPROCESSING_FRAME_SIZE;

    // VAD Decision
    if (vad_is_noise_estimation_done()) {
        if (info.energy >  vad_get_noise_energy()* VAD_ENERGY_MULTIPLIER &&
            info.zcr > VAD_ZCR_LOW && info.zcr < VAD_ZCR_HIGH &&
            info.dominant_freq > VAD_FREQ_MIN && info.dominant_freq < VAD_FREQ_MAX) {
            info.is_voice = 1;
        }
    }

    return info;
}
uint8_t vad_all_mics_agree(VADFrameInfo* mic_infos){
	int voice_count = 0;
	    float freq_sum = 0.0f;

	    // Count how many mics say "voice"
	    for (int i = 0; i < NUM_MICS; i++) {
	        if (mic_infos[i].is_voice) {
	            voice_count++;
	            freq_sum += mic_infos[i].dominant_freq;
	        }
	    }

	    // If fewer than N mics detect voice, reject
	    if (voice_count < 2) return 0;

	    // Optionally check frequency consistency
	    float avg_freq = freq_sum / voice_count;
	    int consistent = 0;

	    for (int i = 0; i < NUM_MICS; i++) {
	        if (mic_infos[i].is_voice &&
	            fabsf(mic_infos[i].dominant_freq - avg_freq) < VAD_FREQ_TOLERANCE) {
	            consistent++;
	        }
	    }
	    return (consistent ==4);
}

