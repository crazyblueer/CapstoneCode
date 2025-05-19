/*
 * beamforming.c
 *
 *  Created on: May 4, 2025
 *      Author: macos
 */


#include "beamforming.h"
#include "arm_math.h"
#include "math.h"
#include "config.h"
#include "vad.h"
#include "myprintf.h"
static float smoothed_angle_prev = 0.0f;
#define SMOOTHING_ALPHA 0.5f


void beamforming_init(){

}

void delay_and_sum_beamforming(MicFFT* mics, VADFrameInfo* infos, int* out_angle, float* out_energy){
    const int num_angles = 360;
    float best_energy = 0.0f;
    int best_angle = 0;

    // Focus on frequencies between 100–800 Hz
//    const int bin_low = (int)(500.0f / ((float)SAMPLE_RATE / FFT_LENGTH));
//    const int bin_high = (int)(2000.0f / ((float)SAMPLE_RATE / FFT_LENGTH)); //previous

    const int bin_low = (int)(200.0f / ((float)SAMPLE_RATE / FFT_LENGTH));
       const int bin_high = (int)(3000.0f / ((float)SAMPLE_RATE / FFT_LENGTH));

    for (int angle = 0; angle < num_angles; angle++) {
        float theta = angle * M_PI / 180.0f;
        float dir[2] = {-cosf(theta), sinf(theta)};
//        float dir[2] = {cosf(theta), sinf(theta)};
        float total_energy = 0.0f;

        for (int bin = bin_low; bin <= bin_high; bin++) {
            float freq = ((float)SAMPLE_RATE / FFT_LENGTH) * bin;
            float omega = 2.0f * M_PI * freq;

            float real_sum = 0.0f;
            float imag_sum = 0.0f;

            for (int mic = 0; mic < NUM_MICS; mic++) {
                if (!infos[mic].is_voice) continue;

                float dx = mic_positions[mic][0] - mic_positions[0][0];
                float dy = mic_positions[mic][1] - mic_positions[0][1];
//                float delay = -(dx * dir[0] + dy * dir[1]) / SPEED_OF_SOUND;
                float delay = (dx * dir[0] + dy * dir[1]) / SPEED_OF_SOUND;
//                if (angle == 180) {
//                                myprintf("Angle: %d, Mic %d, dx=%.3f, dy=%.3f, Delay=%.6f s\n",
//                                         angle, mic, dx, dy, delay);
//                            }


//                float phase_shift = -omega * delay;
                float phase_shift = -omega * delay;

                float real = mics[mic].fft_output[2 * bin];
                float imag = mics[mic].fft_output[2 * bin + 1];

                float shifted_real = real * cosf(phase_shift) - imag * sinf(phase_shift);
                float shifted_imag = real * sinf(phase_shift) + imag * cosf(phase_shift);

                real_sum += shifted_real;
                imag_sum += shifted_imag;
            }

            total_energy += real_sum * real_sum + imag_sum * imag_sum;
        }

        if (total_energy > best_energy) {
            best_energy = total_energy;
            best_angle = angle;
        }
    }

    *out_angle = best_angle;
    *out_energy = best_energy;
}



