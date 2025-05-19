#ifndef GCCPHAT_H_
#define GCCPHAT_H_

#include <arm_math.h>
#include "config.h"

// GCC-PHAT Result Structure
typedef struct {
    int mic_i;
    int mic_j;
    float tdoa;
    float corr_peak;
} GCCPhatResult;

// Initialization
void gccphat_init(void);

// Main GCC-PHAT Calculation (for Two Microphones)
GCCPhatResult gccphat_compute(MicFFT* mics, int mic_count, int mic_i, int mic_j, float fs);

// GCC-PHAT Beamforming (Multiple Pairs)
float gccphat_beamforming_4mics(MicFFT* mics, float fs, float mic_distance);

// Cross-Spectrum Calculation
void gccphat_cross_spectrum(float32_t* fft1, float32_t* fft2, float32_t* cross_spectrum);

// Peak Detection with Quadratic Interpolation
int gccphat_find_peak(float32_t* corr, int length);

// TDOA to Angle Calculation (for Square Array)
float gccphat_tdoa_to_angle(float tdoa, float dx, float dy);

#endif /* GCCPHAT_H_ */

