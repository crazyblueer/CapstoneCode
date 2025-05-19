/*
 * gccphat.c
 *
 *  Created on: May 5, 2025
 *      Author: macos
 */

#include "gccphat.h"
#include <math.h>
#include "myprintf.h"
#include "vad.h"
#include "config.h"
#include <string.h>

static arm_rfft_fast_instance_f32 fft_inst;
static float hanning_window[FFT_LENGTH];
static int hanning_ready = 0;

// Initialization (with Hanning Window Precomputation)
void gccphat_init(void) {
    arm_rfft_fast_init_f32(&fft_inst, FFT_LENGTH);

    // Precompute Hanning window only once
    if (!hanning_ready) {
        for (int i = 0; i < FFT_LENGTH; i++) {
            hanning_window[i] = 0.5f * (1.0f - cosf(2 * M_PI * i / (FFT_LENGTH - 1)));
        }
        hanning_ready = 1;
        myprintf("Hanning window precomputed.\n");
    }
}

// Cross-Spectrum Calculation (GCC-PHAT)
void gccphat_cross_spectrum(float32_t* fft1, float32_t* fft2, float32_t* cross_spectrum) {
    for (int i = 0; i < FFT_LENGTH; i += 2) {
        float a = fft1[i];     // real
        float b = fft1[i + 1]; // imag
        float c = fft2[i];     // real
        float d = -fft2[i + 1];// -imag for conjugate

        float re = a * c - b * d;
        float im = a * d + b * c;
        float mag = sqrtf(re * re + im * im) + 1e-6f; // Magnitude with epsilon to avoid division by zero

        cross_spectrum[i]     = re / mag;
        cross_spectrum[i + 1] = im / mag;
    }
}

// Peak Detection with Quadratic Interpolation
int gccphat_find_peak(float32_t* corr, int length) {
    float max_val = -1e9f;
    int max_idx = 0;

    for (int i = 0; i < length; i++) {
        if (corr[i] > max_val) {
            max_val = corr[i];
            max_idx = i;
        }
    }

    // Quadratic interpolation
    int left = (max_idx - 1 + length) % length;
    int right = (max_idx + 1) % length;
    float delta = 0.0f;
    float denom = 2 * (2 * max_val - corr[right] - corr[left]);

    if (denom != 0) {
        delta = (corr[right] - corr[left]) / denom;
    }

    return max_idx + delta;
}

// Main GCC-PHAT with Hanning Window Applied
GCCPhatResult gccphat_compute(MicFFT* mics, int mic_count, int mic_i, int mic_j, float fs) {
    float32_t sig1[FFT_LENGTH];
    float32_t sig2[FFT_LENGTH];

    // Apply Hanning window to both signals
    for (int i = 0; i < FFT_LENGTH; i++) {
        sig1[i] = (float32_t)mics[mic_i].raw_buffer[i * 2] * hanning_window[i];
        sig2[i] = (float32_t)mics[mic_j].raw_buffer[i * 2] * hanning_window[i];
    }

    // FFT
    float32_t fft1[FFT_LENGTH], fft2[FFT_LENGTH], cross_spectrum[FFT_LENGTH];
    arm_rfft_fast_f32(&fft_inst, sig1, fft1, 0);
    arm_rfft_fast_f32(&fft_inst, sig2, fft2, 0);
    gccphat_cross_spectrum(fft1, fft2, cross_spectrum);

    // IFFT for cross-correlation
    float32_t corr[FFT_LENGTH];
    arm_rfft_fast_f32(&fft_inst, cross_spectrum, corr, 1);

    // Find peak with interpolation
    int peak_idx = gccphat_find_peak(corr, FFT_LENGTH);

    // Calculate TDOA (time delay)
    int shift = peak_idx;
    if (shift > FFT_LENGTH / 2) shift -= FFT_LENGTH;
    float tdoa = (float)shift / fs;

    // Result structure
    GCCPhatResult result = {
        .mic_i = mic_i,
        .mic_j = mic_j,
        .tdoa = tdoa,
        .corr_peak = corr[peak_idx]
    };
    return result;
}

// TDOA to Angle Calculation (for Square Array)
// GCC-PHAT TDOA to Angle Calculation (Safe)
// Enhanced GCC-PHAT TDOA to Angle Calculation (Full 360° Range)
// Corrected GCC-PHAT TDOA to Angle Calculation (Full 360°)
// Corrected GCC-PHAT TDOA to Angle Calculation (Full 360°)
float gccphat_tdoa_to_angle(float tdoa, float dx, float dy) {
    // Speed of sound in m/s (adjust for your environment)
    const float speed_of_sound = 343.0f;

    // Convert TDOA to distance difference
    float delta_distance = tdoa * speed_of_sound;

    // Base angle based on the microphone pair geometry
    float base_angle = atan2f(dy, dx) * (180.0f / M_PI);

    // Adjust angle using delta_distance sign
    float adjusted_angle = base_angle + atan2f(delta_distance, hypotf(dx, dy)) * (180.0f / M_PI);

    // Normalize angle to [0, 360)
    if (adjusted_angle < 0.0f) adjusted_angle += 360.0f;
    if (adjusted_angle >= 360.0f) adjusted_angle -= 360.0f;

    return adjusted_angle;
}

// Enhanced GCC-PHAT Beamforming (Full 360°)
float gccphat_beamforming_4mics(MicFFT* mics, float fs, float mic_distance) {
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    int valid_pairs = 0;

    // Calculate GCC-PHAT for each microphone pair
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            GCCPhatResult result = gccphat_compute(mics, 4, i, j, fs);

            // Calculate dx, dy based on your microphone layout
            float dx = mic_positions[j][0] - mic_positions[i][0];
            float dy = mic_positions[j][1] - mic_positions[i][1];

            // Use the corrected TDOA to angle calculation
            float angle = gccphat_tdoa_to_angle(result.tdoa, dx, dy);
            myprintf("GCC-PHAT Pair (%d-%d): TDOA = %.6f s, Angle = %.2f°\n", i, j, result.tdoa, angle);

            if (!isnan(angle) && angle >= 0.0f && angle <= 360.0f) {
                sum_sin += sinf(angle * M_PI / 180.0f);
                sum_cos += cosf(angle * M_PI / 180.0f);
                valid_pairs++;
            }
        }
    }

    // Average the angles of all pairs (circular averaging)
    if (valid_pairs > 0) {
        float avg_angle = atan2f(sum_sin, sum_cos) * (180.0f / M_PI);
        if (avg_angle < 0.0f) avg_angle += 360.0f; // Ensure [0, 360)
        return avg_angle;
    } else {
        return 0.0f; // Default to 0 if no valid pairs
    }
}
