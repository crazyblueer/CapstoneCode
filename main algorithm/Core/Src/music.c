#include "music.h"
#include "alg.h"
#include "c_math.h"
#include <math.h>
#include <string.h>
#include "config.h"
#include "arm_math.h"
#include "vad.h"
#include "myprintf.h"
#include "preprocessing.h"

extern float B[64]; // column-major 8x8 matrix for SVD input
extern float AUX1[8]; // to store singular values
#define NUM_ANGLES 360

void music_init(void) {
    myprintf("Initializing MUSIC algorithm\n");
    for (int i = 0; i < NUM_MICS; i++) {
//    	myprintf("Mic %d: (%f, %f)\n", i, mic_positions[i][0], mic_positions[i][1]);
    }
}
/*
 * extract fft complex alues (magnitude cal)
 * reads the complex fft values (real and imaginary) from each mic
 * */

void compute_real_covariance_matrix(MicFFT *mic_fft, VADFrameInfo *infos, int bin, float real_cov[8][8]) {
    arm_cmplx_float_t X[NUM_MICS];
    float max_mag = 0.0f;

    // Step 1: Extract and Normalize FFT Values
    for (int i = 0; i < NUM_MICS; i++) {
        X[i] = mic_fft[i].fft_complex[bin];
        float mag = sqrtf(X[i].real * X[i].real + X[i].imag * X[i].imag);
        if (mag > max_mag) max_mag = mag;
    }

    // Normalize FFT to prevent overflow
    if (max_mag > 1e-6f) {
        for (int i = 0; i < NUM_MICS; i++) {
            X[i].real /= max_mag;
            X[i].imag /= max_mag;
        }
    } else {
        // Reset to identity if signal is too weak
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                real_cov[i][j] = (i == j && i < NUM_MICS) ? 1.0f : 0.0f;
            }
        }
        return;
    }

    // Step 2: Apply Energy Weighting
    float noise_energy = vad_get_noise_energy();
    for (int i = 0; i < NUM_MICS; i++) {
        float energy_scale = infos[i].energy / (noise_energy + 1e-6f);
        if (energy_scale < 0.1f) {
            X[i].real = 0.0f;
            X[i].imag = 0.0f;
        }
    }

    // Step 3: Compute Covariance Components
    float R_real[NUM_MICS][NUM_MICS] = {0};
    float R_imag[NUM_MICS][NUM_MICS] = {0};
    float cov_norm = 0.0f;

    for (int i = 0; i < NUM_MICS; i++) {
        for (int j = 0; j < NUM_MICS; j++) {
            R_real[i][j] = X[i].real * X[j].real + X[i].imag * X[j].imag;
            R_imag[i][j] = X[i].imag * X[j].real - X[i].real * X[j].imag;
            cov_norm += R_real[i][j] * R_real[i][j] + R_imag[i][j] * R_imag[i][j];
        }
    }

    // Step 4: Normalize Covariance
    cov_norm = sqrtf(cov_norm);
    if (cov_norm < 1e-3f || isnan(cov_norm) || isinf(cov_norm)) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                real_cov[i][j] = (i == j && i < NUM_MICS) ? 1.0f : 0.0f;
            }
        }
        return;
    }

    for (int i = 0; i < NUM_MICS; i++) {
        for (int j = 0; j < NUM_MICS; j++) {
            R_real[i][j] /= cov_norm;
            R_imag[i][j] /= cov_norm;
        }
    }

    // Step 5: Populate 8x8 Matrix
    for (int i = 0; i < NUM_MICS; i++) {
        for (int j = 0; j < NUM_MICS; j++) {
            real_cov[i][j] = R_real[i][j];
            real_cov[i][j + NUM_MICS] = -R_imag[i][j];
            real_cov[i + NUM_MICS][j] = R_imag[i][j];
            real_cov[i + NUM_MICS][j + NUM_MICS] = R_real[i][j];
        }
    }

    // Step 6: Debugging
    myprintf("Adaptive Noise Threshold: %.6f\n", noise_energy);
    int highest_energy_mic = 0;
    for (int i = 0; i < NUM_MICS; i++) {
        if (infos[i].energy > infos[highest_energy_mic].energy) {
            highest_energy_mic = i;
        }
    }
    myprintf("Microphone with Highest Energy: Mic%d, Energy=%.2f\n", highest_energy_mic + 1, infos[highest_energy_mic].energy);
    myprintf("FFT Values (bin %d):\n", bin);
    for (int i = 0; i < NUM_MICS; i++) {
        myprintf("Mic%d: real=%.6f, imag=%.6f\n", i + 1, X[i].real, X[i].imag);
    }
    myprintf("Covariance Matrix (Real Part):\n");
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            myprintf("%.6f ", real_cov[i][j]);
        }
        myprintf("\n");
    }
}



int compute_noise_subspace_from_bin(MicFFT* mics, VADFrameInfo *infos,int bin, float En[8][3]) {
//	myprintf("Computing noise subspace for bin %d\n", bin);
    float real_cov[8][8];

    compute_real_covariance_matrix(mics, infos, bin, real_cov);


    // Populate B (column-major)
    for (int col = 0; col < 8; col++) {
        for (int row = 0; row < 8; row++) {
            B[col * 8 + row] = real_cov[row][col];
        }
    }
    // Log B before SVD
//    myprintf("B before SVD:\n");
//    for (int i = 0; i < 8; i++) {
//        for (int j = 0; j < 8; j++) {
//        	myprintf("%f ", B[j * 8 + i]);
//        }
//        myprintf("\n");
//    }

    int iterations = svd_one_sided_jacobi_C(8, 8, NULL, NULL);
//    myprintf("SVD iterations: %d\n", iterations);

    // Check singular values
    float max_sigma = 0.0f;
    for (int i = 0; i < 8; i++) {
        max_sigma = fmaxf(max_sigma, AUX1[i]);
    }
//    myprintf("Singular values: ");
//    for (int i = 0; i < 8; i++) {
//    	myprintf("%f ", AUX1[i]);
//    }
//    myprintf("\n");
//    if (max_sigma < 1e-3f) {
//    	myprintf("Error: Invalid singular values, skipping bin\n");
//        return -1;
//    }

    // Populate noise subspace (columns 5, 6, 7)
    for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 8; row++) {
            En[row][col] = B[(5 + col) * 8 + row];
        }
    }
    // Check En norm
    float en_norm = 0.0f;
    for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 8; row++) {
            en_norm += En[row][col] * En[row][col];
        }
    }
    en_norm = sqrtf(en_norm);
//    myprintf("En[0][0]=%f, En[0][1]=%f, En[0][2]=%f, En norm=%f\n", En[0][0], En[0][1], En[0][2], en_norm);
//    if (en_norm < 1e-3f || isnan(en_norm) || isinf(en_norm)) {
//    	myprintf("Error: Invalid noise subspace, skipping bin\n");
//        return -1;
//    }

    return iterations;
}

float compute_music_pseudospectrum_at_angle(float angle_deg, float bin_freq, float En[8][3]) {
    float angle_rad = angle_deg * M_PI / 180.0f;
    float steering[8];

    for (int i = 0; i < NUM_MICS; i++) {
        float dx = mic_positions[i][0];
        float dy = mic_positions[i][1];
        float delay = (dx * cosf(angle_rad) + dy * sinf(angle_rad)) / SPEED_OF_SOUND;
        float phase = 2.0f * M_PI * bin_freq * delay;
        steering[i] = cosf(phase);
        steering[i + NUM_MICS] = sinf(phase);
    }

    float proj[3] = {0};
    for (int k = 0; k < 3; k++) {
        for (int i = 0; i < 8; i++) {
            proj[k] += steering[i] * En[i][k];
        }
    }

    float norm2 = 0.0f;
    for (int k = 0; k < 3; k++) {
        norm2 += proj[k] * proj[k];
    }

    float pseudospectrum = 1.0f / (norm2 + 1e-6f);
//    if ((int)angle_deg % 30 == 0) {
//    	myprintf("Angle %d: norm2=%f, pseudospectrum=%f\n", (int)angle_deg, norm2, pseudospectrum);
//    }

    return pseudospectrum;
}

int music_estimate_angle(MicFFT* mics, VADFrameInfo* infos, int* out_angle, float* out_energy, float* singular_values_out) {
    float dominant_freq = infos[0].dominant_freq;
    int dominant_bin = (int)(dominant_freq * FFT_LENGTH / SAMPLE_RATE);
    float freq_res = SAMPLE_RATE / FFT_LENGTH;
    int bin_low = (int)(100.0f / freq_res);
    int bin_high = (int)(800.0f / freq_res);

//    myprintf("Dominant freq: %f Hz, bin: %d\n", dominant_freq, dominant_bin);
    if (dominant_bin < bin_low || dominant_bin > bin_high) {
//    	myprintf("Invalid dominant bin: %d\n", dominant_bin);
        *out_angle = -1;
        *out_energy = 0.0f;
        return 0;
    }

    float pseudospectrum[NUM_ANGLES] = {0};
    int total_iterations = 0;
    int bin_count = 0;

    for (int delta = -3; delta <= 3; delta++) {
        int bin_k = dominant_bin + delta;
        if (bin_k < bin_low || bin_k > bin_high) continue;

//        myprintf("Processing bin %d\n", bin_k);
        float En[8][3];
        int iterations = compute_noise_subspace_from_bin(mics,infos, bin_k, En);
        if (iterations < 0) {
//        	myprintf("Skipping bin %d due to invalid SVD\n", bin_k);
            continue;
        }
        total_iterations += iterations;

        float bin_freq = bin_k * freq_res;
        for (int theta = 0; theta < NUM_ANGLES; theta++) {
            pseudospectrum[theta] += compute_music_pseudospectrum_at_angle((float)theta, bin_freq, En);
        }
        bin_count++;
    }

    if (bin_count == 0) {
//    	myprintf("No valid bins processed\n");
        *out_angle = -1;
        *out_energy = 0.0f;
        return 0;
    }

    float max_p = 0.0f;
    int best_angle = 0;
    for (int theta = 0; theta < NUM_ANGLES; theta++) {
        if (pseudospectrum[theta] > max_p) {
            max_p = pseudospectrum[theta];
            best_angle = theta;
        }
    }

    *out_angle = best_angle;
    *out_energy = max_p;
    myprintf("DoA: %d°, Pseudospectrum: %f, Bins processed: %d\n", best_angle, max_p, bin_count);

    for (int i = 0; i < 8; i++) {
        singular_values_out[i] = AUX1[i];
    }

    return total_iterations;
}
