/*
 * music.h
 *
 *  Created on: May 5, 2025
 *      Author: macos
 */

#ifndef INC_MUSIC_H_
#define INC_MUSIC_H_

#include "arm_math.h"
#include "config.h"
#include "vad.h"
#include "preprocessing.h"
#include "alg.h"
#include "c_math.h"



void music_init(void);
// music.h (update this)
void compute_real_covariance_matrix(MicFFT *mic_fft, VADFrameInfo *infos, int bin, float real_cov[8][8]);
// In music.h
int compute_noise_subspace_from_bin(MicFFT* mics, VADFrameInfo *infos, int bin, float En[8][3]);
int music_estimate_angle(MicFFT* mics, VADFrameInfo* infos, int* out_angle, float* out_energy, float* singular_values_out);

#endif /* INC_MUSIC_H_ */
