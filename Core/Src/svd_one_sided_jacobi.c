//#include <math.h>
//#include <stdlib.h>
//#include <string.h>
//#include <stdbool.h>
//#include <float.h>
//#include "c_math.h"
//
//
//float B[64];     // column-major matrix input for SVD
//float AUX1[8];
//
//static const float eps = 1e-4;
//static const float thresh = 1e-6;
//
//
//int svd_one_sided_jacobi_C(int M, int N, float *U, float *V) {
//	// input: B (will be changed), column-major order
//	// uses AUX1
//	if (V) eye(V, N);  // row-major
//	bool exit_flag = false;
//	int iterations = 0;
//	while (!exit_flag) {
//		++iterations;
//		exit_flag = true;
//		for (int j = N - 1; j >= 1; --j)
//			for (int i = j - 1; i >= 0; --i) {
//				float alpha = 0, beta = 0, gamm = 0;
//#if !ONE_SIDED_JACOBI_ROW_MAJOR
//				float *pi = B + M * i, *pj = B + M * j;
//				for (int k = 0; k < M; ++k) {
//					alpha += *pi * *pi;
//					beta += *pj * *pj;
//					gamm += *pi++ * *pj++;
//				}
//#else
//				float *pi = B + i, *pj = B + j;
//				for (int k = 0; k < M; ++k) {
//					alpha += *pi * *pi;
//					beta += *pj * *pj;
//					gamm += *pi * *pj;
//					pi += N;
//					pj += N;
//				}
//#endif
//				if (iterations < 50) {
//					const float limit = fabsf(gamm) / sqrtf(alpha * beta);
//					if (limit > eps) exit_flag = false;
//				}
//				float c, s;
//				if (fabsf(gamm) < thresh) {
//					c = 1;
//					s = 0;
//				}
//				else {
//					// some computations (square + square root) need to be done in double precision (64 bits)
//					// or accuracy does not reach values comparable to other algorithms
//					const float tao = (beta - alpha) / (2 * gamm);
//					// t can be computed at 32-bit precision, tests show little loss of accuracy
//					//  but good speed improvement
//					const float t = sign(tao) / (fabsf(tao) + sqrtf(1 + tao * tao));  // t computed at 32-bit precision
//					//const double tao64 = tao;
//					//const float t = sign(tao) / (fabsf(tao) + (float)sqrt(1 + tao64 * tao64));  // t computed at 64-bit precision
//					// tests show that c must instead be computed at 64-bit precision
//					//const float c = 1 / sqrtf(1 + t * t);  // c computed at 32-bit precision
//					//c = 1 / (float)sqrt(1 + (double)t * (double)t);  // c computed at 64-bit precision
//					c = expf(-0.5f * log1pf(t * t));  // new trick by Giorgio! Better than passing to 64 bits.
//					s = c * t;
//				}
//#if !ONE_SIDED_JACOBI_ROW_MAJOR
//				// manual Givens rotation of B because it's column-major
//				pi = B + M * i; pj = B + M * j;
//				for (int k = 0; k < M; ++k) {
//					const float t = *pi;
//					*pi++ = c * t - s * *pj;
//					*pj = s * t + c * *pj;
//					++pj;
//				}
//#else
//				Mat mB = {B, M, N, N};
//				givens_rotation_right(c, s, i, j, false, &mB);
//#endif
//				if (V) {
//					Mat mV = {V, N, N, N};
//					givens_rotation_right(c, s, i, j, false, &mV);
//				}
//			}
//	}
//	for (int j = 0; j < N; ++j) {
//#if !ONE_SIDED_JACOBI_ROW_MAJOR
//		float t = 0, *pj = B + M * j;
//		for (int k = 0; k < M; ++k, ++pj) t += *pj * *pj;
//#else
//		float t = 0, *pj = B + j;
//		for (int k = 0; k < M; ++k, pj += N) t += *pj * *pj;
//#endif
//		AUX1[j] = sqrtf(t);
//	}
//	if (U) {
//		// copy B to U row-major, dividing columns by their singular value
//		for (int j = 0; j < N; ++j) {
//#if !ONE_SIDED_JACOBI_ROW_MAJOR
//			float *pj = B + M * j, *pu = U + j, val = AUX1[j];
//			for (int i = 0; i < M; ++i, pu += M) *pu = *pj++ / val;
//#else
//			float *pj = B + j, *pu = U + j, val = AUX1[j];
//			for (int i = 0; i < M; ++i, pu += M, pj += N) *pu = *pj / val;
//#endif
//		}
//	}
//	return iterations;
//}

#include "music.h"
#include "myprintf.h"
#include <math.h>
#include <string.h>

// Define global variables
float B[64]; // Input/output matrix (column-major 8x8)
float AUX1[8]; // Singular values

// Compute dot product of two columns
static float dot_product(int col1, int col2, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += B[col1 * n + i] * B[col2 * n + i];
    }
    return sum;
}

// Apply Givens rotation to columns col1 and col2
static void apply_rotation(int col1, int col2, int n, float c, float s) {
    for (int i = 0; i < n; i++) {
        float temp1 = B[col1 * n + i];
        float temp2 = B[col2 * n + i];
        B[col1 * n + i] = c * temp1 + s * temp2;
        B[col2 * n + i] = -s * temp1 + c * temp2;
    }
}

int svd_one_sided_jacobi_C(int m, int n, float* U, float* V) {
    if (m != 8 || n != 8 || U != NULL || V != NULL) {
        myprintf("Error: Invalid SVD parameters (m=%d, n=%d)\n", m, n);
        return -1;
    }

    int iterations = 0;
    const int max_iterations = 100;
    const float eps = 1e-6f;
    float norm;

    // Log initial matrix
//    myprintf("Initial B matrix:\n");
//    for (int i = 0; i < 8; i++) {
//        for (int j = 0; j < 8; j++) {
//        	myprintf("%f ", B[j * 8 + i]);
//        }
//        myprintf("\n");
//    }

    // One-sided Jacobi: Orthogonalize columns
    do {
        norm = 0.0f;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                // Compute dot product of columns i and j
                float a = dot_product(i, i, n); // ||B_i||^2
                float b = dot_product(j, j, n); // ||B_j||^2
                float c = dot_product(i, j, n); // B_i^T B_j

                norm += c * c;
                if (fabsf(c) < eps) continue;

                // Compute rotation parameters
                float zeta = (b - a) / (2.0f * c);
                float t = (zeta >= 0.0f ? 1.0f : -1.0f) / (fabsf(zeta) + sqrtf(1.0f + zeta * zeta));
                float cs = 1.0f / sqrtf(1.0f + t * t);
                float sn = cs * t;

                // Apply Givens rotation
                apply_rotation(i, j, n, cs, sn);
                iterations++;
//                myprintf("Rotation (%d, %d): c=%f, s=%f, off-diagonal=%f\n", i, j, cs, sn, c);
            }
        }
        norm = sqrtf(norm);
    } while (norm > eps && iterations < max_iterations);

    // Compute singular values and normalize columns
    for (int j = 0; j < n; j++) {
        float sigma = 0.0f;
        for (int i = 0; i < n; i++) {
            sigma += B[j * n + i] * B[j * n + i];
        }
        sigma = sqrtf(sigma);
        AUX1[j] = sigma;
        if (sigma > eps) {
            for (int i = 0; i < n; i++) {
                B[j * n + i] /= sigma;
            }
        } else {
            AUX1[j] = 0.0f;
            for (int i = 0; i < n; i++) {
                B[j * n + i] = (i == j) ? 1.0f : 0.0f; // Identity for zero columns
            }
        }
    }

    // Sort singular values in descending order
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (AUX1[i] < AUX1[j]) {
                float temp = AUX1[i];
                AUX1[i] = AUX1[j];
                AUX1[j] = temp;
                // Swap columns in B
                for (int k = 0; k < n; k++) {
                    float temp_b = B[i * n + k];
                    B[i * n + k] = B[j * n + k];
                    B[j * n + k] = temp_b;
                }
            }
        }
    }

//    myprintf("Final singular values: ");
//    for (int i = 0; i < n; i++) {
//    	myprintf("%f ", AUX1[i]);
//    }
//    myprintf("\nFinal B matrix (eigenvectors):\n");
//    for (int i = 0; i < 8; i++) {
//        for (int j = 0; j < 8; j++) {
//        	myprintf("%f ", B[j * 8 + i]);
//        }
//        myprintf("\n");
//    }

    return iterations;
}
