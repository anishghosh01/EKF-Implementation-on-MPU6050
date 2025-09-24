#ifndef EKF_H
#define EKF_H

// This is the core header for the CMSIS-DSP library
#include "arm_math.h"

// **FIX**: Define the physical constant for gravity here so it's available to EKF.c
#ifndef G_MPS2
#define G_MPS2 9.80665f // Gravity in m/s^2
#endif

typedef struct {
    // State estimates (Roll and Pitch angles) remain as simple floats
    float phi_r;
    float theta_r;

    // --- CMSIS-DSP Matrix Structures ---
    // We now use special structures from the library to handle matrices.
    // Each matrix needs an instance structure and a float array for its data.

    // State vector [phi, theta]'
    arm_matrix_instance_f32 x;
    float32_t x_data[2];

    // State covariance matrix P (2x2)
    arm_matrix_instance_f32 P;
    float32_t P_data[4];

    // Process noise covariance matrix Q (2x2)
    arm_matrix_instance_f32 Q;
    float32_t Q_data[4];

    // Measurement noise covariance matrix R (3x3)
    arm_matrix_instance_f32 R;
    float32_t R_data[9];

} EKF;

/**
 * @brief Initializes the Attitude EKF with CMSIS-DSP structures.
 */
void EKF_Init(EKF *ekf, float P_initial, float Q_angle, float R_accel, float initial_phi, float initial_theta);

/**
 * @brief Prediction step of the EKF using CMSIS-DSP.
 */
void EKF_Predict(EKF *ekf, float p_rps, float q_rps, float r_rps, float sampleTime_s);

/**
 * @brief Update step of the EKF using CMSIS-DSP.
 */
void EKF_Update(EKF *ekf, float ax_mps2, float ay_mps2, float az_mps2);

#endif

