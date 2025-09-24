#include "EKF.h"
#include <string.h> // Required for memcpy

void EKF_Init(EKF *ekf, float P_initial, float Q_angle, float R_accel, float initial_phi, float initial_theta){
    // Initialize the matrix structures
    arm_mat_init_f32(&ekf->x, 2, 1, ekf->x_data);
    arm_mat_init_f32(&ekf->P, 2, 2, ekf->P_data);
    arm_mat_init_f32(&ekf->Q, 2, 2, ekf->Q_data);
    arm_mat_init_f32(&ekf->R, 3, 3, ekf->R_data);

    // Reset state estimates to the provided initial values
    ekf->phi_r = initial_phi;
    ekf->theta_r = initial_theta;
    ekf->x_data[0] = initial_phi;
    ekf->x_data[1] = initial_theta;

    // Reset state estimates
    ekf->phi_r = 0.0f;
    ekf->theta_r = 0.0f;
    ekf->x_data[0] = 0.0f;
    ekf->x_data[1] = 0.0f;

    // Initialise state covariance matrix P as an identity matrix scaled by P_initial
    ekf->P_data[0] = P_initial; ekf->P_data[1] = 0.0f;
    ekf->P_data[2] = 0.0f;      ekf->P_data[3] = P_initial;

    // Set process noise matrix Q
    ekf->Q_data[0] = Q_angle; ekf->Q_data[1] = 0.0f;
    ekf->Q_data[2] = 0.0f;    ekf->Q_data[3] = Q_angle;

    // Set measurement noise matrix R
    ekf->R_data[0] = R_accel; ekf->R_data[1] = 0.0f;    ekf->R_data[2] = 0.0f;
    ekf->R_data[3] = 0.0f;    ekf->R_data[4] = R_accel; ekf->R_data[5] = 0.0f;
    ekf->R_data[6] = 0.0f;    ekf->R_data[7] = 0.0f;    ekf->R_data[8] = R_accel;
}

void EKF_Predict(EKF *ekf, float p_rps, float q_rps, float r_rps, float sampleTime_s) {
    // --- Predict next state x_k = f(x_{k-1}, u_k) ---
    float sp = sinf(ekf->phi_r);
    float cp = cosf(ekf->phi_r);
    float tt = tanf(ekf->theta_r);

    float dphidt = p_rps + tt * (q_rps * sp + r_rps * cp);
    float dthetadt = q_rps * cp - r_rps * sp;

    // Update state angles directly for use in Jacobians
    ekf->phi_r   += sampleTime_s * dphidt;
    ekf->theta_r += sampleTime_s * dthetadt;

    // Add gimbal lock protection by clamping pitch angle to avoid tan() infinity.
    // Clamp to +/- 89.9 degrees to be safe
    if (ekf->theta_r > (PI / 2.0f) - 0.001f) {
        ekf->theta_r = (PI / 2.0f) - 0.001f;
    } else if (ekf->theta_r < -(PI / 2.0f) + 0.001f) {
        ekf->theta_r = -(PI / 2.0f) + 0.001f;
    }

    // Update the state vector matrix
    ekf->x_data[0] = ekf->phi_r;
    ekf->x_data[1] = ekf->theta_r;

    // --- Predict next covariance P_k = A*P_{k-1}*A' + Q ---
    float ct = cosf(ekf->theta_r);
    float ctInv = 1.0f / ct; // Safe now due to clamping

    // Jacobian of state transition function A (linearized model for covariance update)
    float32_t A_data[4] = {
        1 + sampleTime_s * tt * (q_rps * cp - r_rps * sp), sampleTime_s * (q_rps * sp + r_rps * cp) * ctInv * ctInv,
        -sampleTime_s * (q_rps * sp + r_rps * cp), 1.0f
    };
    arm_matrix_instance_f32 A;
    arm_mat_init_f32(&A, 2, 2, A_data);

    // Transpose of A
    float32_t AT_data[4];
    arm_matrix_instance_f32 AT;
    arm_mat_init_f32(&AT, 2, 2, AT_data);
    arm_mat_trans_f32(&A, &AT);

    // Temporary matrices for calculation
    float32_t temp_P_data[4];
    arm_matrix_instance_f32 temp_P;
    arm_mat_init_f32(&temp_P, 2, 2, temp_P_data);

    // P_k = A * P_{k-1}
    arm_mat_mult_f32(&A, &ekf->P, &temp_P);
    // P_k = (A * P_{k-1}) * A'
    arm_mat_mult_f32(&temp_P, &AT, &ekf->P);
    // P_k = (A * P_{k-1} * A') + Q
    arm_mat_add_f32(&ekf->P, &ekf->Q, &ekf->P);
}

void EKF_Update(EKF *ekf, float ax_mps2, float ay_mps2, float az_mps2) {
    // --- Kalman Gain K = P*C' * inv(C*P*C' + R) ---
    float sp = sinf(ekf->phi_r);
    float cp = cosf(ekf->phi_r);
    float st = sinf(ekf->theta_r);
    float ct = cosf(ekf->theta_r);

    // **FIX**: Corrected the Jacobian of the output function C
    float32_t C_data[6] = {
        0.0f,               G_MPS2 * ct,
        -G_MPS2 * cp * ct,    G_MPS2 * sp * st,
        -G_MPS2 * sp * ct,   -G_MPS2 * cp * st // This row was incorrect
    };
    arm_matrix_instance_f32 C;
    arm_mat_init_f32(&C, 3, 2, C_data);

    // Transpose of C
    float32_t CT_data[6];
    arm_matrix_instance_f32 CT;
    arm_mat_init_f32(&CT, 2, 3, CT_data);
    arm_mat_trans_f32(&C, &CT);

    // Use separate, correctly dimensioned temporary matrices
    float32_t temp_CP_data[6], temp_S_data[9], temp_S_inv_data[9];
    arm_matrix_instance_f32 temp_CP, temp_S, temp_S_inv;
    arm_mat_init_f32(&temp_CP, 3, 2, temp_CP_data); // For C*P, which is 3x2
    arm_mat_init_f32(&temp_S, 3, 3, temp_S_data);
    arm_mat_init_f32(&temp_S_inv, 3, 3, temp_S_inv_data);

    float32_t K_data[6];
    arm_matrix_instance_f32 K;
    arm_mat_init_f32(&K, 2, 3, K_data);

    // Innovation covariance: S = C*P*C' + R
    arm_mat_mult_f32(&C, &ekf->P, &temp_CP);      // temp_CP = C * P (3x2)
    arm_mat_mult_f32(&temp_CP, &CT, &temp_S);     // temp_S = (C*P) * C' (3x3)
    arm_mat_add_f32(&temp_S, &ekf->R, &temp_S);   // temp_S = S + R

    // Invert S
    arm_mat_inverse_f32(&temp_S, &temp_S_inv);

    // Kalman gain: K = P*C'*inv(S)
    float32_t temp_PCt_data[6];
    arm_matrix_instance_f32 temp_PCt;
    arm_mat_init_f32(&temp_PCt, 2, 3, temp_PCt_data); // P*C' is 2x3
    arm_mat_mult_f32(&ekf->P, &CT, &temp_PCt);        // temp_PCt = P * C' (2x3)
    arm_mat_mult_f32(&temp_PCt, &temp_S_inv, &K);     // K = (P*C') * inv(S) (2x3)

    // --- Update state x_k = x_k + K*(y - h(x_k)) ---
    float32_t y_data[3] = {ax_mps2, ay_mps2, az_mps2};
    float32_t h_data[3] = {
        G_MPS2 * st,
        -G_MPS2 * ct * sp,
        G_MPS2 * ct * cp
    };
    float32_t innovation_data[3];
    arm_matrix_instance_f32 y, h, innovation;
    arm_mat_init_f32(&y, 3, 1, y_data);
    arm_mat_init_f32(&h, 3, 1, h_data);
    arm_mat_init_f32(&innovation, 3, 1, innovation_data);

    arm_mat_sub_f32(&y, &h, &innovation);

    float32_t correction_data[2];
    arm_matrix_instance_f32 correction;
    arm_mat_init_f32(&correction, 2, 1, correction_data);

    arm_mat_mult_f32(&K, &innovation, &correction);
    arm_mat_add_f32(&ekf->x, &correction, &ekf->x);

    ekf->phi_r = ekf->x_data[0];
    ekf->theta_r = ekf->x_data[1];

    // --- Update covariance P_k = (I - K*C)*P_{k-1} ---
    float32_t I_data[4] = {1, 0, 0, 1};
    arm_matrix_instance_f32 I;
    arm_mat_init_f32(&I, 2, 2, I_data);

    float32_t KC_data[4];
    arm_matrix_instance_f32 KC;
    arm_mat_init_f32(&KC, 2, 2, KC_data);

    float32_t temp_ImKC_data[4];
    arm_matrix_instance_f32 temp_ImKC;
    arm_mat_init_f32(&temp_ImKC, 2, 2, temp_ImKC_data);

    float32_t P_new_data[4];
    arm_matrix_instance_f32 P_new;
    arm_mat_init_f32(&P_new, 2, 2, P_new_data);

    arm_mat_mult_f32(&K, &C, &KC);          // KC = K * C
    arm_mat_sub_f32(&I, &KC, &temp_ImKC);   // temp_ImKC = I - KC
    arm_mat_mult_f32(&temp_ImKC, &ekf->P, &P_new); // P_new = (I - KC) * P_old

    // Copy the result from the temporary matrix back to the main P matrix data
    memcpy(ekf->P.pData, P_new.pData, 4 * sizeof(float32_t));
}

