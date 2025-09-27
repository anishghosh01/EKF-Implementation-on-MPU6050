#include "new_ekf.h"

void New_EKF_Init(New_EKF_t *ekf, float P_init[2], float Q_init[2], float R_init[3]) {
    // Reset state estimates
    ekf->x_est_data[0] = 0.0f; // phi
    ekf->x_est_data[1] = 0.0f; // theta
    arm_mat_init_f32(&ekf->x_est, 2, 1, ekf->x_est_data);

    // Initialise state covariance matrix P
    // P = [P_init[0] 0; 0 P_init[1]]
    ekf->P_data[0] = P_init[0]; ekf->P_data[1] = 0.0f;
    ekf->P_data[2] = 0.0f;      ekf->P_data[3] = P_init[1];
    arm_mat_init_f32(&ekf->P, 2, 2, ekf->P_data);

    // Set process noise covariance matrix Q
    // Q = [Q_init[0] 0; 0 Q_init[1]]
    ekf->Q_data[0] = Q_init[0]; ekf->Q_data[1] = 0.0f;
    ekf->Q_data[2] = 0.0f;      ekf->Q_data[3] = Q_init[1];
    arm_mat_init_f32(&ekf->Q, 2, 2, ekf->Q_data);

    // Set measurement noise covariance matrix R
    // R = [R_init[0] 0 0; 0 R_init[1] 0; 0 0 R_init[2]]
    ekf->R_data[0] = R_init[0]; ekf->R_data[1] = 0.0f;     ekf->R_data[2] = 0.0f;
    ekf->R_data[3] = 0.0f;      ekf->R_data[4] = R_init[1]; ekf->R_data[5] = 0.0f;
    ekf->R_data[6] = 0.0f;      ekf->R_data[7] = 0.0f;     ekf->R_data[8] = R_init[2];
    arm_mat_init_f32(&ekf->R, 3, 3, ekf->R_data);
}

void New_EKF_Predict(New_EKF_t *ekf, float p_rps, float q_rps, float r_rps, float sampleTime_s) {
    float phi   = ekf->x_est_data[0];
    float theta = ekf->x_est_data[1];

    // Pre-compute trigonometric quantities
    float sp = sinf(phi);
    float cp = cosf(phi);
    float tt = tanf(theta);

    // 1. Update state estimates using the non-linear kinematic model
    float dphidt = p_rps + tt * (q_rps * sp + r_rps * cp);
    float dthetadt = q_rps * cp - r_rps * sp;
    ekf->x_est_data[0] += sampleTime_s * dphidt;
    ekf->x_est_data[1] += sampleTime_s * dthetadt;

    // Re-assign and clamp pitch to avoid singularity
    phi   = ekf->x_est_data[0];
    theta = ekf->x_est_data[1];
    if (theta > (PI / 2.0f) - 0.01f) {
        theta = ekf->x_est_data[1] = (PI / 2.0f) - 0.01f;
    } else if (theta < -(PI / 2.0f) + 0.01f) {
        theta = ekf->x_est_data[1] = -(PI / 2.0f) + 0.01f;
    }

    // Re-compute trig quantities for Jacobian
    sp = sinf(phi);
    cp = cosf(phi);
    tt = tanf(theta);
    float ctInv = 1.0f / cosf(theta);

    // 2. Compute Jacobian of state transition function A = df/dx
    float32_t A_data[4] = {
        tt * (q_rps * cp - r_rps * sp), (q_rps * sp + r_rps * cp) * ctInv * ctInv,
        -(q_rps * sp + r_rps * cp), 0.0f
    };
    arm_matrix_instance_f32 A;
    arm_mat_init_f32(&A, 2, 2, A_data);

    // 3. Update state covariance matrix: P = P + T * (A*P + P*A' + Q)
    // This is the forward Euler integration of the continuous-time Riccati equation.
    float32_t At_data[4];
    float32_t AP_data[4];
    float32_t PAt_data[4];
    float32_t P_dot_data[4];

    arm_matrix_instance_f32 At, AP, PAt, P_dot;
    arm_mat_init_f32(&At, 2, 2, At_data);
    arm_mat_init_f32(&AP, 2, 2, AP_data);
    arm_mat_init_f32(&PAt, 2, 2, PAt_data);
    arm_mat_init_f32(&P_dot, 2, 2, P_dot_data);

    arm_mat_trans_f32(&A, &At);                  // A'
    arm_mat_mult_f32(&A, &ekf->P, &AP);          // A*P
    arm_mat_mult_f32(&ekf->P, &At, &PAt);        // P*A'
    arm_mat_add_f32(&AP, &PAt, &P_dot);          // A*P + P*A'
    arm_mat_add_f32(&P_dot, &ekf->Q, &P_dot);    // A*P + P*A' + Q
    arm_mat_scale_f32(&P_dot, sampleTime_s, &P_dot); // T * (A*P + P*A' + Q)
    arm_mat_add_f32(&ekf->P, &P_dot, &ekf->P);   // P + T*(...)
}

void New_EKF_Update(New_EKF_t *ekf, float ax_mps2, float ay_mps2, float az_mps2) {
    float phi   = ekf->x_est_data[0];
    float theta = ekf->x_est_data[1];

    // 1. Normalise accelerometer readings
    float accNormFactor = 1.0f / sqrtf(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);
    float32_t y_data[3] = {
        ax_mps2 * accNormFactor,
        ay_mps2 * accNormFactor,
        az_mps2 * accNormFactor
    };
    arm_matrix_instance_f32 y;
    arm_mat_init_f32(&y, 3, 1, y_data);

    // 2. Predicted measurement h(x)
    float32_t h_data[3] = {
       -sinf(theta),
       cosf(theta) * sinf(phi), // Note the sign change based on typical sensor frames
       cosf(theta) * cosf(phi)
    };

    // 3. Jacobian of the output function C = dh/dx
    float32_t C_data[6] = {
        0.0f,                   -cosf(theta),
       cosf(theta) * cosf(phi), -sinf(theta) * sinf(phi),
       -cosf(theta) * sinf(phi),-sinf(theta) * cosf(phi)
    };
    arm_matrix_instance_f32 C;
    arm_mat_init_f32(&C, 3, 2, C_data);

    // 4. Calculate Kalman gain: K = P*C' * (R + C*P*C')^-1
    float32_t Ct_data[6], PCt_data[6], S_data[9], S_inv_data[9], K_data[6];
    arm_matrix_instance_f32 Ct, PCt, S, S_inv, K;
    arm_mat_init_f32(&Ct, 2, 3, Ct_data);
    arm_mat_init_f32(&PCt, 2, 3, PCt_data);
    arm_mat_init_f32(&S, 3, 3, S_data);
    arm_mat_init_f32(&S_inv, 3, 3, S_inv_data);
    arm_mat_init_f32(&K, 2, 3, K_data);

    arm_mat_trans_f32(&C, &Ct);                          // C'
    arm_mat_mult_f32(&ekf->P, &Ct, &PCt);                // P*C'
    arm_mat_mult_f32(&C, &PCt, &S);                      // C*(P*C')
    arm_mat_add_f32(&S, &ekf->R, &S);                    // S = R + C*P*C'
    arm_mat_inverse_f32(&S, &S_inv);                     // inv(S)
    arm_mat_mult_f32(&PCt, &S_inv, &K);                  // K = P*C'*inv(S)

    // 5. Update state estimate: x = x + K*(y - h)
    float32_t innovation_data[3];
    float32_t correction_data[2];
    arm_matrix_instance_f32 innovation, correction;
    arm_mat_init_f32(&innovation, 3, 1, innovation_data);
    arm_mat_init_f32(&correction, 2, 1, correction_data);

    arm_sub_f32(y_data, h_data, innovation_data, 3);    // innovation = y - h
    arm_mat_mult_f32(&K, &innovation, &correction);     // K*(y-h)
    arm_mat_add_f32(&ekf->x_est, &correction, &ekf->x_est); // x = x + K*(y-h)

    // 6. Update covariance matrix: P = (I - K*C)*P
    float32_t I_data[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    float32_t KC_data[4];
    float32_t I_KC_data[4];
    float32_t P_new_data[4];
    arm_matrix_instance_f32 I, KC, I_KC, P_new;
    arm_mat_init_f32(&I, 2, 2, I_data);
    arm_mat_init_f32(&KC, 2, 2, KC_data);
    arm_mat_init_f32(&I_KC, 2, 2, I_KC_data);
    arm_mat_init_f32(&P_new, 2, 2, P_new_data);

    arm_mat_mult_f32(&K, &C, &KC);                 // K*C
    arm_mat_sub_f32(&I, &KC, &I_KC);               // I - K*C
    arm_mat_mult_f32(&I_KC, &ekf->P, &P_new);       // (I - K*C)*P
    // Copy new P data back to the main structure
    for(int i=0; i<4; i++) {
        ekf->P_data[i] = P_new_data[i];
    }
}
