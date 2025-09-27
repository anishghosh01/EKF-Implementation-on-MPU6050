#include "new_ekf.h"

void New_EKF_Init(New_EKF_t *ekf, float P[2], float Q[2], float R[3]) {
    // Reset state estimates
    ekf->phi_r = 0.0f;
    ekf->theta_r = 0.0f;

    // Initialise state covariance matrix
    ekf->P[0][0] = P[0]; ekf->P[0][1] = 0.0f;
    ekf->P[1][0] = 0.0f; ekf->P[1][1] = P[1];

    // Set process and measurement noise
    ekf->Q[0] = Q[0];
    ekf->Q[1] = Q[1];
    ekf->R[0] = R[0];
    ekf->R[1] = R[1];
    ekf->R[2] = R[2];
}

void New_EKF_Predict(New_EKF_t *ekf, float p_rps, float q_rps, float r_rps, float sampleTime_s) {
    // Pre-compute trigonometric quantities
    float sp = sinf(ekf->phi_r);
    float cp = cosf(ekf->phi_r);
    float tt = tanf(ekf->theta_r);

    // Update state estimates using the kinematic model
    float dphidt = p_rps + tt * (q_rps * sp + r_rps * cp);
    float dthetadt = q_rps * cp - r_rps * sp;
    ekf->phi_r += sampleTime_s * dphidt;
    ekf->theta_r += sampleTime_s * dthetadt;

    // Clamp pitch to avoid singularity at +/- 90 degrees (gimbal lock)
    if (ekf->theta_r > (PI / 2.0f) - 0.01f) {
        ekf->theta_r = (PI / 2.0f) - 0.01f;
    } else if (ekf->theta_r < -(PI / 2.0f) + 0.01f) {
        ekf->theta_r = -(PI / 2.0f) + 0.01f;
    }

    // Re-compute trigonometric quantities for Jacobian
    sp = sinf(ekf->phi_r);
    cp = cosf(ekf->phi_r);
    tt = tanf(ekf->theta_r);
    float ctInv = 1.0f / cosf(ekf->theta_r);

    // Compute Jacobian of state transition function A = df/dx
    float A[2][2] = {
        { tt * (q_rps * cp - r_rps * sp), (q_rps * sp + r_rps * cp) * ctInv * ctInv },
        { -(q_rps * sp + r_rps * cp), 0.0f }
    };

    // Update state covariance matrix P(n+1) = P(n) + T * (A*P(n) + P(n)*A' + Q)
    float P_dot[2][2];
    P_dot[0][0] = ekf->Q[0] + 2*(A[0][0]*ekf->P[0][0] + A[0][1]*ekf->P[1][0]);
    P_dot[0][1] = A[0][0]*ekf->P[0][1] + A[0][1]*ekf->P[1][1] + A[1][0]*ekf->P[0][0];
    P_dot[1][0] = A[1][0]*ekf->P[0][0] + ekf->P[1][0]*A[0][0] + ekf->P[1][1]*A[0][1];
    P_dot[1][1] = ekf->Q[1] + 2*(A[1][0]*ekf->P[0][1]);

    ekf->P[0][0] += sampleTime_s * P_dot[0][0];
    ekf->P[0][1] += sampleTime_s * P_dot[0][1];
    ekf->P[1][0] += sampleTime_s * P_dot[1][0];
    ekf->P[1][1] += sampleTime_s * P_dot[1][1];
}

void New_EKF_Update(New_EKF_t *ekf, float ax_mps2, float ay_mps2, float az_mps2) {
    // Normalise accelerometer readings
    float accNormFactor = 1.0f / sqrtf(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);
    float ax_norm = ax_mps2 * accNormFactor;
    float ay_norm = ay_mps2 * accNormFactor;
    float az_norm = az_mps2 * accNormFactor;

    // Pre-compute trigonometric quantities
    float sp = sinf(ekf->phi_r);
    float cp = cosf(ekf->phi_r);
    float st = sinf(ekf->theta_r);
    float ct = cosf(ekf->theta_r);

    // --- IMPORTANT FIX: Invert the measurement model to match sensor orientation ---
    // The measurement function h(x) predicts the accelerometer readings
    float h[3] = {
        sinf(ekf->theta_r),
        cosf(ekf->theta_r) * sinf(ekf->phi_r),
        cosf(ekf->theta_r) * cosf(ekf->phi_r)
    };

    // The Jacobian of the output function C = dh/dx
    float C[3][2] = {
        {0.0f,      ct},
        {cp * ct,   -sp * st},
        {-sp * ct,  -cp * st}
    };

    // Calculate Kalman gain K = P*C' * (R + C*P*C')^-1
    // Step 1: P*C'
    float PCt[2][3] = {
        {ekf->P[0][0]*C[0][0] + ekf->P[0][1]*C[0][1], ekf->P[0][0]*C[1][0] + ekf->P[0][1]*C[1][1], ekf->P[0][0]*C[2][0] + ekf->P[0][1]*C[2][1]},
        {ekf->P[1][0]*C[0][0] + ekf->P[1][1]*C[0][1], ekf->P[1][0]*C[1][0] + ekf->P[1][1]*C[1][1], ekf->P[1][0]*C[2][0] + ekf->P[1][1]*C[2][1]}
    };

    // Step 2: S = R + C*P*C'
    float S[3][3];
    S[0][0] = C[0][0]*PCt[0][0] + C[0][1]*PCt[1][0] + ekf->R[0];
    S[0][1] = C[0][0]*PCt[0][1] + C[0][1]*PCt[1][1];
    S[0][2] = C[0][0]*PCt[0][2] + C[0][1]*PCt[1][2];
    S[1][0] = C[1][0]*PCt[0][0] + C[1][1]*PCt[1][0];
    S[1][1] = C[1][0]*PCt[0][1] + C[1][1]*PCt[1][1] + ekf->R[1];
    S[1][2] = C[1][0]*PCt[0][2] + C[1][1]*PCt[1][2];
    S[2][0] = C[2][0]*PCt[0][0] + C[2][1]*PCt[1][0];
    S[2][1] = C[2][0]*PCt[0][1] + C[2][1]*PCt[1][1];
    S[2][2] = C[2][0]*PCt[0][2] + C[2][1]*PCt[1][2] + ekf->R[2];

    // Step 3: inv(S)
    float detS_inv = 1.0f / (S[0][0]*(S[2][2]*S[1][1] - S[2][1]*S[1][2]) - S[0][1]*(S[2][2]*S[1][0] - S[2][0]*S[1][2]) + S[0][2]*(S[2][1]*S[1][0] - S[2][0]*S[1][1]));
    float S_inv[3][3];
    S_inv[0][0] = (S[2][2]*S[1][1] - S[2][1]*S[1][2]) * detS_inv;
    S_inv[0][1] = -(S[2][2]*S[0][1] - S[2][1]*S[0][2]) * detS_inv;
    S_inv[0][2] = (S[1][2]*S[0][1] - S[1][1]*S[0][2]) * detS_inv;
    S_inv[1][0] = -(S[2][2]*S[1][0] - S[2][0]*S[1][2]) * detS_inv;
    S_inv[1][1] = (S[2][2]*S[0][0] - S[2][0]*S[0][2]) * detS_inv;
    S_inv[1][2] = -(S[1][2]*S[0][0] - S[1][0]*S[0][2]) * detS_inv;
    S_inv[2][0] = (S[2][1]*S[1][0] - S[2][0]*S[1][1]) * detS_inv;
    S_inv[2][1] = -(S[2][1]*S[0][0] - S[2][0]*S[0][1]) * detS_inv;
    S_inv[2][2] = (S[1][1]*S[0][0] - S[1][0]*S[0][1]) * detS_inv;

    // Step 4: K = P*C' * inv(S)
    float K[2][3];
    K[0][0] = PCt[0][0]*S_inv[0][0] + PCt[0][1]*S_inv[1][0] + PCt[0][2]*S_inv[2][0];
    K[0][1] = PCt[0][0]*S_inv[0][1] + PCt[0][1]*S_inv[1][1] + PCt[0][2]*S_inv[2][1];
    K[0][2] = PCt[0][0]*S_inv[0][2] + PCt[0][1]*S_inv[1][2] + PCt[0][2]*S_inv[2][2];
    K[1][0] = PCt[1][0]*S_inv[0][0] + PCt[1][1]*S_inv[1][0] + PCt[1][2]*S_inv[2][0];
    K[1][1] = PCt[1][0]*S_inv[0][1] + PCt[1][1]*S_inv[1][1] + PCt[1][2]*S_inv[2][1];
    K[1][2] = PCt[1][0]*S_inv[0][2] + PCt[1][1]*S_inv[1][2] + PCt[1][2]*S_inv[2][2];

    // Update state estimate: x = x + K*(y - h)
    float innovation[3] = {ax_norm - h[0], ay_norm - h[1], az_norm - h[2]};
    ekf->phi_r   += K[0][0]*innovation[0] + K[0][1]*innovation[1] + K[0][2]*innovation[2];
    ekf->theta_r += K[1][0]*innovation[0] + K[1][1]*innovation[1] + K[1][2]*innovation[2];

    // Update covariance matrix: P = (I - K*C)*P
    float I_KC[2][2] = {
        {1.0f - (K[0][0]*C[0][0] + K[0][1]*C[1][0] + K[0][2]*C[2][0]), -(K[0][0]*C[0][1] + K[0][1]*C[1][1] + K[0][2]*C[2][1])},
        {-(K[1][0]*C[0][0] + K[1][1]*C[1][0] + K[1][2]*C[2][0]), 1.0f - (K[1][0]*C[0][1] + K[1][1]*C[1][1] + K[1][2]*C[2][1])}
    };
    float P_new[2][2];
    P_new[0][0] = I_KC[0][0]*ekf->P[0][0] + I_KC[0][1]*ekf->P[1][0];
    P_new[0][1] = I_KC[0][0]*ekf->P[0][1] + I_KC[0][1]*ekf->P[1][1];
    P_new[1][0] = I_KC[1][0]*ekf->P[0][0] + I_KC[1][1]*ekf->P[1][0];
    P_new[1][1] = I_KC[1][0]*ekf->P[0][1] + I_KC[1][1]*ekf->P[1][1];

    ekf->P[0][0] = P_new[0][0];
    ekf->P[0][1] = P_new[0][1];
    ekf->P[1][0] = P_new[1][0];
    ekf->P[1][1] = P_new[1][1];
}

