#ifndef NEW_EKF_H_
#define NEW_EKF_H_

#include <math.h>
#include "arm_math.h" // <-- Include CMSIS-DSP library

#ifndef PI
#define PI 3.14159265358979323846f
#endif

typedef struct {
    // State estimates [phi, theta]
    float32_t x_est_data[2];
    arm_matrix_instance_f32 x_est;

    // State covariance matrix P (2x2)
    float32_t P_data[4];
    arm_matrix_instance_f32 P;

    // Process noise covariance matrix Q (2x2)
    float32_t Q_data[4];
    arm_matrix_instance_f32 Q;

    // Measurement noise covariance matrix R (3x3)
    float32_t R_data[9];
    arm_matrix_instance_f32 R;

} New_EKF_t;

/**
 * @brief Initializes the Extended Kalman Filter.
 * @param ekf Pointer to the EKF structure.
 * @param P_init Initial covariance diagonal values.
 * @param Q_init Process noise diagonal values.
 * @param R_init Measurement noise diagonal values.
 */
void New_EKF_Init(New_EKF_t *ekf, float P_init[2], float Q_init[2], float R_init[3]);

/**
 * @brief Prediction step of the EKF.
 * @param ekf Pointer to the EKF structure.
 * @param p_rps Gyroscope X-axis reading (rad/s).
 * @param q_rps Gyroscope Y-axis reading (rad/s).
 * @param r_rps Gyroscope Z-axis reading (rad/s).
 * @param sampleTime_s Time step in seconds.
 */
void New_EKF_Predict(New_EKF_t *ekf, float p_rps, float q_rps, float r_rps, float sampleTime_s);

/**
 * @brief Update step of the EKF.
 * @param ekf Pointer to the EKF structure.
 * @param ax_mps2 Accelerometer X-axis reading (m/s^2).
 * @param ay_mps2 Accelerometer Y-axis reading (m/s^2).
 * @param az_mps2 Accelerometer Z-axis reading (m/s^2).
 */
void New_EKF_Update(New_EKF_t *ekf, float ax_mps2, float ay_mps2, float az_mps2);


#endif /* NEW_EKF_H_ */
