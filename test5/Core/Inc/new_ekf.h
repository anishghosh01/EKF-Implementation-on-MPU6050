#ifndef NEW_EKF_H_
#define NEW_EKF_H_

#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

typedef struct {
    // State estimates
    float phi_r;   // Roll angle in radians
    float theta_r; // Pitch angle in radians

    // State covariance matrix
    float P[2][2];

    // Process and measurement noise
    float Q[2];
    float R[3];

} New_EKF_t;

/**
 * @brief Initializes the Extended Kalman Filter.
 * @param ekf Pointer to the EKF structure.
 * @param P Initial covariance values.
 * @param Q Process noise values.
 * @param R Measurement noise values.
 */
void New_EKF_Init(New_EKF_t *ekf, float P[2], float Q[2], float R[3]);

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

