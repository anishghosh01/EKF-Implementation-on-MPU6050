#include "INS.h"

void INS_Init(INS_t *ins) {
    ins->position_x_m = 0.0f;
    ins->position_y_m = 0.0f;
    ins->velocity_x_mps = 0.0f;
    ins->velocity_y_mps = 0.0f;
}

void INS_Update(INS_t *ins, float roll_r, float pitch_r, float ax_g, float ay_g, float az_g, float dt_s) {
    // Convert accelerometer readings from g's to m/s^2
    float ax_mps2 = ax_g * G_MPS2;
    float ay_mps2 = ay_g * G_MPS2;

    // 1. Compensate for gravity to get linear acceleration in the body frame
    // This is the corrected gravity compensation math.
    // We SUBTRACT the gravity vector components rotated into the body frame.
    // Gravity vector in world frame: [0, 0, G_MPS2]
    // Rotated gravity in body frame g_b = R_b_w * g_w
    // g_b_x = -G_MPS2 * sin(pitch)
    // g_b_y =  G_MPS2 * cos(pitch) * sin(roll)
    float linear_ax_mps2 = ax_mps2 - (-G_MPS2 * sinf(pitch_r));
    float linear_ay_mps2 = ay_mps2 - ( G_MPS2 * cosf(pitch_r) * sinf(roll_r));

    // For a simple 2D INS, we assume the acceleration is in the horizontal (X-Y) plane.
    // A full 3D implementation would require rotating this linear acceleration vector
    // from the body frame back to the world frame using roll, pitch, AND yaw.

    // 2. Integrate acceleration to get velocity
    ins->velocity_x_mps += linear_ax_mps2 * dt_s;
    ins->velocity_y_mps += linear_ay_mps2 * dt_s;

    // 3. Add a "leaky" factor to the integrator to prevent long-term drift (Simple High-Pass Filter)
    // This slowly bleeds velocity back to zero, assuming the average velocity is 0.
    const float velocity_decay = 0.999f;
    ins->velocity_x_mps *= velocity_decay;
    ins->velocity_y_mps *= velocity_decay;

    // 4. Integrate velocity to get position
    ins->position_x_m += ins->velocity_x_mps * dt_s;
    ins->position_y_m += ins->velocity_y_mps * dt_s;
}
