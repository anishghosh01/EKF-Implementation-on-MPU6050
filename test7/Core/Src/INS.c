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
    float az_mps2 = az_g * G_MPS2;

    // 1. Compensate for gravity to get linear acceleration
    // Note: A full implementation would rotate this linear acceleration
    // from the body frame to the world frame. For small angles, this
    // simplification is acceptable to avoid needing a yaw estimate.
    float linear_ax_mps2 = ax_mps2 - G_MPS2 * sinf(pitch_r);
    float linear_ay_mps2 = ay_mps2 + G_MPS2 * cosf(pitch_r) * sinf(roll_r);
    // Z-axis is not used for 2D distance estimation

    // 2. Integrate acceleration to get velocity
    ins->velocity_x_mps += linear_ax_mps2 * dt_s;
    ins->velocity_y_mps += linear_ay_mps2 * dt_s;

    // 3. Integrate velocity to get position
    ins->position_x_m += ins->velocity_x_mps * dt_s;
    ins->position_y_m += ins->velocity_y_mps * dt_s;
}

// --- **FIX**: Added the missing function definition ---
void INS_Reset_Velocity(INS_t *ins) {
    ins->velocity_x_mps = 0.0f;
    ins->velocity_y_mps = 0.0f;
}

