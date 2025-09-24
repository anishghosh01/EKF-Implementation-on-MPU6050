#ifndef INS_MODEL_H
#define INS_MODEL_H

#include <math.h>

// Use an include guard to prevent redefinition errors
#ifndef G_MPS2
#define G_MPS2 9.80665f // Gravity in m/s^2
#endif

typedef struct {
    // Position in meters
    float position_x_m;
    float position_y_m;

    // Velocity in m/s
    float velocity_x_mps;
    float velocity_y_mps;

} INS_t;

/**
 * @brief Initializes the INS model.
 * @param ins: Pointer to the INS structure.
 */
void INS_Init(INS_t *ins);

/**
 * @brief Updates the INS model with new sensor data.
 * @param ins: Pointer to the INS structure.
 * @param roll_r: Filtered roll angle in radians.
 * @param pitch_r: Filtered pitch angle in radians.
 * @param ax_g: Raw accelerometer x-axis reading in g's.
 * @param ay_g: Raw accelerometer y-axis reading in g's.
 * @param az_g: Raw accelerometer z-axis reading in g's.
 * @param dt_s: Time delta in seconds.
 */
void INS_Update(INS_t *ins, float roll_r, float pitch_r, float ax_g, float ay_g, float az_g, float dt_s);

#endif /* INS_MODEL_H */


