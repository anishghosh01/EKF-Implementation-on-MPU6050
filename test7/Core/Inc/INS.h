#ifndef INS_H_
#define INS_H_

#include <math.h>

// Define the value of g in m/s^2 if it hasn't been defined elsewhere
#ifndef G_MPS2
#define G_MPS2 9.80665f
#endif

typedef struct {
    float position_x_m;
    float position_y_m;
    float velocity_x_mps;
    float velocity_y_mps;
} INS_t;

// Function prototypes
void INS_Init(INS_t *ins);
void INS_Update(INS_t *ins, float roll_r, float pitch_r, float ax_g, float ay_g, float az_g, float dt_s);
void INS_Reset_Velocity(INS_t *ins); // Ensure this is declared

#endif /* INS_MODEL_H_ */

