/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#include <math.h>
#include "core_initialisation_STM32F411xe.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static bool initialized = false;
static float t1_prev = 0, t2_prev = 0;
static int serial_fd = -1;
float Vmax = 12.0f;

static void wrap_angle(float *a) {
    while (*a >  M_PI) *a -= 2.0f * M_PI;
    while (*a < -M_PI) *a += 2.0f * M_PI;
}

float run_compensator_cycle_HIL(float t1, float t2){
    
    // 1. Raw Velocity
    // The value of 0.001 here is necessary, for the 20:1 plant where the dt is 50 micro seconds
    // this value must be 20 times that
    float d1 = (t1 - t1_prev) / 0.001f;
    float d2 = (t2 - t2_prev) / 0.001f;
    t1_prev = t1; 
    t2_prev = t2; 
    
    float angle_err = t2 - M_PI;
    wrap_angle(&angle_err);
    float u = 0;

    // 2. CONTROL LOGIC
    if (fabsf(angle_err) < 0.45f) { 
        float k1 = -101.01f; 
        float k2 = -12.41f;  
        float k3 = 10.00f;  
        float k4 = 7.33f;  

        u = (k1 * angle_err) + (k2 * d2) + (k3 * t1) + (k4 * d1);

         // Deadzone Compensation, still under test so commenting for now
        // if (fabsf(u) > 0.01f) {
        //     u += (u > 0) ? p->deadzone : -p->deadzone;
        // }
        
        if (fabsf(angle_err) < 0.03f) {
            u *= 0.7f; 
        }
    } 
    else {
        float swing_volts = Vmax * 0.85f; 

        if (fabsf(d2) < 0.5f) {
            u = (t1 > 0) ? -swing_volts : swing_volts; 
        } else {
            float direction = (d2 > 0) ? -1.0f : 1.0f;
            u = direction * swing_volts; 
        }
        u -= 12.0f * t1; 
    }
    // 4. Output Processing
    if (u > Vmax){
        u = Vmax;
       }
    if (u < -Vmax){
        u = -Vmax;
       }
    TIM3_Set_DutyCycle((fabsf(u) /Vmax)*999U); //Duty cycle to Timer3 for visualisation
    return u;

}

