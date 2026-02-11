/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
*/

#include <stdint.h>
#include <unistd.h>

/* =========================
   Encoder (encoder only)
   ========================= */
#define MAX_ENCODER_PULSES 2048
#define MAX_PWM_CHANNELS 4
#define DEG_PER_COUNT 0.087890625f
#define MAX_ALLOWED_STATES 2048

/* ===============================
   Encoder state
   =============================== */
typedef struct {
    float prev_angle;
    float remainder;
    int   quad_state;
} Encoder;

/* ===============================
   Decoder state
   =============================== */
typedef struct {
    int prev_state;
    int count;
} Decoder;
                        

typedef struct {
    int8_t  pulses[MAX_ENCODER_PULSES];
    uint8_t count;
    uint8_t direction;
    float   voltage;
    float   current;
} EncoderMessage;


typedef struct {
    float pwm[MAX_PWM_CHANNELS];
    uint8_t count;
} ControllerCommand;       

void encoder_init(Encoder *e);
void decoder_init(Decoder *d);
void encoder_process_angle(Encoder *e,
                           float delta_angle,
                           int *out_states,
                           int *num_states);

void decoder_process(Decoder *d, int new_state); 