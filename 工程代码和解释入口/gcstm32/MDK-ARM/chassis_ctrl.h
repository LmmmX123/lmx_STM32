#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

#include "stm32f4xx_hal.h"  // ????MCU????
#include <stdint.h>
#include <stdbool.h>

/* ================= ?????? ================= */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin_pul;   // ??(PUL)??
    uint16_t pin_dir;   // ??(DIR)??
} MotorPin_t;

/* ???????? (??????????) */
extern const MotorPin_t chassis_motors[4];

/* ================= ??????? ================= */
#define WHEEL_BASE_HALF  0.15f   // ?????? (m)
#define TRACK_HALF       0.12f   // ?????? (m)
// ????:?????????? (???????)
#define ROBOT_RADIUS     sqrtf((WHEEL_BASE_HALF)*(WHEEL_BASE_HALF) + (TRACK_HALF)*(TRACK_HALF))

#define PULSES_PER_METER 2000.0f // ????? (?????/??????)
#define MAX_FREQ_HZ      20000U  // ??????
#define START_FREQ_HZ    300U    // ??????
#define ACC_RATIO        0.5f    // ????? (0.2 = 20%????+??)
#define PULSE_CALIBRATION_FACTOR  0.8333f

/* ================= ???? (????) ================= */
typedef struct {
    int32_t steps[4];
    uint32_t abs_steps[4];
    GPIO_PinState dir[4];
    uint32_t max_steps;
    uint32_t current_step;
    uint32_t target_freq;
    uint32_t acc_steps;
    uint32_t dec_start;
    float freq_now;
    uint32_t bresenham_err[4];
    bool is_running;
} ChassisCtrl_t;

/* ================= ???? ================= */
void chassis_init(void);
void chassis_move(float vx, float vy, float speed);   // ????
void chassis_rotate(int8_t dir, uint16_t angle, float speed); // ????

#endif /* CHASSIS_CTRL_H */
