#ifndef __CHASSIS_CTRL_H
#define __CHASSIS_CTRL_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* ?????? */
#define CHASSIS_EN_PORT   GPIOB
#define CHASSIS_EN_PIN    GPIO_PIN_3

/* ??????? */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin_pul;
    uint16_t pin_dir;
} MotorPin_t;

extern const MotorPin_t chassis_motors[4];

/* ???? */
#define PULSES_PER_REV      400.0f           /* ?????(???????) */
#define WHEEL_DIAMETER_MM   76.2f            /* ?? mm */
/* ????? = 400 / (p * ??(?) * 1.04) */
#define PULSES_PER_METER    (PULSES_PER_REV / (3.14159265f * (WHEEL_DIAMETER_MM / 1000.0f * 1.02f)))

/* ???????? */
#define MAX_FREQ_HZ         6000U            /* ?????? */
#define MIN_FREQ_HZ         300U             /* ???? */
#define ACC_RATIO           0.5f             /* ?????????? */

/* ???????(20µs,??50kHz????) */
#define TIM_PERIOD_US       20U
#define PULSE_WIDTH_US      2U               /* ????2µs(>1.5µs) */

/* ???? */
void chassis_init(void);
void chassis_move(float dist_x, float dist_y, float speed);
bool chassis_is_moving(void);

#endif
