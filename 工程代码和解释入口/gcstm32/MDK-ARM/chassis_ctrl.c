#include "chassis_ctrl.h"
#include <stdlib.h>
#include <math.h>

// ?????? (M1??, M2??, M3??, M4??)
const MotorPin_t chassis_motors[4] = {
    {GPIOB, GPIO_PIN_4, GPIO_PIN_5},   // M1
    {GPIOB, GPIO_PIN_6, GPIO_PIN_7},   // M2
    {GPIOD, GPIO_PIN_0, GPIO_PIN_1},   // M3
    {GPIOD, GPIO_PIN_2, GPIO_PIN_3}    // M4
};

// ??????
static ChassisCtrl_t ctrl = {0};

// ?????? (??DWT)
__STATIC_INLINE void delay_us_precise(uint32_t us) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

// ============================================================
// ????? (???????????)
// ???: +X??, +Y??, +W???(???)
// ????: ????(M2,M3)???????,??????
// ============================================================
static void compute_wheel_speeds(float vx, float vy, float w, float speeds[4]) {
    speeds[0] =  vx + vy + w * ROBOT_RADIUS;  // M1 ?? (??,??)
    speeds[1] = -vx + vy + w * ROBOT_RADIUS;  // M2 ?? (??,????)
    speeds[2] = -vx - vy + w * ROBOT_RADIUS;  // M3 ?? (??,????)
    speeds[3] =  vx - vy + w * ROBOT_RADIUS;  // M4 ?? (??,??)
}

void chassis_init(void) {
    for (int i = 0; i < 4; i++) {
        // ???????????
        chassis_motors[i].port->BSRR = (uint32_t)chassis_motors[i].pin_pul << 16U;
        chassis_motors[i].port->BSRR = (uint32_t)chassis_motors[i].pin_dir << 16U;
    }
    // ?????? (????????)
    #ifdef CHASSIS_EN_PORT
    HAL_GPIO_WritePin(CHASSIS_EN_PORT, CHASSIS_EN_PIN, GPIO_PIN_SET);
    #endif
    delay_us_precise(100);
}

// ============================
// ???? (???)
// ============================
void chassis_move(float vx, float vy, float speed) {
    if (ctrl.is_running) return;

    float wheel_speed[4];
    compute_wheel_speeds(vx, vy, 0.0f, wheel_speed);

    ctrl.max_steps = 0;
    for (int i = 0; i < 4; i++) {
        ctrl.steps[i] = (int32_t)(wheel_speed[i] * PULSES_PER_METER * PULSE_CALIBRATION_FACTOR);
        ctrl.dir[i] = (ctrl.steps[i] >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        ctrl.abs_steps[i] = (uint32_t)abs(ctrl.steps[i]);
        if (ctrl.abs_steps[i] > ctrl.max_steps)
            ctrl.max_steps = ctrl.abs_steps[i];
    }
    if (ctrl.max_steps == 0) return;

    ctrl.target_freq = (uint32_t)(speed * PULSES_PER_METER * PULSE_CALIBRATION_FACTOR);
    if (ctrl.target_freq > MAX_FREQ_HZ) ctrl.target_freq = MAX_FREQ_HZ;
    if (ctrl.target_freq < START_FREQ_HZ) ctrl.target_freq = START_FREQ_HZ;

    // ??????
    for (int i = 0; i < 4; i++) {
        if (ctrl.dir[i] == GPIO_PIN_SET)
            chassis_motors[i].port->BSRR = chassis_motors[i].pin_dir;
        else
            chassis_motors[i].port->BSRR = (uint32_t)chassis_motors[i].pin_dir << 16U;
    }
    delay_us_precise(10);

    // ??????
    ctrl.acc_steps = (uint32_t)(ctrl.max_steps * ACC_RATIO);
    if (ctrl.acc_steps == 0) ctrl.acc_steps = 1;
    if (ctrl.acc_steps > ctrl.max_steps / 2) ctrl.acc_steps = ctrl.max_steps / 2;
    ctrl.dec_start = ctrl.max_steps - ctrl.acc_steps;
    
    ctrl.freq_now = (float)START_FREQ_HZ;
    ctrl.current_step = 0;
    ctrl.is_running = true;
    for (int i = 0; i < 4; i++) ctrl.bresenham_err[i] = 0;

    // ??????
    while (ctrl.current_step < ctrl.max_steps) {
        // S?/?????
        if (ctrl.current_step < ctrl.acc_steps && ctrl.freq_now < ctrl.target_freq) {
            ctrl.freq_now += (float)(ctrl.target_freq - START_FREQ_HZ) / (float)ctrl.acc_steps;
        } else if (ctrl.current_step >= ctrl.dec_start && ctrl.freq_now > START_FREQ_HZ) {
            ctrl.freq_now -= (float)(ctrl.target_freq - START_FREQ_HZ) / (float)ctrl.acc_steps;
        }
        if (ctrl.freq_now > ctrl.target_freq) ctrl.freq_now = (float)ctrl.target_freq;
        if (ctrl.freq_now < START_FREQ_HZ) ctrl.freq_now = (float)START_FREQ_HZ;

        // Bresenham ????
        uint32_t pulse_mask = 0;
        for (int i = 0; i < 4; i++) {
            ctrl.bresenham_err[i] += ctrl.abs_steps[i];
            if (ctrl.bresenham_err[i] >= ctrl.max_steps) {
                ctrl.bresenham_err[i] -= ctrl.max_steps;
                pulse_mask |= (1U << i);
            }
        }

        // ????
        if (pulse_mask & 0x01) GPIOB->BSRR = GPIO_PIN_4;
        if (pulse_mask & 0x02) GPIOB->BSRR = GPIO_PIN_6;
        if (pulse_mask & 0x04) GPIOD->BSRR = GPIO_PIN_0;
        if (pulse_mask & 0x08) GPIOD->BSRR = GPIO_PIN_2;
        delay_us_precise(2); // ???????
        
        if (pulse_mask & 0x01) GPIOB->BSRR = GPIO_PIN_4 << 16U;
        if (pulse_mask & 0x02) GPIOB->BSRR = GPIO_PIN_6 << 16U;
        if (pulse_mask & 0x04) GPIOD->BSRR = GPIO_PIN_0 << 16U;
        if (pulse_mask & 0x08) GPIOD->BSRR = GPIO_PIN_2 << 16U;

        uint32_t period_us = (uint32_t)(1000000.0f / ctrl.freq_now);
        if (period_us > 2) delay_us_precise(period_us - 2);
        ctrl.current_step++;
    }
    ctrl.is_running = false;
}

// ============================
// ???? (???)
// ============================
void chassis_rotate(int8_t dir, uint16_t angle, float speed) {
    if (ctrl.is_running) return;

    // dir: 1=???, -1=???
    float w = (float)dir * speed; 
    float total_rad = angle * 3.1415926f / 180.0f;
    float total_pulses = total_rad * ROBOT_RADIUS * PULSES_PER_METER * PULSE_CALIBRATION_FACTOR;

    float wheel_speed[4];
    compute_wheel_speeds(0.0f, 0.0f, w, wheel_speed);

    ctrl.max_steps = 0;
    for (int i = 0; i < 4; i++) {
        // ?????????,??? kinematics ?????
        ctrl.steps[i] = (wheel_speed[i] > 0) ? (int32_t)total_pulses : -(int32_t)total_pulses;
        ctrl.dir[i] = (ctrl.steps[i] >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        ctrl.abs_steps[i] = (uint32_t)abs(ctrl.steps[i]);
        if (ctrl.abs_steps[i] > ctrl.max_steps) ctrl.max_steps = ctrl.abs_steps[i];
    }
    if (ctrl.max_steps == 0) return;

    ctrl.target_freq = (uint32_t)(speed * ROBOT_RADIUS * PULSES_PER_METER);
    if (ctrl.target_freq > MAX_FREQ_HZ) ctrl.target_freq = MAX_FREQ_HZ;
    if (ctrl.target_freq < START_FREQ_HZ) ctrl.target_freq = START_FREQ_HZ;

    for (int i = 0; i < 4; i++) {
        if (ctrl.dir[i] == GPIO_PIN_SET)
            chassis_motors[i].port->BSRR = chassis_motors[i].pin_dir;
        else
            chassis_motors[i].port->BSRR = (uint32_t)chassis_motors[i].pin_dir << 16U;
    }
    delay_us_precise(10);

    // ????????????? (???????,???????????)
    ctrl.acc_steps = (uint32_t)(ctrl.max_steps * ACC_RATIO);
    if (ctrl.acc_steps == 0) ctrl.acc_steps = 1;
    if (ctrl.acc_steps > ctrl.max_steps / 2) ctrl.acc_steps = ctrl.max_steps / 2;
    ctrl.dec_start = ctrl.max_steps - ctrl.acc_steps;
    ctrl.freq_now = (float)START_FREQ_HZ;
    ctrl.current_step = 0;
    ctrl.is_running = true;
    for (int i = 0; i < 4; i++) ctrl.bresenham_err[i] = 0;

    while (ctrl.current_step < ctrl.max_steps) {
        if (ctrl.current_step < ctrl.acc_steps && ctrl.freq_now < ctrl.target_freq) {
            ctrl.freq_now += (float)(ctrl.target_freq - START_FREQ_HZ) / (float)ctrl.acc_steps;
        } else if (ctrl.current_step >= ctrl.dec_start && ctrl.freq_now > START_FREQ_HZ) {
            ctrl.freq_now -= (float)(ctrl.target_freq - START_FREQ_HZ) / (float)ctrl.acc_steps;
        }
        if (ctrl.freq_now > ctrl.target_freq) ctrl.freq_now = (float)ctrl.target_freq;
        if (ctrl.freq_now < START_FREQ_HZ) ctrl.freq_now = (float)START_FREQ_HZ;

        uint32_t pulse_mask = 0;
        for (int i = 0; i < 4; i++) {
            ctrl.bresenham_err[i] += ctrl.abs_steps[i];
            if (ctrl.bresenham_err[i] >= ctrl.max_steps) {
                ctrl.bresenham_err[i] -= ctrl.max_steps;
                pulse_mask |= (1U << i);
            }
        }

        if (pulse_mask & 0x01) GPIOB->BSRR = GPIO_PIN_4;
        if (pulse_mask & 0x02) GPIOB->BSRR = GPIO_PIN_6;
        if (pulse_mask & 0x04) GPIOD->BSRR = GPIO_PIN_0;
        if (pulse_mask & 0x08) GPIOD->BSRR = GPIO_PIN_2;
        delay_us_precise(2);
        if (pulse_mask & 0x01) GPIOB->BSRR = GPIO_PIN_4 << 16U;
        if (pulse_mask & 0x02) GPIOB->BSRR = GPIO_PIN_6 << 16U;
        if (pulse_mask & 0x04) GPIOD->BSRR = GPIO_PIN_0 << 16U;
        if (pulse_mask & 0x08) GPIOD->BSRR = GPIO_PIN_2 << 16U;

        uint32_t period_us = (uint32_t)(1000000.0f / ctrl.freq_now);
        if (period_us > 2) delay_us_precise(period_us - 2);
        ctrl.current_step++;
    }
    ctrl.is_running = false;
}
