#include "chassis_ctrl.h"

// ?????????:????????,?????????????
#ifndef MAX_FREQ_HZ
#define MAX_FREQ_HZ 6000U
#endif
#ifndef START_FREQ_HZ
#define START_FREQ_HZ 300U
#endif
#ifndef ACC_RATIO
#define ACC_RATIO 0.5f
#endif

/* ??????:??(M1)???(M2)???(M3)???(M4),???????? */
const MotorPin_t chassis_motors[4] = {
    {GPIOB, GPIO_PIN_4, GPIO_PIN_5},   // M1 ??
    {GPIOB, GPIO_PIN_6, GPIO_PIN_7},   // M2 ??(????)
    {GPIOD, GPIO_PIN_0, GPIO_PIN_1},   // M3 ??(????)
    {GPIOD, GPIO_PIN_2, GPIO_PIN_3}    // M4 ??
};

/* ???? */
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
static ChassisCtrl_t ctrl = {0};

/**
 * @brief ??????(??DWT????)
 */
__STATIC_INLINE void delay_us_precise(uint32_t us) {
    if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while((DWT->CYCCNT - start) < ticks);
}

/**
 * @brief ?????
 */
void chassis_init(void)
{
    // 1. ???PUL/DIR????????
    for(int i = 0; i < 4; i++) {
        chassis_motors[i].port->BSRR = (uint32_t)chassis_motors[i].pin_pul << 16U;
        chassis_motors[i].port->BSRR = (uint32_t)chassis_motors[i].pin_dir << 16U;
    }
    
    // 2. ???????
    HAL_GPIO_WritePin(CHASSIS_EN_PORT, CHASSIS_EN_PIN, GPIO_PIN_SET);
    
    // 3. ??????
    delay_us_precise(100);
}

/**
 * @brief ??O?????????,???????
 */
void chassis_move(float dist_x, float dist_y, float speed)
{
    if(ctrl.is_running) return;
    
    // ============== ??O?????????,????????,??? ==============
    // ?????,???????????,????
    // ?????,??????
    // ??????,??????????,??????
    float kin[4] = {
        dist_x + dist_y,   // ??
        dist_x - dist_y,   // ??
        dist_x - dist_y,   // ??
        dist_x + dist_y    // ??
    };
    
    ctrl.max_steps = 0;
    for(int i = 0; i < 4; i++) {
        ctrl.steps[i] = (int32_t)(kin[i] * PULSES_PER_METER);
        ctrl.dir[i] = (ctrl.steps[i] < 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        ctrl.abs_steps[i] = (uint32_t)(ctrl.steps[i] < 0 ? -ctrl.steps[i] : ctrl.steps[i]);
        
        if(ctrl.abs_steps[i] > ctrl.max_steps) {
            ctrl.max_steps = ctrl.abs_steps[i];
        }
    }
    if(ctrl.max_steps == 0) return;
    
    // ??????
    ctrl.target_freq = (uint32_t)(speed * PULSES_PER_METER);
    if(ctrl.target_freq > MAX_FREQ_HZ) ctrl.target_freq = MAX_FREQ_HZ;
    if(ctrl.target_freq < START_FREQ_HZ) ctrl.target_freq = START_FREQ_HZ;
    
    // ============== ????????????,?????????????? ==============
    for(int i = 0; i < 4; i++) {
        GPIO_PinState actual_dir = ctrl.dir[i];
        // ??(M2)???(M3)??????,??????
        if(i == 1 || i == 2) { 
            actual_dir = (actual_dir == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        }
        
        if(actual_dir == GPIO_PIN_SET)
            chassis_motors[i].port->BSRR = chassis_motors[i].pin_dir;
        else
            chassis_motors[i].port->BSRR = (uint32_t)(chassis_motors[i].pin_dir << 16U);
    }
    delay_us_precise(10); // ??????
    
    // ????????
    ctrl.acc_steps = (uint32_t)(ctrl.max_steps * ACC_RATIO);
    if(ctrl.acc_steps == 0) ctrl.acc_steps = 1;
    ctrl.dec_start = ctrl.max_steps - ctrl.acc_steps;
    ctrl.freq_now = (float)START_FREQ_HZ;
    
    // ?????
    ctrl.current_step = 0;
    ctrl.is_running = true;
    for(int i = 0; i < 4; i++) {
        ctrl.bresenham_err[i] = 0;
    }
    
    // ?????????
    while(ctrl.current_step < ctrl.max_steps) {
        // ??????(???)
        if(ctrl.current_step < ctrl.acc_steps && ctrl.freq_now < ctrl.target_freq) {
            ctrl.freq_now += (float)(ctrl.target_freq - START_FREQ_HZ) / (float)ctrl.acc_steps;
        } else if(ctrl.current_step >= ctrl.dec_start && ctrl.freq_now > START_FREQ_HZ) {
            ctrl.freq_now -= (float)(ctrl.target_freq - START_FREQ_HZ) / (float)ctrl.acc_steps;
        }
        if(ctrl.freq_now > ctrl.target_freq) ctrl.freq_now = (float)ctrl.target_freq;
        if(ctrl.freq_now < START_FREQ_HZ) ctrl.freq_now = (float)START_FREQ_HZ;

        // Bresenham????
        uint32_t pulse_mask = 0;
        for(int i = 0; i < 4; i++) {
            ctrl.bresenham_err[i] += ctrl.abs_steps[i];
            if(ctrl.bresenham_err[i] >= ctrl.max_steps) {
                ctrl.bresenham_err[i] -= ctrl.max_steps;
                pulse_mask |= (1U << i);
            }
        }

        // ????
        if(pulse_mask) {
            if(pulse_mask & 0x01) GPIOB->BSRR = GPIO_PIN_4;
            if(pulse_mask & 0x02) GPIOB->BSRR = GPIO_PIN_6;
            if(pulse_mask & 0x04) GPIOD->BSRR = GPIO_PIN_0;
            if(pulse_mask & 0x08) GPIOD->BSRR = GPIO_PIN_2;
        }
        delay_us_precise(2);

        if(pulse_mask) {
            if(pulse_mask & 0x01) GPIOB->BSRR = GPIO_PIN_4 << 16;
            if(pulse_mask & 0x02) GPIOB->BSRR = GPIO_PIN_6 << 16;
            if(pulse_mask & 0x04) GPIOD->BSRR = GPIO_PIN_0 << 16;
            if(pulse_mask & 0x08) GPIOD->BSRR = GPIO_PIN_2 << 16;
        }

        // ??????
        uint32_t period_us = (uint32_t)(1000000.0f / ctrl.freq_now);
        if(period_us > 2) {
            delay_us_precise(period_us - 2);
        }

        ctrl.current_step++;
    }

    // ????,????
    ctrl.is_running = false;
}

bool chassis_is_moving(void)
{
    return ctrl.is_running;
}
