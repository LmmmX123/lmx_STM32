#include "openmv_ctrl.h"
#include "chassis_ctrl.h"
#include "string.h"
#include "stdio.h"

extern UART_HandleTypeDef huart2;

#define STEP_PULSE      40

uint8_t OpenMV_Run_Align(uint8_t cmd, uint32_t timeout_ms)
{
    uint8_t tx_data = cmd;
    uint8_t rx_buf[32] = {0};
    uint32_t start_time = HAL_GetTick();

    // ====================== ?? 1:????????? ======================
    HAL_UART_Transmit(&huart2, &tx_data, 1, HAL_MAX_DELAY);

    // ????
    while (HAL_GetTick() - start_time < timeout_ms)
    {
        // ====================== ?? 2:?????(????) ======================
        if (HAL_UART_Receive(&huart2, rx_buf, sizeof(rx_buf)-1, 1) == HAL_OK)
        {
            // ?? DONE = ??
            if (strstr((char*)rx_buf, "DONE"))
            {
                memset(rx_buf, 0, sizeof(rx_buf));
                return 1;
            }

            // ?? LR / UD
            char lr_sym = 0, ud_sym = 0;
            char *lr_p = strstr((char*)rx_buf, "LR:");
            char *ud_p = strstr((char*)rx_buf, "UD:");

            if (lr_p) lr_sym = lr_p[3];
            if (ud_p) ud_sym = ud_p[3];

            int fwd_back = 0;
            int left_right = 0;

            if (lr_sym == '-') fwd_back =  STEP_PULSE;
            if (lr_sym == '+') fwd_back = -STEP_PULSE;
            if (ud_sym == '+') left_right =  STEP_PULSE;
            if (ud_sym == '-') left_right = -STEP_PULSE;

            // ??????
            if (fwd_back != 0 || left_right != 0)
            {
                chassis_move(fwd_back, left_right, 0.08f);
            }

            memset(rx_buf, 0, sizeof(rx_buf));
        }

        HAL_Delay(5);
    }

    // ????
    return 0;
}
