#include "openmv_ctrl.h"
#include "chassis_ctrl.h"
#include "string.h"
#include "stdio.h"

extern UART_HandleTypeDef huart2;

#define STEP_DISTANCE  0.02f    // ? 每次调整 2cm
#define MOVE_SPEED     0.4f     // ? 运动速度 0.4 m/s（可调，影响单次耗时）

uint8_t OpenMV_Run_Align(uint8_t cmd, uint32_t timeout_ms)
{
    uint8_t tx_data = cmd;
    uint8_t rx_buf[32] = {0};
    uint32_t start_time = HAL_GetTick();

    HAL_UART_Transmit(&huart2, &tx_data, 1, HAL_MAX_DELAY);

    while (HAL_GetTick() - start_time < timeout_ms)
    {
        // 1?? 接收数据（1ms 超时非阻塞）
        if (HAL_UART_Receive(&huart2, rx_buf, sizeof(rx_buf)-1, 1) == HAL_OK)
        {
            rx_buf[31] = '\0';  // ? 安全：确保字符串结尾
            
            // 2?? 完成判断
            if (strstr((char*)rx_buf, "DONE")) {
                memset(rx_buf, 0, sizeof(rx_buf));
                return 1;
            }

            // 3?? 解析 LR/UD 符号（增加健壮性检查）
            char lr_sym = 0, ud_sym = 0;
            char *lr_p = strstr((char*)rx_buf, "LR:");
            char *ud_p = strstr((char*)rx_buf, "UD:");
            
            if (lr_p && (lr_p[3] == '+' || lr_p[3] == '-')) lr_sym = lr_p[3];
            if (ud_p && (ud_p[3] == '+' || ud_p[3] == '-')) ud_sym = ud_p[3];

            // 4?? ? 坐标系映射 + 固定步长
            float vx = 0.0f, vy = 0.0f;
            
            // LR 控制横向（注意符号取反！）
            if (lr_sym == '+') vy = -STEP_DISTANCE;  // 目标在右 → 向右移 → -vy
            if (lr_sym == '-') vy =  STEP_DISTANCE;  // 目标在左 → 向左移 → +vy
            
            // UD 控制纵向（符号一致）
            if (ud_sym == '+') vx =  STEP_DISTANCE;  // 目标在前 → 向前移 → +vx
            if (ud_sym == '-') vx = -STEP_DISTANCE;  // 目标在后 → 向后移 → -vx

            // 5?? 执行运动（距离 + 速度）
            if (vx != 0.0f || vy != 0.0f) {
                chassis_move(vx, vy, MOVE_SPEED);  // vx/vy=距离, speed=最大速度
            }
            memset(rx_buf, 0, sizeof(rx_buf));
        }
        
        // 6?? 50ms 循环延迟（用户设定）
        HAL_Delay(50);
    }
    return 0;  // 超时
}
