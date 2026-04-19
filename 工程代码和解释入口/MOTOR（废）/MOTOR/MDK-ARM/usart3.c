#include "usart3.h"
#include "stdio.h"
#include "main.c"


// ??????: 7E 00 08 01 00 02 01 AB CD
const uint8_t TRIGGER_CMD[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
const uint8_t TRIGGER_CMD_LEN = sizeof(TRIGGER_CMD);

USART3_Receive_Struct usart3_rx = {0};

// ??3??
extern UART_HandleTypeDef huart3;

/**
 * @brief  ????
 * @param  cmd: ????
 * @param  size: ????
 * @retval HAL??
 */
HAL_StatusTypeDef USART3_SendCommand(const uint8_t *cmd, uint16_t size)
{
    // ??????????
    return HAL_UART_Transmit(&huart3, (uint8_t *)cmd, size, 100);
}

/**
 * @brief  ????3??(????)
 */
void USART3_StartReceive(void)
{
    usart3_rx.index = 0;
    usart3_rx.receive_complete = 0;
    usart3_rx.skip_fixed_response = 1;  // ????????
    memset(usart3_rx.buffer, 0, RX_BUFFER_SIZE);
    
    // ??????
    HAL_UART_Receive_IT(&huart3, &usart3_rx.buffer[0], 1);
}

/**
 * @brief  ????????,??????
 * @param  valid_data: ?????????
 * @param  valid_len: ??????
 */
void USART3_ProcessReceivedData(uint8_t *valid_data, uint16_t *valid_len)
{
    uint16_t i;
    uint16_t valid_index = 0;
    
    // ????????7??
    if (usart3_rx.index < FIXED_RESPONSE_LEN) {
        *valid_len = 0;
        return;
    }
    
    // ???7?????????: 02 00 00 01 00 33 31
    if (usart3_rx.buffer[0] == FIXED_RESPONSE_1 &&
        usart3_rx.buffer[1] == FIXED_RESPONSE_2 &&
        usart3_rx.buffer[2] == FIXED_RESPONSE_3 &&
        usart3_rx.buffer[3] == FIXED_RESPONSE_4 &&
        usart3_rx.buffer[4] == FIXED_RESPONSE_5 &&
        usart3_rx.buffer[5] == FIXED_RESPONSE_6 &&
        usart3_rx.buffer[6] == FIXED_RESPONSE_7) {
        
        // ???7??,??????
        for (i = FIXED_RESPONSE_LEN; i < usart3_rx.index; i++) {
            valid_data[valid_index++] = usart3_rx.buffer[i];
        }
        
        *valid_len = valid_index;
    } else {
        // ????????,????????
        memcpy(valid_data, usart3_rx.buffer, usart3_rx.index);
        *valid_len = usart3_rx.index;
    }
}

/**
 * @brief  ??????????
 * @param  huart: ????
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        usart3_rx.index++;
        
        // ?????????
        if (usart3_rx.index >= RX_BUFFER_SIZE) {
            usart3_rx.index = RX_BUFFER_SIZE - 1;
            usart3_rx.receive_complete = 1;
            return;
        }
        
        // ?????????
        HAL_UART_Receive_IT(&huart3, &usart3_rx.buffer[usart3_rx.index], 1);
        
        // ????????????????
        // ??1: ????(?????????)
        // ??2: ??????????
        // ??3: ???????
        
        // ??:???????,??????
        // if (usart3_rx.index >= EXPECTED_LENGTH) {
        //     usart3_rx.receive_complete = 1;
        // }
    }
}

/**
 * @brief  ????????(??????)
 * @retval 1: ??, 0: ???
 */
uint8_t USART3_IsReceiveComplete(void)
{
    return usart3_rx.receive_complete;
}

/**
 * @brief  ??????
 */
void USART3_ResetReceive(void)
{
    usart3_rx.index = 0;
    usart3_rx.receive_complete = 0;
}