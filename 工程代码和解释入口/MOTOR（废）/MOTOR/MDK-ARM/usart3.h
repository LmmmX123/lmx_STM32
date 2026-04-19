#ifndef __USART3_H
#define __USART3_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>

// ???????
#define RX_BUFFER_SIZE 256
// ??????
#define FIXED_RESPONSE_LEN 7
// ??????
#define FIXED_RESPONSE_1 0x02
#define FIXED_RESPONSE_2 0x00
#define FIXED_RESPONSE_3 0x00
#define FIXED_RESPONSE_4 0x01
#define FIXED_RESPONSE_5 0x00
#define FIXED_RESPONSE_6 0x33
#define FIXED_RESPONSE_7 0x31

// ????
extern const uint8_t TRIGGER_CMD[];
extern const uint8_t TRIGGER_CMD_LEN;

// ????
typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    uint16_t index;
    uint8_t receive_complete;
    uint8_t skip_fixed_response;  // ????????
} USART3_Receive_Struct;

extern USART3_Receive_Struct usart3_rx;

void USART3_Init(void);
HAL_StatusTypeDef USART3_SendCommand(const uint8_t *cmd, uint16_t size);
void USART3_ProcessReceivedData(uint8_t *valid_data, uint16_t *valid_len);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif