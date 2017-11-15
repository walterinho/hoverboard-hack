#include "telemetry.h"
#include "uart.h"
#include "application.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern volatile __IO struct UART_dati uartData;
extern volatile __IO struct APPLICATION_dati app;
extern volatile __IO struct BATTERY_dati battery_dati;
volatile __IO struct TELEMETRY_dati telemetry;
volatile COMMAND_data commandsequence;

volatile __IO uint32_t leftToTransfer;


void Telemetry_init(void){
  MX_USART2_UART_Init();
  //HAL_UART_Receive_DMA(&huart2, (uint8_t *)&commandsequence, 8);

}

int16_t getMotorL() {
  return commandsequence.motorRxL * 1000;
}

int16_t getMotorR() {
  return commandsequence.motorRxR * 1000;
}
