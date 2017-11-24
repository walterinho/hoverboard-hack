#include "uart.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"


UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

volatile __IO struct UART_dati uartDati;


/* USART2 init function */
void MX_USART3_UART_Init(void)
{
  __HAL_RCC_USART3_CLK_ENABLE();
  /* DMA1_Channel6_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 6);
  //HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 6);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 7);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 // huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

  uartDati.UartRxReady = 0;
  uartDati.UarttxReady = 1;
}

void Console_Log(char *message)
{
    //while(huart2.State != HAL_UART_STATE_READY);
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)message, strlen(message));
//    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message),200);
}

void Uart3_TX(char *message)
{
    uartDati.UarttxReady = 0;    //occupato
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)message, strlen(message));
}
uint8_t Uart3_IS_TX_free(void){
  return (uartDati.UarttxReady); //(huart2.State == HAL_UART_STATE_READY);
}

uint8_t Uart3_IS_RX_available(void){
  return (uartDati.UartRxReady); //(huart2.State == HAL_UART_STATE_READY);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
     if (huart->Instance == USART3) {
        uartDati.UartRxReady = 1;
     }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
     if (huart->Instance == USART3) {
       uartDati.UarttxReady = 1;
     }
}
