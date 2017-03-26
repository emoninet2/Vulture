/*
 * LCD03Port.cpp
 *
 *  Created on: Mar 26, 2017
 *      Author: emon1
 */

#include "../LCD03Port.h"
#include "../LCD03PortConfig.h"


#if (LCD03Port_STM32 == 1)

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Definition for USARTx clock resources */
#define LCD03_USARTx                           USART1
#define LCD03_USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define LCD03_USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD03_USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define LCD03_USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define LCD03_USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define LCD03_USARTx_TX_PIN                    GPIO_PIN_9
#define LCD03_USARTx_TX_GPIO_PORT              GPIOA
#define LCD03_USARTx_TX_AF                     GPIO_AF7_USART1
#define LCD03_USARTx_RX_PIN                    GPIO_PIN_10
#define LCD03_USARTx_RX_GPIO_PORT              GPIOA
#define LCD03_USARTx_RX_AF                     GPIO_AF7_USART1


/* UART handler declaration */
UART_HandleTypeDef LCD03UartHandle;
GPIO_InitTypeDef  GPIO_InitStruct;

LCD03Port::LCD03Port() {
	// TODO Auto-generated constructor stub

}

LCD03Port::~LCD03Port() {
	// TODO Auto-generated destructor stub
}

int LCD03Port::portSerialInit(){

	LCD03_USARTx_RX_GPIO_CLK_ENABLE();
	LCD03_USARTx_TX_GPIO_CLK_ENABLE();
	LCD03_USARTx_CLK_ENABLE() ;

	LCD03UartHandle.Instance          = LCD03_USARTx;
	LCD03UartHandle.Init.BaudRate     = 9600;
	LCD03UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	LCD03UartHandle.Init.StopBits     = UART_STOPBITS_1;
	LCD03UartHandle.Init.Parity       = UART_PARITY_NONE;
	LCD03UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	LCD03UartHandle.Init.Mode         = UART_MODE_TX_RX;
	LCD03UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;


	/*##-2- Configure peripheral GPIO ##########################################*/
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = LCD03_USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = LCD03_USARTx_TX_AF;

	HAL_GPIO_Init(LCD03_USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = LCD03_USARTx_RX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = LCD03_USARTx_RX_AF;

	HAL_GPIO_Init(LCD03_USARTx_RX_GPIO_PORT, &GPIO_InitStruct);


	if(HAL_UART_Init(&LCD03UartHandle) != HAL_OK)
	{
	/* Initialization Error */
	//Error_Handler();
	}

}
int LCD03Port::portSerialTransmit(uint8_t data){
	return HAL_UART_Transmit(&LCD03UartHandle, &data, sizeof(uint8_t),0xFFFF);
}


int LCD03Port::portI2CInit(){

}
int LCD03Port::portI2CTransmit(uint8_t data){

}

#endif
