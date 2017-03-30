/*
 * LoRaPort.cpp
 *
 *  Created on: Mar 29, 2017
 *      Author: emon1
 */

#include "../LoRaPort.h"

#include "../LoRaPortConfig.h"

#if (LoRaPort_STM32 == 1)




#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */


#define LoRa_SPI				SPI1

#define LoRa_NSS_PORT			GPIOB
#define LoRa_NSS_PIN			GPIO_PIN_6
#define LoRa_RESET_PORT			GPIOC
#define LoRa_RESET_PIN			GPIO_PIN_7
#define LoRa_TX_PORT			GPIOB
#define LoRa_TX_PIN				GPIO_PIN_9
#define LoRa_RX_PORT			GPIOB
#define LoRa_RX_PIN				GPIO_PIN_8

/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

static SPI_HandleTypeDef LoRa_SpiHandle;


static GPIO_InitTypeDef LoRa_NSS_PIN_Struct = {LoRa_NSS_PIN,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH,};
static GPIO_InitTypeDef LoRa_RESET_PIN_Struct = {LoRa_RESET_PIN,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH,};
static GPIO_InitTypeDef LoRa_TX_PIN_Struct = {LoRa_TX_PIN,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH,};
static GPIO_InitTypeDef LoRa_RX_PIN_Struct = {LoRa_RX_PIN,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH,};


LoRaPort::LoRaPort() {
	// TODO Auto-generated constructor stub
	/* Enable GPIOA clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	SPI_Init();

	GPIO_Init(PIN_NSS);
	GPIO_Init(PIN_RESET);
	GPIO_Init(PIN_TX);
	GPIO_Init(PIN_RX);
}

LoRaPort::~LoRaPort() {
	// TODO Auto-generated destructor stub
}

void LoRaPort::GPIO_Init(LoRaPort::LoRaGPIO_t pin){
	switch(pin){
	case PIN_NSS 	: HAL_GPIO_Init(LoRa_NSS_PORT, &LoRa_NSS_PIN_Struct); break;
	case PIN_RESET 	: HAL_GPIO_Init(LoRa_RESET_PORT, &LoRa_RESET_PIN_Struct); break;
	case PIN_TX 	: HAL_GPIO_Init(LoRa_TX_PORT, &LoRa_TX_PIN_Struct); break;
	case PIN_RX 	: HAL_GPIO_Init(LoRa_RX_PORT, &LoRa_RX_PIN_Struct); break;
	default			: break;
	}
}
void LoRaPort::GPIO_DeInit(LoRaPort::LoRaGPIO_t pin){
	switch(pin){
	case PIN_NSS 	: HAL_GPIO_DeInit(LoRa_NSS_PORT, LoRa_NSS_PIN); break;
	case PIN_RESET 	: HAL_GPIO_DeInit(LoRa_RESET_PORT, LoRa_RESET_PIN); break;
	case PIN_TX 	: HAL_GPIO_DeInit(LoRa_TX_PORT, LoRa_RESET_PIN); break;
	case PIN_RX 	: HAL_GPIO_DeInit(LoRa_RX_PORT, LoRa_RESET_PIN); break;
	default			: break;
	}
}
void LoRaPort::GPIO_SetValue(LoRaPort::LoRaGPIO_t pin, bool value){
	switch(pin){
	case PIN_NSS 	: HAL_GPIO_WritePin(LoRa_NSS_PORT, LoRa_NSS_PIN, (GPIO_PinState)value); break;
	case PIN_RESET 	: HAL_GPIO_WritePin(LoRa_RESET_PORT, LoRa_RESET_PIN, (GPIO_PinState)value); break;
	case PIN_TX 	: HAL_GPIO_WritePin(LoRa_TX_PORT, LoRa_TX_PIN, (GPIO_PinState)value); break;
	case PIN_RX 	: HAL_GPIO_WritePin(LoRa_RX_PORT, LoRa_RX_PIN, (GPIO_PinState)value); break;
	default			: break;
	}
}
bool LoRaPort::GPIO_GetValue(LoRaPort::LoRaGPIO_t pin){
	switch(pin){
	case PIN_NSS 	: return HAL_GPIO_ReadPin(LoRa_NSS_PORT, LoRa_NSS_PIN); break;
	case PIN_RESET 	: return HAL_GPIO_ReadPin(LoRa_RESET_PORT, LoRa_RESET_PIN); break;
	case PIN_TX 	: return HAL_GPIO_ReadPin(LoRa_TX_PORT, LoRa_TX_PIN); break;
	case PIN_RX 	: return HAL_GPIO_ReadPin(LoRa_RX_PORT, LoRa_RX_PIN); break;
	default			: break;
	}
}

void LoRaPort::SPI_Init(void){
	// TODO Auto-generated destructor stub

	/*##-1- Configure the SPI peripheral #######################################*/
	/* Set the SPI parameters */
	LoRa_SpiHandle.Instance               = SPIx;
	LoRa_SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	LoRa_SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	LoRa_SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	LoRa_SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	LoRa_SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	LoRa_SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	LoRa_SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	LoRa_SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	LoRa_SpiHandle.Init.CRCPolynomial     = 7;
	//nrf24l01p_SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
	LoRa_SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	//nrf24l01p_SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

	LoRa_SpiHandle.Init.Mode = SPI_MODE_MASTER;
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	SPIx_SCK_GPIO_CLK_ENABLE();
	SPIx_MISO_GPIO_CLK_ENABLE();
	SPIx_MOSI_GPIO_CLK_ENABLE();
	/* Enable SPI clock */
	SPIx_CLK_ENABLE();

	/*##-2- Configure peripheral GPIO ##########################################*/
	/* SPI SCK GPIO pin configuration  */
	GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = SPIx_SCK_AF;
	HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

	/* SPI MISO GPIO pin configuration  */
	GPIO_InitStruct.Pin = SPIx_MISO_PIN;
	GPIO_InitStruct.Alternate = SPIx_MISO_AF;
	HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

	/* SPI MOSI GPIO pin configuration  */
	GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
	GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
	HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);

	HAL_SPI_Init(&LoRa_SpiHandle);
}
void LoRaPort::SPI_TranscieveBuffer( uint8_t *dataInOut, unsigned int size ){
	HAL_SPI_TransmitReceive(&LoRa_SpiHandle, dataInOut, dataInOut, size,1000);
}
uint8_t LoRaPort::SPI_TranscieveByte( uint8_t dataIn ){

}
void LoRaPort::Timeout_init(LoRaPort::LoRaTimeout_t timeout){

}
void LoRaPort::Timeout_SetTime(LoRaPort::LoRaTimeout_t timeout, unsigned int time){

}
void LoRaPort::delayUs(unsigned int us){
	asm("nop");
}
void LoRaPort::delayMs(unsigned int ms){
	asm("nop");
}

#endif
