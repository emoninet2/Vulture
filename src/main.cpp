/*----------------------------------------------------------------------------
 *      main Template for CMSIS RTE C/C++ Project
 *----------------------------------------------------------------------------
 *      Name:    main.c
 *      Purpose: Generic main program body including main() function
 *      Rev.:    1.0.0
 *----------------------------------------------------------------------------*/
/*******************************************************************************
* Copyright (c) 2015 ARM Ltd. and others
* All rights reserved. This program and the accompanying materials
* are made available under the terms of the Eclipse Public License v1.0
* which accompanies this distribution, and is available at
* http://www.eclipse.org/legal/epl-v10.html
*
* Contributors:
* ARM Ltd and ARM Germany GmbH - file template
*******************************************************************************/

#define ARM_MATH_CM4

#ifdef _RTE_
  #include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS                     // when RTE component CMSIS RTOS is used
  #include "cmsis_os.h"                   // CMSIS RTOS header file
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */



#include "../components/components.h"

void SystemClock_Config(void);
static void Error_Handler(void);

//LCD03 lcd(LCD03::LCD03_SERIAL,LCD03::LCD03_20_4,LCD03::LCD03_I2C_ADDRESS_0xc8);
//NRF24L01p *Radio;
SX1272 LoRaRadio;
static GPIO_InitTypeDef  GPIO_InitStruct;

NRF24L01p::RadioConfig_t RadioConfig;
NRF24L01p::RxPipeConfig_t RxPipeConfig[6];


void sx1272_thread(void * ptr)
{

	int i;
	int x;
	for(i=1;i<0x3F;i++){
		x = LoRaRadio.Read(i);
		printf("%x : %x\r\n", i, x);
	}

	while (1)
	{
	  vTaskDelay(1000);
	  printf("we're staying at paris\r\n");
	}
}


/* main function */
int main(void)
{



#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  osKernelInitialize ();                // initialize CMSIS-RTOS
#endif

  /* Initialize device HAL here */

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  // create 'thread' functions that start executing,
  // example: tid_name = osThreadCreate (osThread(name), NULL);
  osKernelStart ();                     // start thread execution
#endif

  //xTaskCreate(nrf24l01p_thread,( const char * ) "t_gpio",configMINIMAL_STACK_SIZE*2,NULL,tskIDLE_PRIORITY+1 ,NULL );
  xTaskCreate(sx1272_thread,( const char * ) "t_gpio",configMINIMAL_STACK_SIZE*2,NULL,tskIDLE_PRIORITY+1 ,NULL );

  vTaskStartScheduler();
  /* Infinite loop */
  while (1)
  {

  }
}


static void Error_Handler(void)
{
  while(1)
  {
  }
}
