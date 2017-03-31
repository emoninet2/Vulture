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
SX1272 myRadio;
static GPIO_InitTypeDef  GPIO_InitStruct;

NRF24L01p::RadioConfig_t RadioConfig;
NRF24L01p::RxPipeConfig_t RxPipeConfig[6];


#define TX_NODE 1
#define RX_NODE 0


void sx1272_thread(void * ptr)
{

    printf("starting up system\r\n");
    myRadio.Reset();


    myRadio.SetModem(SX1272::MODEM_LORA );
    myRadio.LoRaOpMode(LoRa_OpMode_STDBY);
    myRadio.LoRaModemConfig(
    				LoRa_ModemBw_500kHz,
                    LoRa_ModemCodingRate_4Div5,
                    0,
                    1,
                    1,
					LoRa_ModemSpreadingFactor_4096chipsPerSymbol,
                    0,
                    1
                    );
    myRadio.Frf(0xD84CCC);
    myRadio.PaConfig(SX1272_PaSelect_PA_BOOST_pin, 0x07);
    myRadio.PaRamp(0 , SX1272_PaRamp_40us);
    myRadio.OCP(1, 0x1B);
    myRadio.LNA(SX1272_LnaGain_G1, SX1272_LnaBoost_BoostOn)    ;
    myRadio.LoRaWriteFifoAddrPtr(0);
    myRadio.FskNodeAddress(3);
    myRadio.FskBroadcastAddress(0);

    myRadio.LoRaPayloadLength(255);

    int i;
    int x;
    for(i=1;i<0x3F;i++){
        x = myRadio.Read(i);
        printf("%x : %x\r\n", i, x);
    }

    char myname[] = "the way you shake it";

    myRadio.Write( REG_LORA_IRQFLAGS, 0xFF);
    int n=0;
    int count = 0;

	#if (TX_NODE == 1)
		myRadio.FskNodeAddress(3);
		myRadio.antSwSet(1);
	#endif
	#if (RX_NODE == 1)
		myRadio.FskNodeAddress(8);
		myRadio.antSwSet(0);
	#endif

	myRadio.delayMs(1000);

	while (1)
	{
#if (TX_NODE == 1)
        //TX
        sprintf(myname, "count : %d", count++);
        myRadio.LoRaOpMode(LoRa_OpMode_STDBY);
        myRadio.LoRaWriteFifoAddrPtr(0x80);
        myRadio.Write(0, strlen(myname) + 1);
        myRadio.WriteFifo((uint8_t*)myname, strlen(myname) );
        int addr = myRadio.Read( REG_LORA_FIFOADDRPTR);
        myRadio.LoRaOpMode(LoRa_OpMode_TX);

        while(myRadio.LoRaIrqFlags(LoRa_IrqFlags_TxDone) == 0);
        printf("sent\r\n");

        myRadio.LoRaOpMode(LoRa_OpMode_STDBY);
        myRadio.Write( REG_LORA_IRQFLAGS, 0xFF);
        //delay(500);
#endif

#if (RX_NODE == 1)
        myRadio.LoRaDetectOptimize(DetectOptimize_SF7_SF12);
        myRadio.PaRamp(0,SX1272_PaRamp_40us);
        myRadio.LoRaWriteFifoAddrPtr(0x00);
        myRadio.LoRaSymbTImeout(0xFF);
        myRadio.LoRaWriteFifoRxBaseAddr(0x00);
        myRadio.LoRaPayloadLength(255);
        myRadio.LoRaOpMode(LoRa_OpMode_RXContinuous);

        while(myRadio.LoRaIrqFlags(LoRa_IrqFlags_RxDone) == 0);
        //printf("got data\r\n");

        myRadio.LoRaOpMode(LoRa_OpMode_STDBY);
        uint8_t temp;
        myRadio.LoRaWriteFifoAddrPtr(0x00);
        myRadio.ReadFifo(&temp, 1 );
        //printf("size of data is : %d\r\n", temp);
        for(i=0;i<temp-1;i++){
                uint8_t temp2;
                myRadio.ReadFifo(&temp2,1);
                printf("%c",temp2);
        }
        printf("\r\n");
        myRadio.Write( REG_LORA_IRQFLAGS, 0xFF);

 #endif
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
