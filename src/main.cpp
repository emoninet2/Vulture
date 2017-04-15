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
#include <stdlib.h>
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

LCD03 lcd(LCD03::LCD03_SERIAL,LCD03::LCD03_20_4,LCD03::LCD03_I2C_ADDRESS_0xc8);
NRF24L01p *nrfRadio;
SX1272 myRadio;
static GPIO_InitTypeDef  GPIO_InitStruct;

NRF24L01p::RadioConfig_t RadioConfig;
NRF24L01p::RxPipeConfig_t RxPipeConfig[6];


#define TX_NODE 1
#define RX_NODE 0
void *command_handler(char **args,int arg_count){
	if(!strcmp(args[0], "lcd") ) {
		if(!strcmp(args[1], "bl") ) {
			if(!strcmp(args[2], "0")) {
				lcd.backlight(0);
			}
			else if(!strcmp(args[2], "1")) {
				lcd.backlight(1);
			}
		}
		else if(!strcmp(args[1], "cs") ) {
			lcd.clear_screen();
		}
		else if(!strcmp(args[1], "cl") ) {
			lcd.clear_line(atoi(args[2]));
		}
		else if(!strcmp(args[1], "pc") ) {
			int row = atoi(args[2]);
			int col = atoi(args[3]);
			lcd.set_cursor_coordinate(row,col);
			lcd.print_string(args[4], 1);
		}
		else if(!strcmp(args[1], "pl") ) {
			int line = atoi(args[2]);
			lcd.set_cursor_coordinate(line,1);
			lcd.print_line(atoi(args[2]), args[3], strlen(args[3]));
		}
		else if(!strcmp(args[1], "cm") ) {
			lcd.cursor_display_mode(LCD03::LCD03_CURSOR_DISP_t(atoi(args[2])));
		}
	}




}

void command_parse_execute(char *command){

	int arg_index = 0;
	char *pch;
	char *remotch_args[10];
	pch = strtok(command, "/,");
	while(pch != NULL) {
		remotch_args[arg_index] = pch;
		arg_index++;
		if(arg_index >=10) break;
		pch = strtok (NULL, "/,");
	}
	command_handler(remotch_args,arg_index);
}





void NRF24L01p_RadioReset(){

    RadioConfig.DataReadyInterruptEnabled = 0;
    RadioConfig.DataSentInterruptFlagEnabled = 0;
    RadioConfig.MaxRetryInterruptFlagEnabled = 0;
    RadioConfig.Crc = NRF24L01p::CONFIG_CRC_16BIT;
    RadioConfig.AutoReTransmissionCount = 15;
    RadioConfig.AutoReTransmitDelayX250us = 15;
    RadioConfig.frequencyOffset = 2;
    RadioConfig.datarate = NRF24L01p::RF_SETUP_RF_DR_2MBPS;
    RadioConfig.RfPower = NRF24L01p::RF_SETUP_RF_PWR_0DBM;
    RadioConfig.PllLock = 0;
    RadioConfig.ContWaveEnabled = 0;
    RadioConfig.FeatureDynamicPayloadEnabled = 1;
    RadioConfig.FeaturePayloadWithAckEnabled = 1;
    RadioConfig.FeatureDynamicPayloadWithNoAckEnabled = 1;

    RxPipeConfig[0].address = 0x454d4f4e90;
    RxPipeConfig[1].address = 0x1234567891;
    RxPipeConfig[2].address = 0x1234567892;
    RxPipeConfig[3].address = 0x1234567893;
    RxPipeConfig[4].address = 0x1234567894;
    RxPipeConfig[5].address = 0x1234567895;

    int i;

    for(i=0;i<6;i++){
        RxPipeConfig[i].PipeEnabled = 1;
        RxPipeConfig[i].autoAckEnabled = 1;
        RxPipeConfig[i].dynamicPayloadEnabled = 1;
    }

    nrfRadio->ResetConfigValues(&RadioConfig, RxPipeConfig);
}


void lcd03_thread(void * ptr)
{

	while (1)
	{
		vTaskDelay(200);
	}
}


void nrf24l01p_thread(void * ptr)
{
	NRF24L01p myNrfRadio;
	nrfRadio = &myNrfRadio;

	NRF24L01p_RadioReset();


	bool backlight = 0;
	while (1)
	{
		if(nrfRadio->readable()){
			uint8_t RxData[32];

			NRF24L01p::Payload_t payload;
			payload.data = RxData;

			nrfRadio->clear_data_ready_flag();
			nrfRadio->readPayload(&payload);
			payload.data[payload.length] = '\0';
			command_parse_execute((char*)payload.data);

			nrfRadio->flush_rx();
		}

		vTaskDelay(200);
	}
}


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
        printf("sent : %s\r\n", myname);

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

   xTaskCreate(sx1272_thread,( const char * ) "sx1272",configMINIMAL_STACK_SIZE*2,NULL,tskIDLE_PRIORITY+1 ,NULL );
  //xTaskCreate(nrf24l01p_thread,( const char * ) "nrf24l01p",configMINIMAL_STACK_SIZE*2,NULL,tskIDLE_PRIORITY+1 ,NULL );
  xTaskCreate(lcd03_thread,( const char * ) "nrf24l01p",configMINIMAL_STACK_SIZE*2,NULL,tskIDLE_PRIORITY+1 ,NULL );


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
