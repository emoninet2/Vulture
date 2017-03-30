/*
 * LoRaHAL.cpp
 *
 *  Created on: Mar 28, 2017
 *      Author: emon1
 */

#include "LoRaHAL.h"

LoRaHAL::LoRaHAL() {
	// TODO Auto-generated constructor stub

}

LoRaHAL::~LoRaHAL() {
	// TODO Auto-generated destructor stub
}

void LoRaHAL::antSwInit(void){
	GPIO_Init(PIN_TX);
	GPIO_Init(PIN_RX);
}
void LoRaHAL::antSwDeInit(void){
	GPIO_DeInit(PIN_TX);
	GPIO_DeInit(PIN_RX);
}
void LoRaHAL::antSwSet(bool txrx){
	if(txrx){
		GPIO_SetValue(PIN_TX,1);
		GPIO_SetValue(PIN_RX,0);
	}
	else{
		GPIO_SetValue(PIN_TX,0);
		GPIO_SetValue(PIN_RX,1);
	}
}

void LoRaHAL::reset(){
	GPIO_Init(PIN_RESET);
	GPIO_SetValue(PIN_RESET,1);
	delayMs(1);
	GPIO_SetValue(PIN_RESET,0);
	GPIO_DeInit(PIN_RESET);
	delayMs(6);
}

void LoRaHAL::Write(uint8_t addr, uint8_t data){
    uint8_t address = addr | (1<<7);
    GPIO_SetValue(PIN_NSS,0);
    SPI_TranscieveBuffer(&address, 1);
    SPI_TranscieveBuffer(&data, 1);
    GPIO_SetValue(PIN_NSS,1);
}
uint8_t LoRaHAL::Read(uint8_t addr){
    uint8_t address = addr& ~(1<<7);
    uint8_t data;
    GPIO_SetValue(PIN_NSS,0);
    SPI_TranscieveBuffer(&address, 1);
    SPI_TranscieveBuffer(&data, 1);
    GPIO_SetValue(PIN_NSS,1);
    return data;
}
void LoRaHAL::WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size ){
    uint8_t address = addr& ~(1<<7);
    GPIO_SetValue(PIN_NSS,0);
    SPI_TranscieveBuffer(&address, 1);
    SPI_TranscieveBuffer(buffer, size);
    GPIO_SetValue(PIN_NSS,1);
}
void LoRaHAL::ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size ){
    uint8_t address = addr | (1<<7);
    GPIO_SetValue(PIN_NSS,0);
    SPI_TranscieveBuffer(&address, 1);
    SPI_TranscieveBuffer(buffer, size);
    GPIO_SetValue(PIN_NSS,1);
}
