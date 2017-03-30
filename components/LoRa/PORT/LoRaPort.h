/*
 * LoRaPort.h
 *
 *  Created on: Mar 29, 2017
 *      Author: emon1
 */

#ifndef COMPONENTS_LORA_PORT_LORAPORT_H_
#define COMPONENTS_LORA_PORT_LORAPORT_H_


#include <stdint.h>

class LoRaPort {
public:
	LoRaPort();
	virtual ~LoRaPort();


	typedef enum {
		PIN_NSS,PIN_RESET,PIN_TX,PIN_RX,PIN_DIO0, PIN_DIO1,PIN_DIO2,PIN_DIO3,PIN_DIO4,PIN_DIO5
	}LoRaGPIO_t;

	typedef enum {
		txTimeout, rxTimeout, rxTimeoutSyncWord
	}LoRaTimeout_t;


	void GPIO_Init(LoRaGPIO_t pin);
	void GPIO_DeInit(LoRaGPIO_t pin);
	void GPIO_SetValue(LoRaGPIO_t pin, bool value);
	bool GPIO_GetValue(LoRaGPIO_t pin);

	void SPI_Init(void);
	void SPI_TranscieveBuffer( uint8_t *dataInOut, unsigned int size );
	uint8_t SPI_TranscieveByte( uint8_t dataIn );

	void Timeout_init(LoRaTimeout_t timeout);
	void Timeout_SetTime(LoRaTimeout_t timeout, unsigned int time);


	void delayUs(unsigned int us);
	void delayMs(unsigned int ms);
};

#endif /* COMPONENTS_LORA_PORT_LORAPORT_H_ */
