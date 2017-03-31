/*
 * LoRaHAL.h
 *
 *  Created on: Mar 28, 2017
 *      Author: emon1
 */

#ifndef COMPONENTS_LORA_HAL_LORAHAL_H_
#define COMPONENTS_LORA_HAL_LORAHAL_H_


#include <stdint.h>

#include "../PORT/LoRaPort.h"

class LoRaHAL : public LoRaPort {
public:
	LoRaHAL();
	virtual ~LoRaHAL();

	void antSwInit();
	void antSwDeInit();
	void antSwSet(bool txrx);
	void Reset();
    void Write(uint8_t addr, uint8_t data);
    uint8_t Read(uint8_t addr);
    void WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
    void ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );


};

#endif /* COMPONENTS_LORA_HAL_LORAHAL_H_ */
