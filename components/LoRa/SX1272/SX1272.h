/*
 * SX1272.h
 *
 *  Created on: Mar 29, 2017
 *      Author: emon1
 */

#ifndef COMPONENTS_LORA_SX1272_SX1272_H_
#define COMPONENTS_LORA_SX1272_SX1272_H_


#include "SX1272Driver.h"

class SX1272 : public SX1272Driver {
public:
	SX1272();
	virtual ~SX1272();
};

#endif /* COMPONENTS_LORA_SX1272_SX1272_H_ */
