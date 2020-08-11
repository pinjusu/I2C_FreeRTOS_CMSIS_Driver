/*
 * i2c.h
 *
 *  Created on: Aug 10, 2020
 *      Author: user
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f1xx.h"

#define BIT_GET(reg, bit)			((reg) & (bit))
#define BIT_SET(reg, bit)			((reg) |= (bit))
#define BIT_WRITE(reg, bit)			((reg) = (bit))
#define BIT_READ(reg)				(reg)
#define BIT_CLEAR(reg, bit)			((reg) &= ~(bit))
#define BIT_MODIFY(reg, bit, mask)	BIT_WRITE(BIT_READ(reg), (((BIT_READ(reg)) & (~(mask))) | ((mask) & (bit))))

typedef enum {
	I2C_OK,
	I2C_ERR
} I2C_State;

typedef struct {
	I2C_TypeDef *instance;
	uint32_t	clockSpeed;
	uint8_t   	slaveAddr;
	uint8_t   	regAddr;
	uint8_t    	*buff;
	uint8_t   	sizeBuff;
} I2C_Handle;

/* I2C APIs */
I2C_State I2C_Init(I2C_Handle *);
I2C_State I2C_Write(I2C_Handle *, uint8_t, uint8_t, uint8_t *, uint8_t, _Bool);
I2C_State I2C_Read(I2C_Handle *, uint8_t, uint8_t, uint8_t *, uint8_t);

extern void debugPrint(char []);

#endif /* INC_I2C_H_ */
