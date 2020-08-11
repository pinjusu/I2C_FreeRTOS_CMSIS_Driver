/*
 * i2c.h
 *
 *  Created on: Aug 10, 2020
 *      Author: user
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f1xx.h"

#define BIT_SET(reg, bit)			((reg) |= (bit))
#define BIT_CLEAR(reg, bit)			((reg) &= ~(bit))
#define BIT_MODIFY(reg, bit, mask)	((reg) |= ((bit) & (mask)))

typedef enum {
	I2C_OK,
	I2C_ERR
} I2C_State;

typedef struct {
	I2C_TypeDef *instance;
	uint32_t	clockSpeed;
} I2C_Handle;

/* I2C APIs */
I2C_State I2C_Init(I2C_Handle *);
//void i2c_read;
//void i2c_write;

#endif /* INC_I2C_H_ */
