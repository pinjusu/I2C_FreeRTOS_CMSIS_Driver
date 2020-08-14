/*
 * i2c.h
 *
 *  Created on: Aug 10, 2020
 *      Author: user
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define BIT_GET(reg, bit)			((reg) & (bit))
#define BIT_SET(reg, bit)			((reg) |= (bit))
#define BIT_WRITE(reg, bit)			((reg) = (bit))
#define BIT_READ(reg)				(reg)
#define BIT_CLEAR(reg, bit)			((reg) &= ~(bit))
#define BIT_MODIFY(reg, bit, mask)	BIT_WRITE(BIT_READ(reg), (((BIT_READ(reg)) & (~(mask))) | ((mask) & (bit))))

#define CHECK_IT(Tout)	do {	\
							/* Do nothing */	\
							} while(ulTaskNotifyTake(pdTRUE, Tout) == 0 )

typedef enum {
	I2C_OK,
	I2C_ERR

} I2C_State;

typedef enum
{
	I2C_READ,
	I2C_WRITE

} I2C_RW;

typedef enum
{
	I2C_NORMAL,
	I2C_WRITE_FIRST_READ,
	I2C_START_READ,
	I2C_READ_FIRST_AND_LAST_FRAME

} I2C_TransferOp;
typedef struct _I2C_Handle{
	I2C_TypeDef 	*instance;
	uint32_t		clockSpeed;
	uint8_t   		slaveAddr;
	uint8_t   		regAddr;
	uint8_t    		*buff;
	uint8_t   		sizeBuff;
	uint8_t			currSizeBuff;

	I2C_RW			RW;
	I2C_TransferOp	TransferOptions;
} I2C_Handle;

extern TaskHandle_t defaultTaskHandle;

/* I2C APIs */
I2C_State I2C_Init(I2C_Handle *);
I2C_State I2C_Write_IT(I2C_Handle *,uint8_t, uint8_t, uint8_t *, uint8_t);
I2C_State I2C_Read_IT(I2C_Handle *,uint8_t, uint8_t, uint8_t *, uint8_t);
void I2C1_EV_IRQHandler(void);

extern void debugPrint(char []);

#endif /* INC_I2C_H_ */
