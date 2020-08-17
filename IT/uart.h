/*
 * uart.h
 *
 *  Created on: Aug 14, 2020
 *      Author: user
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#define UART_BUFFER_SIZE			128
#define BIT_GET(reg, bit)			((reg) & (bit))
#define BIT_SET(reg, bit)			((reg) |= (bit))
#define BIT_WRITE(reg, bit)			((reg) = (bit))
#define BIT_READ(reg)				(reg)
#define BIT_CLEAR(reg, bit)			((reg) &= ~(bit))
#define BIT_MODIFY(reg, bit, mask)	BIT_WRITE(BIT_READ(reg), (((BIT_READ(reg)) & (~(mask))) | ((mask) & (bit))))

#define UART_LOCK(__HANDLE__)    	do{                                        \
                                    if((__HANDLE__)->lock == UART_LOCKED)   \
                                    {                                      \
                                       return UART_ERR;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->lock = UART_LOCKED;    \
                                    }                                      \
                            		}while (0U)
#define UART_UNLOCK(__HANDLE__)  	do{                                       \
                                      (__HANDLE__)->lock = UART_UNLOCKED;    \
                                    }while (0U)

#define CHECK_IT(Tout)	do {	\
							/* Do nothing */	\
							} while(ulTaskNotifyTake(pdTRUE, Tout) == 0 )

typedef enum {
	UART_OK,
	UART_ERR
} UART_State;

typedef enum {
	UART_UNLOCKED,
	UART_LOCKED
} UART_Lock;

typedef struct UART_Handle {
	USART_TypeDef 	*instance;
	uint32_t		baudRate;
	uint8_t			*buffPtr;
	uint16_t		sizeBuff;
	uint8_t			lock;

	_Bool			enableIT;
	uint16_t		currSizeBuff;
	QueueHandle_t	txQueueHandle;
	QueueHandle_t	rxQueueHandle;

} UART_Handle;

extern void debugPrint(char *);

UART_State UART_Init(UART_Handle *);
UART_State UART_Write(UART_Handle *, uint8_t *, uint16_t);
UART_State UART_Read(UART_Handle *, uint8_t *, uint32_t);
UART_State UART_Write_IT(UART_Handle *, uint8_t *, uint16_t);
UART_State UART_Read_IT(UART_Handle *, uint8_t *, TickType_t);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);

#endif /* SRC_UART_H_ */
