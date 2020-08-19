/*
 * uart.c
 *
 *  Created on: Aug 14, 2020
 *      Author: user
 */
#include "uart.h"

extern UART_Handle huart1;
UART_Handle *huart1Ptr = &huart1;
extern UART_Handle huart2;
UART_Handle *huart2Ptr = &huart2;


static StaticQueue_t rxQueue, txQueue;
static uint8_t rxQueueBuff[UART_BUFFER_SIZE], txQueueBuff[UART_BUFFER_SIZE];


static void GPIO_Init(UART_Handle *uartPtr) {
   if (!uartPtr)
      return;

   /* PA9     ------> USART1_TX
    * PA10    ------> USART1_RX
    *
    * PA2     ------> USART2_TX
    * PA3     ------> USART2_RX
    * */
   if (uartPtr->instance == USART1) {
      // Enable IO Port A and UART2 clocks
      BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
      BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);

      /* CRL: PA2
       * CNFy[1:0]=10 (push-pull,50MHz)
       * MODEy[1:0]=11 (output mode)
       * CRL: PA3
       * MODEy[1:0]=00 (input mode)
       */
      BIT_CLEAR(GPIOA->CRH, GPIO_CRH_CNF9);
      BIT_SET(GPIOA->CRH, GPIO_CRH_CNF9_1|GPIO_CRH_MODE9);
      BIT_CLEAR(GPIOA->CRH, GPIO_CRH_MODE10);

   } else if (uartPtr->instance == USART2) {
      // Enable IO Port A and UART2 clocks
      BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
      BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);

      /* CRL: PA2
       * CNFy[1:0]=10 (push-pull,50MHz)
       * MODEy[1:0]=11 (output mode)
       * CRL: PA3
       * MODEy[1:0]=00 (input mode)
       */
      BIT_CLEAR(GPIOA->CRL, GPIO_CRL_CNF2);
      BIT_SET(GPIOA->CRL, GPIO_CRL_CNF2_1|GPIO_CRL_MODE2);
      BIT_CLEAR(GPIOA->CRL, GPIO_CRL_MODE3);
   }
}

static void NVIC_Init(UART_Handle *uartPtr) {
   if (!uartPtr)
      return;

   if (uartPtr->instance == USART1) {
      NVIC_SetPriority(USART1_IRQn, 6);
      NVIC_EnableIRQ(USART1_IRQn);

   } else if (uartPtr->instance == USART2) {
      NVIC_SetPriority(USART2_IRQn, 6);
      NVIC_EnableIRQ(USART2_IRQn);
   }
}

UART_State UART_Init(UART_Handle *uartPtr) {
	uint32_t PCLK1;
	uint32_t mantissa_final, fractional_final;
	float usartdiv, fractional;

	if (!uartPtr)
			return UART_ERR;

	GPIO_Init(uartPtr);

	/* Disable UART */
	BIT_CLEAR(uartPtr->instance->CR1, USART_CR1_UE);

	// Stop bit = 0, ( = 1 bit length )
	BIT_CLEAR(uartPtr->instance->CR2, USART_CR2_STOP);

	// Disable parity check
	BIT_CLEAR(uartPtr->instance->CR1, USART_CR1_PCE);

	// Enable TE and RE
	BIT_SET(uartPtr->instance->CR1, USART_CR1_TE | USART_CR1_RE);

	// Write CR3 to 0
	BIT_WRITE(uartPtr->instance->CR3, 0);

	PCLK1 = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);

	usartdiv = (float) PCLK1/(uartPtr->baudRate * 16);

	//Get fractional part of USARTDIV
	fractional = usartdiv - (long)usartdiv;

	//Get whole part of USARTDIV
	mantissa_final = usartdiv - fractional;

	//Get fractional part by multiplying by 16 and rounding up. (fractional*16)+0.5 will never be higher than 16.499(9)
	//meaning that fractonal_final will never be higher than 16!
	fractional_final = (uint32_t) ((fractional * 16) + 0.5);

	//If the fraction is bigger than 4bits (i.e. it's 0d16), carry 1 to the mantissa and subtract 0d15 or 0xF to the fractional
	if (fractional_final > 0xF) {
		fractional_final = fractional_final &  0x0FU;
		mantissa_final++;
	}
	BIT_SET(uartPtr->instance->BRR, (mantissa_final << 4) | fractional_final);

	/* Interrupt UART Enable Procedure */
	if (uartPtr->enableIT) {
		NVIC_Init(uartPtr);

		uartPtr->txQueueHandle = xQueueCreateStatic(
				UART_BUFFER_SIZE, sizeof(uint8_t), txQueueBuff, &txQueue);
		uartPtr->rxQueueHandle = xQueueCreateStatic(
				UART_BUFFER_SIZE, sizeof(uint8_t), rxQueueBuff, &rxQueue);

		if (!uartPtr->txQueueHandle || !uartPtr->rxQueueHandle)
			return UART_ERR;


		BIT_SET(uartPtr->instance->CR1, USART_CR1_RXNEIE);
	}

	/* Enable UART */
	BIT_SET(uartPtr->instance->CR1, USART_CR1_UE);

	return UART_OK;
}

UART_State UART_Write(UART_Handle *uartPtr, uint8_t *dataPtr, uint16_t sizeData) {
	if (!uartPtr || !dataPtr || !sizeData)
		return UART_ERR;

	UART_LOCK(uartPtr);

	uartPtr->buffPtr = dataPtr;
	uartPtr->sizeBuff = sizeData;

	while (uartPtr->sizeBuff--)
	{
		while(BIT_GET(uartPtr->instance->SR, USART_SR_TXE) == 0);
		uartPtr->instance->DR = (*(uartPtr->buffPtr)++ & (uint8_t)0xFF);
	}

	while(BIT_GET(uartPtr->instance->SR, USART_SR_TC) == 0);

	UART_UNLOCK(uartPtr);

	return UART_OK;
}

UART_State UART_Read(UART_Handle *uartPtr, uint8_t *dataPtr, uint32_t sizeData) {
	if (!uartPtr || !dataPtr || !sizeData)
			return UART_ERR;

	UART_LOCK(uartPtr);

	uartPtr->buffPtr = dataPtr;
	uartPtr->sizeBuff = sizeData;

	while (uartPtr->sizeBuff--) {
		while(BIT_GET(uartPtr->instance->SR, USART_SR_RXNE) == 0);
		*(uartPtr->buffPtr)++ = (uint8_t)(uartPtr->instance->DR & (uint8_t)0xFF);
	}

	UART_UNLOCK(uartPtr);

	return UART_OK;
}


UART_State UART_Write_IT(UART_Handle *uartPtr, uint8_t *dataPtr, uint16_t sizeData) {

	if (!uartPtr || !dataPtr || !sizeData)
			return UART_ERR;

	UART_LOCK(uartPtr);

	uartPtr->buffPtr = dataPtr;
	uartPtr->sizeBuff = sizeData;
	uartPtr->currSizeBuff = 0;

	for (int i=0; i<uartPtr->sizeBuff; i++) {
		xQueueSend(uartPtr->txQueueHandle, uartPtr->buffPtr + i, portMAX_DELAY);
		BIT_SET(uartPtr->instance->CR1, USART_CR1_TXEIE);
	}


	UART_UNLOCK(uartPtr);

	return UART_OK;
}

UART_State UART_Read_IT(UART_Handle *uartPtr, uint8_t *dataPtr, TickType_t delay) {
	if (!uartPtr || !dataPtr)
			return UART_ERR;

	UART_LOCK(uartPtr);

	if (xQueueReceive(uartPtr->rxQueueHandle, dataPtr, delay) == pdFALSE) {
		UART_UNLOCK(uartPtr);
		return UART_ERR;
	}

	UART_UNLOCK(uartPtr);

	return UART_OK;
}

void USART1_IRQHandler(void) {
   BaseType_t rxWakeup = pdFALSE;
   BaseType_t txWakeup = pdFALSE;
   uint8_t tmp;

   if (BIT_GET(huart1Ptr->instance->SR, USART_SR_RXNE)) {
      tmp = huart1Ptr->instance->DR;
      xQueueSendFromISR(huart1Ptr->rxQueueHandle, &tmp, &rxWakeup);
   }

   if (BIT_GET(huart1Ptr->instance->SR, USART_SR_TXE)) {
      if (xQueueReceiveFromISR(huart1Ptr->txQueueHandle, &tmp, &txWakeup))
         huart1Ptr->instance->DR = tmp;
      else
         BIT_CLEAR(huart1Ptr->instance->CR1, USART_CR1_TXEIE);
   }

   if (rxWakeup || txWakeup)
	   portYIELD_FROM_ISR(rxWakeup || txWakeup);
}

void USART2_IRQHandler(void) {
	BaseType_t rxWakeup = pdFALSE;
	BaseType_t txWakeup = pdFALSE;
	uint8_t tmp;

	if (BIT_GET(huart2Ptr->instance->SR, USART_SR_RXNE)) {
		tmp = huart2Ptr->instance->DR;
		xQueueSendFromISR(huart2Ptr->rxQueueHandle, &tmp, &rxWakeup);
	}

	if (BIT_GET(huart2Ptr->instance->SR, USART_SR_TXE)) {
		if (xQueueReceiveFromISR(huart2Ptr->txQueueHandle, &tmp, &txWakeup))
			huart2Ptr->instance->DR = tmp;
		else
			BIT_CLEAR(huart2Ptr->instance->CR1, USART_CR1_TXEIE);
	}

	 if (rxWakeup || txWakeup)
		 portYIELD_FROM_ISR(rxWakeup || txWakeup);
}




