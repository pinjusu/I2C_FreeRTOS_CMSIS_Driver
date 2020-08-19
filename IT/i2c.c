/*
 * i2c.c
 *
 *  Created on: Aug 10, 2020
 *      Author: Joe, Ruby
 */
#include "i2c.h"
#include "main.h"

extern I2C_Handle hi2c1;
I2C_Handle *hi2c1Ptr = &hi2c1;

extern void JS_I2C1_Init(void);


/* I2C Initial for 400KHz Fast Mode */
I2C_State I2C_Init(I2C_Handle *hi2cPtr, _Bool enableIT) {
	uint32_t PCLK1;
	uint32_t i2c_clk_freq;
	uint32_t i2c_trise;
	uint32_t i2c_ccr;


	if (!hi2cPtr)
		return I2C_ERR;

	/* GPIO Init
	 * PB6 ------> I2C1_SCL
	 * PB7 ------> I2C1_SDA
	*/
	// Enable IO Port B and I2C1 clocks
	BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
	BIT_SET(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

	/* CRL
	 * CNFy[1:0]=11 (open drain,50MHz)
	 * MODEy[1:0]=11 (output mode)
	 */
	BIT_SET(GPIOB->CRL, GPIO_CRL_CNF6|GPIO_CRL_MODE6);
	BIT_SET(GPIOB->CRL, GPIO_CRL_CNF7|GPIO_CRL_MODE7);

	if (enableIT) {
	/* Interrupt Init*/
	/* Sets the priority of a device specific interrupt or a processor exception.
	 * IRQn: Interrupt number, you can check the IRQn in stm32f103xb.h .
	 * priority: 0-15, 0: highest priority.
	 */
		NVIC_SetPriority(I2C1_EV_IRQn, 5);
		NVIC_EnableIRQ(I2C1_EV_IRQn);

	//Enable interrupts, not really necessary since this is the default value.
		__enable_irq();
	}

	/* CR1
	 * bit 0: 0 (Disable the selected I2C peripheral.)
	 */
	BIT_CLEAR(hi2cPtr->instance->CR1, I2C_CR1_PE);

	// Set I2C clock freq via APB1/PCLK1, SystemCoreClock = 8MHz
	PCLK1 = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
	//Unit in MHz
	i2c_clk_freq = PCLK1/1000000U;

	/* CR2
	 * bit 5-0: 8.(I2C Peripheral clock frequency).
	 */
	BIT_MODIFY(hi2cPtr->instance->CR2, i2c_clk_freq, I2C_CR2_FREQ);


	/*TRISE
	 * bit 5-0: 9(the rising time for SCL)
	 * TRISE = (rise time in seconds / PCLK1 in seconds) + 1.
     * Max allowed TRISE for standard mode is 1000ns. 1000ns/(1/8M) + 1 = 8 + 1 = 9, 8M Hz is PCLK1.
     * Max allowed TRISE for Fast Mode is 300ns.
	 */
	i2c_trise = 0.3 * i2c_clk_freq + 1;
	BIT_MODIFY(hi2cPtr->instance->TRISE, i2c_trise, I2C_TRISE_TRISE);

	/*CCR
	 * bit 15   : 1(Fm)
	 * bit 14   : 0(only for Fm. 0:Tlow/Thigh=2 1:Tlow/Thigh=16/9)
     * bit 11-0 : 6(The SCL clock for master.)
	 * For Sm, Thigh=Tlow=CCR*Tpclk1=CCR*125ns.
	 * The frequency of I2C is 100kHz, 1/(Thigh+Tlow) = 100K = 1/(250n*CCR) => CCR = 1/(250n*100k) = 40.
	 * For Fm, Thigh=CCR*Tpclk1, Tlow=2*CCR*Tpclk1.
	 */
	i2c_ccr = ( (PCLK1 -1) / ( hi2cPtr->clockSpeed * 3) + 1 );
	BIT_MODIFY(hi2cPtr->instance->CCR, i2c_ccr | I2C_CCR_FS,
			(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR));

	/*CR1
	 * bit 7: 0(Clock stretching enable,only for slave mode)
	 * bit 6: 0(General call disable)
	 */
	BIT_CLEAR(hi2cPtr->instance->CR1, I2C_CR1_ENGC);
	BIT_CLEAR(hi2cPtr->instance->CR1, I2C_CR1_NOSTRETCH);

	/*ORA1
	 * bit 15  : 0(ADDMode, 0 is 7-bit address for slave mode)
	 * bit 14  : 1(must be)
	 * bit 9-8 : NONE.(bit 9-8 for 10-bit's address)
	 * bit 7-1 : 0 (bit 7-1 of address.)
	 * bit 0   : NONE. (bit 0 for 10-bit's address)
	 */
	BIT_CLEAR(hi2cPtr->instance->OAR1, I2C_OAR1_ADDMODE);
	BIT_SET(hi2cPtr->instance->OAR1, 1 << 14);
	BIT_MODIFY(hi2cPtr->instance->OAR1, 0, I2C_OAR1_ADD1_7);

	/*ORA2
	 * bit 7-1 : NONE(the dual address).
	 * bit 0   : 0(disable dual addressing mode)
	 */
	BIT_CLEAR(hi2cPtr->instance->OAR2, I2C_OAR2_ENDUAL);

	/* CR1
	 * bit 0: 1 (Peripheral enable.)
	 */
	BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_PE);

	return I2C_OK;
}

I2C_State I2C_Write(I2C_Handle *hi2cPtr, uint8_t slaveAddr, uint8_t regAddr,
		uint8_t *data, uint8_t dataSize, _Bool isRead) {
	uint32_t tmp;

	if (!hi2cPtr)
		return I2C_ERR;

	for (int i=0; i<6; i++) {
		if (!BIT_GET(hi2cPtr->instance->SR2, I2C_SR2_BUSY))
			break;

		if (i == 5) {
			debugPrint("[ERR] I2C busy...\r\n");
			return I2C_ERR;
		}
	}

	I2C_LOCK(hi2cPtr);

	hi2cPtr->slaveAddr = slaveAddr << 1;
	hi2cPtr->regAddr = regAddr;
	hi2cPtr->sizeBuff = dataSize;
	hi2cPtr->regAddrEvIsSend = 0;

	for (int i=0; i<dataSize; i++) {
		hi2cPtr->buffPtr[i] = data[i];
	}

	//Generate a start condition and turn on ACKs
	BIT_SET(I2C1->CR1, I2C_CR1_START | I2C_CR1_ACK);

	while (!BIT_GET(hi2cPtr->instance->SR1, I2C_SR1_SB));
	hi2cPtr->instance->DR = hi2cPtr->slaveAddr;

	while (!BIT_GET(hi2cPtr->instance->SR1, I2C_SR1_ADDR));
	tmp = hi2cPtr->instance->SR2;
	UNUSED(tmp);
	hi2cPtr->instance->DR = hi2cPtr->regAddr;

	while (!BIT_GET(hi2cPtr->instance->SR1, I2C_SR1_TXE));

	for (int i=0; i<hi2cPtr->sizeBuff; i++) {
		hi2cPtr->instance->DR = hi2cPtr->buffPtr[i];
		while (!BIT_GET(hi2cPtr->instance->SR1, I2C_SR1_TXE));
	}

	if (!isRead)
		BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_STOP);

	I2C_UNLOCK(hi2cPtr);

	return I2C_OK;
}

I2C_State I2C_Read(I2C_Handle *hi2cPtr, uint8_t slaveAddr, uint8_t regAddr,
		uint8_t *data, uint8_t dataSize) {
	uint32_t tmp;
	if (!hi2cPtr || dataSize <= 0)
		return I2C_ERR;

	if (I2C_Write(hi2cPtr, slaveAddr, regAddr, NULL, 0, 1) == I2C_ERR) {
		debugPrint("[ERR] I2C Write failed\r\n");
		return I2C_ERR;
	}

	I2C_LOCK(hi2cPtr);

	hi2cPtr->slaveAddr = (slaveAddr << 1) + 1;
	hi2cPtr->regAddr = regAddr;
	hi2cPtr->sizeBuff = dataSize;
	hi2cPtr->buffPtr = data;
	hi2cPtr->regAddrEvIsSend = 0;

	//Generate a start condition and turn on ACKs
	BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_START | I2C_CR1_ACK);

	while (!BIT_GET(hi2cPtr->instance->SR1, I2C_SR1_SB));
	hi2cPtr->instance->DR = hi2cPtr->slaveAddr;

	while (!BIT_GET(hi2cPtr->instance->SR1, I2C_SR1_ADDR));
	tmp = hi2cPtr->instance->SR2;
	UNUSED(tmp);

	if (hi2cPtr->sizeBuff == 1) {
		BIT_CLEAR(hi2cPtr->instance->CR1, I2C_CR1_ACK);
		BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_STOP);
	}

	for (int i=0; i<hi2cPtr->sizeBuff; i++) {
		while (!BIT_GET(hi2cPtr->instance->SR1, I2C_SR1_RXNE));
		hi2cPtr->buffPtr[i] = hi2cPtr->instance->DR;

		if (i == hi2cPtr->sizeBuff - 2) {
			BIT_CLEAR(hi2cPtr->instance->CR1, I2C_CR1_ACK);
			BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_STOP);
		}
	}

	I2C_UNLOCK(hi2cPtr);

	return I2C_OK;
}

I2C_State I2C_Write_IT(I2C_Handle *hi2cPtr, uint8_t slaveAddr,
		uint8_t regAddr, uint8_t *data, uint8_t dataSize) {
	if (!hi2cPtr)
		return I2C_ERR;

	for (int i=0; i<6; i++) {
		if (!BIT_GET(hi2cPtr->instance->SR2, I2C_SR2_BUSY))
			break;

		if (i == 5) {
			debugPrint("[ERR] I2C busy...\r\n");
			/* Busy handle, reset the I2C*/
			BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_SWRST);
			BIT_CLEAR(hi2cPtr->instance->CR1, I2C_CR1_SWRST);
			JS_I2C1_Init();
			return I2C_ERR;
		}
	}

	I2C_LOCK(hi2cPtr);

	hi2cPtr->slaveAddr = slaveAddr << 1;
	hi2cPtr->regAddr = regAddr;
	hi2cPtr->buffPtr = data;
	hi2cPtr->sizeBuff = dataSize;
	hi2cPtr->currSizeBuff = 0U;
	hi2cPtr->Mode = I2C_WRITE;
	hi2cPtr->TransferOptions = I2C_NORMAL;
	hi2cPtr->regAddrEvIsSend = 0;

	//Generate a start condition and turn on ACKs
	BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_START | I2C_CR1_ACK);

	I2C_UNLOCK(hi2cPtr);

	/* CR2
	 * bit 10: 1(Buffer interrupt enable. When TxE=1 or RxNE=1, interrupt happen.)
	 * bit 9: 1(Event interrupt enable. when  SB = 1, ADDR = 1, STOPF = 1 and BTF = 1, interrupt happen.)
	 */
	BIT_SET(hi2cPtr->instance->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN |I2C_CR2_ITERREN );

	return I2C_OK;
}

I2C_State I2C_Read_IT(I2C_Handle *hi2cPtr, uint8_t slaveAddr,
		uint8_t regAddr, uint8_t *data, uint8_t dataSize) {
	if (!hi2cPtr || dataSize <= 0)
		return I2C_ERR;

	for (int i=0; i<6; i++) {
		if (!BIT_GET(hi2cPtr->instance->SR2, I2C_SR2_BUSY))
			break;

		if (i == 5) {
			debugPrint("[ERR] I2C busy...\r\n");
			/* Busy handle, reset the I2C*/
			BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_SWRST);
			BIT_CLEAR(hi2cPtr->instance->CR1, I2C_CR1_SWRST);
			JS_I2C1_Init();
			return I2C_ERR;
		}
	}

	I2C_LOCK(hi2cPtr);

	hi2cPtr->slaveAddr = slaveAddr << 1 ;
	hi2cPtr->regAddr = regAddr;
	hi2cPtr->buffPtr = data;
	hi2cPtr->sizeBuff = dataSize;
	hi2cPtr->currSizeBuff = 0U;
	hi2cPtr->Mode = I2C_READ;
	hi2cPtr->TransferOptions = I2C_WRITE_FIRST_READ;
	hi2cPtr->regAddrEvIsSend = 0;

	//Generate a start condition and turn on ACKs
	BIT_SET(hi2cPtr->instance->CR1, I2C_CR1_START | I2C_CR1_ACK);

	I2C_UNLOCK(hi2cPtr);

	/* CR2
	 * bit 10: 1(Buffer interrupt enable. When TxE=1 or RxNE=1, interrupt happen.)
	 * bit 9: 1(Event interrupt enable. when  SB = 1, ADDR = 1, STOPF = 1 and BTF = 1, interrupt happen.)
	 */
	BIT_SET(hi2cPtr->instance->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN |I2C_CR2_ITERREN );

	return I2C_OK;
}

void I2C1_EV_IRQHandler(void) {
	uint32_t tmp;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (BIT_GET(hi2c1Ptr->instance->SR1, I2C_SR1_SB)) {
		/* read start */
		if (hi2c1Ptr->TransferOptions == I2C_START_READ) {
			if(hi2c1Ptr->sizeBuff == 1)
				hi2c1Ptr->TransferOptions = I2C_READ_FIRST_AND_LAST_FRAME;
			else
				hi2c1Ptr->TransferOptions = I2C_NORMAL;

			hi2c1Ptr->instance->DR = hi2c1Ptr->slaveAddr + 1;
		} else
			hi2c1Ptr->instance->DR = hi2c1Ptr->slaveAddr;

	} else if (BIT_GET(hi2c1Ptr->instance->SR1, I2C_SR1_ADDR)) {
		tmp = hi2c1Ptr->instance->SR1;
		tmp = hi2c1Ptr->instance->SR2;
		UNUSED(tmp);

		if (hi2c1Ptr->TransferOptions == I2C_READ_FIRST_AND_LAST_FRAME){
			BIT_CLEAR(hi2c1Ptr->instance->CR1, I2C_CR1_ACK);
			BIT_SET(hi2c1Ptr->instance->CR1, I2C_CR1_STOP);
		}


	} else if (BIT_GET(hi2c1Ptr->instance->SR1, I2C_SR1_TXE) ||
			(BIT_GET(hi2c1Ptr->instance->SR1, I2C_SR1_BTF) && BIT_GET(hi2c1Ptr->instance->SR2, I2C_SR2_TRA)) ) {
		/* Send the addr of register. */
		if(!hi2c1Ptr->regAddrEvIsSend){
			hi2c1Ptr->instance->DR = hi2c1Ptr->regAddr;
			hi2c1Ptr->regAddrEvIsSend = 1;

		/* I2C write mode */
		} else if (hi2c1Ptr->Mode == I2C_WRITE) {
			/* the last output byte. */
			if (hi2c1Ptr->currSizeBuff >= hi2c1Ptr->sizeBuff) {
				BIT_SET(hi2c1Ptr->instance->CR1, I2C_CR1_STOP);
				BIT_CLEAR(hi2c1Ptr->instance->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN |I2C_CR2_ITERREN);

				/* notify the task at main. */
				vTaskNotifyGiveFromISR(IMU_TaskHandle, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			} else
				hi2c1Ptr->instance->DR = hi2c1Ptr->buffPtr[hi2c1Ptr->currSizeBuff++];

		/* I2C read mode */
		} else if(hi2c1Ptr->TransferOptions == I2C_WRITE_FIRST_READ) {
				hi2c1Ptr->TransferOptions = I2C_START_READ;
				BIT_SET(hi2c1Ptr->instance->CR1, I2C_CR1_START | I2C_CR1_ACK);
		}

	} else if (BIT_GET(hi2c1Ptr->instance->SR1, I2C_SR1_RXNE) || (BIT_GET(hi2c1Ptr->instance->SR1, I2C_SR1_BTF)) ){
		hi2c1Ptr->buffPtr[hi2c1Ptr->currSizeBuff++] = hi2c1Ptr->instance->DR;
		/* The last two bytes data. */
		if (hi2c1Ptr->currSizeBuff == hi2c1Ptr->sizeBuff - 1) {
			BIT_CLEAR(hi2c1Ptr->instance->CR1, I2C_CR1_ACK);
			BIT_SET(hi2c1Ptr->instance->CR1, I2C_CR1_STOP);

		/* The last one byte data. */
		} else if (hi2c1Ptr->currSizeBuff >= hi2c1Ptr->sizeBuff) {
			BIT_CLEAR(hi2c1Ptr->instance->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN |I2C_CR2_ITERREN );

			/* notify the task at main. */
			vTaskNotifyGiveFromISR(IMU_TaskHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
	}
}




