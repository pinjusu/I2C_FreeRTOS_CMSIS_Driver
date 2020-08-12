/*
 * i2c.c
 *
 *  Created on: Aug 10, 2020
 *      Author: Joe, Ruby
 */
#include "i2c.h"

/* I2C Initial for 400KHz Fast Mode */
I2C_State I2C_Init(I2C_Handle *hi2c) {
	uint32_t PCLK1;
	uint32_t i2c_clk_freq;
	uint32_t i2c_trise;
	uint32_t i2c_ccr;

	if (!hi2c)
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

	/* CR1
	 * bit 0: 0 (Disable the selected I2C peripheral.)
	 */
	BIT_CLEAR(hi2c->instance->CR1, I2C_CR1_PE);

	// Set I2C clock freq via APB1/PCLK1, SystemCoreClock = 8MHz
	PCLK1 = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
	//Unit in MHz
	i2c_clk_freq = PCLK1/1000000U;

	/* CR2
	 * bit 5-0: 8.(I2C Peripheral clock frequency).
	 */
	BIT_MODIFY(hi2c->instance->CR2, i2c_clk_freq, I2C_CR2_FREQ);
	hi2c->instance->CR2 = i2c_clk_freq;

	/*TRISE
	 * bit 5-0: 9(the rising time for SCL)
	 * TRISE = (rise time in seconds / PCLK1 in seconds) + 1.
     * Max allowed TRISE for standard mode is 1000ns. 1000ns/(1/8M) + 1 = 8 + 1 = 9, 8M Hz is PCLK1.
     * Max allowed TRISE for Fast Mode is 300ns.
	 */
	i2c_trise = 0.3 * i2c_clk_freq + 1;
	BIT_MODIFY(hi2c->instance->TRISE, i2c_trise, I2C_TRISE_TRISE);

	/*CCR
	 * bit 15   : 1(Fm)
	 * bit 14   : 0(only for Fm. 0:Tlow/Thigh=2 1:Tlow/Thigh=16/9)
     * bit 11-0 : 6(The SCL clock for master.)
	 * For Sm, Thigh=Tlow=CCR*Tpclk1=CCR*125ns.
	 * The frequency of I2C is 100kHz, 1/(Thigh+Tlow) = 100K = 1/(250n*CCR) => CCR = 1/(250n*100k) = 40.
	 * For Fm, Thigh=CCR*Tpclk1, Tlow=2*CCR*Tpclk1.
	 */
	i2c_ccr = ( (PCLK1 -1) / ( hi2c->clockSpeed * 3) + 1 );
	BIT_MODIFY(hi2c->instance->CCR, i2c_ccr | I2C_CCR_FS, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR));

	/*CR1
	 * bit 7: 0(Clock stretching enable,only for slave mode)
	 * bit 6: 0(General call disable)
	 */
	BIT_CLEAR(hi2c->instance->CR1,I2C_CR1_ENGC);
	BIT_CLEAR(hi2c->instance->CR1,I2C_CR1_NOSTRETCH);

	/*ORA1
	 * bit 15  : 0(ADDMode, 0 is 7-bit address for slave mode)
	 * bit 14  : 1(must be)
	 * bit 9-8 : NONE.(bit 9-8 for 10-bit's address)
	 * bit 7-1 : 0 (bit 7-1 of address.)
	 * bit 0   : NONE. (bit 0 for 10-bit's address)
	 */
	BIT_CLEAR(hi2c->instance->OAR1, I2C_OAR1_ADDMODE);
	BIT_SET(hi2c->instance->OAR1, 1 << 14);
	BIT_MODIFY(hi2c->instance->OAR1, 0, I2C_OAR1_ADD1_7);

	/*ORA2
	 * bit 7-1 : NONE(the dual address).
	 * bit 0   : 0(disable dual addressing mode)
	 */
	BIT_CLEAR(hi2c->instance->OAR2,I2C_OAR2_ENDUAL);

	/* CR1
	 * bit 0: 1 (Peripheral enable.)
	 */
	BIT_SET(hi2c->instance->CR1, I2C_CR1_PE);

	return I2C_OK;
}

I2C_State I2C_Write(I2C_Handle *hi2c, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data, uint8_t dataSize, _Bool isRead) {
	uint32_t tmp;

	if (!hi2c)
		return I2C_ERR;

	for (int i=0; i<6; i++) {
		if (!BIT_GET(hi2c->instance->SR2, I2C_SR2_BUSY))
			break;

		if (i == 5) {
			return I2C_ERR;
		}
	}

	hi2c->slaveAddr = slaveAddr << 1;
	hi2c->regAddr = regAddr;
	hi2c->sizeBuff = dataSize;

	for (int i=0; i<dataSize; i++) {
		hi2c->buff[i] = data[i];
	}

	//Generate a start condition and turn on ACKs
	BIT_SET(I2C1->CR1, I2C_CR1_START | I2C_CR1_ACK);

	while (!BIT_GET(hi2c->instance->SR1, I2C_SR1_SB));
	hi2c->instance->DR = hi2c->slaveAddr;

	while (!BIT_GET(hi2c->instance->SR1, I2C_SR1_ADDR));
	tmp = hi2c->instance->SR2;
	UNUSED(tmp);
	hi2c->instance->DR = hi2c->regAddr;

	while (!BIT_GET(hi2c->instance->SR1, I2C_SR1_TXE));

	for (int i=0; i<hi2c->sizeBuff; i++) {
		hi2c->instance->DR = hi2c->buff[i];
		while (!BIT_GET(hi2c->instance->SR1, I2C_SR1_TXE));
	}

	if (!isRead)
		BIT_SET(hi2c->instance->CR1, I2C_CR1_STOP);

	return I2C_OK;
}

I2C_State I2C_Read(I2C_Handle *hi2c, uint8_t slaveAddr, uint8_t regAddr, uint8_t *data, uint8_t dataSize) {
	uint32_t tmp;

	if (!hi2c || dataSize <= 0)
		return I2C_ERR;

	if (I2C_Write(hi2c, slaveAddr, regAddr, NULL, 0, 1) == I2C_ERR)
		return I2C_ERR;

	hi2c->slaveAddr = (slaveAddr << 1) + 1;
	hi2c->regAddr = regAddr;
	hi2c->sizeBuff = dataSize;
	hi2c->buff = data;

	//Generate a start condition and turn on ACKs
	BIT_SET(hi2c->instance->CR1, I2C_CR1_START | I2C_CR1_ACK);

	while (!BIT_GET(hi2c->instance->SR1, I2C_SR1_SB));
	hi2c->instance->DR = hi2c->slaveAddr;

	while (!BIT_GET(hi2c->instance->SR1, I2C_SR1_ADDR));
	tmp = hi2c->instance->SR2;
	UNUSED(tmp);

	if (hi2c->sizeBuff == 1) {
		BIT_CLEAR(hi2c->instance->CR1, I2C_CR1_ACK);
		BIT_SET(hi2c->instance->CR1, I2C_CR1_STOP);
	}

	for (int i=0; i<hi2c->sizeBuff; i++) {
		while (!BIT_GET(hi2c->instance->SR1, I2C_SR1_RXNE));
		hi2c->buff[i] = hi2c->instance->DR;

		if (i == hi2c->sizeBuff - 2) {
			BIT_CLEAR(hi2c->instance->CR1, I2C_CR1_ACK);
			BIT_SET(hi2c->instance->CR1, I2C_CR1_STOP);
		}
	}

	return I2C_OK;
}






