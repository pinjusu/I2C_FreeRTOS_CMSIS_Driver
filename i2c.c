/*
 * i2c.c
 *
 *  Created on: Aug 10, 2020
 *      Author: Joe
 */
#include "i2c.h"

void I2CGPIOinit(void){
	/* GPIO Init
	 * PB6 ------> I2C1_SCL
	 * PB7 ------> I2C1_SDA
	*/
	// Enable IO Port B clock
	BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);

	/* CRL
	 * CNFy[1:0]=11 (open drain,50MHz)
	 * MODEy[1:0]=11 (output mode)
	 */
	BIT_SET(GPIOB->CRL, GPIO_CRL_CNF6|GPIO_CRL_MODE6);
	BIT_SET(GPIOB->CRL, GPIO_CRL_CNF7|GPIO_CRL_MODE7);
}

/* I2C Initial for 400KHz Fast Mode */
I2C_State I2C_Init(I2C_Handle *hi2c) {
	uint32_t PCLK1;
	uint32_t i2c_clk_freq;
	uint32_t i2c_trise;
	uint32_t i2c_ccr;

	if (!hi2c)
		return I2C_ERR;

	I2CGPIOinit();

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

//void i2c_read;
//
//void i2c_write;
