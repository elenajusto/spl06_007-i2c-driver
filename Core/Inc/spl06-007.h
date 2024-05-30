/*
 * spl06-007.h
 *
 *	SPL06-007 Air Pressure Sensor I2C Driver
 *
 *  Created on: May 26, 2024
 *      Author: elena
 */

#ifndef INC_SPL06_007_H_
#define INC_SPL06_007_H_

#include "stm32g0xx_hal.h"				/* For I2C */

/*
 * DEFINES
 */
#define SPL06007_I2C_ADDR		(0x76 << 1)	/* SDO -> GND (p.9) */
#define SPL06007_I2C_ID_ADDR	0x0D		/* Product and Revision ID address (p.27) */
#define SPL06007_I2C_REV_ID		0x10		/* Product and Revision ID reset value (p.27) */

/*
 * REGISTERS (p.17)
 */

/* Air Pressure Registers */
#define SPL06_REG_PSR_B2_ADDR	0x00	/* highest byte of the three bytes measured pressure value */
#define SPL06_REG_PSR_B1_ADDR	0x01	/* middle byte of the three bytes measured pressure value */
#define SPL06_REG_PSR_B0_ADDR	0x02	/* lowest byte of the three bytes measured pressure value */

/* Temperature Registers */
#define SPL06_REG_TMP_B2_ADDR	0x03	/* highest byte of the three bytes measured temperature value */
#define SPL06_REG_TMP_B1_ADDR	0x04	/* middle byte of the three bytes measured temperature value */
#define SPL06_REG_TMP_B0_ADDR	0x05	/* lowest part of the three bytes measured temperature value */

/* Pressure Configuration Register */
#define SPL06_REG_PRS_CFG_ADDR	0x06

/* Temperature Configuration Register */
#define SPL06_REG_TMP_CFG_ADDR	0x07

/* Sensor Operating Mode and Status Register */
#define SPL06_REG_MEAS_CFG_ADDR	0x08

/* Interrupt and FIFO Configuration Register */
#define SPL06_REG_CFG_REG_ADDR	0x09

/* Interrupt Status Register */
#define SPL06_REG_INT_STS_ADDR	0x0A

/* FIFO Status Register */
#define SPL06_REG_FIFO_STS_ADDR	0x0B

/* Soft Reset and FIFO Flush Register */
#define SPL06_REG_RESET_ADDR	0x0C

/* Calibration Coefficient Registers */
#define SPL06_REG_C0			0x10		/* c0 [11:4] 			 */
#define	SPL06_REG_C01C1			0x11		/* c0 [3:0] 	c1 [11:8]*/
#define SPL06_REG_C1			0x12
#define SPL06_REG_C00_1			0x13
#define SPL06_REG_C00_2			0x14
#define SPL06_REG_C00C10		0x15
#define SPL06_REG_C10_1			0x16
#define SPL06_REG_C10_2			0x17
#define SPL06_REG_C01_1			0x18
#define SPL06_REG_C01_2			0x19
#define SPL06_REG_C11_1			0x1A
#define SPL06_REG_C11_2			0x1B
#define SPL06_REG_C20_1			0x1C
#define SPL06_REG_C20_2			0x1D
#define SPL06_REG_C21_1			0x1E
#define SPL06_REG_C21_2			0x1F
#define SPL06_REG_C30_1			0x20
#define SPL06_REG_C30_2			0x21

/*
 * SENSOR STRUCT
 */
typedef struct {
	/* I2C Handle */
	I2C_HandleTypeDef *i2cHandle;

	/* Pressure Data */
	float compensatedPressure;

	/* Temperature Data */
	float compensatedTemperature;

	/* Scale Factor */
	uint16_t scaleFactor;

} SPL06_007;

/*
 * INITIALISATION
 */
uint8_t SPL06_007_Initialise( SPL06_007 *dev, I2C_HandleTypeDef *i2cHandle );

/*
 * DATA ACQUISITION
 */
uint8_t SPL06_007_calcCompPressure( SPL06_007 *dev );
uint8_t SPL06_007_calcCompTemp( SPL06_007 *dev );
uint8_t SPL06_007_getRawPressure( SPL06_007 *dev );
uint8_t SPL06_007_getRawTemp( SPL06_007 *dev );


/*
 * LOW-LEVEL FUNCTIONS
 */
uint8_t SPL06_007_getRegisterValue( SPL06_007 *dev, uint8_t reg );
HAL_StatusTypeDef SPL06_007_ReadRegister( SPL06_007 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef SPL06_007_WriteRegister( SPL06_007 *dev, uint8_t reg, uint8_t *data );

#endif /* INC_SPL06_007_H_ */
