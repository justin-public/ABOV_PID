/**
 *******************************************************************************
 * @file        a96l116_adc.h
 * @author      ABOV R&D Division
 * @brief       ADC Header File
 *
 * Copyright 2020 ABOV Semiconductor Co.,Ltd. All rights reserved.
 *
 * This file is licensed under terms that are found in the LICENSE file
 * located at Document directory.
 * If this file is delivered or shared without applicable license terms,
 * the terms of the BSD-3-Clause license shall be applied.
 * Reference: https://opensource.org/licenses/BSD-3-Clause
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H_
#define __ADC_H_
/*******************************************************************************
* Included File
*******************************************************************************/
#include "A96L116.h"
//#include "A96L116_clock.h"
#include "typedef.h"
#include "Intrins.h"

/*******************************************************************************
* Public Macro
*******************************************************************************/
#define ADC_MAX_BUFFER_SIZE     16

#define ADC_CLK_DIV1						0
#define ADC_CLK_DIV2						1
#define ADC_CLK_DIV3						2
#define ADC_CLK_DIV4						3
#define ADC_CLK_DIV5						4
#define ADC_CLK_DIV6						5
#define ADC_CLK_DIV8						6
#define ADC_CLK_DIV16						7

#define ADC_SW_TRIG             0
#define ADC_T0A_TRIG            1
#define ADC_T1A_TRIG            2
#define ADC_T2A_TRIG            3

#define ADC_MSB  0
#define ADC_LSB	 1

/*******************************************************************************
* Public Typedef
*******************************************************************************/
enum  adc_channel{ 
	ADC_CH0		 	= 0,
	ADC_CH1,
	ADC_CH2,
	ADC_CH3,
	ADC_CH4,
	ADC_CH5,
	ADC_CH6,
	ADC_CH7,
	ADC_VBGR		= 15,
};

/*******************************************************************************
* Exported Public Function
*******************************************************************************/
//void ADC_Initial(uint8_t clock_sel, uint8_t trigger_sel, uint8_t align_sel);
//void ADC_Enable(uint8_t enable);
//void ADC_SelectChannel(uint8_t channel);

void ADC_StartSoftwareTrigger(void);
uint8_t ADC_GetConversionStatus(void);

void ADC_GetDataWithInterrupt(uint16_t *adc_data, uint8_t count);
void ADC_GetDataWithPolling(uint16_t *adc_data, uint8_t count);

void ADC_ConfigureInterrupt(uint8_t enable);
void ADC_ClearInterruptStatus(void);

#endif  /* End of __ADC_H_ */
/* --------------------------------- End Of File ------------------------------ */