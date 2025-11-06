/**
 *******************************************************************************
 * @file        a96l116_adc.c
 * @author      ABOV R&D Division
 * @brief       ADC Source File
 *
 * Copyright 2020 ABOV Semiconductor Co.,Ltd. All rights reserved.
 *
 * This file is licensed under terms that are found in the LICENSE file
 * located at Document directory.
 * If this file is delivered or shared without applicable license terms,
 * the terms of the BSD-3-Clause license shall be applied.
 * Reference: https://opensource.org/licenses/BSD-3-Clause
 ******************************************************************************/

/*******************************************************************************
* Included File
*******************************************************************************/
#include "A96L116_adc.h"

/*******************************************************************************
* Private Pre-processor Definition & Macro
*******************************************************************************/

/*******************************************************************************
* Private Typedef
*******************************************************************************/

/*******************************************************************************
* Private Variable
*******************************************************************************/
uint8_t adc_count = 0;
uint16_t *adc_buff;
uint8_t buff_cnt = 0;
/*******************************************************************************
* Private Function Prototype
*******************************************************************************/

/*******************************************************************************
* Public Function
*******************************************************************************/
/**
* @brief		Get data of ADC conversion for ADC interrupt.
* @param   adc_data		This parameter contains the data of conversion ADC.
* @param   count		This parameter contains the number of count.
* @return		None
*/
void ADC_GetDataWithInterrupt(uint16_t *adc_data, uint8_t count)
{
	adc_count = count;
	adc_buff = (uint16_t*)adc_data;
	buff_cnt = 0;
	
	IE3  |= 0x01; 

	if( ((ADCCRH & 0x30) == 0x00)) //SW trigger
	{
		ADCCRL |= (1 << 6);
	}
	
	while(0 < adc_count);  //adc interrupt subroutine execute
	ADCCRH &= ~0x80;
	IE3 &= ~0x01;
}

void ADC_Int_Handler(void) interrupt ADC_VECT
{
	adc_buff[buff_cnt++] = (ADCDRH << 8) + (ADCDRL << 0);
	
	adc_count--;
	if(((ADCCRH & 0x30) == 0x00) && (adc_count != 0)) //SW trigger
		ADCCRL |= (1 << 6);

	ADCCRH &= ~0x80;
}

/* --------------------------------- End Of File ------------------------------ */