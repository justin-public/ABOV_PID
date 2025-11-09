//**
//*******************************************************************************
//* @file main.c
//* @author ABOV R&D Division
//* @brief Main program
//*
//* Copyright 2022 ABOV Semiconductor Co.,Ltd. All rights reserved.
//*
//* This file is licensed under terms that are found in the LICENSE file
//* located at Document directory.
//* If this file is delivered or shared without applicable license terms,
//* the terms of the BSD-3-Clause license shall be applied.
//* Reference: https://opensource.org/licenses/BSD-3-Clause
//*******************************************************************************
//======================================================
// Main program routine
// - Device name  : A96L116
// - Package type : 20TSSOP
// Generated    : Sun, Nov 09, 2025 (12:31:50)
//======================================================
// MAIN is used for XDATA variable : V1.041.00 ~
#define		MAIN	1	// Do not delete this line

#include	"A96L116.h"
#include  "A96L116_adc.h"
#include	"func_def.h"

#define PWM_NO1     0
#define PWM_NO2     1

#define ADC_BUFFER_SIZE     10

#define ADC_MAIN_PROC	 1
#define PWM_MAIN_PROC	 1
#define UART_MAIN_PROC	 0

#define SERVO_FREQ  50          // 50Hz (20ms 주기)
#define SERVO_PERIOD 40000      // 2MHz / 50Hz = 40,000 카운트 (20ms)

#define SERVO_PULSE_MIN     900
#define SERVO_PULSE_CENTER  1500
#define SERVO_PULSE_MAX     2100

uint16_t avg_data = 0;
uint16_t adc_data[ADC_BUFFER_SIZE];

uint32_t capture_cnt, t_cap_duty;
bit pwm_start_flag = 0;

void LPUART_SendChar(unsigned char ch);
void LPUART_SendStr(const char* str);
void LPUART_SendNum(unsigned long num);
void delay_us(unsigned long us);

void set_servo_us(uint8_t pwm_no,uint16_t pulse_us);

uint16_t Timer1_GetCaptureValue(void);

void Timer0_SetPPGDutyCounter(uint16_t count);
void Timer0_SetPPGPeriodCounter(uint16_t count);

void delay_us(unsigned long us)  
{
    unsigned long i;
    for(i = 0; i < us; i++)
    {
        NOP;  
    }
}

void set_servo_us(uint8_t pwm_no,uint16_t pulse_us)
{
	uint16_t p_cnt = SERVO_PERIOD;
    uint16_t duty_cnt;

	if(pulse_us < SERVO_PULSE_MIN) pulse_us = SERVO_PULSE_MIN;
    if(pulse_us > SERVO_PULSE_MAX) pulse_us = SERVO_PULSE_MAX;

    duty_cnt = pulse_us * 2;
	
	if(pwm_no == PWM_NO1)
	{
		Timer0_SetPPGPeriodCounter(p_cnt);
		Timer0_SetPPGDutyCounter(duty_cnt);	
	}
	else
	{
		//Timer0_SetPPGPeriodCounter(p_cnt);
		//Timer0_SetPPGDutyCounter(duty_cnt);	
	} 
}

void main()
{
	uint8_t i = 0;
	uint32_t t_cap_duty_cpy = 0;
	
	GLOBAL_INTERRUPT_DIS();          	
	port_init();    	// initialize ports
	clock_init();   	// initialize operation clock
	ADC_init();     	// initialize A/D convertor
	ADC_start(7);
	ExINT_init();   	// initialize external interrupt
#if UART_MAIN_PROC	
	LPUART_init();  	// initialize LPUART
#endif	
	Timer0_init();  	// initialize Timer0
	Timer1_init();  	// initialize Timer1
	Timer2_init();  	// initialize Timer2
	GLOBAL_INTERRUPT_EN();          	
	
	// TODO: add your main code here
	P1 |= (1 << 1);     // HIGH P1.1 
	P1 &= ~(1 << 2);    // LOW  P1.2
	
	set_servo_us(PWM_NO1,900);
	
	while(1)
	{
		#if ADC_MAIN_PROC	
			avg_data = 0;
			ADC_GetDataWithInterrupt(adc_data, ADC_BUFFER_SIZE);    
			
			for(i = 0; i < ADC_BUFFER_SIZE; i++)
			{  
				avg_data += adc_data[i];
			}
			avg_data /= ADC_BUFFER_SIZE;
		#if UART_MAIN_PROC	
			LPUART_SendNum((unsigned long)avg_data);
			LPUART_SendChar(0x0D);
			LPUART_SendChar(0x0A);
			delay_us(1000);
		#endif
		#endif
		#if PWM_MAIN_PROC
			if(pwm_start_flag)
			{
				
				t_cap_duty_cpy = t_cap_duty;	
				//LPUART_SendNum((unsigned long)t_cap_duty);
				//LPUART_SendChar(0x0D);
				//LPUART_SendChar(0x0A);
				pwm_start_flag = 0;
			}
		#endif
	}
}

//======================================================
// Interrupt routines
//======================================================

void INT_Ext11() interrupt 1
{
	// External interrupt 11
	// TODO: add your code here
	if (P0 & (1<<3))
	{
		capture_cnt = 0;		
	}
	else
	{
		capture_cnt += Timer1_GetCaptureValue();
		t_cap_duty = capture_cnt/2;

		pwm_start_flag = 1;
	}
	EIFLAG2	= ~(1<<1);
}

void INT_Timer0() interrupt 12
{
	// Timer0 interrupt
	// TODO: add your code here
}

void INT_Timer1() interrupt 13
{
	// Timer1 interrupt
	// TODO: add your code here
	capture_cnt += 0x10000; 
}

void INT_Timer2() interrupt 14
{
	// Timer1 interrupt
	// TODO: add your code here
}

#if 0
void INT_ADC() interrupt 18
{
	// ADC interrupt
	// TODO: add your code here
}
#endif

//======================================================
// Peripheral setting routines
//======================================================
#if 0
unsigned int ADC_read()
{
	// read A/D convertor
	unsigned int adcVal;
	
	while(!(ADCCRL & 0x10));	// wait ADC busy
	adcVal = ADCDRH << 8;	// read ADC high
	adcVal |= ADCDRL;	// read ADC low
	ADCCRL &= ~0x80;	// disable ADC
	return	adcVal;
}
#endif

void ADC_init()
{
	// initialize A/D convertor
	PPCLKEN0 |= 0x40;	// Enable clock for ADC
	ADCCRL = 0x00;  	// setting
	ADCCRH = 0x0E;  	// trigger source, alignment, frequency
	//IE3 |= 0x01;    	// enable ADC interrupt
}

void ADC_start(unsigned char ch)
{
	// start A/D convertor
	ADCCRL |= 0x80; 	// enable ADC
	ADCCRL = (ADCCRL & 0xf0) | (ch & 0x0f);	// select channel
	ADCCRL |= 0x40; 	// start ADC
}

void ExINT_init()
{
	// initialize external interrupt
	EIPOL2L = 0x0C; 	// edge : external INT.10 ~ 12
	IE |= 0x02;     	// Enable Ext.INT 10 ~ 12
}

#if UART_MAIN_PROC
void LPUART_init()
{
	// initialize LPUART
	PPCLKEN2 |= 0x80;	// Enable clock for LPUART

	LPUT0CR0 = 0x06;	// Control 0
	LPUT0CR1 = 0x0D;	// Control 1

	// Set DE pin : Disabled
	LPUT0CR2 = 0x00;	// Active level start time (0.006500ms)
	LPUT0CR3 = 0x00;	// Active level finish time (0.006500ms)
	LPUT0CR4 = 0x00;	// Control 4

	// Set communication
	// - Source clock : fx
	// - Baudrate : 9600
	// - Data bit count : 8
	// - Stop bit count : 1
	// - Parity : No
	// - Oversampling : 16
	LPUT0BDR = 0x67;	// Baudrate
	LPUT0BCPH = 0x80;	// Bit compensation High
	LPUT0BCPL = 0x49;	// Bit compensation Low
	LPUT0RTDRH = 0xFF;	// Rx time out High (6815.744000ms)
	LPUT0RTDRL = 0xFF;	// Rx time out Low
	LPUT0RCDR = 0x00;	// Rx character detection data
	LPUT0DLY = 0x00;	// Tx delay time (0.000000ms)
}

void LPUART_SendChar(unsigned char ch)
{
    LPUT0TDR = ch;
}

void LPUART_SendStr(const char* str)
{
	while(*str != '\0')
	{
		LPUART_SendChar(*str);
		delay_us(100);
		str++;
	}
}

void LPUART_SendNum(unsigned long num)
{
	char buffer[12];
	int i = 0;

	if(num == 0)
	{
		LPUART_SendChar('0');
		return;
	}

	while(num > 0)
	{
		buffer[i++] = (num % 10) + '0';
		num /= 10;
	}

	while(i > 0)
	{
		LPUART_SendChar(buffer[--i]);
		delay_us(100);
	}
}
#endif

void Timer0_init()
{
	// initialize Timer0
	PPCLKEN1 |= 0x01;	// Enable clock for Timer0

	// 16bit PWM0, period = 20.000000mS ( 50.000000Hz )
	//     PWM duty = 50.000000%
	T0CRH = 0x30;   	// PWM setting High, repeat mode
	T0CRL = 0x20;   	// PWM setting Low
	T0BDRH = 0x4E;  	// duty High
	T0BDRL = 0x1F;  	// duty Low
	T0ADRH = 0x9C;  	// period count High
	T0ADRL = 0x3F;  	// period count Low
	//T0CRL = 0x08; 	// disable Timer reload signal
	IE2 |= 0x01;    	// Enable Timer0 interrupt

	T0CRH |= 0x80;  	// enable counter
}

void Timer0_SetPPGDutyCounter(uint16_t count)
{
    T0BDRH = 0x00FF & (count >> 8);
    T0BDRL = 0x00FF & (count >> 0);
}

void Timer0_SetPPGPeriodCounter(uint16_t count)
{
    T0ADRH = 0x00FF & (count >> 8);
    T0ADRL = 0x00FF & (count >> 0);
}

void Timer1_init()
{
	// initialize Timer1
	PPCLKEN1 |= 0x02;	// Enable clock for Timer1

	// 16bit capture1, period = 20.000000mS
	T1CRH = 0x10;   	// capture setting High
	T1CRL = 0x60;   	// capture setting Low
	T1ADRH = 0x9C;  	// period count High
	T1ADRL = 0x3F;  	// period count Low
	//T1CRL = 0x08; 	// disable Timer reload signal
	IE2 |= 0x02;    	// Enable Timer1 interrupt

	T1CRH |= 0x80;  	// enable counter
}

uint16_t Timer1_GetCaptureValue(void) 
{
	uint16_t count = 0;
	count = T1CAPL;
	count += (T1CAPH << 8);
	return count;
}

void Timer2_init()
{
	// initialize Timer2
	PPCLKEN1 |= 0x04;	// Enable clock for Timer2

	// 16bit PWM2, period = 20.000000mS ( 50.000000Hz )
	//     PWM duty = 50.000000%
	T2CRH = 0x30;   	// PWM setting High, repeat mode
	T2CRL = 0x60;   	// PWM setting Low
	T2BDRH = 0x4E;  	// duty High
	T2BDRL = 0x1F;  	// duty Low
	T2ADRH = 0x9C;  	// period count High
	T2ADRL = 0x3F;  	// period count Low
	//T2CRL = 0x08; 	// disable Timer reload signal
	IE2 |= 0x04;    	// Enable Timer2 interrupt

	T2CRH |= 0x80;  	// enable counter
}

void clock_init()
{
	// HFIRC clock (16.000000MHz)
	unsigned char tmp;	// variable for HFIRC

	tmp = OSCCR;    	// get default
	tmp &= ~(0x07 << 3);	// clear divider
	tmp |= 0x28;    	// set divider
	OSCCR = tmp;    	// Set HFIRC
	SCCR = 0x00;    	// Use HFIRC
}

void port_init()
{
	// initialize ports
	// Initialize P0x ports
	PPCLKEN0 |= 0x01;	// Enable clock for P0x ports
	P0PU = 0x00;    	// pullup
	P0OD = 0x00;    	// open drain
	P0IOH = 0xAA;   	// direction High
	P0IOL = 0x2A;   	// direction Low
	P0DB = 0x08;    	// bit7~6 = debounce clock
	P0 = 0x00;      	// port initial value
	P0FSRH = 0x18;  	// P0 selection High
	P0FSRL = 0x10;  	// P0 selection Low

	// Initialize P1x ports
	PPCLKEN0 |= 0x02;	// Enable clock for P1x ports
	P1PU = 0x00;    	// pullup
	P1OD = 0x00;    	// open drain
	P1IOH = 0xAA;   	// direction High
	//P1IOL = 0xAA;   	// direction Low
	P1IOL = 0x96;
	P1DB = 0x00;    	// debounce
	P1 = 0x00;      	// port initial value
	P1FSRH = 0x00;  	// P1 selection High
	P1FSRL = 0x00;  	// P1 selection Low

	// Initialize P2x ports
	PPCLKEN0 |= 0x04;	// Enable clock for P2x ports
	P2PU = 0x00;    	// pullup
	P2OD = 0x00;    	// open drain
	P2IOL = 0xAA;   	// direction Low
	P2 = 0x00;      	// port initial value
	P2FSRL = 0x00;  	// P2 selection Low
}


