//*******************************************************************************
//* @file func_def.h
//* @author ABOV R&D Division
//* @brief Define functions
//*
//* Copyright 2022 ABOV Semiconductor Co.,Ltd. All rights reserved.
//*
//* This file is licensed under terms that are found in the LICENSE file
//* located at Document directory.
//* If this file is delivered or shared without applicable license terms,
//* the terms of the BSD-3-Clause license shall be applied.
//* Reference: https://opensource.org/licenses/BSD-3-Clause
//*******************************************************************************
#ifndef _FUNC_DEF_H_
#define _FUNC_DEF_H_

//======================================================
// Function and global variables definition
//======================================================
void port_init();	 	// initialize ports
void clock_init();	 	// initialize operation clock
void ADC_init();	 	// initialize A/D convertor
void ADC_start(unsigned char ch);	// start A/D convertor
unsigned int ADC_read();	// read A/D convertor
void ExINT_init();	 	// initialize external interrupt
void LPUART_init();	 	// initialize LPUART
void Timer0_init();	 	// initialize Timer0
void Timer1_init();	 	// initialize Timer1
void Timer2_init();	 	// initialize Timer2

#endif // _FUNC_DEF_H_

