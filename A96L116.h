/**
 *******************************************************************************
 * @file        a96l116.h
 * @author      ABOV R&D Division
 * @brief       A96L116 Header File
 *
 * Copyright 2020 ABOV Semiconductor Co.,Ltd. All rights reserved.
 *
 * This file is licensed under terms that are found in the LICENSE file
 * located at Document directory.
 * If this file is delivered or shared without applicable license terms,
 * the terms of the BSD-3-Clause license shall be applied.
 * Reference: https://opensource.org/licenses/BSD-3-Clause
 ******************************************************************************/
#include <intrins.h>
#define NOP _nop_()

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __A96L116_H_
#define __A96L116_H_
//-----------------------------------------------------
//  MC96F8204 Special Function Register Definitions
//----------------------------------------------------- 0x80
sfr     P0          =   0x80;       // P0 Data Register
sbit    P00         =   0x80;
sbit    P01         =   0x81;
sbit    P02         =   0x82;
sbit    P03         =   0x83;
sbit    P04         =   0x84;
sbit    P05         =   0x85;
sbit    P06         =   0x86;
sbit    P07         =   0x87;

sfr     SP          =   0x81;       // Stack Pointer
sfr     DPL         =   0x82;       // Data Pointer Register Low
sfr     DPH         =   0x83;       // Data Pointer Register High
sfr     DPL1        =   0x84;       // Data Pointer Register Low 1
sfr     DPH1        =   0x85;       // Data Pointer Register High 1
sfr     LVICR       =   0x86;       // Low Voltage Indicator Control Register
sfr     PCON        =   0x87;       // Power Control Register
//----------------------------------------------------- 0x88
sfr     P1        =   0x88;       // P1 DATA Register
sbit    P10         =   0x88;
sbit    P11         =   0x89;
sbit    P12         =   0x8A;
sbit    P13         =   0x8B;
sbit    P14         =   0x8C;
sbit    P15         =   0x8D;
sfr     FCDIN        =   0x89;       // FLASH CONTROL DATA IN Register
sfr     SCCR        =   0x8A;       // System and Clock Control Register
sfr     BITCR       =   0x8B;       // Basic Interval Timer Control Register
sfr     BITCNT      =   0x8C;       // Basic Interval Timer Counter Register (Read Only)
sfr     WDTCR       =   0x8D;       // Watch Dog Timer Control Register
sfr     WDTDR       =   0x8E;       // Watch Dog Timer Data Register (Write Only)
sfr     WDTCNT      =   0x8E;       // Watch Dog Timer Counter Register (Read Only)
sfr     IRCTCR      =   0x8F;       // 
//----------------------------------------------------- 0x90
sfr     P2        =   0x90;       
sbit    P20         =   0x90;
sbit    P21         =   0x91;
sbit    P22         =   0x92;
sbit    P23         =   0x93;
sfr     EIFLAG0       =   0x91;   
sfr     ADCCRL        =   0x92;   
sfr     ADCCRH       =   0x93;    
sfr     ADCDRL        =   0x94;   
sfr     ADCDRH        =   0x95;   
//sfr           =   0x96;     
//sfr           =   0x97;     
//----------------------------------------------------- 0x98
//sfr            =   0x98;       // 
sfr     EIFLAG1     =   0x99;    
sfr     EIPOL0L     =   0x9A;     
//sfr           =   0x9B;       	
sfr     SPI0CR      =   0x9C;      
sfr     SPI0DR      =   0x9D;      
sfr     SPI0SR      =   0x9E;    
sfr     IRCIDR      =   0x9F;    
//----------------------------------------------------- 0xA0
//sfr           =   0xA0;       
sfr     EIFLAG2      =   0xA1;  
sfr     EO          =   0xA2;   
//sfr           =   0xA4;   
//sfr           =   0xA5;   
//sfr            =   0xA7;   
//----------------------------------------------------- 0xA8
sfr     IE          =   0xA8;       // Interrupt Enable Register
sbit    EINT10_INT_EN   =   0xA8;
sbit    EINT11_INT_EN   =   0xA9;
sbit    EINT12_INT_EN   =   0xAA;
sbit    EINT0_1_2_3_INT_EN    =   0xAD;
sbit    EA              =   0xAF;

sfr     IE1         =   0xA9;       // Interrupt Enable Register 1
sfr     IE2         =   0xAA;       // Interrupt Enable Register 2
sfr     IE3         =   0xAB;       // Interrupt Enable Register 3
sfr     EIPOL2L          =   0xAC;       // P1 Data Register
//sfr             =   0xAD;       
//sfr             =   0xAE;       
//sfr             =   0xAF;       
//----------------------------------------------------- 0xB0
//sfr            =   0xB0;       
sfr     P0DB       =   0xB1;     
sfr     P0IOL      =   0xB2;     
sfr     P0IOH      =   0xB3;     
sfr     P0OD      =   0xB4;      
sfr     P0PU      =   0xB5;      
sfr     P0FSRL      =   0xB6;    
sfr     P0FSRH      =   0xB7;    
//----------------------------------------------------- 0xB8
sfr     IP          =   0xB8;       
sfr     P1DB        =   0xB9;       
sfr		P1IOL 		=	0xBA;		
sfr		P1IOH 		=	0xBB;		
sfr		P1OD 		=	0xBC;		
sfr		P1PU 		=	0xBD;		
sfr		P1FSRL 		=	0xBE;		
sfr    	P1FSRH     	=   0xBF;       
//----------------------------------------------------- 0xC0
sfr     P2IOL       =   0xC2;      
sfr     P2OD        =   0xC4;      
sfr     P2PU        =   0xC5;      
sfr     P2FSRL      =   0xC6;      
//----------------------------------------------------- 0xC8
sfr     OSCCR       =   0xC8;       
sfr     SUBISET     =   0xC9;       
sfr     T0CRL       =   0xCA;       
sfr     T0CRH       =   0xCB;       
sfr     T0ADRL      =   0xCC;       
sfr     T0ADRH      =   0xCD;       
sfr     T0BDRL      =   0xCE;       
sfr     T0BDRH      =   0xCF;       
//----------------------------------------------------- 0xD0
sfr     PSW         =   0xD0;       // Program Status Word Register
sbit    F1          =   0xD1;
sbit    OV          =   0xD2;
sbit    RS0         =   0xD3;
sbit    RS1         =   0xD4;
sbit    F0          =   0xD5;
sbit    AC          =   0xD6;
sbit    CY          =   0xD7;

sfr     PPCLKEN0    =   0xD1;       
sfr     LPUT0BCPL   =   0xD2;       
sfr     LPUT0BCPH   =   0xD3;       
sfr     LPUT0RTDRL  =   0xD4;       
sfr     LPUT0RTDRH  =   0xD5;       
sfr     LPUT0RCDR   =   0xD6;       
sfr     LPUT0DLY    =   0xD7;       
//----------------------------------------------------- 0xD8
sfr     LVRCR       =   0xD8;       
sfr     PPCLKEN1    =   0xD9;       
sfr     LPUT0ISRL   =   0xDA;       
sfr     LPUT0ISRH   =   0xDB;       
sfr     LPUT0RDR    =   0xDC;       
sfr     LPUT0TDR    =   0xDD;       
sfr     LPUT0BDR    =   0xDE;       
//----------------------------------------------------- 0xE0
sfr     ACC         =   0xE0;       // Accumulator Register
sfr		PPCLKEN2	=	0xE1;
sfr		LPUT0CR0	=	0xE2;
sfr		LPUT0CR1	=	0xE3;
sfr		LPUT0CR2	=	0xE4;
sfr		LPUT0CR3	=	0xE5;
sfr		LPUT0CR4	=	0xE6;
sfr		LPUT0IER	=	0xE7;

//----------------------------------------------------- 0xE8
sfr     RSTFR       =   0xE8;       // Reset Flag Register
sfr     PPCLKEN3	=	0xE9;
sfr     I2C0CR		=	0xEA;
sfr     I2C0SR		=	0xEB;
sfr     I2C0DR		=	0xEC;
sfr     I2C0SDHR	=	0xED;
sfr     I2C0SCLR	=	0xEE;
sfr     I2C0SCHR	=	0xEF;
//----------------------------------------------------- 0xF0
sfr     B           =   0xF0;       // B Register
sfr     MPWRCR		=	0xF1;
sfr     DFSADRL		=	0xF2;
sfr     DFSADRH		=	0xF3;
sfr     DFIDR		=	0xF4;
sfr     DFMCR		=	0xF5;
sfr     I2C0SAR0	=	0xF6;
sfr     I2C0SAR1	=	0xF7;
//----------------------------------------------------- 0xF8
sfr     IP1         =   0xF8;       // Interrupt Priority Register 1
sfr     FSADRH      =   0xFA;       // Flash Sector Address High Register
sfr     FSADRM      =   0xFB;       // Flash Sector Address Middle Register
sfr     FSADRL      =   0xFC;       // Flash Sector Address Low Register
sfr     FIDR        =   0xFD;       // Flash Identification Register
sfr     FMCR        =   0xFE;       // Flash Mode Control Register

//==================================================================== //
// XSFR                                                                //
//==================================================================== //
#define     IRCCTRM       *(volatile unsigned char xdata *) 0x1000        
#define     IRCFTRM       *(volatile unsigned char xdata *) 0x1001        
#define     T0CAPL	      *(volatile unsigned char xdata *) 0x1006       
#define     T0CAPH	      *(volatile unsigned char xdata *) 0x1007       
#define     T1CRL	      *(volatile unsigned char xdata *) 0x1008       
#define     T1CRH	      *(volatile unsigned char xdata *) 0x1009       
#define     T1ADRL	      *(volatile unsigned char xdata *) 0x100a       
#define     T1ADRH	      *(volatile unsigned char xdata *) 0x100b       
#define     T1BDRL	      *(volatile unsigned char xdata *) 0x100c       
#define     T1BDRH	      *(volatile unsigned char xdata *) 0x100d       
#define     T1CAPL	      *(volatile unsigned char xdata *) 0x100e       
#define     T1CAPH	      *(volatile unsigned char xdata *) 0x100f       
#define     T2CRL	      *(volatile unsigned char xdata *) 0x1010       
#define     T2CRH	      *(volatile unsigned char xdata *) 0x1011       
#define     T2ADRL	      *(volatile unsigned char xdata *) 0x1012       
#define     T2ADRH	      *(volatile unsigned char xdata *) 0x1013       
#define     T2BDRL	      *(volatile unsigned char xdata *) 0X1014
#define     T2BDRH	      *(volatile unsigned char xdata *) 0x1015       
#define     T2CAPL	      *(volatile unsigned char xdata *) 0x1016       
#define     T2CAPH	      *(volatile unsigned char xdata *) 0X1017	

#define     UNIQUEID0       *(volatile unsigned char xdata *) 0x1060        
#define     UNIQUEID1       *(volatile unsigned char xdata *) 0x1061        
#define     UNIQUEID2       *(volatile unsigned char xdata *) 0x1062       
#define     UNIQUEID3       *(volatile unsigned char xdata *) 0x1063       
#define     UNIQUEID4       *(volatile unsigned char xdata *) 0x1064       
#define     UNIQUEID5       *(volatile unsigned char xdata *) 0x1065       
#define     UNIQUEID6       *(volatile unsigned char xdata *) 0x1066       
#define     UNIQUEID7       *(volatile unsigned char xdata *) 0x1067       
#define     UNIQUEID8       *(volatile unsigned char xdata *) 0x1068       
#define     UNIQUEID9       *(volatile unsigned char xdata *) 0x1069       
#define     UNIQUEID10      *(volatile unsigned char xdata *) 0x106a       
#define     UNIQUEID11      *(volatile unsigned char xdata *) 0x106b       
#define     UNIQUEID12      *(volatile unsigned char xdata *) 0x106c       
#define     UNIQUEID13      *(volatile unsigned char xdata *) 0x106d       
#define     UNIQUEID14      *(volatile unsigned char xdata *) 0x106e       
#define     UNIQUEID15      *(volatile unsigned char xdata *) 0x106f       

#define     RTCCRL	      *(volatile unsigned char xdata *) 0x10b0        
#define     RTCCRH	      *(volatile unsigned char xdata *) 0x10b1        
#define     RTCSCTL       *(volatile unsigned char xdata *) 0x10b2       
#define     RTCSCTH       *(volatile unsigned char xdata *) 0x10b3       
#define     RTCECR	      *(volatile unsigned char xdata *) 0x10b4       
#define     RTCSEC	      *(volatile unsigned char xdata *) 0x10b5       
#define     RTCMIN	      *(volatile unsigned char xdata *) 0x10b6       
#define     RTCHR	      *(volatile unsigned char xdata *) 0x10b7       
#define     RTCDAY	      *(volatile unsigned char xdata *) 0x10b8       
#define     RTCWK	      *(volatile unsigned char xdata *) 0x10b9       
#define     RTCMTH	      *(volatile unsigned char xdata *) 0x10ba       
#define     RTCYR	      *(volatile unsigned char xdata *) 0x10bb       
#define     RTCAMIN       *(volatile unsigned char xdata *) 0x10bc       
#define     RTCAHR	      *(volatile unsigned char xdata *) 0x10bd       
#define     RTCAWK	      *(volatile unsigned char xdata *) 0x10be       


#define     FCSARH      *(volatile unsigned char xdata *) 0x5050        // Flash CRC Start Address High Register
#define     FCEARH      *(volatile unsigned char xdata *) 0x5051        // Flash CRC End Address High Register
#define     FCSARM      *(volatile unsigned char xdata *) 0x5052        // Flash CRC Start Address Middle Register
#define     FCEARM      *(volatile unsigned char xdata *) 0x5053        // Flash CRC End Address Middle Register
#define     FCSARL      *(volatile unsigned char xdata *) 0x5054        // Flash CRC Start Address Low Register
#define     FCEARL      *(volatile unsigned char xdata *) 0x5055        // Flash CRC End Address Low Register
#define     FCCR        *(volatile unsigned char xdata *) 0x5056        // Flash CRC Control Register
#define     FCDRH       *(volatile unsigned char xdata *) 0x5057        // Flash CRC Data High Register (Read Only)
#define     FCDRL       *(volatile unsigned char xdata *) 0x5058        // Flash CRC Data Low Register (Read Only)
#define     LVRIDR      *(volatile unsigned char xdata *) 0x505F        

//------------------------------------------------------------------------------
// Interrupt vectors address of A96L116 SFR.
#define HW_RESET_VECTOR             0x0000
#define EXTERNAL_INTERRUPT_10       0x0003
#define EXTERNAL_INTERRUPT_11       0x000B
#define EXTERNAL_INTERRUPT_12       0x0013
#define EXTERNAL_INTERRUPT_0_1_2_3  0x002B
#define I2C0_INTERRUPT              0x0033
#define SPI0_INTERRUPT          	0x003B
#define LPUART0_INTERRUPT           0x0043
#define T0_MACTCH_INTERRUPT         0x0063
#define T1_MACTCH_INTERRUPT         0x006B
#define T2_MACTCH_INTERRUPT         0x0073
#define ADC_INTERRUPT               0x0093
#define LVI_INTERRUPT               0x009B
#define RTCC_INTERRUPT              0x00A3
#define WDT_INTERRUPT               0x00AB
#define BIT_INTERRUPT               0x00B3

// Interrupt vectors Prioirty of A96L116 SFR. (For Keil C51)
#define EINT10_VECT         0   // IE.0
#define EINT11_VECT         1   // IE.1
#define EINT12_VECT         2   // IE.2
#define EINT0_1_2_3_VECT    5   // IE.5

#define I2C0_VECT           6   // IE1.0
#define SPI0_VECT       	7   // IE1.1
#define LPUART0_VECT        8   // IE1.2

#define T0_MATCH_VECT       12  // IE2.0
#define T1_MATCH_VECT       13  // IE2.1
#define T2_MATCH_VECT       14  // IE2.2

#define ADC_VECT            18  // IE3.0
#define LVI_VECT            19  // IE3.1
#define RTCC_VECT           20  // IE3.2
#define WDT_VECT            21  // IE3.3
#define BIT_VECT            22  // IE3.4


//------------------------------------------------------------------------------
// Interrupt enable/disable control
// EA REG
#define EINT10_EN()         (EINT10_INT_EN = 1)
#define EINT10_DIS()        (EINT10_INT_EN = 0)

#define EINT11_EN()         (EINT11_INT_EN = 1)
#define EINT11_DIS()        (EINT11_INT_EN = 0)

#define EINT0_1_2_3_EN()          (EINT0_1_2_3_INT_EN = 1)
#define EINT0_1_2_3_DIS()         (EINT0_1_2_3_INT_EN = 0)

#define GLOBAL_INTERRUPT_EN()       (EA = 1)
#define GLOBAL_INTERRUPT_DIS()      (EA = 0)

#define EI()                (EA = 1)
#define DI()                (EA = 0)

// IE1 REG
#define I2C_INT_EN()       (IE1 |=0X01)
#define I2C_INT_DIS()      (IE1 &= ~(0X01))

#define SPI0_INT_EN()  (IE1 |=0X02)
#define SPI0_INT_DIS() (IE1 &= ~(0X02))

#define LPUART0_INT_EN()  (IE1 |=0X04)
#define LPUART0_INT_DIS() (IE1 &= ~(0X04))

// IE2 REG
#define TIMER0_MATCH_EN()   (IE2 |=0X01)
#define TIMER0_MATCH_DIS()  (IE2 &= ~(0X01))

#define TIMER1_MATCH_EN()   (IE2 |=0X02)
#define TIMER1_MATCH_DIS()  (IE2 &= ~(0X02))

#define TIMER2_MATCH_EN()   (IE2 |=0X04)
#define TIMER2_MATCH_DIS()  (IE2 &= ~(0X04))

// IE3 REG
#define ADC_INT_EN()        (IE3 |=0X01)
#define ADC_INT_DIS()       (IE3 &= ~(0X01))

#define LVI_INT_EN()         (IE3 |=0X02)
#define LVI_INT_DIS()        (IE3 &= ~(0X02))

#define RTCC_INT_EN()         (IE3 |=0X04)
#define RTCC_INT_DIS()        (IE3 &= ~(0X04))

#define WDT_INT_EN()        (IE3 |=0X08)
#define WDT_INT_DIS()       (IE3 &= ~(0X08))

#define BIT_INT_EN()        (IE3 |=0X10)
#define BIT_INT_DIS()       (IE3 &= ~(0X10))

// P0DB REG
#define P00DB_EN()          (P0DB |=0X01)
#define P00DB_DIS()         (P0DB &= ~(0X01))

#define P01DB_EN()          (P0DB |=0X02)
#define P01DB_DIS()         (P0DB &= ~(0X02))

#define P02DB_EN()          (P0DB |=0X04)
#define P02DB_DIS()         (P0DB &= ~(0X04))

#define P03DB_EN()          (P0DB |=0X08)
#define P03DB_DIS()         (P0DB &= ~(0X08))

// P1DB REG
#define P13DB_EN()          (P1DB |=0X08)
#define P13DB_DIS()         (P1DB &= ~(0X08))

#define P14DB_EN()          (P1DB |=0X10)
#define P14DB_DIS()         (P1DB &= ~(0X10))

#define P15DB_EN()          (P1DB |=0X20)
#define P15DB_DIS()         (P1DB &= ~(0X20))
//------------------------------------------------------------------------------
/* CLOCK Control Register bits */
//------------------------------------------------------------------------------
//	SCCR (System and clock control Register : 0x8A, Initial Value : 0x00 )
//------------------------------------------------------------------------------
#define SCCR_HFIRC_OSC      ((INT8U)0x00)   /*!< HF Internal RC OSC for system clock */
#define SCCR_EXT_MOSC       ((INT8U)0x01)   /*!< External Main OSC for system clock  */
#define SCCR_EXT_SOSC       ((INT8U)0x02)   /*!< External Sub OSC for system clock  */
#define SCCR_LFIRC_OSC      ((INT8U)0x03)   /*!< LF Internal RC OSC for system clock */
//------------------------------------------------------------------------------
//	OSCCR (Oscillator Control Register : 0xC8, Initial Value : 0x08 )
//------------------------------------------------------------------------------
#define OSCCR_LFIRCE_DIS    ((INT8U)0x00)   /*!< OSCCR[7]
                                            Control the Operation of the low frequency internal RC oscillator
                                            0 : Disable
                                            */
#define OSCCR_LFIRCE_EN     ((INT8U)0x80)   /*!< OSCCR[7]
                                            Control the Operation of the low frequency internal RC oscillator
                                            1 : Enable
                                            */
#define OSCCR_HFIRCS_500kHz ((INT8U)0x00)   /*!< OSCCR[5:3]
                                            HF Internal RC Oscillator Post-devider Selection
                                            HF INT-RC/32 = 0.5MHz
                                            */
#define OSCCR_HFIRCS_1MHz   ((INT8U)0x08)   /*!< OSCCR[5:3]
                                            HF Internal RC Oscillator Post-devider Selection
                                            HF INT-RC/16 = 1MHz
                                            */
#define OSCCR_HFIRCS_2MHz   ((INT8U)0x10)   /*!< OSCCR[5:3]
                                            Internal RC Oscillator Post-devider Selection
                                            HF INT-RC/8 = 2MHz
                                            */
#define OSCCR_HFIRCS_4MHz   ((INT8U)0x18)   /*!< OSCCR[5:3]
                                            HF Internal RC Oscillator Post-devider Selection
                                            HF INT-RC/4 = 4MHz
                                            */
#define OSCCR_HFIRCS_8MHz   ((INT8U)0x20)   /*!< OSCCR[5:3]
                                            HF Internal RC Oscillator Post-devider Selection
                                            HF INT-RC/2 = 8MHz
                                            */
#define OSCCR_HFIRCS_16MHz   ((INT8U)0x28)   /*!< OSCCR[5:3]
											HF Internal RC Oscillator Post-devider Selection
											HF INT-RC/1 = 16MHz
											*/
#define OSCCR_HFIRCE_EN     ((INT8U)0x00)   /*!< OSCCR[2]
                                            Control the Operation of the HF Internal RC oscillator
                                            !! Caution 1 : Disable
                                            */
#define OSCCR_HFIRCE_DIS    ((INT8U)0x04)   /*!< OSCCR[2]
                                            Control the Operation of the HF Internal RC oscillator
                                            !! Caution 0 : Enable
                                            */                                                
#define OSCCR_XCLKE_DIS     ((INT8U)0x00)   /*!< OSCCR[1]
                                            Control the Operation of the External Main Oscillator
                                            0 : Disable
                                            */
#define OSCCR_XCLKE_EN      ((INT8U)0x02)   /*!< OSCCR[1]
                                            Control the Operation of the External Main Oscillator
                                            1 : Enable
                                            */                                            
#define OSCCR_SCLKE_DIS     ((INT8U)0x00)   /*!< OSCCR[0]
                                            Control the Operation of the External Sub Oscillator
                                            0 : Disable
                                            */
#define OSCCR_SCLKE_EN      ((INT8U)0x01)   /*!< OSCCR[0]
                                            Control the Operation of the External Sub Oscillator
                                            1 : Enable
                                            */

#endif  /* End of __A96L116_H_ */
/* --------------------------------- End Of File ------------------------------ */
