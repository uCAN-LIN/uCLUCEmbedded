/**
  LIN Peripheral Hardware Driver
	
  Company:
    Microchip Technology Inc.

  File Name:
    lin_hardware.h

  Summary:
    LIN Peripheral Hardware Driver

  Description:
    This header file links device specific peripherals to LIN API

 */

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#ifndef HARDWARE_H
#define	HARDWARE_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <xc.h>

#include "../eusart.h"
#include "../tmr0.h"

//Device EUSART
#define LIN_EUSART_DataReady    EUSART_DataReady
#define LIN_EUSART_Read         EUSART_Read
#define LIN_EUSART_Write        EUSART_Write
#define LIN_TXIE                TXIE
#define LIN_RCIE                RCIE

#define LIN_TRMT                TXSTAbits.TRMT
#define LIN_CREN                RCSTAbits.CREN
#define LIN_SENDB               TXSTAbits.SENDB
#define LIN_FERR                RCSTAbits.FERR

//Device Timer
#define LIN_WriteTimer          TMR0_WriteTimer
#define LIN_StartTimer          NOP
#define LIN_StopTimer           NOP
#define LIN_SetInterruptHandler TMR0_SetInterruptHandler


#endif	/* HARDWARE_H */
