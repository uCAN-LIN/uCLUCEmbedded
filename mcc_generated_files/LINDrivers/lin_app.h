/**
  LIN Slave Application
	
  Company:
    Microchip Technology Inc.

  File Name:
    lin_app.h

  Summary:
    LIN Slave Application

  Description:
    This header file provides the interface between the user and 
    the LIN drivers.

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


#ifndef LIN_APP_H
#define	LIN_APP_H
#define LIN_MASTER_DEF_ID 0


#include "lin_slave.h"
#include "lin_master.h"

//uint8_t LIN_Master_Data[8 * 16];

uint8_t LIN_Master_Data[8 * 3];



lin_cmd_packet_t scheduleTable[15]; 
//= {
//    //Command, Type, TX/RX Length, Timeout, Period, Data Address
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 100, 0, &LIN_Master_Data[0] },
//    {LIN_MASTER_DEF_ID, TRANSMIT, 8, 0, 10, &LIN_Master_Data[8] },
//    {LIN_MASTER_DEF_ID, TRANSMIT, 8, 0, 10, &LIN_Master_Data[16] },
//    {LIN_MASTER_DEF_ID, TRANSMIT, 8, 0, 10, &LIN_Master_Data[24] },
//    {LIN_MASTER_DEF_ID, TRANSMIT, 8, 0, 10, &LIN_Master_Data[32] },
//    {LIN_MASTER_DEF_ID, TRANSMIT, 8, 0, 10, &LIN_Master_Data[40] },
//    {LIN_MASTER_DEF_ID, TRANSMIT, 8, 0, 10, &LIN_Master_Data[48] },
//    {LIN_MASTER_DEF_ID, TRANSMIT, 8, 0, 10, &LIN_Master_Data[56] },
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 0, 10, &LIN_Master_Data[64] },
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 0, 10, &LIN_Master_Data[72] },
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 0, 10, &LIN_Master_Data[80] },
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 0, 10, &LIN_Master_Data[88] },
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 0, 10, &LIN_Master_Data[96] },
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 0, 10, &LIN_Master_Data[104] },
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 0, 10, &LIN_Master_Data[112] },
//    {LIN_MASTER_DEF_ID, RECEIVE, 8, 0, 10, &LIN_Master_Data[120] },
//};
#define TABLE_SIZE  (sizeof(scheduleTable)/sizeof(lin_cmd_packet_t))

void LIN_Slave_Initialize(void);
void LIN_Master_Initialize(void);

void processLIN(void);


#endif	/* LIN_APP_H */

