/**
  LIN Slave Driver
	
  Company:
    Microchip Technology Inc.

  File Name:
    lin_slave.c

  Summary:
    LIN Slave Driver

  Description:
    This source file provides the driver for LIN slave nodes

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

#include "lin_slave.h"
#include "lin_hardware.h"
#include "../../slcan/slcan.h"
#include "../../mcc_generated_files/usb/usb_device_cdc.h"

#define READ_TIMEOUT    10  //ms

lin_packet_t LIN_packet;
bool LIN_rxInProgress = false;
const lin_rx_cmd_t* LIN_rxCommand;
uint8_t LIN_rxCommandLength;

static uint8_t LIN_timeout = 30; //TODO: Make dependent on Baudrate
static bool LIN_timerRunning = false;
static volatile uint8_t CountCallBack = 0;



void LIN_Slave_Initialize() {
    
    LIN_stopTimer();
    LIN_enableRx();
    LIN_setTimerHandler();
}

void LIN_queuePacket(uint8_t cmd) {
    const lin_rx_cmd_t* tempSchedule = LIN_rxCommand; //copy table pointer so we can modify it

    cmd &= 0x3F; //clear possible parity bits
    for (uint8_t i = 0; i < LIN_rxCommandLength; i++) {
        if (cmd == tempSchedule->cmd) {
            break;
        }
        tempSchedule++; //go to next entry
    }

    LIN_packet.type = tempSchedule->type;
    LIN_packet.length = tempSchedule->length;

    //Build Packet - User defined data
    //add data
    memcpy(LIN_packet.data, tempSchedule->data, LIN_packet.length);

    //Add Checksum
    LIN_packet.checksum = LIN_getChecksum(LIN_packet.length, LIN_packet.PID, LIN_packet.data);
    LIN_sendPacket(LIN_packet.length, LIN_packet.PID, LIN_packet.data);


}

void DBG(const char *x)
{
//    while(!USBUSARTIsTxTrfReady())
//    {
//       CDCTxService();   
//    }
//    putrsUSBUSART(x);
//    CDCTxService();   
//    while(!USBUSARTIsTxTrfReady())
//    {
//       CDCTxService();   
//    }
}

lin_rx_state_t LIN_Slave_handler(void) {
    static lin_rx_state_t LIN_rxState = LIN_RX_IDLE;
    static uint8_t rxDataIndex = 0;

    if (LIN_rxInProgress == true) {
        if (LIN_timerRunning == false) {
            //Timeout
//            LIN_rxState = LIN_RX_ERROR;
        }
    }

    switch (LIN_rxState) {
        case LIN_RX_IDLE:
            if (LIN_EUSART_DataReady > 0) {
                //Start Timer
                LIN_startTimer(READ_TIMEOUT);
                LIN_rxInProgress = true;
                LIN_rxState = LIN_RX_BREAK;
                DBG("!");
            }
            break;
        case LIN_RX_BREAK:
            if (LIN_EUSART_DataReady > 0) {
                if (LIN_breakCheck() == true) { //Read Break
                    LIN_rxState = LIN_RX_SYNC;
                    DBG("@1");
                } else {
                    LIN_rxState = LIN_RX_ERROR;
                    DBG("@2");
                }
            }
            break;
        case LIN_RX_SYNC:
            if (LIN_EUSART_DataReady > 0) {
                if (LIN_EUSART_Read() == 0x55) { //Read sync - discard
                    LIN_rxState = LIN_RX_PID;
                } else {
                    LIN_rxState = LIN_RX_ERROR;
                }
                DBG("#");
            }
            break;
        case LIN_RX_PID:
            if (LIN_EUSART_DataReady > 0) {
                LIN_packet.PID = LIN_EUSART_Read();
                LIN_rxState = LIN_RX_DATA;
                LIN_packet.length = 8; // len always max then check for timeout                
                DBG("$");
            }
            break;
        case LIN_RX_DATA:
            {
            extern uint8_t EUSART_read_timeout;
            uint8_t uart_rx_byte = LIN_EUSART_Read();
//            if ((LIN_breakCheck() == true))
//            {
//                DBG("%0");
//                goto LIN_PACKET_TOTAL;
//            }
            if (EUSART_read_timeout == 0)
            {
                DBG("%1");
                LIN_packet.data[rxDataIndex] = uart_rx_byte;
                if (++rxDataIndex >= LIN_packet.length) {
                    DBG("_");
                    //received all data bytes
                    rxDataIndex = 0;
                    LIN_rxState = LIN_RX_CHECKSUM;
                } 
            } else 
            {
LIN_PACKET_TOTAL:
                DBG("%2");
                // frame was shorter then 8 bits go to checksum
                LIN_packet.length = rxDataIndex - 1;
                LIN_packet.checksum = LIN_packet.data[rxDataIndex-1]; 
                if (LIN_packet.checksum != LIN_getChecksum(LIN_packet.length, LIN_packet.PID, LIN_packet.data)) {
                    DBG("^");
                    LIN_rxState = LIN_RX_ERROR;
                    slcanReciveCanFrame(&LIN_packet, 't');
                } else {
                    DBG("&");
                    LIN_rxState = LIN_RX_ERROR;
                    slcanReciveCanFrame(&LIN_packet, 'T');
                }  
            }
            }
            break;
        case LIN_RX_CHECKSUM:
            if (LIN_EUSART_DataReady > 0) {
                DBG("*");
                LIN_packet.checksum = LIN_EUSART_Read();
                if (LIN_packet.checksum != LIN_getChecksum(LIN_packet.length, LIN_packet.PID, LIN_packet.data)) {
                    LIN_rxState = LIN_RX_ERROR;
                    slcanReciveCanFrame(&LIN_packet, 't');
                    DBG("(");
                } else {
                    LIN_rxState = LIN_RX_ERROR;
                    slcanReciveCanFrame(&LIN_packet, 'T');
                }
            }
            break;
        case LIN_RX_TX_DATA:
            LIN_queuePacket(LIN_packet.PID); //Send response automatically
            LIN_rxState = LIN_RX_RDY;
        case LIN_RX_RDY:
            DBG(")");
            slcanReciveCanFrame(&LIN_packet, '?');
        case LIN_RX_ERROR:
            DBG("_");
            LIN_stopTimer();
            LIN_EUSART_Restart();
            rxDataIndex = 0;
            LIN_rxInProgress = false;
            memset(LIN_packet.rawPacket, 0, sizeof (LIN_packet.rawPacket)); //clear receive data
            LIN_rxState = LIN_RX_WAIT;
        case LIN_RX_WAIT:
            if (LIN_TRMT) {
                LIN_enableRx();
                LIN_rxState = LIN_RX_IDLE;
            } else {
                LIN_rxState = LIN_RX_WAIT;
            }
            break;
    }
    return LIN_rxState;
}

void LIN_sendPacket(uint8_t length, uint8_t pid, uint8_t* data) {

    //Write data    
    for (uint8_t i = 0; i < length; i++) {
        LIN_EUSART_Write(*(data + i));
    }
    //Add Checksum
    LIN_EUSART_Write(LIN_getChecksum(length, pid, data));
}

uint8_t LIN_getPacket(uint8_t* data) {
    uint8_t cmd = LIN_packet.PID & 0x3F;

    memcpy(data, LIN_packet.data, sizeof (LIN_packet.data));
    memset(LIN_packet.rawPacket, 0, sizeof (LIN_packet.rawPacket));

    return cmd;
}

uint8_t LIN_getFromTable(uint8_t cmd, lin_sch_param_t param) {
    const lin_rx_cmd_t* rxCommand = LIN_rxCommand; //copy table pointer so we can modify it

    cmd &= 0x3F; //clear possible parity bits
    //check table
    for (uint8_t length = 0; length < LIN_rxCommandLength; length++) {
        if (cmd == rxCommand->cmd) {
            break;
        }
        rxCommand++; //go to next entry

        if (length == (LIN_rxCommandLength - 1)) {
            return ERROR; //command not in schedule table
        }
    }

    switch (param) {
        case CMD:
            return rxCommand->cmd;
        case TYPE:
            return rxCommand->type;
        case LENGTH:
            return rxCommand->length;
        default:
            break;
    }

    return ERROR;
}

bool LIN_checkPID(uint8_t pid) {
    if (LIN_getFromTable(pid, TYPE) == ERROR)
        return false; //PID not in schedule table

    if (pid == LIN_calcParity(pid & 0x3F))
        return true;

    return false; //Parity Error

}

uint8_t LIN_calcParity(uint8_t CMD) {
    lin_pid_t PID;
    PID.rawPID = CMD;

    //Workaround for compiler bug:
    //    PID.P0 = PID.ID0 ^ PID.ID1 ^ PID.ID2 ^ PID.ID4;
    //    PID.P1 = ~(PID.ID1 ^ PID.ID3 ^ PID.ID4 ^ PID.ID5);
    PID.P0 = PID.ID0 ^ PID.ID1;
    PID.P0 = PID.P0 ^ PID.ID2;
    PID.P0 = PID.P0 ^ PID.ID4;
    PID.P1 = PID.ID1 ^ PID.ID3;
    PID.P1 = PID.P1 ^ PID.ID4;
    PID.P1 = PID.P1 ^ PID.ID5;
    PID.P1 = ~PID.P1;

    return PID.rawPID;
}

uint8_t extern lin_checksum_type;

uint8_t LIN_getChecksum(uint8_t length, uint8_t pid, uint8_t* data) {

    uint16_t checksum = pid;

    if (lin_checksum_type == 'c')
        checksum = 0;

    for (uint8_t i = 0; i < length; i++) {
        checksum = checksum + *data++;
        if (checksum > 0xFF)
            checksum -= 0xFF;
    }
    checksum = ~checksum;

    return (uint8_t) checksum;
}

void LIN_startTimer(uint8_t timeout) {
    LIN_timeout = timeout;
    LIN_WriteTimer(0);
    LIN_StartTimer();
    LIN_timerRunning = true;
}

extern volatile uint8_t EUSART_Tmr;
void LIN_timerHandler(void) {

    // callback function
    EUSART_Tmr ++;
    if (++CountCallBack >= LIN_timeout) {
        // ticker function call
        LIN_stopTimer();
    }
}

void LIN_setTimerHandler(void) {
    LIN_SetInterruptHandler(LIN_timerHandler);
}

void LIN_stopTimer(void) {
    LIN_StopTimer();
    // reset ticker counter
    CountCallBack = 0;
    LIN_timerRunning = false;
}

void LIN_enableRx(void) {
    LIN_CREN = 1;
    LIN_RCIE = 1;
}

void LIN_disableRx(void) {
    LIN_CREN = 0;
    LIN_RCIE = 0;
}

bool LIN_breakCheck(void) {

    if (LIN_FERR == 1) {
        LIN_EUSART_Read();
        return true;
    }

    return false;
}