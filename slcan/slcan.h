/*
 * slcan_port.h
 *
 *  Created on: Apr 4, 2016
 *      Author: PLLUJUR1
 */

#ifndef SLCAN_PORT_H_
#define SLCAN_PORT_H_

#include "stdint.h"
#include "../mcc_generated_files/LINDrivers/lin_slave.h"

#define VERSION_FIRMWARE_MAJOR 0
#define VERSION_FIRMWARE_MINOR 1

#define VERSION_HARDWARE_MAJOR 0
#define VERSION_HARDWARE_MINOR 1

#define CAN_BR_10K 0
#define CAN_BR_20K 1
#define CAN_BR_50K 2
#define CAN_BR_100K 3
#define CAN_BR_125K 4
#define CAN_BR_250K 5
#define CAN_BR_500K 6
#define CAN_BR_800K 7
#define CAN_BR_1M 8

//ex TZ12020506
#define LINE_MAXLEN 30  

void slcanClose();

typedef union {
    struct {
        uint8_t PID;
        uint8_t data[8];
        uint8_t checksum;
        uint8_t type;
        int8_t length;
    };
    uint8_t rawPacket[13];
}sl_lin_packet_t;

uint8_t slcanReciveCanFrame(sl_lin_packet_t *pRxMsg, uint8_t prefix);
int slCanProccesInput(uint8_t ch);
void slCanCheckCommand();
void slCanHandler(void);

typedef enum {
    LIN_MASTER,
    LIN_SLAVE
} LinType_t ;



extern LinType_t lin_type;

#endif /* SLCAN_PORT_H_ */
