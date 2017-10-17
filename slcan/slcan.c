/*
 * slcan_interface.c
 *
 *  Created on: Apr 2, 2016
 *      Author: Vostro1440
 */


#include "string.h"
#include "slcan.h"
#include "./../mcc_generated_files/LINDrivers/lin_slave.h"
#include "./../mcc_generated_files/mcc.h"
#include "./../mcc_generated_files/LINDrivers/lin_hardware.h"



#define LINE_MAXLEN 100
#define SLCAN_BELL 7
#define SLCAN_CR 13
#define SLCAN_LR 10

#define STATE_CONFIG 0
#define STATE_LISTEN 1
#define STATE_OPEN 2

#define HAL_OK 1


extern volatile int32_t serialNumber;
// internal slcan_interface state
static uint8_t state = STATE_CONFIG;
static uint8_t timestamping = 0;


static uint8_t terminator = SLCAN_CR;

uint8_t sl_frame[32];
uint8_t sl_frame_len=0;
/**
  * @brief  Adds data to send buffer
  * @param  c - data to add
  * @retval None
  */
static void slcanSetOutputChar(uint8_t c)
{
	if (sl_frame_len < sizeof(sl_frame))
	{
		sl_frame[sl_frame_len] = c;
		sl_frame_len ++;
	}
}

/**
  * @brief  Add given nible value as hexadecimal string to bufferr
  * @param  c - data to add
  * @retval None
  */
static void slCanSendNibble(uint8_t ch)
{
	ch = ch > 9 ? ch - 10 + 'A' : ch + '0';
	slcanSetOutputChar(ch);
}

/**
  * @brief  Add given byte value as hexadecimal string to buffer
  * @param  value - data to add
  * @retval None
  */
static void slcanSetOutputAsHex(uint8_t ch) {
	slCanSendNibble(ch >> 4);
	slCanSendNibble(ch & 0x0F);
}

// frame buffer
#define FRAME_BUFFER_SIZE 128
uint8_t frameBuffer[FRAME_BUFFER_SIZE];
uint32_t dataToSend = 0;

void slcanClose()
{
	dataToSend = 0;
	state = STATE_CONFIG;
}

static void slcanOutputFlush(void)
{
    while(!USBUSARTIsTxTrfReady()) //TODO to jest ?le !
    {
        putUSBUSART(sl_frame,sl_frame_len);
        sl_frame_len = 0;
    }
}

/**
  * @brief  Add to input buffer data from interfaces
  * @param  ch - data to add
  * @retval None
  */
static uint8_t command[LINE_MAXLEN];
int slCanProccesInput(uint8_t ch)
{
	static uint8_t line[LINE_MAXLEN];
	static uint8_t linepos = 0;

    if (ch == SLCAN_CR) {
        line[linepos] = 0;
        memcpy(command,line,linepos);
        linepos = 0;
        return 1;
    } else if (ch != SLCAN_LR) {
        line[linepos] = ch;
        if (linepos < LINE_MAXLEN - 1) linepos++;
    }
    return 0;
}


/**
  * @brief  Parse hex value of given string
  * @param  canmsg - line Input string
  * 		len    - of characters to interpret
  * 		value  - Pointer to variable for the resulting decoded value
  * @retval 0 on error, 1 on success
  */
static uint8_t parseHex(uint8_t* line, uint8_t len, uint32_t* value) {
    *value = 0;
    while (len--) {
        if (*line == 0) return 0;
        *value <<= 4;
        if ((*line >= '0') && (*line <= '9')) {
           *value += *line - '0';
        } else if ((*line >= 'A') && (*line <= 'F')) {
           *value += *line - 'A' + 10;
        } else if ((*line >= 'a') && (*line <= 'f')) {
           *value += *line - 'a' + 10;
        } else return 0;
        line++;
    }
    return 1;
}

/**
 * @brief  Interprets given line and transmit can message
 * @param  line Line string which contains the transmit command
 * @retval HAL status
 */
static uint8_t transmitStd(uint8_t* line) {
    uint32_t temp;
    lin_packet_t pck = {0};
    
    // id
    if (!parseHex(&line[1], 2, &temp)) return 0;
    pck.PID = temp;
    // type
    if (!parseHex(&line[3], 1, &temp)) return 0;
    pck.type = temp;
    // len
    if (!parseHex(&line[4], 1, &temp)) return 0;
    pck.length = temp;
    
    if (pck.length > 8) return 0;

    uint8_t i;
    for (i = 0; i < pck.length; i++) {
        if (!parseHex(&line[5 + i*2], 2, &temp)) return 0;
        pck.data[i] = temp;
    }
    
    //Add Checksum
    pck.checksum = LIN_getChecksum(pck.length, pck.data);
  // check lin master  
    LIN_EUSART_Write(pck.PID[i]);
    for(uint8_t i = 0; i < pck.length; i++){
        LIN_EUSART_Write(pck.data[i]);
    }
    LIN_EUSART_Write(pck.checksum);
    
    return 1;
}


/**
 * @brief  Parse given command line
 * @param  line Line string to parse
 * @retval None
 */
void RebootToBootloader();
void slCanCheckCommand()
{
	uint8_t result = SLCAN_BELL;
	uint8_t *line = command;
	if (line[0] == 0)
	{
		return ;
	}
    switch (line[0]) {
    	case 'a':
    	{
    		if (terminator == SLCAN_CR)
    			terminator = SLCAN_LR;
    		else
    			terminator = SLCAN_CR;
    		result = terminator;
    		break;
    	}
        case 'S': 
        case 'G': 
        case 'W':
        case 's': 
			result = terminator;
            break;
        case 'V': // Get hardware version
            {
                slcanSetOutputChar('V');
                slcanSetOutputAsHex(VERSION_HARDWARE_MAJOR);
                slcanSetOutputAsHex(VERSION_HARDWARE_MINOR);
                result = terminator;
            }
            break;
        case 'v': // Get firmware version
            {
                slcanSetOutputChar('v');
                slcanSetOutputAsHex(VERSION_FIRMWARE_MAJOR);
                slcanSetOutputAsHex(VERSION_FIRMWARE_MINOR);
                result = terminator;
            }
            break;
        case 'N': // Get serial number
            {
                slcanSetOutputChar('N');
                slcanSetOutputAsHex((uint8_t)(serialNumber));
                slcanSetOutputAsHex((uint8_t)(serialNumber>>8));
                slcanSetOutputAsHex((uint8_t)(serialNumber>>16));
                slcanSetOutputAsHex((uint8_t)(serialNumber>>24));
                result = terminator;
            }
            break;
        case 'o':
        case 'O': // Open CAN channel
   
            break;
        case 'l': // Loop-back mode
            break;
        case 'L': // Open CAN channel in listen-only mode
            break;
        case 'C': // Close CAN channel
            break;
        case 'r': // Transmit standard RTR (11 bit) frame
        case 'R': // Transmit extended RTR (29 bit) frame
        case 't': // Transmit standard (11 bit) frame
        case 'T': // Transmit extended (29 bit) frame
            if (state == STATE_OPEN)
            {
                if (transmitStd(line) == HAL_OK) {
                    if (line[0] < 'Z') slcanSetOutputChar('Z');
                    else slcanSetOutputChar('z');
                    result = terminator;
                }
            }
            break;
        case 'F': // Read status flags
            {
//                unsigned char status = HAL_CAN_GetError(&hcan);
//                unsigned char flags = HAL_CAN_GetError(&hcan);
//                if (flags & 0x01) status |= 0x04; // error warning
//                if (flags & 0xC0) status |= 0x08; // data overrun
//                if (flags & 0x18) status |= 0x20; // passive error
//                if (flags & 0x20) status |= 0x80; // bus error
                slcanSetOutputChar('F');
//                slcanSetOutputAsHex(status);
                result = terminator;
            }
            break;
         case 'Z': // Set time stamping
            {
                unsigned long stamping;
                if (parseHex(&line[1], 1, &stamping)) {
                    timestamping = (stamping != 0);
                    result = terminator;
                }
            }
            break;
         case 'b':
        	 line[0] = 0;
        	 RebootToBootloader();
        	 break;

    }
   line[0] = 0;
   slcanSetOutputChar(result);
   slcanOutputFlush();
}


/**
 * @brief  reciving CAN frame
 * @param  canmsg Pointer to can message
 * 			step Current step
 * @retval Next character to print out
 */
uint8_t slcanReciveCanFrame(lin_packet_t *pRxMsg)
{
	uint8_t i;

    // @TODO check lin frame type
    slcanSetOutputChar('T');
    slcanSetOutputAsHex(pRxMsg->PID);
	slCanSendNibble(pRxMsg->length);
	if (pRxMsg->length > 0)
	{
		for (i = 0;  i != pRxMsg->length; i ++)
		{
			slcanSetOutputAsHex(pRxMsg->data[i]);
		}
	}
	slcanSetOutputChar(terminator);
	slcanOutputFlush();
	return 0;
}