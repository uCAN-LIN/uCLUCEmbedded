/*
 * slcan_interface.c
 *
 *  Created on: Apr 2, 2016
 *      Author: Vostro1440
 */
#include "string.h"
#include "slcan.h"
#include "./../mcc_generated_files/LINDrivers/lin_slave.h"
#include "./../mcc_generated_files/LINDrivers/lin_master.h"
#include "./../mcc_generated_files/mcc.h"
#include "./../mcc_generated_files/LINDrivers/lin_hardware.h"

#define SLCAN_BELL 7
#define SLCAN_CR 13
#define SLCAN_LR 10

#define STATE_CONFIG 0
#define STATE_LISTEN 1
#define STATE_OPEN 2

#define HAL_OK 1

const int32_t serialNumber = 5;
// internal slcan_interface state

static uint8_t timestamping = 0;

static uint8_t state = STATE_OPEN ;
LinType_t lin_type = LIN_MASTER;

static uint8_t terminator = SLCAN_CR;

extern lin_cmd_packet_t scheduleTable[MAX_LIN_SLAVE_COUNT];

uint8_t sl_frame[LINE_MAXLEN];
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

void slcanClose()
{
	state = STATE_CONFIG;
}

static void slcanOutputFlush(void)
{
    
    while(!USBUSARTIsTxTrfReady())
    {
       CDCTxService();   
    }
    {
        putUSBUSART(sl_frame,sl_frame_len);
    }
    /* send all data */
    CDCTxService();   
    while(!USBUSARTIsTxTrfReady())
    {
       CDCTxService();   
    }
    
    sl_frame_len = 0;
}

void slCanHandler(void)
{
    if (state == STATE_OPEN)
    {
        if (lin_type == LIN_MASTER)
            LIN_Master_handler();
        else 
            LIN_Slave_handler();
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

void LIN_sendHeaderPacket(lin_packet_t LIN_packet_master, bool send_header, bool send_data){
    
    if (send_header == true)
    {
        //Add Break
        LIN_SENDB = 1;
        LIN_EUSART_Write(0x00); //send dummy transmission
        //Add Preamble
        LIN_EUSART_Write(0x55);
        //Add ID
        LIN_EUSART_Write(LIN_packet_master.PID);
        //Build Packet - User defined data
    }
    //add data 
    if (send_data)
    {
        if (LIN_packet_master.length > 0)
        {
            for(uint8_t i = 0; i < LIN_packet_master.length; i++){
                LIN_EUSART_Write(LIN_packet_master.data[i]);
            }
            //Add Checksum
            LIN_EUSART_Write(LIN_packet_master.checksum);
        }
    }    
}

static uint8_t wakeUpLin(void)
{
    LIN_SENDB = 1;
    LIN_EUSART_Write(0x00); //send dummy transmission
    LIN_SENDB = 1;
    LIN_EUSART_Write(0x00); 
    LIN_SENDB = 1;
    LIN_EUSART_Write(0x00); 

}

static uint8_t addLinMasterRow(uint8_t* line) {
    uint32_t temp;
    lin_cmd_packet_t pck;
    uint16_t tFrame_Max_ms;
     
    // reset schedule table
    if (line[1] == '2')
    {
        scheduleLength = 0;
        return 1;
    }
    
    // start sending
    if (line[1] == '1'){
        LIN_Master_init();               
        state = STATE_OPEN;
        if (lin_type == LIN_MASTER){
            wakeUpLin();
        }
        return 1;
    }
    
    // id
    if (!parseHex(&line[2], 2, &temp)) return 0;
    pck.cmd = temp; // add parity
    
    // len
    if (!parseHex(&line[4], 1, &temp)) return 0;
    pck.length = temp;
    if (pck.length > 8) return 0;
     
    // type
    pck.type = ((line[0] == 'r') || (line[0] == 'R'));
    // data
    pck.data = (uint8_t*)(LIN_Master_Data + scheduleLength * sizeof(lin_cmd_packet_t));
    // period
    
//    if (!parseHex(&line[5], 2, &temp)) return 0;
    pck.timeout = 15;
    // timeout
//    if (!parseHex(&line[7], 2, &temp)) return 0;
    tFrame_Max_ms = (((uint16_t)pck.length * 10 + 44) * 7 / 100) + 1;
    pck.period = (uint8_t)(tFrame_Max_ms) + pck.timeout;
    
    if (pck.type == MASTER_TRANSMIT)
    {
        for (uint8_t i = 0; i < pck.length; i++)
        {
            if (!parseHex(&line[5+i*2], 2, &temp)) return 0;
            pck.data[i] = temp;
        }
    }
        
    LIN_Master_Set_Table_Row(&pck);
     
   
    return 1;
}

/**
 * @brief  Interprets given line and transmit can message
 * @param  line Line string which contains the transmit command
 * @retval HAL status
 */
static uint8_t transmitStd(uint8_t* line, bool lin_header, bool lin_data) {
    uint32_t temp;
    lin_packet_t pck;
    
    // id
    if (!parseHex(&line[1], 3, &temp)) return 0;
        pck.PID = LIN_calcParity(temp); // add parity
    // len
    if (!parseHex(&line[4], 1, &temp)) return 0;
    pck.length = temp;
    
    if (pck.length > 8) return 0;
    if (lin_data)
    {
        uint8_t i;
        for (i = 0; i < pck.length; i++) {
            if (!parseHex(&line[5 + i*2], 2, &temp)) return 0;
            pck.data[i] = temp;
        }
        //Add Checksum
        pck.checksum = LIN_getChecksum(pck.length,pck.PID, pck.data);
    }
    LIN_disableRx();  
    LIN_sendHeaderPacket(pck, lin_header, lin_data);
    LIN_enableRx();
    
    return 1;
}


/**
 * @brief  Parse given command line
 * @param  line Line string to parse
 * @retval None
 */
//void RebootToBootloader();
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
        case 'o':  // master mode
        case 'O': 
            if (state == STATE_CONFIG) 
            {
                lin_type = LIN_MASTER;
                result = terminator;
            }
            break;
        case 'L': // Slave mode
        case 'l': 
            if (state == STATE_CONFIG) 
            {
                LIN_Slave_Initialize();
                result = terminator;
                lin_type = LIN_SLAVE;
                state = STATE_OPEN; 
            }
            break;

        case 'C': // Close LIN channel
            state = STATE_CONFIG;
            result = terminator;
            lin_type = LIN_MASTER;
            break;
        case 'r': // Transmit header
        case 'R': 
            if (lin_type == LIN_MASTER)
            {
                addLinMasterRow(line);
                if (line[0] < 'Z') slcanSetOutputChar('Z');
                    else slcanSetOutputChar('z');
                    result = terminator;
            } else 
            {
                if (state == STATE_OPEN)
                {   
                    if (transmitStd(line,true,false) == HAL_OK) {
                        if (line[0] < 'Z') slcanSetOutputChar('Z');
                        else slcanSetOutputChar('z');
                        result = terminator;
                    }
                }   
            }
            break;
        case 't': // Transmit full frame
        case 'T': 
            if (lin_type == LIN_MASTER)
            {
                if (addLinMasterRow(line) == HAL_OK)
                {
                    slcanSetOutputChar('z');
                    result = terminator;
                }
            } else 
            {
                if (state == STATE_OPEN)
                {
                    if (transmitStd(line, true, true) == HAL_OK) {
                        if (line[0] < 'Z') slcanSetOutputChar('Z');
                        else slcanSetOutputChar('z');
                        result = terminator;
                    }
                }
            }
            break;
        case 'F': // Read status flags
            {
                slcanSetOutputChar('F');
                result = terminator;
            }
            break;
         case 'Z': // Call wakeup
            break;
         case 'b':
//        	 RebootToBootloader();
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
uint8_t slcanReciveCanFrame(sl_lin_packet_t *pRxMsg, uint8_t prefix)
{
	uint8_t i;
    lin_pid_t pid;
    
    // @TODO check lin frame type
    slcanSetOutputChar(prefix);

    
    pid.rawPID = pRxMsg->PID;
    pid.P0 = 0;
    pid.P1 = 0;
    slCanSendNibble(0); // for slcan compatibility
    slcanSetOutputAsHex(pid.rawPID);
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
