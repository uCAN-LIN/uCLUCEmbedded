/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC16F1455
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
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

#include "mcc_generated_files/mcc.h"
#include "slcan/slcan.h"
#include "mcc_generated_files/LINDrivers/lin_master.h"

/*
                         Main application
 */

uint8_t lin_checksum_type = 'c';

extern LinType_t lin_type;

char * data_data[] = {"abc\r\n"};

void UserApplication(void)
{
    volatile static uint8_t buffer[LINE_MAXLEN] = {"Test\n\r"};
    uint8_t numBytes;
        
    numBytes = getsUSBUSART(buffer,sizeof(buffer)); //until the buffer is free.
    if(numBytes > 0)
    {
        if(USBUSARTIsTxTrfReady())
        {
            putsUSBUSART(buffer);
        }    
    }
}
//void UserApplication(void)
//{
//    if (lin_type == LIN_MASTER)
//         LIN_Master_handler();
//     else 
//         LIN_Slave_handler();
//
//     {
//         static uint8_t buffer[LINE_MAXLEN];
//         uint8_t numBytes;
//         numBytes = getsUSBUSART(buffer,sizeof(buffer)); //until the buffer is free.
//         if(numBytes > 0)
//         {
//             for (uint8_t i; i < numBytes; i++)
//             {
//                 if (slCanProccesInput(buffer[i]))
//                 {
//                     slCanCheckCommand();
//                 }
//             }
//         }
//     }
//}

void main(void)
{
    SYSTEM_Initialize();
    
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

//    LIN_Slave_Initialize();

    while (1)
    {
//        USBDeviceTasks();
        if((USBGetDeviceState() < CONFIGURED_STATE) ||
           (USBIsDeviceSuspended() == true))
        {
            //Either the device is not configured or we are suspended
            //  so we don't want to do execute any application code
            continue;   //go back to the top of the while loop
        }
        else
        {
            //Keep trying to send data to the PC as required
            CDCTxService();
            //Run application code.
            UserApplication();
        }
    }
}
/**
 End of File
*/