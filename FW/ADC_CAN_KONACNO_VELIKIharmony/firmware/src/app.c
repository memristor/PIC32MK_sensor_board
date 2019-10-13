/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
COLOR_SENSOR color_sensor[COLOR_SENSOR_MAX];
COLOR_SENSOR prev_color_sensor[COLOR_SENSOR_MAX];
IR_SENSOR ir_sensor;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_9);
   
    T1CONbits.TECS = 1;
    T1CONbits.TCS = 1;
    T1CONbits.ON = 1;
    T1CONbits.TSYNC = 0;
   
    T7CONbits.TCS = 1;
    T7CONbits.ON = 1;
    T7CONbits.TCS = 1;
    
    T5CONbits.TCS = 1;
    T5CONbits.ON = 1;
    T5CONbits.TCS = 1;
    
    T6CONbits.TCS = 1;
    T6CONbits.ON = 1;
    T6CONbits.TCS = 1;
    
    T4CONbits.TCS = 1;
    T4CONbits.ON = 1;
    T4CONbits.TCS = 1;
       
    T3CONbits.TCS = 1;
    T3CONbits.ON = 1;
    
    T2CONbits.TCS = 1;
    T2CONbits.ON = 1;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

uint8_t CAN_transmit(uint32_t addr, uint8_t data_len, void* CAN_message ){
    return DRV_CAN0_ChannelMessageTransmit(CAN_CHANNEL1, addr, data_len, (uint8_t*)CAN_message);
}

CAN_RX_MSG_BUFFER* CAN_receive(void){
    CAN_CHANNEL_EVENT ChannelEvent = PLIB_CAN_ChannelEventGet(CAN_ID_4, CAN_CHANNEL0);
    if( ChannelEvent & CAN_RX_CHANNEL_NOT_EMPTY){
        CAN_RX_MSG_BUFFER *receivedMsg = PLIB_CAN_ReceivedMessageGet(CAN_ID_4, CAN_CHANNEL0);
        if(receivedMsg ){
            PLIB_CAN_ChannelUpdate(CAN_ID_4, CAN_CHANNEL0);
        }
        return receivedMsg;
    }
    return NULL;
}

void ADC_Read(uint8_t channel){
    uint32_t adc_data;
    switch (channel){
    /*case 0:
        DRV_ADC0_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(0));
        adc_data = DRV_ADC_SamplesRead(0);
        DRV_ADC_Stop();
        DRV_ADC0_Close();
        while(CAN_transmit(0x7800, 4, &adc_data) == false);
        break;
    case 1:
        ADCTRGMODE &= 0xFFF3FFFF;
        DRV_ADC1_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(1));
        adc_data = DRV_ADC_SamplesRead(1);
        DRV_ADC_Stop();
        DRV_ADC1_Close();
        while(CAN_transmit(0x7801, 4, &adc_data) == false);
        break;
    case 2:
        DRV_ADC2_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(2));
        adc_data = DRV_ADC_SamplesRead(2);
        DRV_ADC_Stop();
        DRV_ADC2_Close();
        while(CAN_transmit(0x7802, 4, &adc_data) == false);
        break;
    case 3:
        ADCTRGMODE &= 0xFF7FFFFF;
        DRV_ADC3_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(3));
        adc_data = DRV_ADC_SamplesRead(3);
        DRV_ADC_Stop();
        DRV_ADC3_Close();
        while(CAN_transmit(0x7803, 4, &adc_data) == false);
        break;*/
    case 0:
        ADCTRGMODE &= 0xFCFFFFFF;
        DRV_ADC4_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(4));
        adc_data = DRV_ADC_SamplesRead(4);
        DRV_ADC_Stop();
        DRV_ADC4_Close();
        while(CAN_transmit(0x7800, 4, &adc_data) == false);
        break;
    case 1:
        ADCTRGMODE &= 0xF3FFFFFF;
        DRV_ADC5_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(5));
        adc_data = DRV_ADC_SamplesRead(5);
        DRV_ADC_Stop();
        DRV_ADC5_Close();
        while(CAN_transmit(0x7801, 4, &adc_data) == false);
        break;
    case 2:
        ADCTRGMODE |= 0x08000000;
        DRV_ADC5_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(5));
        adc_data = DRV_ADC_SamplesRead(5);
        DRV_ADC_Stop();
        DRV_ADC5_Close();
        while(CAN_transmit(0x7802, 4, &adc_data) == false);
        break;
    /*case 7:
        ADCTRGMODE |= 0x00080000;
        DRV_ADC1_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(1));
        adc_data = DRV_ADC_SamplesRead(1);
        DRV_ADC_Stop();
        DRV_ADC1_Close();
        while(CAN_transmit(0x7807, 4, &adc_data) == false);
        break;*/
    case 3:
        ADCTRGMODE |= 0x00800000;
        DRV_ADC3_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(3));
        adc_data = DRV_ADC_SamplesRead(3);
        DRV_ADC_Stop();
        DRV_ADC3_Close();
        while(CAN_transmit(0x7803, 4, &adc_data) == false);
        break;
    /*case 9:
        ADCTRGMODE |= 0x02000000;
        DRV_ADC4_Open();
        DRV_ADC_Start();
        while(!DRV_ADC_SamplesAvailable(4));
        adc_data = DRV_ADC_SamplesRead(4);
        DRV_ADC_Stop();
        DRV_ADC4_Close();
        while(CAN_transmit(0x7809, 4, &adc_data) == false);
        break;*/
    }
}

void color_send(uint8_t channel){
    uint16_t color_data[4];
    if(channel >= COLOR_SENSOR_MAX){
        return;
    }
    color_data[0] = prev_color_sensor[channel].red;
    color_data[1] = prev_color_sensor[channel].green;
    color_data[2] = prev_color_sensor[channel].blue;
    color_data[3] = prev_color_sensor[channel].clear;
    while(CAN_transmit(0x7810+channel, 8, color_data) == false);
}

void send_IR_report(){
    uint8_t ir_data=0;
    
    ir_data = (PORTAbits.RA11)|(PORTAbits.RA0<<1)|(PORTAbits.RA1<<2)|(PORTBbits.RB0<<3)|(PORTBbits.RB1<<4);
    
    while(CAN_transmit(0x7820, 1, &ir_data) == false);
}

void check_IR_sensor(){
    ir_sensor.current = (PORTAbits.RA11)|(PORTAbits.RA0<<1)|(PORTAbits.RA1<<2)|(PORTBbits.RB0<<3)|(PORTBbits.RB1<<4);
    //while(CAN_transmit(0x7821, 1, &ir_sensor.current) == false);
    if(ir_sensor.current != ir_sensor.old){
        uint8_t s = 1;
        uint8_t i;
        uint8_t temp = (ir_sensor.current ^ ir_sensor.old);// & ir_sensor.current;
        for(i=0;i<8;i++){
            if(temp & (1<<i)){
                if(ir_sensor.current & (1<<i)){
                    s=1;
                    while(CAN_transmit(0x7821 + i, 1, &s) == false);            
                }
                else
                {
                    s=0;
                    while(CAN_transmit(0x7821 + i, 1, &s) == false);
                }
            }
        }
    }
}
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            uint8_t cnt;
            for(cnt=0; cnt < COLOR_SENSOR_MAX; cnt++){
                color_sensor[cnt].state = CURRENT_RED;
            }
            
            ir_sensor.old = 0;
            ir_sensor.current = 0;
            
        //PLIB_ADC_MuxAInputScanEnable(ADCHS_AN0);
            if (appInitialized)
            {
                 DRV_TMR0_Start(); 
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            
            CAN_RX_MSG_BUFFER* buff = CAN_receive();
            
            if(buff != NULL){
               uint8_t adc_channel,col_channel,ir;
               if(buff->msgEID.eid == 0x7800){
                    adc_channel = buff->data[0];
                    ADC_Read(adc_channel);
               }
               if(buff->msgEID.eid == 0x7810){
                   col_channel = buff->data[0];
                   
                   color_send(col_channel);
               }
               if(buff->msgEID.eid == 0x7820){
                   ir = buff->data[0];
                   if( ir == 0)
                       send_IR_report();
               }

            }
            check_IR_sensor();
            ir_sensor.old = ir_sensor.current;
            break;

        }
        

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
