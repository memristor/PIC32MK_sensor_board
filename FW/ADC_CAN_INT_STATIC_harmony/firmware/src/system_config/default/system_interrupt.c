/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************


 
void __ISR(_TIMER_9_VECTOR, ipl4AUTO) IntHandlerDrvTmrInstance0(void)
{

    
    uint8_t i;
    switch(color_sensor[0].state){
        case CURRENT_RED:
            color_sensor[0].red = TMR1;
            TMR1 = 0;
            color_sensor[1].red = TMR2;
            TMR2 = 0;
            color_sensor[2].red = TMR3;
            TMR3 = 0;
            color_sensor[3].red = TMR4;
            TMR4 = 0;
            color_sensor[4].red = TMR5;
            TMR5 = 0;
            color_sensor[5].red = TMR6;
            TMR6 = 0;
            color_sensor[6].red = TMR7;
            TMR7 = 0;
            //color_sensor[7].red = TMR8;
            //TMR8 = 0;
            for(i=0;i<COLOR_SENSOR_MAX;i++)
                color_sensor[i].state = CURRENT_GREEN;
            LATBbits.LATB14 = 1; //S2
            LATCbits.LATC9 = 1; //S3
            break;
        case CURRENT_GREEN:
            color_sensor[0].green = TMR1;
            TMR1 = 0;
            color_sensor[1].green = TMR2;
            TMR2 = 0;
            color_sensor[2].green = TMR3;
            TMR3 = 0;
            color_sensor[3].green = TMR4;
            TMR4 = 0;
            color_sensor[4].green = TMR5;
            TMR5 = 0;
            color_sensor[5].green = TMR6;
            TMR6 = 0;
            color_sensor[6].green = TMR7;
            TMR7 = 0;
            //color_sensor[7].green = TMR8;
            //TMR8 = 0;
            for(i=0;i<COLOR_SENSOR_MAX;i++)
                color_sensor[i].state = CURRENT_BLUE;
            LATBbits.LATB14 = 0; //S2
            LATCbits.LATC9 = 1; //S3
            break;
        case CURRENT_BLUE:
            color_sensor[0].blue = TMR1;
            TMR1 = 0;
            color_sensor[1].blue = TMR2;
            TMR2 = 0;
            color_sensor[2].blue = TMR3;
            TMR3 = 0;
            color_sensor[3].blue = TMR4;
            TMR4 = 0;
            color_sensor[4].blue = TMR5;
            TMR5 = 0;
            color_sensor[5].blue = TMR6;
            TMR6 = 0;
            color_sensor[6].blue = TMR7;
            TMR7 = 0;
           // color_sensor[7].blue = TMR8;
            //TMR8 = 0;
            for(i=0;i<COLOR_SENSOR_MAX;i++)
                color_sensor[i].state = CURRENT_CLEAR;
            LATBbits.LATB14 = 1; //S2
            LATCbits.LATC9 = 0; //S3
            break;
        case CURRENT_CLEAR:
            color_sensor[0].clear = TMR1;
            TMR1 = 0;
            color_sensor[1].clear = TMR2;
            TMR2 = 0;
            color_sensor[2].clear = TMR3;
            TMR3 = 0;
            color_sensor[3].clear = TMR4;
            TMR4 = 0;
            color_sensor[4].clear = TMR5;
            TMR5 = 0;
            color_sensor[5].clear = TMR6;
            TMR6 = 0;
            color_sensor[6].clear = TMR7;
            TMR7 = 0;
           // color_sensor[7].clear = TMR8;
           // TMR8 = 0;
            for(i=0;i<COLOR_SENSOR_MAX;i++){
                color_sensor[i].state = CURRENT_RED;
                prev_color_sensor[i] = color_sensor[i];
                prev_color_sensor[i].state = FINISHED;
            }
            LATBbits.LATB14 = 0; //S2
            LATCbits.LATC9 = 0; //S3
            break;
    }
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_9);
}
 



/*******************************************************************************
 End of File
*/
