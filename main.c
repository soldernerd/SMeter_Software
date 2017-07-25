/*
 * File:   main.c
 * Author: Luke
 *
 * Created on 19. July 2017, 22:41
 */

/** INCLUDES *******************************************************/

#include "system.h"
#include <xc.h>
#include <stdint.h>

#include "usb.h"
#include "usb_device_hid.h"
#include "usb_device_msd.h"
//
#include "internal_flash.h"
//
#include "app_device_custom_hid.h"
#include "app_device_msd.h"


//User defined code
#include "os.h"
#include "lcd.h"
#include "i2c.h"
//#include "adc.h"
//#include "flash.h" 


/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
MAIN_RETURN main(void)
{
    //uint8_t cntr;
    uint8_t contrast = 0;
    uint8_t buffer[16]; 
    
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);

    USBDeviceInit();
    USBDeviceAttach();
    
    //PPSOut(RP10,U2RTS_FUNC);
    
    //This is a user defined function
    system_init();
    
    //initialize display
    
    //lcd_off();

    while(1)
    {
        SYSTEM_Tasks();

        //Do this as often as possible
        APP_DeviceMSDTasks();
        
        if(!os.done)
        {
            
            //Run periodic tasks
            switch(os.timeSlot&0b00001111)
            {
                case 1:
                    contrast += 2;
                    if(contrast>120)
                    {
                        contrast = 50;
                    }
                    //CCPR2 = contrast;
                    break;
                    
                case 8:
                    APP_DeviceCustomHIDTasks();
                    break;
                    
                case 9:
                    //i2c_eeprom_read(0x0000, &buffer[0], 2);
                    break;
                    
                case 11:
                    APP_DeviceCustomHIDTasks();
                    break; 
                    
                case 2:
                    
                    break;
            }
            os.done = 1;
        }
    }//end while(1)
}//end main

/*******************************************************************************
 End of File
*/

