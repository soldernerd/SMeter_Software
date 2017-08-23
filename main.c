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
#include "display.h"

static void _calculate_adc_sum(void);
static void _calculate_db_value(void);
static void _calculate_s_value(void);


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
    uint8_t startup_timer;
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);

    USBDeviceInit();
    USBDeviceAttach();
    
    //This is a user defined function
    system_init();

    startup_timer = 0;
    while(1)
    {
        SYSTEM_Tasks();

        //Do this as often as possible
        APP_DeviceMSDTasks();

        if(!os.done)
        { 
            //Do this every time
            os.adc_values[os.timeSlot&0b00001111] = i2c_adc_read();
            i2c_adc_start(I2C_ADC_RESOLUTION_14BIT, I2C_ADC_GAIN_1);
            APP_DeviceCustomHIDTasks();
            
            //Run periodic tasks
            switch(os.timeSlot&0b00001111)
            {       
                case 14:
                    _calculate_adc_sum();
                    _calculate_db_value();
                    _calculate_s_value();
                    break;
                    
                case 15:
                    if(startup_timer<10)
                    {    
                        ++startup_timer;
                    }
                    else
                    {
                        display_prepare();
                        lcd_refresh_all();
                    }    
                    break;
            }
            os.done = 1;
        }
    }//end while(1)
}//end main




static void _calculate_adc_sum(void)
{
    uint8_t cntr;
    os.adc_sum = 0;
    for(cntr=0;cntr<16;++cntr)
    {
        os.adc_sum += os.adc_values[cntr];
    }
}

static void _calculate_db_value(void)
{
    int16_t cntr;
    int16_t returnValue;
    int32_t tmp;
   
    if (os.adc_sum < os.calibration[0])
    {
        //Indicate an unterflow
        os.db_value = -32768;
        return;
    }
    if (os.adc_sum > os.calibration[13])
    {
        //Indicate an overflow
        os.db_value =  32767;
        return;
    }
    for(cntr=0; cntr<13; ++cntr)
    {
        if(os.adc_sum<os.calibration[cntr+1])
        {
            returnValue = -12000;
            returnValue += (cntr * 1000);
            tmp = 1000 * (os.adc_sum - os.calibration[cntr]);
            tmp /=  (os.calibration[cntr + 1] - os.calibration[cntr]);
            returnValue += (int16_t) tmp;
            os.db_value = returnValue;
        }
    }
}

static void _calculate_s_value(void)
{
    int16_t tmp;
    //Calculate full S value
    tmp = os.db_value + 12749;
    os.s_value = (uint8_t) (tmp / 600); 
    //Calculate S fraction
    tmp = tmp % 600;
    os.s_fraction = (uint8_t) (tmp / 100);
};

/*******************************************************************************
 End of File
*/
