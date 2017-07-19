/*
 * File:   main.c
 * Author: Luke
 *
 * Created on 19. July 2017, 22:41
 */

/** INCLUDES *******************************************************/

#include "system.h"

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
//#include "i2c.h"
//#include "display.h"
//#include "ui.h"
//#include "rtcc.h"
//#include "buck.h"
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
    
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);

    USBDeviceInit();
    USBDeviceAttach();
    
    //This is a user defined function
    system_init();

    while(1)
    {
        
        /*
        SYSTEM_Tasks();

        #if defined(USB_POLLING)
            // Interrupt or polling method.  If using polling, must call
            // this function periodically.  This function will take care
            // of processing and responding to SETUP transactions
            // (such as during the enumeration process when you first
            // plug in).  USB hosts require that USB devices should accept
            // and process SETUP packets in a timely fashion.  Therefore,
            // when using polling, this function should be called
            // regularly (such as once every 1.8ms or faster** [see
            // inline code comments in usb_device.c for explanation when
            // "or faster" applies])  In most cases, the USBDeviceTasks()
            // function does not take very long to execute (ex: <100
            // instruction cycles) before it returns.
            USBDeviceTasks();
        #endif
        */
        
        //Do this as often as possible
        APP_DeviceMSDTasks();
        
        if(!os.done)
        {
            //Do this every time
            
            //Shut down outputs when battery voltage drops too low
            if(os.output_voltage<USB_CHARGING_VOLTAGE_MINIMUM)
            {
                system_output_off(OUTPUT_USB);
            }
            if(os.output_voltage<POWER_OUTPUTS_VOLTAGE_MINIMUM)
            {
                system_output_off(OUTPUT_1);
                system_output_off(OUTPUT_2);
                system_output_off(OUTPUT_3);
                system_output_off(OUTPUT_4);
            }
            
            //Run scheduled EEPROM write tasks
            //i2c_eeprom_tasks();
            
            //Run user interface
            //ui_run();
            
            //Measure temperature
            //os.temperature_onboard_adc += adc_read(ADC_CHANNEL_TEMPERATURE_ONBOARD);
            //os.temperature_external_1_adc += adc_read(ADC_CHANNEL_TEMPERATURE_EXTERNAL_1);
            //os.temperature_external_2_adc += adc_read(ADC_CHANNEL_TEMPERATURE_EXTERNAL_2);

            //Run periodic tasks
            switch(os.timeSlot&0b00001111)
            {
                case 1:
                    APP_DeviceCustomHIDTasks();
                    break;
                    
                case 8:
                    APP_DeviceCustomHIDTasks();
                    break;
                    
                case 11:
                    APP_DeviceCustomHIDTasks();
                    break; 
            }
            os.done = 1;
        }

        /*
        //Application specific tasks
        APP_DeviceCustomHIDTasks();
        APP_DeviceMSDTasks();
        
        for(cntr=0;cntr<16;++cntr)
        {
            ui_run();
            flash_dummy_read();
            
            adc_calibrate();
            os.temperature_onboard_adc += adc_read(ADC_CHANNEL_TEMPERATURE_ONBOARD);
            os.temperature_external_1_adc += adc_read(ADC_CHANNEL_TEMPERATURE_EXTERNAL_1);
            os.temperature_external_2_adc += adc_read(ADC_CHANNEL_TEMPERATURE_EXTERNAL_2);
            
            if(cntr==14)
            {
                os.temperature_onboard = adc_calculate_temperature(os.temperature_onboard_adc);
                os.temperature_onboard_adc = 0;
                os.temperature_external_1 = adc_calculate_temperature(os.temperature_external_1_adc);
                os.temperature_external_1_adc = 0;
                os.temperature_external_2 = adc_calculate_temperature(os.temperature_external_2_adc);
                os.temperature_external_2_adc = 0;
                display_prepare(os.display_mode);
                
                adc_calibrate();
            }
            if(cntr==15)
            {
                display_update();
            }
            system_delay_ms(8);
        }
        */
    }//end while(1)
}//end main

/*******************************************************************************
 End of File
*/
