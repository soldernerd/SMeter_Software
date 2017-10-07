/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
*******************************************************************************/

/** INCLUDES *******************************************************/
#include "usb.h"
#include "usb_device_hid.h"
#include <string.h>
#include "system.h"

#include "os.h"
#include "lcd.h"
//#include "rtcc.h"
//#include "display.h"
//#include "buck.h"


/** VARIABLES ******************************************************/
unsigned char ReceivedDataBuffer[64];
unsigned char ToSendDataBuffer[64];

volatile USB_HANDLE USBOutHandle;    
volatile USB_HANDLE USBInHandle;

/** DEFINITIONS ****************************************************/
typedef enum
{
    //These commands are specific to this application
    COMMAND_GET_STATUS = 0x10,
} CUSTOM_HID_DEMO_COMMANDS;

/** FUNCTIONS ******************************************************/
static void _fill_buffer_get_status(void);
static void _parse_command_short(uint8_t cmd);
static void _parse_command_long(uint8_t cmd, uint8_t data);
static void _parse_command_calibration(uint8_t cmd, uint8_t item, uint8_t dat1, uint8_t dat2, uint8_t dat3);

/*********************************************************************
* Function: void APP_DeviceCustomHIDInitialize(void);
*
* Overview: Initializes the Custom HID demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCustomHIDInitialize()
{
    //initialize the variable holding the handle for the last
    // transmission
    USBInHandle = 0;

    //enable the HID endpoint
    USBEnableEndpoint(CUSTOM_DEVICE_HID_EP, USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = (volatile USB_HANDLE)HIDRxPacket(CUSTOM_DEVICE_HID_EP,(uint8_t*)&ReceivedDataBuffer,64);
}

/*********************************************************************
* Function: void APP_DeviceCustomHIDTasks(void);
*
* Overview: Keeps the Custom HID demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_DeviceCustomHIDInitialize() and APP_DeviceCustomHIDStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCustomHIDTasks()
{   
    uint8_t idx;
    
    /* If the USB device isn't configured yet, we can't really do anything
     * else since we don't have a host to talk to.  So jump back to the
     * top of the while loop. */
    if( USBGetDeviceState() < CONFIGURED_STATE )
    {
        return;
    }

    /* If we are currently suspended, then we need to see if we need to
     * issue a remote wakeup.  In either case, we shouldn't process any
     * keyboard commands since we aren't currently communicating to the host
     * thus just continue back to the start of the while loop. */
    if( USBIsDeviceSuspended()== true )
    {
        return;
    }
    
    //Check if we have received an OUT data packet from the host
    if(HIDRxHandleBusy(USBOutHandle) == false)
    {   
        //We just received a packet of data from the USB host.
        //Check the first uint8_t of the packet to see what command the host
        //application software wants us to fulfill.
        //Look at the data the host sent, to see what kind of application specific command it sent.
        switch(ReceivedDataBuffer[0])				
        {
            case COMMAND_GET_STATUS:
                //Check to make sure the endpoint/buffer is free before we modify the contents
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    //Call function to fill the buffer with general information
                    _fill_buffer_get_status();
                    //Prepare the USB module to send the data packet to the host
                    USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ToSendDataBuffer[0],64);
                }
                break;
        }
        
        //Check if the host expects us to do anything else
        idx = 1;
        while(idx<62)
        {
            switch(ReceivedDataBuffer[idx] & 0xF0)
            {
                case 0x30:
                    _parse_command_short(ReceivedDataBuffer[idx]);
                    ++idx;
                    break;
                case 0x40:
                    _parse_command_long(ReceivedDataBuffer[idx], ReceivedDataBuffer[idx+1]);
                    idx += 2;
                    break;
                case 0x60:
                    _parse_command_calibration(ReceivedDataBuffer[idx], ReceivedDataBuffer[idx+1], ReceivedDataBuffer[idx+2], ReceivedDataBuffer[idx+3], ReceivedDataBuffer[idx+4]);
                    idx += 5;
                    break;
                default:
                    idx = 65; //exit loop
            }
        }

        //Re-arm the OUT endpoint, so we can receive the next OUT data packet 
        //that the host may try to send us.
        USBOutHandle = HIDRxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ReceivedDataBuffer, 64);
    }
}


//Fill buffer with general status information
static void _fill_buffer_get_status(void)
{
    uint8_t i;
    //Echo back to the host PC the command we are fulfilling in the first uint8_t
    ToSendDataBuffer[0] = COMMAND_GET_STATUS;
    //Copy entire os struct into buffer
    //memcpy(&ToSendDataBuffer[1], &os, sizeof os);
    ToSendDataBuffer[1] = (uint8_t) os.adc_values[os.timeSlot&0b00001111]; //LSB
    ToSendDataBuffer[2] = (uint8_t) (os.adc_values[os.timeSlot&0b00001111] >> 8); //MSB
    ToSendDataBuffer[3] = (uint8_t) os.adc_sum; //LSB
    ToSendDataBuffer[4] = (uint8_t) (os.adc_sum >> 8);
    ToSendDataBuffer[5] = (uint8_t) (os.adc_sum >> 16);
    ToSendDataBuffer[6] = (uint8_t) (os.adc_sum >> 24); //MSB
    ToSendDataBuffer[7] = (uint8_t) os.db_value; //LSB
    ToSendDataBuffer[8] = (uint8_t) (os.db_value >> 8);
    ToSendDataBuffer[9] = os.s_value;
    ToSendDataBuffer[10] = os.s_fraction;
    ToSendDataBuffer[11] = lcd_get_brightness();
    ToSendDataBuffer[12] = lcd_get_contrast();
    ToSendDataBuffer[13] = lcd_get_saved_brightness();
    ToSendDataBuffer[14] = lcd_get_saved_contrast();
    //Calibration. 3 bytes each
    for(i=0; i<14; ++i)
    {
        ToSendDataBuffer[3*i+21] = (uint8_t) (os.calibration[i]);
        ToSendDataBuffer[3*i+22] = (uint8_t) (os.calibration[i] >> 8);
        ToSendDataBuffer[3*i+23] = (uint8_t) (os.calibration[i] >> 16);  
    }
}


static void _parse_command_short(uint8_t cmd)
{
    switch(cmd)
    {
        case 0x30:
            lcd_save_brightness_contrast();
            break;
        case 0x31:
            lcd_reset_brightness_contrast();
            break;
        case 0x32:
            system_restore_default_calibration();
            break;
    }
}

static void _parse_command_long(uint8_t cmd, uint8_t data)
{
    switch(cmd)
    {
        //Display brightness
        case 0x40:
            lcd_set_brightness(data);
            break;
        //Display contrast
        case 0x41: 
            lcd_set_contrast(data);
            break;
    }    
}

static void _parse_command_calibration(uint8_t cmd, uint8_t item, uint8_t dat1, uint8_t dat2, uint8_t dat3)
{
    int32_t parameter = 0;
    parameter |= dat1;
    parameter <<= 8;
    parameter |= dat2;
    parameter <<= 8;
    parameter |= dat3;
    
    switch(cmd)
    {
        //Calibration
        case 0x60:
            os.calibration[item] = parameter;
            break;
    }  
}

