#include <xc.h>
#include "i2c.h"
#include "os.h"

#define _XTAL_FREQ 8000000

#define I2C_WRITE 0x00
#define I2C_READ 0x01

#define I2C_ADC_SLAVE_ADDRESS 0b11010000
#define I2C_EEPROM_SLAVE_ADDRESS 0b10100000

//eeprom_write_task_t task_list[16];
//uint8_t task_list_read_index = 0;
//uint8_t task_list_write_index = 0;

/* ****************************************************************************
 * General I2C functionality
 * ****************************************************************************/

//Replacements for the PLIB functions

void i2c_init(void)
{
    I2C_SDA_TRIS = PIN_INPUT;
    I2C_SCL_TRIS = PIN_INPUT;
    SSP1STATbits.SMP = 0; //Enable slew rate control
    SSP1STATbits.CKE = 0; //Disable SMBus inputs
    SSP1ADD = 29; //400kHz at 48MHz system clock
    SSP1CON1bits.WCOL = 0; //Clear write colision bit
    SSP1CON1bits.SSPOV = 0; //Clear receive overflow bit bit
    SSP1CON1bits.SSPM = 0b1000; //I2C master mode
    SSP1CON1bits.SSPEN = 1; //Enable module
}

static void _i2c_wait_idle(void)
{
    while(SSP1CON2bits.ACKEN | SSP1CON2bits.RCEN1 | SSP1CON2bits.PEN | SSP1CON2bits.RSEN | SSP1CON2bits.SEN | SSP1STATbits.R_W ){}
}

static void _i2c_start(void)
{
    SSP1CON2bits.SEN=1;
    while(SSP1CON2bits.SEN){}
}

static void _i2c_send(uint8_t dat)
{
    SSP1BUF = dat;
}

static uint8_t _i2c_get(void)
{
    SSP1CON2bits.RCEN = 1 ; //initiate I2C read
    while(SSP1CON2bits.RCEN){} //wait for read to complete
    return SSP1BUF; //return the value in the buffer
}

static void _i2c_stop(void)
{
    SSP1CON2bits.PEN = 1;
    while(SSP1CON2bits.PEN){}
}

static void _i2c_acknowledge(void)
{
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN){}
}

static void _i2c_not_acknowledge(void)
{
    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN){}
}


static void _i2c_write(uint8_t slave_address, uint8_t *data, uint8_t length)
{
    uint8_t cntr;

    _i2c_wait_idle();
    _i2c_start();
    _i2c_wait_idle();
    _i2c_send(slave_address);
    _i2c_wait_idle();
    
    for(cntr=0; cntr<length; ++cntr)
    {
        _i2c_send(data[cntr]);
        _i2c_wait_idle();      
    } 
    
    _i2c_stop();    
}

static void _i2c_read(uint8_t slave_address, uint8_t *data, uint8_t length)
{
    uint8_t cntr;

    _i2c_wait_idle();
    _i2c_start();
    _i2c_wait_idle();
    _i2c_send(slave_address | I2C_READ);
    _i2c_wait_idle();
    /*
    for(cntr=0; cntr<length; ++cntr)
    {
        data[cntr] = _i2c_get();
        _i2c_acknowledge();       
    } 
    _i2c_not_acknowledge();
    */
    
    for(cntr=0; cntr<length-1; ++cntr)
    {
        data[cntr] = _i2c_get();
        _i2c_acknowledge();       
    } 
    data[cntr] = _i2c_get();
    _i2c_not_acknowledge();
     
    _i2c_stop();
}


/* ****************************************************************************
 * I2C ADC Functionality
 * ****************************************************************************/


void i2c_adc_start(i2cAdcResolution_t resolution, i2cAdcGain_t gain)
 {
     uint8_t configuration_byte;
     configuration_byte = 0b10000000;
     configuration_byte |= (resolution<<2);
     configuration_byte |= gain;
     
     _i2c_write(I2C_ADC_SLAVE_ADDRESS, &configuration_byte, 1);
 }
 
 int16_t i2c_adc_read(void)
 {
    int16_t result;
    _i2c_wait_idle();
    _i2c_start();
    _i2c_wait_idle();
    _i2c_send(I2C_ADC_SLAVE_ADDRESS | I2C_READ);
    _i2c_wait_idle();
    result = _i2c_get();
    result <<= 8;
    _i2c_acknowledge();
    result |= _i2c_get();
    _i2c_not_acknowledge();
    _i2c_stop(); 
    return result;
 };
 
/* ****************************************************************************
 * I2C EEPROM Functionality
 * ****************************************************************************/
 
#define EEPROM_CALIBRATION_ADDRESS 0x0100
 
//void _i2c_eeprom_load_default_calibration(calibration_t *buffer, calibrationIndex_t index);
 
void i2c_eeprom_writeByte(uint16_t address, uint8_t data)
{
    uint8_t slave_address;
    uint8_t dat[2];
    
    slave_address = I2C_EEPROM_SLAVE_ADDRESS | ((address&0b0000011100000000)>>7);
    dat[0] = address & 0xFF;
    dat[1] = data;
    
    _i2c_write(slave_address, &dat[0], 2);
}

uint8_t i2c_eeprom_readByte(uint16_t address)
{
    uint8_t slave_address;
    uint8_t addr;
    slave_address = I2C_EEPROM_SLAVE_ADDRESS | ((address&0b0000011100000000)>>7);
    addr = address & 0xFF;
    
    _i2c_write(slave_address, &addr, 1);
    _i2c_read(slave_address, &addr, 1);
    return addr;
}


void i2c_eeprom_write(uint16_t address, uint8_t *data, uint8_t length)
{
    uint8_t cntr;
    uint8_t slave_address;
    uint8_t dat[17];

    slave_address = I2C_EEPROM_SLAVE_ADDRESS | ((address&0b0000011100000000)>>7);
    dat[0] = address & 0xFF;

    length &= 0b00001111;
    for(cntr=0; cntr<length; ++cntr)
    {
        dat[cntr+1] = data[cntr];
    }
    
    _i2c_write(slave_address, &dat[0], length+1);
}

void i2c_eeprom_read(uint16_t address, uint8_t *data, uint8_t length)
{
    uint8_t slave_address;
    uint8_t addr;
    addr = address & 0xFF;
    address &= 0b0000011100000000;
    address >>= 7;
    slave_address = I2C_EEPROM_SLAVE_ADDRESS | address;
    
    _i2c_write(slave_address, &addr, 1);
    _i2c_read(slave_address, &data[0], length);
}

/*
void i2c_eeprom_read_calibration(void)
{
    uint8_t buffer[4];
    uint8_t cntr;
    uint16_t addr;

    for(cntr=0; cntr<CALIBRATION_INDEX_COUNT; ++cntr)
    {
        //Read 4 byte signature
        addr = EEPROM_CALIBRATION_ADDRESS + (cntr<<4);
        i2c_eeprom_read(addr, &buffer[0], 4);
        //Check signature
        if((buffer[0]==0x77) && (buffer[1]==0x55) && (buffer[2]==0x33) && (buffer[3]==cntr))
        {
            //Valid data in EEPROM -> read data
            addr += 4;
            i2c_eeprom_read(addr, (uint8_t*) &calibrationParameters[cntr], 12);
        }
        else
        {
            //No valid data in EEPROM -> write default data
            _i2c_eeprom_load_default_calibration(&calibrationParameters[cntr], cntr);
            //Schedule data to be written
            switch((calibrationIndex_t) cntr)
            {
                case CALIBRATION_INDEX_INPUT_VOLTAGE:
                    schedule_eeprom_write_task(EEPROM_WRITE_TASK_CALIBRATION_INPUT_VOLTAGE);
                    break;
                case CALIBRATION_INDEX_OUTPUT_VOLTAGE:
                    schedule_eeprom_write_task(EEPROM_WRITE_TASK_CALIBRATION_OUTPUT_VOLTAGE);
                    break;
                case CALIBRATION_INDEX_INPUT_CURRENT:
                    schedule_eeprom_write_task(EEPROM_WRITE_TASK_CALIBRATION_INPUT_CURRENT);
                    break;
                case CALIBRATION_INDEX_OUTPUT_CURRENT:
                    schedule_eeprom_write_task(EEPROM_WRITE_TASK_CALIBRATION_OUTPUT_CURRENT);
                    break;
                case CALIBRATION_INDEX_ONBOARD_TEMPERATURE:
                    schedule_eeprom_write_task(EEPROM_WRITE_TASK_CALIBRATION_ONBOARD_TEMPERATURE);
                    break;
                case CALIBRATION_INDEX_EXTERNAL_TEMPERATURE_1:
                    schedule_eeprom_write_task(EEPROM_WRITE_TASK_CALIBRATION_EXTERNAL_TEMPERATURE_1);
                    break;
                case CALIBRATION_INDEX_EXTERNAL_TEMPERATURE_2:
                    schedule_eeprom_write_task(EEPROM_WRITE_TASK_CALIBRATION_EXTERNAL_TEMPERATURE_2);
                    break;
            }
            addr += 4;
            i2c_eeprom_write(addr, (uint8_t*) &calibrationParameters[cntr], 12);
            //Wait for a while
            system_delay_ms(7);
            //Update signature to indicate that data is now valid
            addr -= 4;
            buffer[0] = 0x77;
            buffer[1] = 0x55;
            buffer[2] = 0x33;
            buffer[3] = cntr;
            i2c_eeprom_write(addr, &buffer[0], 4);
            //Wait for a while
            system_delay_ms(7);
             * 
        }
    }   
}

void _i2c_eeprom_load_default_calibration(calibration_t *buffer, calibrationIndex_t index)
{
    switch(index)
    {
        case CALIBRATION_INDEX_INPUT_VOLTAGE:
            (*buffer).NeutralOffset = 0;
            (*buffer).NeutralMultiplier = 11;
            (*buffer).NeutralShift = 4;
            (*buffer).Offset = 0;
            (*buffer).Multiplier = 11;
            (*buffer).Shift = 4;
            (*buffer).AutoCalibration = 0;
            break;
        case CALIBRATION_INDEX_OUTPUT_VOLTAGE:   
            (*buffer).NeutralOffset = 0;
            (*buffer).NeutralMultiplier = 17;
            (*buffer).NeutralShift = 5;
            (*buffer).Offset = 0;
            (*buffer).Multiplier = 17;
            (*buffer).Shift = 5;
            (*buffer).AutoCalibration = 0;
            break;
        case CALIBRATION_INDEX_INPUT_CURRENT:
            (*buffer).NeutralOffset = 0;
            (*buffer).NeutralMultiplier = 5851;
            (*buffer).NeutralShift = 15;
            (*buffer).Offset = 0;
            (*buffer).Multiplier = 5851;
            (*buffer).Shift = 15;
            (*buffer).AutoCalibration = 0;
            break;
        case CALIBRATION_INDEX_OUTPUT_CURRENT:   
            (*buffer).NeutralOffset = 0;
            (*buffer).NeutralMultiplier = 5851;
            (*buffer).NeutralShift = 15;
            (*buffer).Offset = 0;
            (*buffer).Multiplier = 5851;
            (*buffer).Shift = 15;
            (*buffer).AutoCalibration = 0;
            break;
        case CALIBRATION_INDEX_ONBOARD_TEMPERATURE:   
            (*buffer).NeutralOffset = -13769;
            (*buffer).NeutralMultiplier = -11479;
            (*buffer).NeutralShift = 13;
            (*buffer).Offset = -13769;
            (*buffer).Multiplier = -11479;
            (*buffer).Shift = 13;
            (*buffer).AutoCalibration = 0;
            break;
        case CALIBRATION_INDEX_EXTERNAL_TEMPERATURE_1:   
            (*buffer).NeutralOffset = -13769;
            (*buffer).NeutralMultiplier = -11479;
            (*buffer).NeutralShift = 13;
            (*buffer).Offset = -13769;
            (*buffer).Multiplier = -11479;
            (*buffer).Shift = 13;
            (*buffer).AutoCalibration = 0;
            break;
        case CALIBRATION_INDEX_EXTERNAL_TEMPERATURE_2:   
            (*buffer).NeutralOffset = -13769;
            (*buffer).NeutralMultiplier = -11479;
            (*buffer).NeutralShift = 13;
            (*buffer).Offset = -13769;
            (*buffer).Multiplier = -11479;
            (*buffer).Shift = 13;
            (*buffer).AutoCalibration = 0;
            break;
    }
}

void _write_calibration(calibrationIndex_t index)
{
    uint8_t buffer[16];
    uint8_t *ptr;
    uint8_t cntr;
    uint16_t addr;
    buffer[0] = 0x77;
    buffer[1] = 0x55;
    buffer[2] = 0x33;
    buffer[3] = (uint8_t) index;
    ptr = (uint8_t*) &calibrationParameters[index];
    for(cntr=4; cntr<16; ++cntr)
    {
        buffer[cntr] = *ptr;
        ++ptr;
    }
    addr = EEPROM_CALIBRATION_ADDRESS + (index<<4);
    i2c_eeprom_write(addr, &buffer[0], 16);
}


uint8_t get_eeprom_write_task_count(void)
{
       return (task_list_write_index - task_list_read_index) & 0x0F;
}

void schedule_eeprom_write_task(eeprom_write_task_t task)
{
       uint8_t idx;
       //Check if task is already scheduled
       for(idx=task_list_read_index; idx!=task_list_write_index; idx=(idx+1) & 0x0F)
       {
             if (task_list[idx] == task)
             {
                    //Return (i.e. do nothing) if task is already scheduled
                    return;
             }
       }
       //Add task to list
       task_list[task_list_write_index] = task;
       ++task_list_write_index;
       task_list_write_index &= 0x0F;
}

eeprom_write_task_t get_next_eeprom_write_task(void)
{
       eeprom_write_task_t task;
       if (task_list_read_index == task_list_write_index)
       {
             //Buffer is empty
             return EEPROM_WRITE_TASK_NONE;
       }
       else
       {
             //Save task to return
             task = task_list[task_list_read_index];
             //Increment read index
             ++task_list_read_index;
             task_list_read_index &= 0x0F;
             //Return first task
             return task;
       }
}

void i2c_eeprom_tasks()
{
    switch(get_next_eeprom_write_task())
    {
        case EEPROM_WRITE_TASK_REAL_TIME_CLOCK:
            break;
        case EEPROM_WRITE_TASK_CALIBRATION_INPUT_VOLTAGE:
            _write_calibration(CALIBRATION_INDEX_INPUT_VOLTAGE);
            break;
        case EEPROM_WRITE_TASK_CALIBRATION_OUTPUT_VOLTAGE:
            _write_calibration(CALIBRATION_INDEX_OUTPUT_VOLTAGE);
            break;
        case EEPROM_WRITE_TASK_CALIBRATION_INPUT_CURRENT:
            _write_calibration(CALIBRATION_INDEX_INPUT_CURRENT);
            break;
        case EEPROM_WRITE_TASK_CALIBRATION_OUTPUT_CURRENT:
            _write_calibration(CALIBRATION_INDEX_OUTPUT_CURRENT);
            break;
        case EEPROM_WRITE_TASK_CALIBRATION_ONBOARD_TEMPERATURE:
            _write_calibration(CALIBRATION_INDEX_ONBOARD_TEMPERATURE);
            break;
        case EEPROM_WRITE_TASK_CALIBRATION_EXTERNAL_TEMPERATURE_1:
            _write_calibration(CALIBRATION_INDEX_EXTERNAL_TEMPERATURE_1);
            break;
        case EEPROM_WRITE_TASK_CALIBRATION_EXTERNAL_TEMPERATURE_2:
            _write_calibration(CALIBRATION_INDEX_EXTERNAL_TEMPERATURE_2);
            break;
    }
}
*/
