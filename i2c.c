#include <xc.h>
#include "i2c.h"
#include "os.h"

#define _XTAL_FREQ 8000000

#define I2C_WRITE 0x00
#define I2C_READ 0x01

#define I2C_ADC_SLAVE_ADDRESS 0b11010000
#define I2C_EEPROM_SLAVE_ADDRESS 0b10100000

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
 
static uint8_t _i2c_eeprom_busy(void)
{
    uint8_t busy;
    _i2c_wait_idle();
    _i2c_start();
    _i2c_wait_idle();
    _i2c_send(I2C_EEPROM_SLAVE_ADDRESS);
    _i2c_wait_idle();
    //ACKSTAT: Acknowledge Status bit (Master Transmit mode only)
    //1 = Acknowledge was not received from slave
    //0 = Acknowledge was received from slave
    busy = SSP1CON2bits.ACKSTAT;
    _i2c_stop(); 
}
 
void i2c_eeprom_writeByte(uint16_t address, uint8_t data)
{
    uint8_t slave_address;
    uint8_t dat[2];
    
    //Wait for device to be available
    while(_i2c_eeprom_busy());
    
    slave_address = I2C_EEPROM_SLAVE_ADDRESS | ((address&0b0000011100000000)>>7);
    dat[0] = address & 0xFF;
    dat[1] = data;
    
    _i2c_write(slave_address, &dat[0], 2);
}

uint8_t i2c_eeprom_readByte(uint16_t address)
{
    uint8_t slave_address;
    uint8_t addr;
    
    //Wait for device to be available
    while(_i2c_eeprom_busy());
    
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
    
    //Wait for device to be available
    while(_i2c_eeprom_busy());

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
    
    //Wait for device to be available
    while(_i2c_eeprom_busy());
    
    addr = address & 0xFF;
    address &= 0b0000011100000000;
    address >>= 7;
    slave_address = I2C_EEPROM_SLAVE_ADDRESS | address;
    
    _i2c_write(slave_address, &addr, 1);
    _i2c_read(slave_address, &data[0], length);
}

void i2c_eeprom_calibration_write(int32_t value, uint8_t index)
{
    uint16_t address;
    address = I2C_EEPROM_CALIBRATION_ADDRESS + (4*index);
    i2c_eeprom_write(address, (uint8_t *) &value, 4);
}

int32_t i2c_eeprom_calibration_read(uint8_t index)
{
    uint16_t address;
    int32_t tmp;
    address = I2C_EEPROM_CALIBRATION_ADDRESS + (4*index);
    i2c_eeprom_read(address, (uint8_t *) &tmp, 4);
    return tmp;
}
