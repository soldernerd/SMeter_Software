
#include <stdint.h>
#include <xc.h>
#include "os.h"
#include "lcd.h"
#include "i2c.h"
//#include "adc.h"


//8ms for normal load, 1ms for short load
#define TIMER0_LOAD_HIGH_48MHZ 0xD1
#define TIMER0_LOAD_LOW_48MHZ 0x20
#define TIMER0_LOAD_SHORT_HIGH_48MHZ 0xFA
#define TIMER0_LOAD_SHORT_LOW_48MHZ 0x24

void tmr_isr(void)
{ 
    //Timer 0
    if(INTCONbits.T0IF)
    {
        if(os.done) 
        {
            //8ms until overflow
            TMR0H = TIMER0_LOAD_HIGH_48MHZ;
            TMR0L = TIMER0_LOAD_LOW_48MHZ;
            ++os.timeSlot;
            if(os.timeSlot==NUMBER_OF_TIMESLOTS)
            {
                os.timeSlot = 0;
            }
            os.done = 0;
        }
        else //Clock stretching
        {
            //1ms until overflow
            TMR0H = TIMER0_LOAD_SHORT_HIGH_48MHZ;
            TMR0L = TIMER0_LOAD_SHORT_LOW_48MHZ;
        }
        INTCONbits.T0IF = 0;
    }
}


static void _system_timer0_init(void)
{
    //Clock source = Fosc/4
    T0CONbits.T0CS = 0;
    //Operate in 16bit mode
    T0CONbits.T08BIT = 0;
    //Prescaler=8
    T0CONbits.T0PS2 = 0;
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS0 = 0;
    //Use prescaler
    T0CONbits.PSA = 0;
    //8ms until overflow
    TMR0H = TIMER0_LOAD_HIGH_48MHZ;
    TMR0L = TIMER0_LOAD_LOW_48MHZ;
    //Turn timer0 on
    T0CONbits.TMR0ON = 1;
            
    //Enable timer0 interrupts
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIE = 1;
    
    //Initialize timeSlot
    os.timeSlot = 0;
}

void system_init(void)
{
    uint16_t dat;

    //Set up timer0 for timeSlots
    _system_timer0_init();
    
    //Set up I2C
    i2c_init();
    
    //dat = i2c_eeprom_readByte(0x00);
    i2c_adc_start(I2C_ADC_RESOLUTION_16BIT, I2C_ADC_GAIN_1);
//    __delay_ms(500);
//    __delay_ms(500);
//    __delay_ms(500);
//    dat = i2c_adc_read();
    
    
    //Set up LCD and display startup screen
    lcd_setup();
    lcd_init_4bit();
    lcd_refresh_all();
    
    
}

