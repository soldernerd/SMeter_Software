
#include <stdint.h>
#include <xc.h>
#include "os.h"
#include "lcd.h"
#include "i2c.h"
//#include "adc.h"


//12ms for normal load, 1ms for short load
#define TIMER0_LOAD_HIGH_48MHZ 0xB9
#define TIMER0_LOAD_LOW_48MHZ 0xB0
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
    //Set up timer0 for timeSlots
    _system_timer0_init();
    
    //Set up I2C
    i2c_init();

    //Set up LCD and display startup screen
    lcd_setup();
    lcd_init_4bit();
    lcd_refresh_all(); 
    
    //Load calibration
    os.calibration[0] = 2400 * 2;
    os.calibration[1] = 7200 * 2;
    os.calibration[2] = 12000 * 2;
    os.calibration[3] = 16800 * 2;
    os.calibration[4] = 21600 * 2;
    os.calibration[5] = 26400 * 2;
    os.calibration[6] = 31200 * 2;
    os.calibration[7] = 36000 * 2;
    os.calibration[8] = 40800 * 2;
    os.calibration[9] = 54600 * 2;
    os.calibration[10] = 50400 * 2;
    os.calibration[11] = 55200 * 2;
    os.calibration[12] = 60000 * 2;
    os.calibration[13] = 64800 * 2;
}

