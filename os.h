/* 
 * File:   system.h
 * Author: Luke
 *
 * Created on 5. September 2016, 21:17
 */

#ifndef OS_H
#define	OS_H

#include <stdint.h>

/*
 * General definitions
 */

#define _XTAL_FREQ 8000000

#define NUMBER_OF_TIMESLOTS 16

#define  PPSUnLock()    {EECON2 = 0b01010101; EECON2 = 0b10101010; PPSCONbits.IOLOCK = 0;}
#define  PPSLock() 		{EECON2 = 0b01010101; EECON2 = 0b10101010; PPSCONbits.IOLOCK = 1;}

#define PPS_FUNCTION_CCP1_OUTPUT 14
#define PPS_FUNCTION_CCP2_OUTPUT 18

#define PIN_INPUT           1
#define PIN_OUTPUT          0
#define PIN_DIGITAL         1
#define PIN_ANALOG          0

#define LCD_BACKLIGHT_TRIS TRISCbits.TRISC2
#define LCD_BACKLIGHT_PIN LATCbits.LATC2
#define LCD_BACKLIGHT_PPS RPOR13

#define LCD_CONTRAST_TRIS TRISCbits.TRISC7
#define LCD_CONTRAST_PIN LATCbits.LATC7
#define LCD_CONTRAST_PPS RPOR18

#define LCD_D0_TRIS TRISBbits.TRISB3
#define LCD_D0_PIN LATBbits.LATB3

#define LCD_D1_TRIS TRISAbits.TRISA0
#define LCD_D1_PIN LATAbits.LATA0

#define LCD_D2_TRIS TRISAbits.TRISA1
#define LCD_D2_PIN LATAbits.LATA1

#define LCD_D3_TRIS TRISAbits.TRISA2
#define LCD_D3_PIN LATAbits.LATA2

#define LCD_D4_TRIS TRISAbits.TRISA3
#define LCD_D4_PIN LATAbits.LATA3

#define LCD_D5_TRIS TRISAbits.TRISA5
#define LCD_D5_PIN LATAbits.LATA5

#define LCD_D6_TRIS TRISCbits.TRISC0
#define LCD_D6_PIN LATCbits.LATC0

#define LCD_D7_TRIS TRISCbits.TRISC1
#define LCD_D7_PIN LATCbits.LATC1

#define LCD_E_TRIS TRISBbits.TRISB2
#define LCD_E_PIN LATBbits.LATB2

#define LCD_RW_TRIS TRISBbits.TRISB1
#define LCD_RW_PIN LATBbits.LATB1

#define LCD_RS_TRIS TRISBbits.TRISB0
#define LCD_RS_PIN LATBbits.LATB0

#define I2C_SDA_TRIS TRISBbits.TRISB3
#define I2C_SDA_PIN LATBbits.LATB3

#define I2C_SCL_TRIS TRISBbits.TRISB4
#define I2C_SCL_PIN LATBbits.LATB4


/*
 * Type definitions
 */

typedef struct
{
    volatile uint8_t timeSlot;
    volatile uint8_t done;
} os_t;


/*
 * Global variables
 */

os_t os;


/*
 * Function prototypes
 */


void tmr_isr(void);
void system_init(void);

#endif	/* OS_H */

