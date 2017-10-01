
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "display.h"
#include "lcd.h"
#include "os.h"

static void _display_clear(void);
static void _display_itoa(int16_t value, uint8_t decimals, char *text);

static void _display_clear(void)
{
    uint8_t row;
    uint8_t col;
    for(row=0;row<2;++row)
    {
        for(col=0;col<16;++col)
        {
            lcd_content[row][col] = ' ';
        }
    }
}

static void _display_itoa(int16_t value, uint8_t decimals, char *text)
{
    uint8_t pos;
    uint8_t len;
    int8_t missing;
    char tmp[10];
    itoa(tmp, value, 10);
    len = strlen(tmp);
    
    if(value<0) //negative values
    {
        missing = decimals + 2 - len;
        if(missing>0) //zero-padding needed
        {
            for(pos=decimals;pos!=0xFF;--pos)
            {
                if(pos>=missing) //there is a character to copy
                {
                    tmp[pos+1] = tmp[pos+1-missing];
                }
                else //there is no character
                {
                    tmp[pos+1] = '0';
                }
            }
            len = decimals + 2;
        }  
    }
    else
    {
        missing = decimals + 1 - len;
        if(missing>0) //zero-padding needed
        {
            for(pos=decimals;pos!=0xFF;--pos)
            {
                if(pos>=missing) //there is a character to copy
                {
                    tmp[pos] = tmp[pos-missing];
                }
                else //there is no character
                {
                    tmp[pos] = '0';
                }
            }
            len = decimals + 1;
        }       
    }
 
    decimals = len - decimals - 1;
    
    for(pos=0;pos<len;++pos)
    {
        text[pos] = tmp[pos];
        if(pos==decimals)
        {
            //Insert decimal point
            ++pos;
            text[pos] = '.';
            break;
        }
    }
    for(;pos<len;++pos)
    {
        text[pos+1] = tmp[pos];
    }
    text[pos+1] = 0;
}

void display_prepare()
{
    char buffer[10];
    uint8_t cntr;
    uint8_t offset;
    int16_t threshold;
    _display_clear();
    //Line 1
    
    //Write ADC value
    /*
    _display_itoa((int16_t) (os.adc_value>>1), 3, &buffer[0]);
    cntr = 0;
    while(buffer[cntr])
        lcd_content[0][cntr] = buffer[cntr++];
     * */
    
    if(os.db_value==-32768)
    {
        //There is no signal
        buffer[0] = '<';
        buffer[1] = '-';
        buffer[2] = '1';
        buffer[3] = '2';
        buffer[4] = '0';
        buffer[5] = 'd';
        buffer[6] = 'B';
        buffer[7] = 'm';
        buffer[8] = 0;
        cntr = 0;
        while(buffer[cntr])
            lcd_content[0][cntr+4] = buffer[cntr++];        
    }
    else if(os.db_value==32767)
    {
        //Overload
        buffer[0] = '>';
        buffer[1] = '+';
        buffer[2] = '1';
        buffer[3] = '0';
        buffer[4] = 'd';
        buffer[5] = 'B';
        buffer[6] = 'm';
        buffer[7] = 0;
        cntr = 0;
        while(buffer[cntr])
            lcd_content[0][cntr+4] = buffer[cntr++];          
    }
    else
    {
        //Write db value
        offset = 2;
        if(os.db_value<0)
            offset--;
        if(os.db_value<-9999)
            offset--;
        if(os.db_value>0)
            lcd_content[0][offset-1] = '+';
        _display_itoa((int16_t) (os.db_value), 2, &buffer[0]);
        cntr = 0;
        while(buffer[cntr])
            lcd_content[0][offset+cntr] = buffer[cntr++];
        lcd_content[0][offset+cntr++] = 'd';
        lcd_content[0][offset+cntr++] = 'B';
        lcd_content[0][offset+cntr++] = 'm';
        
        //Write S value
        lcd_content[0][11] = 'S';
        _display_itoa((int16_t) (os.s_value), 0, &buffer[0]);
        lcd_content[0][12] = buffer[0];
        if(os.s_fraction!=0)
        {
            lcd_content[0][13] = '+';
            _display_itoa((int16_t) (os.s_fraction), 0, &buffer[0]);
            lcd_content[0][14] = buffer[0]; 
            if(os.s_fraction>9)
            {
                lcd_content[0][15] = buffer[1]; 
            }       
        }
    }
        
    if(os.db_value==-32768)
    {
        //There is no signal
        buffer[0] = 'N';
        buffer[1] = 'o';
        buffer[2] = ' ';
        buffer[3] = 'S';
        buffer[4] = 'i';
        buffer[5] = 'g';
        buffer[6] = 'n';
        buffer[7] = 'a';
        buffer[8] = 'l';
        buffer[9] = 0;
        cntr = 0;
        while(buffer[cntr])
            lcd_content[1][cntr+4] = buffer[cntr++];
    }
    else if(os.db_value==32767)
    {
        //Signal overload
        buffer[0] = 'O';
        buffer[1] = 'v';
        buffer[2] = 'e';
        buffer[3] = 'r';
        buffer[4] = 'l';
        buffer[5] = 'o';
        buffer[6] = 'a';
        buffer[7] = 'd';
        buffer[8] = 0;
        cntr = 0;
        while(buffer[cntr])
            lcd_content[1][cntr+5] = buffer[cntr++];
    }
    else
    {
        //Bar chart
        cntr = 0;
        threshold = -11340;
        while(os.db_value>threshold)
        {
            lcd_content[1][cntr] = LCD_CUSTOM_CHARACTER_BAR5;
            ++cntr;
            threshold += 825;
        }
        threshold = os.db_value - threshold + 825;
        if(threshold>660)
        {
            lcd_content[1][cntr] = LCD_CUSTOM_CHARACTER_BAR4;
        }
        else if(threshold>495)
        {
            lcd_content[1][cntr] = LCD_CUSTOM_CHARACTER_BAR3;
        }
        else if(threshold>330)
        {
            lcd_content[1][cntr] = LCD_CUSTOM_CHARACTER_BAR2;
        }
        else if(threshold>165)
        {
            lcd_content[1][cntr] = LCD_CUSTOM_CHARACTER_BAR1;
        }
    }
}

