#ifndef LCD_H
#define LCD_H

#include <stdint.h>

/******************************************************************************
 * LCD setup                                                                  *
 ******************************************************************************/
#define LCD_STARTUP_TIME1 50
#define LCD_STARTUP_TIME2 10
#define LCD_STARTUP_TIME3 1
#define LCD_STARTUP_TIME4 10
#define LCD_DATA 0
#define LCD_INSTRUCTION 1
#define LCD_ADDRESS_SETUP_TIME 1
#define LCD_ENABLE_PULSE_WIDTH 2
#define LCD_EXECUTION_TIME_SHORT 80
#define LCD_EXECUTION_TIME_LONG 3000
#define LCD_LINE_1_ADDR 0X00
#define LCD_LINE_2_ADDR 0X40
#define LCD_LINE_3_ADDR 0X10
#define LCD_LINE_4_ADDR 0X50


/******************************************************************************
 * LCD instructions                                                           *
 ******************************************************************************/
#define LCD_CLEAR 0b00000001
#define LCD_HOME 0b00000010
#define LCD_FUNCTIONSET_8BIT 0b00111100
#define LCD_FUNCTIONSET_4BIT 0b00101100
#define LCD_ENTRYMODE 0b00000110
#define LCD_SET_DDRAM_ADDRESS 0b10000000
#define LCD_SET_CGRAM_ADDRESS 0b01000000
#define LCD_ON 0b00001100
#define LCD_OFF 0b00001000

/******************************************************************************
 * Custom Characters                                                          *
 ******************************************************************************/
#define LCD_CUSTOM_CHARACTER_ae 0x00
#define LCD_CUSTOM_CHARACTER_oe 0x01
#define LCD_CUSTOM_CHARACTER_ue 0x02
#define LCD_CUSTOM_CHARACTER_BAR1 0x03
#define LCD_CUSTOM_CHARACTER_BAR2 0x04
#define LCD_CUSTOM_CHARACTER_BAR3 0x05
#define LCD_CUSTOM_CHARACTER_BAR4 0x06
#define LCD_CUSTOM_CHARACTER_BAR5 0xFF


/******************************************************************************
 * default values                                                             *
 ******************************************************************************/
#define LCD_STARTUP_SCREEN {{' ', 's','o','l','d','e','r','n','e','r','d','.','c','o','m',' '},{'d','B','-','L','i','n','e','a','r',' ','S','M','e','t','e','r'}}
#define LCD_DEFAULT_BRIGHTNESS 150
#define LCD_DEFAULT_CONTRAST 80
#define LCD_MINIMUM_CONTRAST 30
#define LCD_MAXIMUM_CONTRAST 130


/******************************************************************************
 * LCD typedef                                                                *
 ******************************************************************************/
/*
typedef struct
{
    uint8_t content[2][16];
    uint8_t contrast;
    uint8_t brightness;
} lcd_t;
*/

/******************************************************************************
 * Variable definitions                                                       *
 ******************************************************************************/
char lcd_content[2][16] = LCD_STARTUP_SCREEN;

/******************************************************************************
 * Public functions                                                           *
 ******************************************************************************/
void lcd_setup(void);
void lcd_init_4bit(void);
void lcd_refresh_all(void);
void lcd_off(void);
void lcd_on(void);

void lcd_set_contrast(uint8_t contrast);
void lcd_set_brightness(uint8_t brightness);
uint8_t lcd_get_contrast(void);
uint8_t lcd_get_brightness(void);

#endif /* LCD_H */