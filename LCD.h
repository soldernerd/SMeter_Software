#ifndef LCD_H
#define LCD_H

#include "header.h"

/******************************************************************************
 * Port locations                                                             *
 ******************************************************************************/
#define LCD_RS 0b00000010 // 0 is instructions, 1 is data
#define LCD_RW 0b00000100 // 0 is write, 1 is read
#define LCD_EN 0b00001000


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
 * LCD instructions                                                           *
 ******************************************************************************/
extern uint8_t SHADOW_A;
extern lcd_struct lcd;


/******************************************************************************
 * Public functions                                                           *
 ******************************************************************************/
void lcd_init_4bit();
void lcd_refresh_all();
void lcd_off();
void lcd_on();
void lcd_read_contrast();
void lcd_read_brightness();
void lcd_write_contrast();
void lcd_set_contrast(uint8_t contrast);
void lcd_write_brightness();
void lcd_set_brightness(uint8_t brightness);


#endif /* LCD_H */