#include <xc.h>
#include "lcd.h"
#include "os.h"


/******************************************************************************
 * Static functions                                                           *
 ******************************************************************************/
static void _lcd_write_8_bitfixwait(uint8_t type, uint8_t dat);
static void _lcd_write_4_bitfixwait(uint8_t type, uint8_t dat);
static void _lcd_write_4(uint8_t type, uint8_t dat);
static void _lcd_a_umlaut();
static void _lcd_wait_while_busy();

/******************************************************************************
 * Port and pwm setup                                                         *
 ******************************************************************************/
void lcd_setup(void)
{
    //Configure ports
    LCD_BACKLIGHT_TRIS = PIN_OUTPUT;
    LCD_BACKLIGHT_PIN = 0;
    LCD_BACKLIGHT_PPS = PPS_FUNCTION_CCP1_OUTPUT;
    
    LCD_CONTRAST_TRIS = PIN_OUTPUT;
    LCD_CONTRAST_PIN = 0;
    LCD_CONTRAST_PPS = PPS_FUNCTION_CCP2_OUTPUT;
    
    LCD_D0_TRIS = PIN_OUTPUT;
    LCD_D0_PIN = 0;
    
    LCD_D1_TRIS = PIN_OUTPUT;
    LCD_D1_PIN = 0;
    
    LCD_D2_TRIS = PIN_OUTPUT;
    LCD_D2_PIN = 0;
    
    LCD_D3_TRIS = PIN_OUTPUT;
    LCD_D3_PIN = 0;
    
    LCD_D4_TRIS = PIN_OUTPUT;
    LCD_D4_PIN = 0;
    
    LCD_D5_TRIS = PIN_OUTPUT;
    LCD_D5_PIN = 0;
    
    LCD_D6_TRIS = PIN_OUTPUT;
    LCD_D6_PIN = 0;
    
    LCD_D7_TRIS = PIN_OUTPUT;
    LCD_D7_PIN = 0;
    
    LCD_E_TRIS = PIN_OUTPUT;
    LCD_E_PIN = 0;
    
    LCD_RW_TRIS = PIN_OUTPUT;
    LCD_RW_PIN = 0;
    
    LCD_RS_TRIS = PIN_OUTPUT;
    LCD_RS_PIN = 0;
    
    //Both CCP modules use timer 2 in PWM mode
    TCLKCONbits.T3CCP2 = 0;
    TCLKCONbits.T3CCP1 = 0;
    
    //Configure timer 2
    //Prescaler = 1
    T2CONbits.T2CKPS = 0b00;
    //Postscaler = 1
    T2CONbits.T2OUTPS = 0b0000; 
    //Enable timer 2
    T2CONbits.TMR2ON = 1;
    
    //Configure backlight PWM
    //Single output
    CCP1CONbits.P1M = 0b00;
    //LSB of duty cycle
    CCP1CONbits.DC1B = 0b00;
    //Active high PWM mode
    CCP1CONbits.CCP1M = 0b1100;
    //Duty cycle MSB
    CCPR1 = LCD_DEFAULT_BRIGHTNESS;
    
    //Configure contrast PWM
    //Single output
    CCP2CONbits.P2M = 0b00;
    //LSB of duty cycle
    CCP2CONbits.DC2B = 0b00;
    //Active high PWM mode
    CCP2CONbits.CCP2M = 0b1100;
    //Duty cycle MSB
    CCPR2 = LCD_DEFAULT_CONTRAST;    
}

/******************************************************************************
 * Read contrast from EEPROM                                                  *
 ******************************************************************************/
//void lcd_read_contrast()
//{
//  lcd.contrast = eeprom_read(LCD_CONTRAST_EEPROM_ADDRESS);
//}

/******************************************************************************
 * Read brightness from EEPROM                                                *
 ******************************************************************************/
//void lcd_read_brightness()
//{
//  lcd.brightness = eeprom_read(LCD_BRIGHTNESS_EEPROM_ADDRESS);
//}

/******************************************************************************
 * Write contrast to EEPROM                                                   *
 ******************************************************************************/
//void lcd_write_contrast()
//{
//  eeprom_write(LCD_CONTRAST_EEPROM_ADDRESS, lcd.contrast);
//}

/******************************************************************************
 * Write brightness to EEPROM                                                 *
 ******************************************************************************/
//void lcd_write_brightness()
//{
//  eeprom_write(LCD_BRIGHTNESS_EEPROM_ADDRESS, lcd.brightness);
//}

/******************************************************************************
 * Adjust contrast                                                            *
 ******************************************************************************/
//void lcd_set_contrast(uint8_t contrast)
//{
//  lcd.contrast = contrast;
//  contrast *= LCD_CONTRAST_MULTIPLIER;
//  contrast += LCD_CONTRAST_OFFSET;
//  PWM2_Set_Duty(contrast);
//}

/******************************************************************************
 * Adjust brightness                                                          *
 ******************************************************************************/
//void lcd_set_brightness(uint8_t brightness)
//{
//  lcd.brightness = brightness;
//  PWM1_Set_Duty(lcd.dutycycles[brightness]);
//}

/******************************************************************************
 * Wait until LCD display is ready for next instruction                       *
 ******************************************************************************/
//void lcd_wait_while_busy()
//{
//  uint8_t shadow_save = SHADOW_A;
//  TRISA = 0b11110000; //configure D7:D4 as inputs
//  SHADOW_A = LCD_RW; //read
//  PORTA = SHADOW_A;
//  SHADOW_A |= LCD_EN; //EN high
//  PORTA = SHADOW_A;
//  while(PORTA & 0b10000000)
//  {
//    delay_us(1);
//  }
//  SHADOW_A = shadow_save;
//  PORTA = SHADOW_A;
//  TRISA = 0x00; //configure D7:D4 as outputs
//}

/******************************************************************************
 * Write to LCD                                                               * 
 * Used during first half of initialization while display is in 8-bit mode    *
 * Wait times are fixed since busy flag is not valid before initialization    *
 * is complete                                                                *
 ******************************************************************************/
static void lcd_write_8bit_fixwait(uint8_t type, uint8_t dat)
{
    //Set up RS, RW, EN
    //LCD_RS_PIN = (type==LCD_DATA); // high for data, low for instruction
    if(type==LCD_DATA)
    {
      LCD_RS_PIN = 1; //data
    }
    if(type==LCD_INSTRUCTION)
    {
      LCD_RS_PIN = 0; //instruction
    }
    LCD_RW_PIN = 0; //write
    LCD_E_PIN = 0; // EN low
    //Set up data
    //LCD_D0_PIN = dat & 0b00000001;
    //LCD_D1_PIN = dat & 0b00000010;
    //LCD_D2_PIN = dat & 0b00000100;
    //LCD_D3_PIN = dat & 0b00001000;
    //LCD_D4_PIN = dat & 0b00010000;
    //LCD_D5_PIN = dat & 0b00100000;
    //LCD_D6_PIN = dat & 0b01000000;
    //LCD_D7_PIN = dat & 0b10000000;
    LCD_D7_PIN = (dat>>7) & 0b1;
    LCD_D6_PIN = (dat>>6) & 0b1;
    LCD_D5_PIN = (dat>>5) & 0b1;
    LCD_D4_PIN = (dat>>4) & 0b1;
    //Wait
    __delay_us(10*LCD_ADDRESS_SETUP_TIME);
    //Set enable high
    LCD_E_PIN = 1; // EN high
    //Wait
    __delay_us(10*LCD_ENABLE_PULSE_WIDTH);
    //Set enable low
    LCD_E_PIN = 0; // EN low
    //Wait
    if((type==LCD_INSTRUCTION) & (dat==LCD_CLEAR | dat==LCD_HOME))
    {
        __delay_us(10*LCD_EXECUTION_TIME_LONG);
    }
    else
    {
        __delay_us(10*LCD_EXECUTION_TIME_SHORT);
    }
}

/*
static void lcd_write_8bit_fixwait(uint8_t type, uint8_t dat)
{
  //Set up output pins
  if(type==LCD_DATA)
  {
    SHADOW_A |= LCD_RS; //data
  }
  if(type==LCD_INSTRUCTION)
  {
    SHADOW_A &= (~LCD_RS); //instruction
  }
  SHADOW_A &= (~LCD_RW); //write
  SHADOW_A &= (~LCD_EN); //EN low
  PORTA = SHADOW_A; //prepare RS, RW, EN
  //Set up data
  SHADOW_A &= 0b00001111;
  SHADOW_A |= (0b11110000 & dat);
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ADDRESS_SETUP_TIME);
  //Set enable high
  SHADOW_A |= LCD_EN; //EN high
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ENABLE_PULSE_WIDTH);
  //Set enable low
  SHADOW_A &= (~LCD_EN); //EN low
  PORTA = SHADOW_A;
  //Wait
  if(type=LCD_INSTRUCTION & (dat==LCD_CLEAR | dat==LCD_HOME))
  {
    delay_us(LCD_EXECUTION_TIME_LONG);
  }
  else
  {
    delay_us(LCD_EXECUTION_TIME_SHORT);
  }
} 
 */

/******************************************************************************
 * Write to LCD                                                               *
 * Used during second half of initialization while display is in 4-bit mode   *
 * Wait times are fixed since busy flag is not valid before initialization    *
 * is complete                                                                *
 ******************************************************************************/
static void lcd_write_4bit_fixwait(uint8_t type, uint8_t dat)
{
    //Set up RS, RW, EN
    //LCD_RS_PIN = (type==LCD_DATA); // high for data, low for instruction
    if(type==LCD_DATA)
    {
      LCD_RS_PIN = 1; //data
    }
    if(type==LCD_INSTRUCTION)
    {
      LCD_RS_PIN = 0; //instruction
    }
    LCD_RW_PIN = 0; //write
    LCD_E_PIN = 0; // EN low
    //Set up first nibble
    //LCD_D4_PIN = dat & 0b00010000;
    //LCD_D5_PIN = dat & 0b00100000;
    //LCD_D6_PIN = dat & 0b01000000;
    //LCD_D7_PIN = dat & 0b10000000;
    LCD_D7_PIN = (dat>>7) & 0b1;
    LCD_D6_PIN = (dat>>6) & 0b1;
    LCD_D5_PIN = (dat>>5) & 0b1;
    LCD_D4_PIN = (dat>>4) & 0b1;
    //Wait
    __delay_us(10*LCD_ADDRESS_SETUP_TIME);
    //Set enable high
    LCD_E_PIN = 1; // EN high
    //Wait
    __delay_us(10*LCD_ENABLE_PULSE_WIDTH);
    //Set enable low
    LCD_E_PIN = 0; // EN low
    //Wait
    __delay_us(10*LCD_EXECUTION_TIME_SHORT);
    //Set up second nibble
    //LCD_D4_PIN = dat & 0b00000001;
    //LCD_D5_PIN = dat & 0b00000010;
    //LCD_D6_PIN = dat & 0b00000100;
    //LCD_D7_PIN = dat & 0b00001000;
    LCD_D7_PIN = (dat>>3) & 0b1;
    LCD_D6_PIN = (dat>>2) & 0b1;
    LCD_D5_PIN = (dat>>1) & 0b1;
    LCD_D4_PIN = (dat>>0) & 0b1;
    //Wait
    __delay_us(10*LCD_ADDRESS_SETUP_TIME);
    //Set enable high
    LCD_E_PIN = 1; // EN high
    //Wait
    __delay_us(10*LCD_ENABLE_PULSE_WIDTH);
    //Set enable low
    LCD_E_PIN = 0; // EN low
    //Wait
    if((type==LCD_INSTRUCTION) & (dat==LCD_CLEAR | dat==LCD_HOME))
    {
        __delay_us(10*LCD_EXECUTION_TIME_LONG);
    }
    else
    {
        __delay_us(10*LCD_EXECUTION_TIME_SHORT);
    }
}

/*
static void lcd_write_4bit_fixwait(uint8_t type, uint8_t dat)
{
  //Set up output pins
  if(type==LCD_DATA)
  {
    SHADOW_A |= LCD_RS; //data
  }
  if(type==LCD_INSTRUCTION)
  {
    SHADOW_A &= (~LCD_RS); //instruction
  }
  SHADOW_A &= (~LCD_RW); //write
  SHADOW_A &= (~LCD_EN); //EN low
  PORTA = SHADOW_A; //prepare RS, RW, EN
  //Set up first nibble
  SHADOW_A &= 0b00001111;
  SHADOW_A |= (0b11110000 & dat);
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ADDRESS_SETUP_TIME);
  //Set enable high
  SHADOW_A |= LCD_EN; //EN high
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ENABLE_PULSE_WIDTH);
  //Set enable low
  SHADOW_A &= (~LCD_EN); //EN low
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_EXECUTION_TIME_SHORT);
  //Set up second nibble
  dat <<= 4;
  SHADOW_A &= 0b00001111;
  SHADOW_A |= (0b11110000 & dat);
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ADDRESS_SETUP_TIME);
  //Set enable high
  SHADOW_A |= LCD_EN; //EN high
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ENABLE_PULSE_WIDTH);
  //Set enable low
  SHADOW_A &= (~LCD_EN); //EN low
  PORTA = SHADOW_A;
  //Wait
  if(type=LCD_INSTRUCTION & (dat==LCD_CLEAR | dat==LCD_HOME))
  {
    delay_us(LCD_EXECUTION_TIME_LONG);
  }
  else
  {
    delay_us(LCD_EXECUTION_TIME_SHORT);
  }
}
*/


/******************************************************************************
 * Write to LCD                                                               *
 * Used once initialization is complete                                       *
 * Wait times depend on busy flag to maximize performance                     *
 ******************************************************************************/

static void lcd_write_4bit(uint8_t type, uint8_t dat)
{
    lcd_write_4bit_fixwait(type, dat);
  /*
  //Wait
  lcd_wait_while_busy();
  //Set up output pins
  if(type==LCD_DATA)
  {
    SHADOW_A |= LCD_RS; //data
  }
  if(type==LCD_INSTRUCTION)
  {
    SHADOW_A &= (~LCD_RS); //instruction
  }
  SHADOW_A &= (~LCD_RW); //write
  SHADOW_A &= (~LCD_EN); //EN low
  //PORTA = SHADOW_A; //prepare RS, RW, EN
  //Set up first nibble
  SHADOW_A &= 0b00001111;
  SHADOW_A |= (0b11110000 & dat);
  //Send Data
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ADDRESS_SETUP_TIME);
  //Set enable high
  SHADOW_A |= LCD_EN; //EN high
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ENABLE_PULSE_WIDTH);
  //Set enable low
  SHADOW_A &= (~LCD_EN); //EN low
  PORTA = SHADOW_A;
  //Wait
  lcd_wait_while_busy();
  //Set up second nibble
  dat <<= 4;
  SHADOW_A &= 0b00001111;
  SHADOW_A |= (0b11110000 & dat);
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ADDRESS_SETUP_TIME);
  //Set enable high
  SHADOW_A |= LCD_EN; //EN high
  PORTA = SHADOW_A;
  //Wait
  delay_us(LCD_ENABLE_PULSE_WIDTH);
  //Set enable low
  SHADOW_A &= (~LCD_EN); //EN low
  PORTA = SHADOW_A;
  */
}


/******************************************************************************
 * Sets up character "ä" as a custom character                                *
 ******************************************************************************/
/*
static void lcd_a_umlaut()
{
  lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_CGRAM_ADDRESS | 0));
  lcd_write_4bit(LCD_DATA, 0b00001010);
  lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_CGRAM_ADDRESS | 1));
  lcd_write_4bit(LCD_DATA, 0b00000000);
  lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_CGRAM_ADDRESS | 2));
  lcd_write_4bit(LCD_DATA, 0b00001110);
  lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_CGRAM_ADDRESS | 3));
  lcd_write_4bit(LCD_DATA, 0b00000001);
  lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_CGRAM_ADDRESS | 4));
  lcd_write_4bit(LCD_DATA, 0b00001111);
  lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_CGRAM_ADDRESS | 5));
  lcd_write_4bit(LCD_DATA, 0b00010001);
  lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_CGRAM_ADDRESS | 6));
  lcd_write_4bit(LCD_DATA, 0b00001111);
  lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_CGRAM_ADDRESS | 7));
  lcd_write_4bit(LCD_DATA, 0b00000000);
}
*/

/******************************************************************************
 * Initialize LCD display in 4-bit mode                                       *
 ******************************************************************************/
void lcd_init_4bit()
{
    //Variable declarations
    //uint8_t dutycycles[LCD_DUTYCYCLE_COUNT] = LCD_DUTYCYCLE_VALUES;
    //uint8_t line;
    //uint8_t character;
    //Prepare duty cycle array
    //for(line=0;line<LCD_DUTYCYCLE_COUNT;++line)
    //{
    //    lcd.dutycycles[line] = dutycycles[line];
    //}
    //Configure Port
    //TRISA = 0x00;
    //ANSELA = 0x00;
    //Configure LCD brightness (PWM1) and contrast (PWM2)
    //PWM1_Init(5000);
    //PWM2_Init(5000);
    //lcd_read_brightness();
    //lcd_set_brightness(lcd.brightness);
    //lcd_read_contrast();
    //lcd_set_contrast(lcd.contrast);
    //PWM1_Start();
    //PWM2_Start();
    //Wait
    __delay_ms(10*LCD_STARTUP_TIME1);
    //Initialize LCD display
    lcd_write_8bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_8BIT);
    __delay_ms(10*LCD_STARTUP_TIME2);
    //Initialize LCD display
    lcd_write_8bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_8BIT);
    __delay_ms(10*LCD_STARTUP_TIME3);
    //Initialize LCD display
    lcd_write_8bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_8BIT);
    __delay_ms(10*LCD_STARTUP_TIME3);
    //Initialize LCD display
    lcd_write_8bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_4BIT);
    __delay_ms(10*LCD_STARTUP_TIME3);
    //Initialize LCD display
    lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_4BIT);
    __delay_ms(10*LCD_STARTUP_TIME3);
    //Turn LCD off
    lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_OFF);
    __delay_ms(10*LCD_STARTUP_TIME3);
    //Clear Display
    lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_CLEAR);
    __delay_ms(10*LCD_STARTUP_TIME3);
    //Set entry mode
    lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_ENTRYMODE);
    __delay_ms(10*LCD_STARTUP_TIME4);
    //Turn LCD on
    lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_ON);
    //Add 'ä' as a custom character
    //lcd_a_umlaut();
}

/******************************************************************************
 * Refreshes content of entire display                                        *
 * Takes approximately 3.7ms                                                  *
 ******************************************************************************/
void lcd_refresh_all()
{
  //Variable declarations
  uint8_t lcd_addr[2] = {LCD_LINE_1_ADDR, LCD_LINE_2_ADDR};
  uint8_t line;
  uint8_t character;
  //Refresh screen
  for(line=0; line<2; ++line)
  {
    lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_DDRAM_ADDRESS | lcd_addr[line]));
    for(character=0; character<16; ++character)
    {
        lcd_write_4bit(LCD_DATA, lcd_content[line][character]);
    }
  }
}

/******************************************************************************
 * Turn LCD on                                                                *
 ******************************************************************************/
void lcd_on()
{
  lcd_write_4bit(LCD_INSTRUCTION, LCD_ON);
}

/******************************************************************************
 * Turn LCD off                                                               *
 ******************************************************************************/
void lcd_off()
{
  lcd_write_4bit(LCD_INSTRUCTION, LCD_OFF);
}

/******************************************************************************
 * Set contrast                                                               *
 ******************************************************************************/
void lcd_set_contrast(uint8_t contrast)
{
    CCPR2 = contrast;
}