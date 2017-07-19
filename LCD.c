#include "LCD.h"


/******************************************************************************
 * Static functions                                                           *
 ******************************************************************************/
static void lcd_write_8_bitfixwait(uint8_t type, uint8_t dat);
static void lcd_write_4_bitfixwait(uint8_t type, uint8_t dat);
static void lcd_write_4(uint8_t type, uint8_t dat);
static void lcd_a_umlaut();
static void lcd_wait_while_busy();

/******************************************************************************
 * Read contrast from EEPROM                                                  *
 ******************************************************************************/
void lcd_read_contrast()
{
  lcd.contrast = eeprom_read(LCD_CONTRAST_EEPROM_ADDRESS);
}

/******************************************************************************
 * Read brightness from EEPROM                                                *
 ******************************************************************************/
void lcd_read_brightness()
{
  lcd.brightness = eeprom_read(LCD_BRIGHTNESS_EEPROM_ADDRESS);
}

/******************************************************************************
 * Write contrast to EEPROM                                                   *
 ******************************************************************************/
void lcd_write_contrast()
{
  eeprom_write(LCD_CONTRAST_EEPROM_ADDRESS, lcd.contrast);
}

/******************************************************************************
 * Write brightness to EEPROM                                                 *
 ******************************************************************************/
void lcd_write_brightness()
{
  eeprom_write(LCD_BRIGHTNESS_EEPROM_ADDRESS, lcd.brightness);
}

/******************************************************************************
 * Adjust contrast                                                            *
 ******************************************************************************/
void lcd_set_contrast(uint8_t contrast)
{
  lcd.contrast = contrast;
  contrast *= LCD_CONTRAST_MULTIPLIER;
  contrast += LCD_CONTRAST_OFFSET;
  PWM2_Set_Duty(contrast);
}

/******************************************************************************
 * Adjust brightness                                                          *
 ******************************************************************************/
void lcd_set_brightness(uint8_t brightness)
{
  lcd.brightness = brightness;
  PWM1_Set_Duty(lcd.dutycycles[brightness]);
}

/******************************************************************************
 * Wait until LCD display is ready for next instruction                       *
 ******************************************************************************/
void lcd_wait_while_busy()
{
  uint8_t shadow_save = SHADOW_A;
  TRISA = 0b11110000; //configure D7:D4 as inputs
  SHADOW_A = LCD_RW; //read
  PORTA = SHADOW_A;
  SHADOW_A |= LCD_EN; //EN high
  PORTA = SHADOW_A;
  while(PORTA & 0b10000000)
  {
    delay_us(1);
  }
  SHADOW_A = shadow_save;
  PORTA = SHADOW_A;
  TRISA = 0x00; //configure D7:D4 as outputs
}

/******************************************************************************
 * Write to LCD                                                               * 
 * Used during first half of initialization while display is in 8-bit mode    *
 * Wait times are fixed since busy flag is not valid before initialization    *
 * is complete                                                                *
 ******************************************************************************/
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

/******************************************************************************
 * Write to LCD                                                               *
 * Used during second half of initialization while display is in 4-bit mode   *
 * Wait times are fixed since busy flag is not valid before initialization    *
 * is complete                                                                *
 ******************************************************************************/
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

/******************************************************************************
 * Write to LCD                                                               *
 * Used once initialization is complete                                       *
 * Wait times depend on busy flag to maximize performance                     *
 ******************************************************************************/
static void lcd_write_4bit(uint8_t type, uint8_t dat)
{
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
}

/******************************************************************************
 * Sets up character "ä" as a custom character                                *
 ******************************************************************************/
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

/******************************************************************************
 * Initialize LCD display in 4-bit mode                                       *
 ******************************************************************************/
void lcd_init_4bit()
{
  //Variable declarations
  uint8_t dutycycles[LCD_DUTYCYCLE_COUNT] = LCD_DUTYCYCLE_VALUES;
  uint8_t line;
  uint8_t character;
  //Prepare duty cycle array
  for(line=0;line<LCD_DUTYCYCLE_COUNT;++line)
  {
    lcd.dutycycles[line] = dutycycles[line];
  }
  //Configure Port
  TRISA = 0x00;
  ANSELA = 0x00;
  //Configure LCD brightness (PWM1) and contrast (PWM2)
  PWM1_Init(5000);
  PWM2_Init(5000);
  lcd_read_brightness();
  lcd_set_brightness(lcd.brightness);
  lcd_read_contrast();
  lcd_set_contrast(lcd.contrast);
  //PWM1_Start();
  PWM2_Start();
  //Wait
  delay_ms(LCD_STARTUP_TIME1);
  //Initialize LCD display
  lcd_write_8bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_8BIT);
  delay_ms(LCD_STARTUP_TIME2);
  //Initialize LCD display
  lcd_write_8bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_8BIT);
  delay_ms(LCD_STARTUP_TIME3);
  //Initialize LCD display
  lcd_write_8bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_8BIT);
  delay_ms(LCD_STARTUP_TIME3);
  //Initialize LCD display
  lcd_write_8bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_4BIT);
  delay_ms(LCD_STARTUP_TIME3);
  //Initialize LCD display
  lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_FUNCTIONSET_4BIT);
  delay_ms(LCD_STARTUP_TIME3);
  //Turn LCD off
  lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_OFF);
  delay_ms(LCD_STARTUP_TIME3);
  //Clear Display
  lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_CLEAR);
  delay_ms(LCD_STARTUP_TIME3);
  //Set entry mode
  lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_ENTRYMODE);
  delay_ms(LCD_STARTUP_TIME4);
  //Turn LCD on
  lcd_write_4bit_fixwait(LCD_INSTRUCTION, LCD_ON);
  //Add 'ä' as a custom character
  lcd_a_umlaut();
}

/******************************************************************************
 * Refreshes content of entire display                                        *
 * Takes approximately 3.7ms                                                  *
 ******************************************************************************/
void lcd_refresh_all()
{
  //Variable declarations
  uint8_t lcd_addr[4] = {LCD_LINE_1_ADDR, LCD_LINE_2_ADDR, LCD_LINE_3_ADDR, LCD_LINE_4_ADDR};
  uint8_t line;
  uint8_t character;
  //Refresh screen
  for(line=0; line<4; ++line)
  {
    lcd_write_4bit(LCD_INSTRUCTION, (LCD_SET_DDRAM_ADDRESS | lcd_addr[line]));
    for(character=0; character<16; ++character)
    {
      lcd_write_4bit(LCD_DATA, lcd.display[line][character]);
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
