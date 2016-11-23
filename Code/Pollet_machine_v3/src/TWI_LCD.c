/*
 * TWI_LCD.c
 *
 * Created: 2016-05-11 11:15:44
 *  Author: Atomic
 */ 

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "TWI_LCD.h"

extern I2C_HandleTypeDef hi2c1;

void LCD_Init(void){
	
	// Basic initialization
	LCD_Byte(0x33, LCD_CMD); 
	LCD_Byte(0x32, LCD_CMD);
	 
	// Entry mode, move cursor left to right 
	LCD_Byte(0x06, LCD_CMD); 
	
	// Display on, Cursor off, Blink off
	LCD_Byte(0x0C, LCD_CMD); 
	
	// Set to 4-bit ooperation, 2 lines and 5x7 dots
	LCD_Byte(0x28, LCD_CMD); 
	
	// Clears display and DDRAM
	LCD_Byte(0x01, LCD_CMD); 
	//_delay_ms(DELAY_MS*5); //5 ms delay
	osDelay(5);
	
	uint8_t welcome[] =  {"   System is    "};
	uint8_t welcome2[] = {"  starting up   "};
	LCD_String(welcome,16, welcome2,16);
	//_delay_ms(1500);
	osDelay(500);
	
}

void LCD_Byte(uint8_t bits, uint8_t mode){
	
	uint8_t bits_high, bits_low;
	
	//Divides the data in two 4 bit values
	bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT;
	bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT;
	
	// Send the high bits
	//Write_Byte(LCD_ADDR, bits_high);

	uint16_t addr = ((LCD_ADDR << 1) | LCD_WRITE);

	HAL_I2C_Master_Transmit(&hi2c1, addr, &bits_high, 1, 50);
	LCD_Toggle_enable(bits_high);
	
	// Send the low bits
	//Write_Byte(LCD_ADDR, bits_low);
	HAL_I2C_Master_Transmit(&hi2c1, addr, &bits_low, 1, 50);
	LCD_Toggle_enable(bits_low);
	
}

void LCD_Toggle_enable(uint8_t bits){
	
	// Toggles the enable pin on the display to activate the code
	//_delay_ms(1);
	osDelay(1);

	uint16_t addr = ((LCD_ADDR << 1) | LCD_WRITE);

	uint8_t bits_enable = (bits | ENABLE);
	uint8_t bits_disable = (bits & ~ENABLE);

	//Write_Byte(LCD_ADDR, (bits | ENABLE));
	HAL_I2C_Master_Transmit(&hi2c1, addr, &bits_enable, 1, 50);

	//_delay_ms(1);
	osDelay(1);

	//Write_Byte(LCD_ADDR, (bits & ~ENABLE));
	HAL_I2C_Master_Transmit(&hi2c1, addr, &bits_disable, 1, 50);
	//_delay_ms(1);
	osDelay(1);
		
}

void LCD_String(uint8_t row1[],int row1_elems, uint8_t row2[], int row2_elems){
	
	// Command to print on first line
	LCD_Byte(LCD_LINE_1, LCD_CMD);
	
	// Prints the characters on that line
	for(int i = 0; i < row1_elems ;i++){
		LCD_Byte(row1[i], LCD_CHR);
	}
	
	// Command to print on second line
	LCD_Byte(LCD_LINE_2, LCD_CMD);
	
	// Prints the characters on that line
	for(int i = 0; i < row2_elems ;i++){
		LCD_Byte(row2[i], LCD_CHR);
	}

}

void LCD_Singlestring(uint8_t place, uint8_t *str){
	
	LCD_Byte(place, LCD_CMD);
	while(*str){
		LCD_Byte(*str++,LCD_CHR);
	}
}

void LCD_Turn_off(void){
	LCD_String("    Welcome     ",16, " coffee maniac! ", 16);
}
