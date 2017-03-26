/*
 * LCD03.cpp
 *
 *  Created on: Mar 26, 2017
 *      Author: emon1
 */

#include "LCD03.h"

LCD03::LCD03() {
	// TODO Auto-generated constructor stub

}

LCD03::~LCD03() {
	// TODO Auto-generated destructor stub
}


LCD03::LCD03(COMM_MODE_t mode = LCD03_SERIAL,LCD_SIZE_t type = LCD03_20_4,LCD03_I2C_ADDRESS_t addr = LCD03_I2C_ADDRESS_0xc8){
	if(mode == LCD03_I2C){
		psend = &LCD03::portI2CTransmit;
	}
	else if (mode == LCD03_SERIAL){
		psend = &LCD03::portSerialTransmit;
	}
}

void LCD03::send_command(LCD03_COMMAND_t cmd){
	(this->*psend) (cmd);
}
void LCD03::send_data(uint8_t data){
	(this->*psend) (data);
}
void LCD03::cursor_home(){
	send_command(LCD03_CMD_CURSOR_HOME);
}
void LCD03::set_cursor_pos(uint8_t pos){
	send_command(LCD03_CMD_SET_CURSOR_POS);
}
void LCD03::set_cursor_coordinate(uint8_t line, uint8_t col){
	send_command(LCD03_CMD_SET_CURSOR_COORDINATE);
}
void LCD03::cursor_display_mode(LCD03_CURSOR_DISP_t mode){
	switch(mode){
		case 	LCD03_CURSOR_HIDE		:	send_command(LCD03_CMD_HIDE_CURSOR);break;
		case	LCD03_CURSOR_UNDERLINE	:	send_command(LCD03_CMD_SHOW_UNDERLINE_CURSOR);break;
		case	LCD03_CURSOR_BLINKING	:	send_command(LCD03_CMD_SHOW_BLINKING_CURSOR);break;
		default							:	break;
	}
}
void LCD03::backspace(){
	send_command(LCD03_CMD_BACKSPACE);
}
void LCD03::tab(){
	send_command(LCD03_CMD_HORIZONTAL_TAB);
}
void LCD03::smart_line_feed(){
	send_command(LCD03_CMD_SMART_LINE_FEED);
}
void LCD03::vertical_tab(){
	send_command(LCD03_CMD_VERTICAL_TAB);
}
void LCD03::clear_screen(){
	send_command(LCD03_CMD_CLEAR_SCREEN);
}
void LCD03::carriage_return(){
	send_command(LCD03_CMD_CARRIAGE_RETURN);
}
void LCD03::clear_column(){
	send_command(LCD03_CMD_CLEAR_COLUMN);
}
void LCD03::tab_set_size(uint8_t size){
	send_command(LCD03_CMD_TAB_SET);
	send_data(size);
}
void LCD03::backlight(bool mode){
	if(mode) {send_command(LCD03_CMD_BACKLIGHT_ON);}
	else {send_command(LCD03_CMD_BACKLIGHT_OFF);}
}
void LCD03::startup_message_mode(bool mode){
	if(mode) {send_command(LCD03_CMD_ENABLE_STARTUP_MESSAGE);}
	else {send_command(LCD03_CMD_DISABLE_STARTUP_MESSAGE);}
}
void LCD03::change_i2c_addr(LCD03_I2C_ADDRESS_t addr){
	if(psend == (&LCD03::portI2CTransmit)){
		send_command(LCD03_CMD_CHANGE_ADDRESS);
		send_data(0xA0);
		send_data(0xAA);
		send_data(0xA5);
		send_data(addr);
	}
}
void LCD03::custom_char_ger(){

}
uint8_t LCD03::get_fifo_status(){

}
uint8_t LCD03::get_software_version(){

}

void LCD03::print_string(char *strn_add, int len){
	for(int i=0;i<len;i++){
		send_data(*(strn_add+i));
	}
}
