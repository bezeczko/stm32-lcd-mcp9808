#include "i2c-lcd.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t timeout = 0;
uint8_t timeoutTime = 20;
uint8_t timeoutTimer = 0;

uint8_t lcd_init(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;
	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* init sequence */
	HAL_Delay(40);
	if(lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin)) {
		HAL_Delay(5);
	} else return 0;

	if(lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin)) {
		HAL_Delay(1);
	} else return 0;

	if(lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin) == 0) {
		return 0;
	}

	/* set 4-bit mode */
	if(lcd_write(lcd->addr, INIT_4_BIT_MODE, xpin) == 0) {
		return 0;
	}

	/* set cursor mode */
	if(lcd_write(lcd->addr, UNDERLINE_OFF_BLINK_OFF, xpin) == 0) {
		return 0;
	}

	/* clear */
	if(lcd_clear(lcd) == 0) {
		return 0;
	}

	return 1;
}


int read_bf(uint8_t addr){

	uint8_t tx_data[4];
	uint8_t busy_flag;

	tx_data[0] = EN_PIN | 0x02;
	tx_data[1] = 0x02;

	if(HAL_I2C_Master_Transmit(&hi2c1, addr, tx_data, 2, 100) == HAL_OK){
		if(HAL_I2C_Master_Receive(&hi2c1, addr | 0x01, &busy_flag, 1, 100) == HAL_OK) {
			busy_flag = busy_flag & 0x80;
		} else return -1;
	} else return -1;

	return busy_flag;

}

uint8_t lcd_write(uint8_t addr, uint8_t data, uint8_t xpin)
{
	uint8_t tx_data[4];
	uint8_t busy_flag;

	/* split data */
	tx_data[0] = (data & 0xF0) | EN_PIN | xpin;
	tx_data[1] = (data & 0xF0) | xpin;
	tx_data[2] = (data << 4) | EN_PIN | xpin;
	tx_data[3] = (data << 4) | xpin;

	timeoutTimer = 0;
	timeout = 0;
	do {
		busy_flag = read_bf(addr);
		if(timeout == 1 || busy_flag == -1) {
			timeout = 0;
			return 0;
		}
	} while(busy_flag != 0);

	/* send data via i2c */
	HAL_I2C_Master_Transmit(&hi2c1, addr, tx_data, 4, 100);

	return 1;
}

uint8_t lcd_display(struct lcd_disp * lcd)
{
	uint8_t xpin = 0, i = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	if(lcd_clear(lcd) == 0) {
		return 0;
	}

	/* send first line data */
	if(lcd_write(lcd->addr, FIRST_CHAR_LINE_1, xpin) == 0) {
		return 0;
	}
	while(lcd->f_line[i])
	{
		if(lcd_write(lcd->addr, lcd->f_line[i], (xpin | RS_PIN))) {
			i++;
		} else return 0;
	}

	/* send second line data */
	i = 0;
	if(lcd_write(lcd->addr, FIRST_CHAR_LINE_2, xpin) == 0) {
		return 0;
	}
	while(lcd->s_line[i])
	{
		if(lcd_write(lcd->addr, lcd->s_line[i], (xpin | RS_PIN))) {
			i++;
		} else return 0;
	}

	return 1;
}

uint8_t lcd_clear(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* clear display */
	if(lcd_write(lcd->addr, CLEAR_LCD, xpin) == 0) {
		return 0;
	}

	return 1;
}
