/*
 * oledc.h
 *
 *  Created on: 24 Jan 2022
 *      Author: steph
 */

#ifndef INC_OLEDC_H_
#define INC_OLEDC_H_

#include <stdint.h>
#include <stdbool.h>

void oledc_init();
void oledc_fill_screen( uint16_t color );
void hal_spiWrite(uint8_t *pBuf, uint16_t nBytes);
void oledc_set_font( const uint8_t *font, uint16_t color);
void oledc_text( uint8_t *text, uint16_t col_off, uint16_t row_off );
void oledc_text_p(char *txt);
void character( uint16_t ch );
void pixel( uint8_t col, uint8_t row, uint16_t color );
void oledc_text_two_lines(char *firstLine, char *secondLine);
void hal_gpio_csSet(bool pinState);
void hal_gpio_pwmSet(bool pinState);
void hal_gpio_intSet(bool pinState);
void hal_gpio_rstSet(bool pinState);
void delay(int delay);
unsigned int spi_read(unsigned int buffer);
void hal_spiWrite(uint8_t *pBuf, uint16_t nBytes);
void oledc_text_at_line(char* line, uint8_t lineNumber);
void clear_area(uint8_t start_col, uint8_t start_row, uint8_t end_col, uint8_t end_row);

#endif /* INC_OLEDC_H_ */
