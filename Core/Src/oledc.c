/*
 * oledc.c
 *
 *  Created on: 24 Jan 2022
 *      Author: steph
 */

#include "oledc.h"
#include <stdint.h>
#include <stdbool.h>

// Font Direction
const uint8_t  _OLEDC_FO_HORIZONTAL       = 0x00;
const uint8_t  _OLEDC_FO_VERTICAL         = 0x01;
const uint8_t  _OLEDC_FO_VERTICAL_COLUMN  = 0x02;

// OLED REMAMP SET
const uint8_t _OLEDC_RMP_INC_HOR         = 0x00;
const uint8_t _OLEDC_RMP_INC_VER         = 0x01;
const uint8_t _OLEDC_RMP_COLOR_NOR       = 0x00;
const uint8_t _OLEDC_RMP_COLOR_REV       = 0x02;
const uint8_t _OLEDC_RMP_SEQ_RGB         = 0x00;
const uint8_t _OLEDC_RMP_SEQ_BGR         = 0x04;
const uint8_t _OLEDC_RMP_SCAN_NOR        = 0x00;
const uint8_t _OLEDC_RMP_SCAN_REV        = 0x10;
const uint8_t _OLEDC_RMP_SPLIT_DISABLE   = 0x00;
const uint8_t _OLEDC_RMP_SPLIT_ENABLE    = 0x20;
const uint8_t _OLEDC_COLOR_65K           = 0x00;
const uint8_t _OLEDC_COLOR_262K          = 0x80;
const uint8_t _OLEDC_IMG_HEAD            = 0x06;

// Device Properties
const uint8_t  _OLEDC_SCREEN_WIDTH    = 0x60;
const uint8_t  _OLEDC_SCREEN_HEIGHT   = 0x60;
const uint16_t _OLEDC_SCREEN_SIZE     = 0x2400;
const uint8_t  _OLEDC_ROW_OFF         = 0x00;
const uint8_t  _OLEDC_COL_OFF         = 0x10;

// SSD1355 Commands
const uint8_t  _OLEDC_SET_COL_ADDRESS   = 0x15;
const uint8_t  _OLEDC_SET_ROW_ADDRESS   = 0x75;
const uint8_t  _OLEDC_WRITE_RAM         = 0x5C;
const uint8_t  _OLEDC_READ_RAM          = 0x5D;
const uint8_t  _OLEDC_SET_REMAP         = 0xA0;
const uint8_t  _OLEDC_SET_START_LINE    = 0xA1;
const uint8_t  _OLEDC_SET_OFFSET        = 0xA2;
const uint8_t  _OLEDC_MODE_OFF          = 0xA4;
const uint8_t  _OLEDC_MODE_ON           = 0xA5;
const uint8_t  _OLEDC_MODE_NORMAL       = 0xA6;
const uint8_t  _OLEDC_MODE_INVERSE      = 0xA7;
const uint8_t  _OLEDC_FUNCTION          = 0xAB;
const uint8_t  _OLEDC_SLEEP_ON          = 0xAE;
const uint8_t  _OLEDC_SLEEP_OFF         = 0xAF;
const uint8_t  _OLEDC_NOP              = 0xB0;
const uint8_t  _OLEDC_SET_RESET_PRECH  = 0xB1;
const uint8_t  _OLEDC_ENHANCEMENT      = 0xB2;
const uint8_t  _OLEDC_CLOCK_DIV         = 0xB3;
const uint8_t  _OLEDC_VSL               = 0xB4;
const uint8_t  _OLEDC_GPIO              = 0xB5;
const uint8_t  _OLEDC_SETSEC_PRECH      = 0xB6;
const uint8_t  _OLEDC_GREY_SCALE        = 0xB8;
const uint8_t  _OLEDC_LUT               = 0xB9;
const uint8_t  _OLEDC_PRECH_VOL         = 0xBB;
const uint8_t  _OLEDC_VCOMH             = 0xBE;
const uint8_t  _OLEDC_CONTRAST          = 0xC1;
const uint8_t  _OLEDC_MASTER_CONTRAST   = 0xC7;
const uint8_t  _OLEDC_MUX_RATIO         = 0xCA;
const uint8_t  _OLEDC_COMMAND_LOCK      = 0xFD;
const uint8_t  _OLEDC_SCROLL_HOR        = 0x96;
const uint8_t  _OLEDC_START_MOV         = 0x9E;
const uint8_t  _OLEDC_STOP_MOV          = 0x9F;


static uint8_t _OLEDC_DEFAULT_MUX_RATIO      = 95;
static uint8_t _OLEDC_DEFAULT_START_LINE     = 0x80;
static uint8_t _OLEDC_DEFAULT_OFFSET         = 0x20;

static uint8_t _OLEDC_DEFAULT_OLED_LOCK      = 0x12;
static uint8_t _OLEDC_DEFAULT_CMD_LOCK       = 0xB1;
static uint8_t _OLEDC_DEFAULT_DIVSET         = 0xF1;
static uint8_t _OLEDC_DEFAULT_PRECHARGE      = 0x32;
static uint8_t _OLEDC_DEFAULT_VCOMH          = 0x05;
static uint8_t _OLEDC_DEFAULT_MASTER_CONT    = 0xCF;
static uint8_t _OLEDC_DEFAULT_PRECHARGE_2    = 0x01;

static uint8_t _OLEDC_DEFAULT_REMAP = _OLEDC_RMP_INC_HOR | _OLEDC_RMP_COLOR_REV |
                                _OLEDC_RMP_SEQ_RGB | _OLEDC_RMP_SCAN_REV |
                                _OLEDC_RMP_SPLIT_ENABLE | _OLEDC_COLOR_65K;

static  uint8_t _OLEDC_DEFAULT_VSL[ 3 ]       = { 0xA0, 0xB5, 0x55 };
static  uint8_t _OLEDC_DEFAULT_CONTRAST[ 3 ]  = { 0x8A, 0x51, 0x8A };

static uint8_t cols[ 2 ]    = { _OLEDC_COL_OFF, _OLEDC_COL_OFF + 95 };
static uint8_t rows[ 2 ]    = { _OLEDC_ROW_OFF, _OLEDC_ROW_OFF + 95 };
static const uint8_t*   _font;
static uint16_t         _font_color;
static uint8_t          _font_orientation;
static uint16_t         _font_first_char;
static uint16_t         _font_last_char;
static uint16_t         _font_height;
static uint16_t         x_cord;
static uint16_t         y_cord;

void oledc_command( uint8_t command, uint8_t *args, uint16_t args_len )
{
    uint8_t *ptr = args;

    hal_gpio_csSet(0);
    hal_gpio_pwmSet(0);
    hal_spiWrite( &command, 1 );
    hal_gpio_pwmSet(1);
    if( args_len )
            hal_spiWrite( ptr, args_len );
    hal_gpio_csSet(1);
}



static void box_area( uint8_t start_col, uint8_t start_row, uint8_t end_col, uint8_t end_row, uint16_t color )
{
    uint8_t   cmd       = _OLEDC_WRITE_RAM;
    uint16_t  cnt       = ( end_col - start_col ) * ( end_row - start_row );
    uint8_t   clr[ 2 ]  = { 0 };

    if( ( start_col > _OLEDC_SCREEN_WIDTH ) ||
        ( end_col > _OLEDC_SCREEN_WIDTH ) )
        return;

    if( ( start_row > _OLEDC_SCREEN_HEIGHT ) ||
        ( end_row > _OLEDC_SCREEN_HEIGHT ) )
        return;

    if( ( end_col < start_col ) ||
        ( end_row < start_row ) )
        return;

    cols[ 0 ] = _OLEDC_COL_OFF + start_col;
    cols[ 1 ] = _OLEDC_COL_OFF + end_col - 1;
    rows[ 0 ] = _OLEDC_ROW_OFF + start_row;
    rows[ 1 ] = _OLEDC_ROW_OFF + end_row - 1;
    clr[ 0 ] |= color >> 8;
    clr[ 1 ] |= color & 0x00FF;

    oledc_command( _OLEDC_SET_COL_ADDRESS, cols, 2 );
    oledc_command( _OLEDC_SET_ROW_ADDRESS, rows, 2 );
    hal_gpio_csSet(0);
    hal_gpio_pwmSet(0);
    hal_spiWrite( &cmd, 1 );
    hal_gpio_pwmSet(1);

    while( cnt-- )
        hal_spiWrite( clr, 2 );
    hal_gpio_csSet(1);
}

void clear_area(uint8_t start_col, uint8_t start_row, uint8_t end_col, uint8_t end_row){
	box_area(start_col, start_row, end_col, end_row, 0x0000);
}

void oledc_fill_screen( uint16_t color )
{
    box_area( 0, 0, 96, 96, color );
}

void oledc_enable( bool pinState )
{
    hal_gpio_intSet(pinState);
}

void oledc_reset( void )
{
    hal_gpio_rstSet(1);
    delay(1);
    hal_gpio_rstSet(0);
    delay(1);
    hal_gpio_rstSet(1);
    delay(100);
}

void oledc_init()
{
    oledc_enable(1);
    oledc_reset();

    /* Unlock display and turn off */
    oledc_command( _OLEDC_COMMAND_LOCK,    &_OLEDC_DEFAULT_OLED_LOCK,     1 );
    oledc_command( _OLEDC_COMMAND_LOCK,    &_OLEDC_DEFAULT_CMD_LOCK,      1 );
    oledc_command( _OLEDC_SLEEP_ON,        0,                      0 );

    /* Setup SSD1351 */
    oledc_command( _OLEDC_SET_REMAP,       &_OLEDC_DEFAULT_REMAP,         1 );
    oledc_command( _OLEDC_MUX_RATIO,       &_OLEDC_DEFAULT_MUX_RATIO,     1 );
    oledc_command( _OLEDC_SET_START_LINE,  &_OLEDC_DEFAULT_START_LINE,    1 );
    oledc_command( _OLEDC_SET_OFFSET,      &_OLEDC_DEFAULT_OFFSET,        1 );
    oledc_command( _OLEDC_VCOMH,           &_OLEDC_DEFAULT_VCOMH,         1 );
    oledc_command( _OLEDC_CLOCK_DIV,       &_OLEDC_DEFAULT_DIVSET,        1 );
    oledc_command( _OLEDC_SET_RESET_PRECH, &_OLEDC_DEFAULT_PRECHARGE,     1 );
    oledc_command( _OLEDC_SETSEC_PRECH,    &_OLEDC_DEFAULT_PRECHARGE_2,   1 );
    oledc_command( _OLEDC_MASTER_CONTRAST, &_OLEDC_DEFAULT_MASTER_CONT,   1 );
    oledc_command( _OLEDC_CONTRAST,        _OLEDC_DEFAULT_CONTRAST,       3 );
    oledc_command( _OLEDC_VSL,             _OLEDC_DEFAULT_VSL,            3 );

    /* Set normal mode and turn on display */
    oledc_command( _OLEDC_MODE_NORMAL, 0, 0 );
    oledc_command( _OLEDC_SLEEP_OFF,   0, 0 );

    oledc_fill_screen( 0 );
}

void oledc_set_font( const uint8_t *font, uint16_t color)
{
    _font               = font;
    _font_first_char    = font[2] + (font[3] << 8);
    _font_last_char     = font[4] + (font[5] << 8);
    _font_height        = font[6];
    _font_color         = color;
    _font_orientation   = _OLEDC_FO_HORIZONTAL ;
}

void oledc_text_p(char *txt){
	oledc_text((uint8_t*)txt, 2,2);
}

void oledc_text_two_lines(char *firstLine, char *secondLine){
	oledc_text((uint8_t*)firstLine, 2,2);
	oledc_text((uint8_t*)secondLine, 2,_font_height + 2);
}

void oledc_text_at_line(char* line, uint8_t lineNumber){
	oledc_text((uint8_t*)line, 2, (lineNumber * _font_height) + 2);
}

void oledc_text(uint8_t *text, uint16_t x, uint16_t y )
{
    uint8_t *ptr = text;

    if( ( x >= _OLEDC_SCREEN_WIDTH ) || (y >= _OLEDC_SCREEN_HEIGHT ) )
        return;

    x_cord = x;
    y_cord = y;

    while( *ptr )
        character( *ptr++ );
}

void character( uint16_t ch )
{
    uint8_t     ch_width = 0;
    uint8_t     x_cnt;
    uint8_t     y_cnt;
    uint16_t    x = 0;
    uint16_t    y = 0;
    uint16_t    tmp;
    uint8_t     temp = 0;
    uint8_t     mask = 0;
    uint32_t    offset;
    const uint8_t *ch_table;
    const uint8_t *ch_bitmap;

    if( ch < _font_first_char )
        return;

    if( ch > _font_last_char )
        return;

    offset = 0;
    tmp = (ch - _font_first_char) << 2;
    ch_table = _font + 8 + tmp;
    ch_width = *ch_table;

    offset = (uint32_t)ch_table[1] + ((uint32_t)ch_table[2] << 8) + ((uint32_t)ch_table[3] << 16);

    ch_bitmap = _font + offset;

    if( ( _font_orientation == _OLEDC_FO_HORIZONTAL ) ||
        ( _font_orientation == _OLEDC_FO_VERTICAL_COLUMN ) )
    {
        y = y_cord;
        for (y_cnt = 0; y_cnt < _font_height; y_cnt++)
        {
            x = x_cord;
            mask = 0;
            for( x_cnt = 0; x_cnt < ch_width; x_cnt++ )
            {
                if( !mask )
                {
                    temp = *ch_bitmap++;
                    mask = 0x01;
                }

                if( temp & mask )
                    pixel( x, y, _font_color );

                x++;
                mask <<= 1;
            }
            y++;
        }

        if ( _font_orientation == _OLEDC_FO_HORIZONTAL )
            x_cord = x + 1;
        else
            y_cord = y;
    }
    else
    {
        y = x_cord;

        for( y_cnt = 0; y_cnt < _font_height; y_cnt++ )
        {
            x = y_cord;
            mask = 0;

            for( x_cnt = 0; x_cnt < ch_width; x_cnt++ )
            {
                if( mask == 0 )
                {
                    temp = *ch_bitmap++;
                    mask = 0x01;
                }

                if( temp & mask )
                {
                    pixel( y, x, _font_color );
                }

                x--;
                mask <<= 1;
            }
            y++;
        }
        y_cord = x - 1;
    }
}

void pixel( uint8_t col, uint8_t row, uint16_t color )
{
    uint8_t cmd       = _OLEDC_WRITE_RAM;
    uint8_t clr[ 2 ]  = { 0 };

    if( ( col > _OLEDC_SCREEN_WIDTH ) || ( row > _OLEDC_SCREEN_HEIGHT ) )
        return;

    cols[ 0 ] = _OLEDC_COL_OFF + col;
    cols[ 1 ] = _OLEDC_COL_OFF + col;
    rows[ 0 ] = _OLEDC_ROW_OFF + row;
    rows[ 1 ] = _OLEDC_ROW_OFF + row;
    clr[ 0 ] |= color >> 8;
    clr[ 1 ] |= color & 0x00FF;

    oledc_command( _OLEDC_SET_COL_ADDRESS, cols, 2 );
    oledc_command( _OLEDC_SET_ROW_ADDRESS, rows, 2 );

    hal_gpio_csSet(0);
    hal_gpio_pwmSet(0);
    hal_spiWrite( &cmd, 1 );
    hal_gpio_pwmSet(1);
    hal_spiWrite( clr, 2 );
    hal_gpio_csSet(1);
}



