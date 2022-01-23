/* HEARTRATE HAL.h
 *   by Nikolaus Rieder
 *
 * Created:
 *   December 28, 2021, 5:52:16 PM GMT+1
 * Last edited:
 *   January 23, 2022, 12:36:58 PM GMT+1
 * Auto updated?
 *   Yes
 *
 * Description:
 *   Hardware abstraction layer for heartrate click module. (HEADER)
**/

/* NOTE This Library is an adaption and was written specifically with the STM32L432KC in mind
The hardware abstraction library from the manufacturer didn't work out-of-the-box, so I decided to write my own adaption.
Original sources can be found here: https://github.com/MikroElektronika/Click_Heart_Rate_MAX30100 
*/

#ifndef HEARTRATE_HAL_H
#define HEARTRATE_HAL_H

/*--- COMMON LIBRARIES ---*/
#include <stdint.h>

/*--- CUSTOM LIBRARIES ---*/
#include "stm32l4xx_hal.h"

/*--- MACROS ---*/
#define WRITE 0
#define READ 1
// #define MAX_WRITE_SIZE 1
#define MAX_READ_SIZE 64
#define MAX30100_I2C_ADR 0x57

//---- Function prototypes

/**Tut
 * @brief Initializes i2c module
 * @param address_id - slave device address
 */
int8_t heartrate1_i2c_init( uint8_t address_id, I2C_HandleTypeDef i2c_handle, UART_HandleTypeDef uart_handle);

/**
 * @brief Writes data through the i2c line
 * @param address - memory adress of the slave device
 * @param num     - number of bytes to send
 * @param buff    - buffer containing the bytes
 */
void heartrate1_i2c_hal_write( uint8_t address, uint16_t num, uint8_t *buff );

/**
 * @brief Reads bytes from i2c bus, stores to a desired buffer
 * @param address - desired memory address on the slave device
 * @param num     - number of bytes to be read
 * @param buff    - buffer to where the read bytes will be stored
 */
void heartrate1_i2c_hal_read(  uint8_t address, uint16_t num, uint8_t *buff );

/************************************************
 * @brief Logging function which writes to UART
 * Needs to be improved because it currently delays the program significantly
 ***********************************************/
void uart_dev_log(uint8_t *log);

/************************************************
 * @brief Turns on the RED RGB Led to signal a transmission error has occured
 * 
 ***********************************************/
void led_error_blink();

#endif /* HEARTRATE_HAL_H */