/* HEARTRATE HAL.h
 *   by Nikolaus Rieder
 *
 * Created:
 *   December 28, 2021, 5:52:16 PM GMT+1
 * Last edited:
 *   December 28, 2021, 6:59:29 PM GMT+1
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

/*--- MACROS ---*/
#define WRITE 0
#define READ 1
// #define MAX_WRITE_SIZE 1
#define MAX_READ_SIZE 64
#define MAX30100_I2C_ADR 0x57

//---- Function prototypes

/**
 * @brief Initializes i2c module
 * @param address_id - slave device address
 */
int heartrate1_i2c_init( uint8_t address_id );

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

#endif /* HEARTRATE_HAL_H */