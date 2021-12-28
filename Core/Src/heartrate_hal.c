/* HEARTRATE HAL.c
 *   by Nikolaus Rieder
 *
 * Created:
 *   December 28, 2021, 5:53:16 PM GMT+1
 * Last edited:
 *   December 28, 2021, 6:59:15 PM GMT+1
 * Auto updated?
 *   Yes
 *
 * Description:
 *   Hardware abstraction layer for heartrate click module. (SOURCE)
**/

/* NOTE This Library is an adaption and was written specifically with the STM32L432KC in mind
The hardware abstraction library from the manufacturer didn't work out-of-the-box, so I decided to write my own adaption.
Original sources can be found here: https://github.com/MikroElektronika/Click_Heart_Rate_MAX30100 
*/

/*--- COMMON LIBRARIES ---*/
#include <stdint.h>

/*--- CUSTOM LIBRARIES ---*/
#include "heartrate_hal.h"

// This handle must be overwritten in main.c
#ifndef HANDLE
#define HANDLE hi2c
#endif

/*--- Module variable definitions ---*/
static uint8_t i2c_address;

int heartrate1_i2c_init(uint8_t address_id)
{
    i2c_address = (address_id << 1);
    return 0;
}

void heartrate1_i2c_hal_write(uint8_t address, uint16_t num, uint8_t *buff)
{
    // Variable declaration
    uint8_t buffer[MAX_READ_SIZE];
    HAL_StatusTypeDef ret;

    // Local copy of buffer
    buffer[0] = address;
    memcpy(&buffer[1], buff, num);

    // HAL Library usage for transmission

    ret = HAL_I2C_Master_Transmit(hi2c, (uint16_t) address, buffer, 1, HAL_MAX_DELAY));
    if (ret != HAL_OK)
    {
        // TODO Error-Message during Transmit
    }
    return;
}

void heartrate1_i2c_hal_read(uint8_t address, uint16_t num, uint8_t *buff)
{
    // Variable declaration
    uint8_t buffer[MAX_READ_SIZE];
    HAL_StatusTypeDef ret;

    // Local copy of buffer
    buffer[0] = address;
    memcpy(&buffer[1], buff, num);

    // HAL Library usage for transmission

    ret = HAL_I2C_Master_Receive(hi2c, (uint16_t) address, buffer, 2, HAL_MAX_DELAY));
    if (ret != HAL_OK)
    {
        // TODO Error-Message during Receive
    }
    return;
}