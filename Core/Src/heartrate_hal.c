/* HEARTRATE HAL.c
 *   by Nikolaus Rieder
 *
 * Created:
 *   December 28, 2021, 5:53:16 PM GMT+1
 * Last edited:
 *   January 23, 2022, 1:05:40 PM GMT+1
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
#include <string.h>

/*--- CUSTOM LIBRARIES ---*/
#include "heartrate_hal.h"
#include "main.h"

// This handle must be overwritten in main.c
#ifndef HANDLE
#define HANDLE NULL
#endif

/*--- Module variable definitions ---*/
static uint8_t i2c_address;
static I2C_HandleTypeDef hi2c;
static UART_HandleTypeDef huart;
static char *receive_error = "Error on RECEIVE\r\n";
static char *send_error = "Error on SEND\r\n";

int8_t heartrate1_i2c_init(uint8_t address_id, I2C_HandleTypeDef i2c_handle, UART_HandleTypeDef uart_handle)
{
    i2c_address = (address_id << 1);
    hi2c = i2c_handle;
    huart = uart_handle;
    return 0;
}

void heartrate1_i2c_hal_write(uint8_t address, uint16_t num, uint8_t *buff)
{
    // Variable declaration
    uint8_t buffer[MAX_READ_SIZE];
    HAL_StatusTypeDef ret;

    // // Local copy of buffer
    buffer[0] = address;
    memcpy(&buffer[1], buff, num);
    uart_dev_log(buffer);

    // HAL Library usage for transmission

    ret = HAL_I2C_Master_Transmit(&hi2c, (uint16_t)address, buff, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK)
    {
        // TODO Error-Message during Transmit
        uart_dev_log((uint8_t *)send_error);
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
    uart_dev_log(buffer);

    // HAL Library usage for transmission
    ret = HAL_I2C_Master_Receive(&hi2c, (uint16_t)address, buff, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK)
    {
        // TODO Error-Message during Receive
        uart_dev_log((uint8_t *)receive_error);
    }
    return;
}