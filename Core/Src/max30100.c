/* MAX 30100.c
 *   by Nikolaus Rieder
 *
 * Created:
 *   January 25, 2022, 7:21:13 PM GMT+1
 * Last edited:
 *   January 27, 2022, 4:50:12 AM GMT+1
 * Auto updated?
 *   Yes
 *
 * Description:
 *   Source file of MAX30100 library
**/

/*--- COMMON LIBRARIES ---*/
#include <stdio.h>
#include <stdlib.h>

/*--- CUSTOM LIBRARIES ---*/
#include "max30100.h"
#include "stm32l4xx_hal.h"

/*--- MACROS ---*/

/*--- Global variables ---*/

/* -------- INITIALIZATION -------- */

INIT_STATUS MAX30100_initialize(MAX30100 *sensor, I2C_HandleTypeDef *i2c_handle)
{
    /* NOTE Set structure parameters to initial values */
    sensor->i2c_handle = i2c_handle;
    // FIXME can't set the sample arrays to 0 for some weird compiler reason
    // sensor->IR_data = {{0}};
    // sensor->RED_data = {{0}};
    sensor->temperature = 0.0f;

    /* NOTE Function variables for storage */
    uint8_t data;
    HAL_StatusTypeDef status;

    /* NOTE Initial communication - basic checks
     * Check device revision & part id - Serves as a basic check if the device responds
     */
    /* NOTE The documentation did not provide a revision or part number, which I could hardcode 
     * Also since my device is broken, I was not able to read it out - this part is therefor skipped
     */
    // status = MAX30100_read_register(sensor, REV_ID, &data);
    // if (status != HAL_OK) return REVISION_FAILED;
    // if ( data != MAX30100_REVISION) return REVISION_FALSE;
    // status = MAX30100_read_register(sensor, PART_ID, &data);
    // if (status != HAL_OK) return PART_ID_FAILED;
    // if( data != MAX30100_PART_ID) return PART_ID_FALSE;

    /* NOTE Reset MAX30100 */
    data = 0x00 | MODE_RESET;
    status = MAX30100_write_register(sensor, MODE_CONFIG, &data);
    if (status != HAL_OK)
        return RESET_FAILED;

    /* NOTE Mode-configuration MAX30100 */
    data = 0x00;
#ifdef HR_ONLY_MODE
    data |= (MODE & HR_ONLY);
#endif
#ifdef SPO2_MODE
    data |= (MODE & SPO2_EN) | MODE_TEMP_EN;
#endif
    status = MAX30100_write_register(sensor, MODE_CONFIG, &data);
    if (status != HAL_OK)
        return CONFIG_FAILED;

    /* NOTE SPO2 configuration MAX30100 */
    data = 0x00;
#ifdef HR_ONLY_MODE // NOTE HERE one can configure sample rate & pulse width of HR_ONLY_MODE
    data |= (SPO2_SR & SAMPLE_RATE_100) | (LED_PW & PULSE_WIDTH_400);
#endif
#ifdef SPO2_MODE // NOTE HERE one can configure sample rate & pulse width of SPO2_MODE
    data |= (SPO2_SR & SAMPLE_RATE_100) | (LED_PW & PULSE_WIDTH_400);
#endif
    status = MAX30100_write_register(sensor, SPO2_CONFIG, &data);
    if (status != HAL_OK)
        return SPO2_CONFIG_FAILED;

#ifdef SPO2_MODE
    /* NOTE First & Initial temperature reading */
    status = MAX30100_read_temperature(sensor);
    if (status != HAL_OK)
        return INITIAL_TEMP_FAILED;
#endif

    /* NOTE Enable Interrupts */
    data = 0x00;
#ifdef HR_ONLY_MODE
    data |= (FIFO_FULL | HEARTRATE_READY);
#endif
#ifdef SPO2_MODE
    data |= (FIFO_FULL | TEMP_READY | SPO2_READY);
#endif
#ifdef TEMP_ONLY_MODE
    data |= (TEMP_READY);
#endif
    status = MAX30100_write_register(sensor, INT_ENABLE, &data);
    if (status != HAL_OK)
        return INTERRUPT_CONFIG_FAILED;

    /* NOTE LED configuration MAX30100 */
    data = 0x00;
    data |= (RED_PA & (PA_40 << 4)) | (IR_PA & (PA_40)); // IR_PA & RED_PA = 40.2mA => 0xCC
    status = MAX30100_write_register(sensor, LED_CONFIG, &data);
    if (status != HAL_OK)
        return LED_CONFIG_FAILED;

    /* NOTE End of initialization */
    return INIT_OK;
}

/* -------- DATA ACQUISITION --------  */

HAL_StatusTypeDef MAX30100_read_temperature(MAX30100 *sensor)
{
    HAL_StatusTypeDef status[4];
    uint8_t temp_done = 0x00;
    uint8_t temperature = 0x00;
    uint8_t fraction = 0x00;
    uint8_t data = MODE_TEMP_EN;
    status[0] = MAX30100_write_register(sensor, MODE_CONFIG, &data); // Initiate temperature reading
    uint8_t retry_counter = 0;
    // do // Wait for temperature reading to be ready
    // {
    //     // Using status[3] since an error in temperature reading is more important and therefor prioritized
    //     // Read interrupt status flag
    status[3] = MAX30100_read_register(sensor, MODE_CONFIG, &temp_done);
    //     // Increment error counter
    //     retry_counter++;
    //     // Ignore all bits other than TEMP_READY
    //     temp_done &= MODE_TEMP_EN;
    //     // Loop until temperature is ready or retried 20 times in total
    // } while (temp_done && retry_counter <= MAX_RETRY); /* NOTE temp_done == 0 when temperature is ready (a bit counter-intuitive) */
    status[1] = MAX30100_read_register(sensor, TEMP_INTEGER, &temperature); /* removed (uint8_t *) */
    status[2] = MAX30100_read_register(sensor, TEMP_FRACTION, &fraction);
    fraction &= 0x07; // Ignoring all other fraction bits, if wrongly set.
    // sensor->temperature = (float)temperature + (fraction * 0.0625);
    // Check if there were errors, otherwise return HAL_OK
    // for (int i = 0; i < 4; i++)
    // {
    //     if (status[i] != HAL_OK)
    //     {
    //         return status[i]; // Will return the first error that occured
    //         /* NOTE When an error occured, the returned status could be used to discard the reading */
    //     }
    // }
    return HAL_OK; // No errors above, so we can return HAL_OK
}
HAL_StatusTypeDef MAX30100_read_samples(MAX30100 *sensor)
{
    uint8_t sample_number = 0;
    uint8_t writePtr = 0;
    uint8_t readPtr = 0;
    // uint8_t sample[4]; // Currently read samples
    HAL_StatusTypeDef status;

    // Get current pointers of samples
    status = MAX30100_read_register(sensor, FIFO_WRITE_PTR, &writePtr);
    status = MAX30100_read_register(sensor, FIFO_READ_PTR, &readPtr);

    sample_number = abs(16 + writePtr - readPtr) % 16; // Calculate amount of samples in memory
    // Read the whole memory bank of samples
    if (sample_number >= 1)
    {
        for (int i = 0; i < sample_number; i++)
        { // i == sample
            // Read one sample
            status = MAX30100_read_sample(sensor);
            if (status == HAL_OK)
            {
                sensor->IR_data[i] = sensor->hr_sample;
                sensor->RED_data[i] = sensor->spo2_sample;
            }
            else
            {
                sensor->IR_data[i] = 0;
                sensor->RED_data[i] = 0;
            }
        }
    }
    return status;
}

HAL_StatusTypeDef MAX30100_read_sample(MAX30100 *sensor)
{
    uint8_t sample[4];
    HAL_StatusTypeDef status;

    status = MAX30100_read_registers(sensor, FIFO_DATA_REG, sample, SAMPLE_SIZE);

    sensor->hr_sample = (uint16_t)sample[1];         // Save lower-half of sample IR[7:0]
    sensor->hr_sample |= (uint16_t)sample[0] << 8;   // Shift upper-half of sample IR[15:8]
    sensor->spo2_sample = (uint16_t)sample[3];       // Save lower-half of sample RED[7:0]
    sensor->spo2_sample |= (uint16_t)sample[2] << 8; // Shift upper-half of sample RED[15:8]
    return status;
}

uint8_t MAX30100_read_interrupts(MAX30100 *sensor)
{
    uint8_t interrupt_status = 0;
    HAL_StatusTypeDef status;

    status = MAX30100_read_register(sensor, INT_STATUS, &interrupt_status);
    if (status != HAL_OK)
    {
        return 0x00;
    }
    else
    {
        return (interrupt_status & 0xF0); // Only B7,B6,B5 & B5 contain relevant data
    }
}

/* -------- ABSTRACTION FUNCTIONS -------- */

// Read 1 byte from register
HAL_StatusTypeDef MAX30100_read_register(MAX30100 *sensor, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(sensor->i2c_handle, MAX30100_I2C_READ, reg, sizeof(reg), data, sizeof(data), I2C_TIMEOUT);
}
// Read x byte from register
HAL_StatusTypeDef MAX30100_read_registers(MAX30100 *sensor, uint8_t reg, uint8_t *data, uint8_t size)
{
    return HAL_I2C_Mem_Read(sensor->i2c_handle, MAX30100_I2C_READ, reg, sizeof(reg), data, size, I2C_TIMEOUT);
}
// Write 1 byte to register
HAL_StatusTypeDef MAX30100_write_register(MAX30100 *sensor, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Write(sensor->i2c_handle, MAX30100_I2C_WRITE, reg, sizeof(reg), data, sizeof(data), I2C_TIMEOUT);
}