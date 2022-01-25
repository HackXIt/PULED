/* MAX 30100.c
 *   by Nikolaus Rieder
 *
 * Created:
 *   January 25, 2022, 7:21:13 PM GMT+1
 * Last edited:
 *   January 25, 2022, 11:45:31 PM GMT+1
 * Auto updated?
 *   Yes
 *
 * Description:
 *   Source file of MAX30100 library
**/

/*--- COMMON LIBRARIES ---*/

/*--- CUSTOM LIBRARIES ---*/
#include "max30100.h"
#include "stm32l4xx_hal.h"

/*--- MACROS ---*/
#define HR_ONLY_MODE
// #define SPO2_MODE

/*--- Global variables ---*/
uint16_t ir_samples[MAX_SAMPLES];
uint16_t red_samples[MAX_SAMPLES];

/* -------- INITIALIZATION -------- */

INIT_STATUS MAX30100_initialize(MAX30100 *sensor, I2C_HandleTypeDef *i2c_handle)
{
    /* NOTE Set structure parameters to initial values */
    sensor->i2c_handle = i2c_handle;
    sensor->IR_data = 0x00;
    sensor->RED_data = 0x00;
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
    data |= (MODE & SPO2_EN);
#endif
    status = MAX30100_write_register(sensor, MODE_CONFIG, &data);
    if (status != HAL_OK)
        return CONFIG_FAILED;

    /* NOTE SPO2 configuration MAX30100 */
    data = 0x00;
#ifdef HR_ONLY_MODE
    data |= (SPO2_SR & SAMPLE_RATE_100) | (LED_PW & PULSE_WIDTH_1600);
#endif
#ifdef SPO2_MODE
    data |= (SPO2_SR & SAMPLE_RATE_100) | (LED_PW & PULSE_WIDTH_1600);
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

    /* NOTE LED configuration MAX30100 */
    data = 0x00 | 0xEF; // RED_PA = 1110 (46.8mA) / IR_PA = 1111 (50mA) => 0xEF
    status = MAX30100_write_register(sensor, LED_CONFIG, &data);
    if (status != HAL_OK)
        return LED_CONFIG_FAILED;

    /* NOTE Enable Interrupts */
    data = 0x00 | (FIFO_FULL);
#ifdef HR_ONLY_MODE
    data |= HEARTRATE_READY;
#endif
#ifdef SPO2_MODE
    data |= (TEMP_READY | HEARTRATE_READY | SPO2_READY);
#endif
    status = MAX30100_write_register(sensor, INT_ENABLE, &data);
    if (status != HAL_OK)
        return INTERRUPT_CONFIG_FAILED;

    /* NOTE End of initialization */
    return INIT_OK;
}

/* -------- DATA ACQUISITION -------- */

HAL_StatusTypeDef MAX30100_read_temperature(MAX30100 *sensor)
{
    HAL_StatusTypeDef status[4];
    uint8_t temp_ready = 0x00;
    int8_t temperature = 0x00;
    uint8_t fraction = 0x00;
    status[0] = MAX30100_write_register(sensor, MODE_CONFIG, MODE_TEMP_EN); // Initiate temperature reading
    uint8_t interrupt_error_counter = 0 do                                  // Wait for temperature reading to be ready
    {
        // Using status[4] since an error in temperature reading is more important and therefor prioritized
        // Read interrupt status flag
        status[4] = MAX30100_read_register(sensor, INT_STATUS, &temp_ready);
        // Increment error counter
        interrupt_error_counter += (status[4] != HAL_OK);
        // Ignore all bits other than TEMP_READY
        temp_ready &= TEMP_READY;
        // Loop until temperature is ready or communication failed 20 times in total
    }
    while (!temp_ready && interrupt_error_counter <= 20)
        ;
    status[1] = MAX30100_read_register(sensor, TEMP_INTEGER, &temperature);
    status[2] = MAX30100_read_register(sensor, TEMP_FRACTION, &fraction);
    fraction &= 0x07; // Ignoring all other fraction bits, if wrongly set.
    sensor->temperature = (float)temperature + (fraction * 0.0625);
    // Check if there were errors, otherwise return HAL_OK
    for (int i = 0; i < 4; i++)
    {
        if (status[i] != HAL_OK)
        {
            return status[i]; // Will return the first error that occured
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef MAX30100_read_samples(MAX30100 *sensor)
{
    uint8_t sample_number = 0;
    uint8_t writePtr = 0;
    uint8_t readPtr = 0;
    uint8_t samples[MAX_SAMPLES] = {0}; // All samples
    uint8_t sample[4];                  // Currently read samples
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
            status = MAX30100_read_registers(sensor, FIFO_DATA_REG, &sample, SAMPLE_SIZE);

            ir_samples[i] = (uint16_t)sample[1];        // Save lower-half of sample IR[7:0]
            ir_samples[i] |= (uint16_t)sample[0] << 8;  // Shift upper-half of sample IR[15:8]
            red_samples[i] = (uint16_t)sample[3];       // Save lower-half of sample RED[7:0]
            red_samples[i] |= (uint16_t)sample[2] << 8; // Shift upper-half of sample RED[15:8]
        }
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