/* MAX30100.h
 *   by Nikolaus Rieder (ic20b028@technikum-wien.at | hackxit@gmail.com)
 *
 * Created:
 *   January 25, 2022, 7:20:44 PM GMT+1
 * Last edited:
 *   January 27, 2022, 9:48:46 AM GMT+1
 * Auto updated?
 *   Yes
 *
 * Description:
 *   Header of MAX30100 library
**/

#ifndef MAX30100_H
#define MAX30100_H

/* -------- IMPORTS -------- */

#include "stm32l4xx_hal.h" /* Required for HAL_Library Functions */

/* -------- FLAGS -------- */
/* NOTE Activate "Heartrate only mode" or "SpO2 Mode"
 * Does not affect the sample rate or pulse width!
 */
/* FIXME Currently only heartrate only mode is working, dunno why, temperature just won't work
 * Also SpO2 generally doesn't work, even when ignoring temperature, the LED just doesn't turn on.
 */
#define HR_ONLY_MODE
// #define SPO2_MODE
// #define TEMP_ONLY_MODE

/* -------- DEFINITIONS -------- */

/* NOTE PLEASE IGNORE the compiler warning about "#pragma region" - It doesn't have any negative effect */

/* NOTE Generic definitions for MAX30100 */
#define MAX_READ_SIZE 64 // Total bytes of FIFO data according to P.13
#define MAX_SAMPLES 16 // Total number of samples in memory according to P.13
#define SAMPLE_SIZE 4 // 4 bytes per sample
#define MAX_RETRY 20 // Maximum amount of retries for I2C communication retries

/* NOTE I2C definitions for MAX30100 */
#pragma region I2C
#define MAX30100_I2C_WRITE 0xAE // This already takes the write-bit into account
#define MAX30100_I2C_READ 0xAF // This already takes the read-bit into account
// #define MAX30100_REVISION 0xFF // NOTE The documentation didn't provide a revision number
// #define MAX30100_PART_ID 0xFF // NOTE The documentation didn't provide a part id
#define SETUP_DELAY 20
#define I2C_TIMEOUT 30
#define UART_TIMEOUT 30
#define WRITE 0x00
#define READ 0x01
#pragma endregion END_OF_I2C

/* NOTE 8-Bit Registers of MAX30100 - Datasheet P.10 Table 1 */
#pragma region REGISTERS
#define INT_STATUS 0x00 // Interrupt status
#define INT_ENABLE 0x01 // Interrupt enable
#define FIFO_WRITE_PTR 0x02 // FIFO Write Pointer
#define OVER_FLOW_CNT 0x03 // Over Flow Counter
#define FIFO_READ_PTR 0x04 // FIFO Read Pointer
#define FIFO_DATA_REG 0x05 // FIFO Data Register
#define MODE_CONFIG 0x06 // Mode configuration
#define SPO2_CONFIG 0x07 // SPO2 Configuration
// RESERVED 0x08
#define LED_CONFIG 0x09 // LED Configuration
// RESERVED 0x0A - 0x15
#define TEMP_INTEGER 0x16 // Temperature Integer Register
#define TEMP_FRACTION 0x17 // Temperature Fraction
#define REV_ID 0xFE // Revision ID
#define PART_ID 0xFF // Part ID
#pragma endregion END_OF_REGISTERS

/* NOTE Masks for defining specific bit(s) in code as readable words */
#pragma region MASKS
/* NOTE Masks for Interrupt Status & Enable - Datasheet P. 11+12 INT_STATUS & INT_ENABLE
 * The masks are the same for both registers,
 */
#define FIFO_FULL 0x80 // B7 - FIFO Almost Full Flag
#define TEMP_READY 0x40 // B6 - Temperature Ready Flag
#define HEARTRATE_READY 0x20 // B5 - Heart Rate Data Ready Flag
#define SPO2_READY 0x10 // B4 - SpO2 Data Ready Flagg
// RESERVED B3, B2, B1
#define POWER_READY 0x01 // B0 - Power Ready Flag (Not usable with INT_ENABLE)

/* NOTE Masks for FIFO Registers - Datasheet P. 12 FIFO (0x02-0x05) */
#define FIFO_MASK 0x0F // Mask for Bit [3:0] in FIFO_WRITE_PTR, OVER_FLOW_CNT and FIFO_READ_PTR
#define FIFO_DATA 0xFF // Mask for FIFO_DATA [7:0] - not really necessary, but for completeness

/* NOTE Masks for Mode Configuration - Datasheet P. 15 MODE_CONFIG 0x06 */
#define MODE_SHDN 0x80 // B7 - Shutdown Control
#define MODE_RESET 0x40 // B6 - Reset Control
// UNUSED B5, B4
#define MODE_TEMP_EN 0x08 // B3 - Temperature Enable
#define MODE 0x07 // B2, B1, B0 - Mode Control [2:0]
#define HR_ONLY 0x02 // 010 - HR only enabled
#define SPO2_EN 0x03 // 011 - SPO2 enabled

/* NOTE Masks for SpO2 Configuration - Datasheet P. 16+17 SPO2_CONFIG 0x07 */
// UNUSED B7
#define SPO2_HI_RES_EN 0x40 // B6 - SpO2 High Resolution Enable (ADC resolution 16-bit w/ 1.6ms pulse width)
// RESERVED B5
// NOTE Sample Rate Control - Configuration Masks
#define SPO2_SR 0x1C // B4, B3, B2 - SpO2 Sample Rate Control [2:0]
#define SAMPLE_RATE_50 0x00 // 000 - 50 samples per second
#define SAMPLE_RATE_100 0x01 // 001 - 100 samples per second
#define SAMPLE_RATE_167 0x02 // 010 - 167 samples per second
#define SAMPLE_RATE_200 0x03 // 011 - 200 samples per second
#define SAMPLE_RATE_400 0x04 // 100 - 400 samples per second
#define SAMPLE_RATE_600 0x05 // 101 - 600 samples per second
#define SAMPLE_RATE_800 0x06 // 110 - 800 samples per second
#define SAMPLE_RATE_1000 0x07 // 111 - 1000 samples per second
// NOTE Pulse Width Control - Configuration Masks
#define LED_PW 0x03 // B1, B0 - LED Pulse Width Control [1:0]
#define PULSE_WIDTH_200 0x00 // 00 - Pulse Width 200us + 13 ADC Resolution Bits
#define PULSE_WIDTH_400 0x01 // 01 - Pulse Width 400us + 14 ADC Resolution Bits
#define PULSE_WIDTH_800 0x02 // 10 - Pulse Width 800us + 15 ADC Resolution Bits
#define PULSE_WIDTH_1600 0x03 // 11 - Pulse Width 1600us + 16 ADC Resolution Bits

/* NOTE Masks for LED Current Control - Datasheet P. 17 LED_CONFIG 0x09 */
#define RED_PA 0xF0 // B7, B6, B5, B4 - Red LED Current Control [3:0]
#define IR_PA 0x0F // B3, B2, B1, B0 - IR LED Current Control [3:0]
/* NOTE To use the following LED currents for RED_PA, the value needs to be left-shifted by 4 bits */
#define PA_0 0x00 // 0000 - Typical LED current 0.0mA
#define PA_4 0x01 // 0001 - Typical LED current 4.4mA
#define PA_7 0x02 // 0010 - Typical LED current 7.6mA
#define PA_11 0x03 // 0011 - Typical LED current 11.0mA
#define PA_14 0x04 // 0100 - Typical LED current 14.2mA
#define PA_17 0x05 // 0101 - Typical LED current 17.4mA
#define PA_20 0x06 // 0110 - Typical LED current 20.8mA
#define PA_24 0x07 // 0111 - Typical LED current 24.0mA
#define PA_27 0x08 // 1000 - Typical LED current 27.1mA
#define PA_30 0x09 // 1001 - Typical LED current 30.6mA
#define PA_33 0x0A // 1010 - Typical LED current 33.8mA
#define PA_37 0x0B // 1011 - Typical LED current 37.0mA
#define PA_40 0x0C // 1100 - Typical LED current 40.2mA
#define PA_43 0x0D // 1101 - Typical LED current 43.6mA
#define PA_46 0x0E // 1110 - Typical LED current 46.8mA
#define PA_50 0x0F // 1111 - Typical LED current 50.0mA


/* NOTE Masks for Temperature Data - Datasheet P. 18 TEMP_INTEGER & TEMP_FRACTION */
#define TINT 0xFF // 8-Bits - Temperature Integer Value
#define TFRAC 0x0F // 4-Bits - Temperature Fraction Value
#pragma endregion END_OF_MASKS

/* -------- STRUCTURES -------- */

/* NOTE Sensor Datastructure holding information */
typedef struct {
    I2C_HandleTypeDef *i2c_handle; // Handle for I2C communication
    uint16_t IR_data[MAX_SAMPLES]; // All infrared ADC samples
    uint16_t RED_data[MAX_SAMPLES]; // All red led ADC samples
    uint16_t hr_sample; // Current heartrate sample
    uint16_t spo2_sample; // Current SPO2 sample
    float temperature; // Converted temperature reading according to formula
} MAX30100;

/* NOTE Error codes which may happen during initialization */
typedef enum {
    INIT_OK = 0x00,
    REVISION_FAILED,
    REVISION_FALSE,
    PART_ID_FAILED,
    PART_ID_FALSE,
    RESET_FAILED,
    CONFIG_FAILED,
    SPO2_CONFIG_FAILED,
    INITIAL_TEMP_FAILED,
    LED_CONFIG_FAILED,
    INTERRUPT_CONFIG_FAILED    
} INIT_STATUS;

/* -------- INITIALIZATION -------- */

/************************************************
 * @brief Initializes the MAX30100 sensor and prepares sensor datastructure
 * @note The initialization is dependant on MACROS
 * @param sensor - Sensor datastructure
 * @param i2c_handle - I2C Communication handle to use & store in datastructure
 * @return INIT_STATUS - Returns initialization status code
 ***********************************************/
INIT_STATUS MAX30100_initialize(MAX30100 *sensor, I2C_HandleTypeDef *i2c_handle);

/* -------- DATA ACQUISITION -------- */

/************************************************
 * @brief Reads & properly converts the temperature of MAX30100
 * 
 * @param sensor - Sensor datastructure
 * @return HAL_StatusTypeDef - Returns communication status
 ***********************************************/
HAL_StatusTypeDef MAX30100_read_temperature(MAX30100 *sensor);
/************************************************
 * @brief Reads all currently available samples of MAX30100
 * Currently available is calculated with the Read & Write Pointer of the FIFO
 * @param sensor - Sensor datastructure
 * @return HAL_StatusTypeDef  - Returns communication status
 ***********************************************/
HAL_StatusTypeDef MAX30100_read_samples(MAX30100 *sensor);
/************************************************
 * @brief Reads one sample of MAX30100
 * 
 * @param sensor - Sensor datastructure
 * @return HAL_StatusTypeDef - Returns communication status
 ***********************************************/
HAL_StatusTypeDef MAX30100_read_sample(MAX30100 *sensor);
/************************************************
 * @brief Reads the status of the interrupt register of MAX30100
 * 
 * @param sensor - Sensor datastructure
 * @return uint8_t - Returns data of the interrupt register
 ***********************************************/
uint8_t MAX30100_read_interrupts(MAX30100 *sensor);

/* -------- ABSTRACTION FUNCTIONS -------- */

/************************************************
 * @brief Communication function, which reads 1 byte of data from given register of MAX30100
 * 
 * @param sensor - Sensor datastructure
 * @param reg - Register of MAX30100 to be read
 * @param data - Pointer to target
 * @return HAL_StatusTypeDef - Returns communication status
 ***********************************************/
HAL_StatusTypeDef MAX30100_read_register(MAX30100 *sensor, uint8_t reg, uint8_t *data);
/************************************************
 * @brief Communication function, which reads X byte of data from given register of MAX30100
 * @note The only register where this is applicable is the FIFO, all others are only 1 byte
 * @param sensor - Sensor datastructure
 * @param reg - Register of MAX30100 to be read
 * @param data - Pointer to target (should have appropriate size)
 * @param size - Amount of bytes to be read
 * @return HAL_StatusTypeDef - Returns communication status
 ***********************************************/
HAL_StatusTypeDef MAX30100_read_registers(MAX30100 *sensor, uint8_t reg, uint8_t *data, uint8_t size);
/************************************************
 * @brief Communication function, which writes 1 byte of data into given register of MAX30100
 * 
 * @param sensor - Sensor datastructure
 * @param reg - Register of MAX30100 to be written to
 * @param data - Pointer of source
 * @return HAL_StatusTypeDef - Returns communication status
 ***********************************************/
HAL_StatusTypeDef MAX30100_write_register(MAX30100 *sensor, uint8_t reg, uint8_t *data);
#endif