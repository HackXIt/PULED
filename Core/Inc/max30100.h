/* MAX30100.h
 *   by Nikolaus Rieder (ic20b028@technikum-wien.at | hackxit@gmail.com)
 *
 * Created:
 *   January 25, 2022, 7:20:44 PM GMT+1
 * Last edited:
 *   January 25, 2022, 11:38:37 PM GMT+1
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

/* -------- DEFINITIONS -------- */

/* NOTE Generic definitions for MAX30100 */
#define MAX_READ_SIZE 64 // Total bytes of FIFO data according to P.13
#define MAX_SAMPLES 16 // Total number of samples in memory according to P.13
#define SAMPLE_SIZE 4 // 4 bytes per sample

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
#define SAMPLE_RATE_100 0x07 // 111 - 1000 samples per second
// NOTE Pulse Width Control - Configuration Masks
#define LED_PW 0x03 // B1, B0 - LED Pulse Width Control [1:0]
#define PULSE_WIDTH_200 0x00 // 00 - Pulse Width 200us + 13 ADC Resolution Bits
#define PULSE_WIDTH_400 0x01 // 01 - Pulse Width 400us + 14 ADC Resolution Bits
#define PULSE_WIDTH_800 0x02 // 10 - Pulse Width 800us + 15 ADC Resolution Bits
#define PULSE_WIDTH_1600 0x03 // 11 - Pulse Width 1600us + 16 ADC Resolution Bits

/* NOTE Masks for LED Current Control - Datasheet P. 17 LED_CONFIG 0x09 */
#define RED_PA 0xF0 // B7, B6, B5, B4 - Red LED Current Control [3:0]
#define IR_PA 0x0F // B3, B2, B1, B0 - IR LED Current Control [3:0]
// FIXME I didn't provide configuration MASKS for the current control

/* NOTE Masks for Temperature Data - Datasheet P. 18 TEMP_INTEGER & TEMP_FRACTION */
#define TINT 0xFF // 8-Bits - Temperature Integer Value
#define TFRAC 0x0F // 4-Bits - Temperature Fraction Value

#pragma endregion END_OF_MASKS

/* -------- STRUCTURES -------- */

/* NOTE Sensor Datastructure holding information */
typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    uint16_t IR_data; // One sample of IR data
    uint16_t RED_data; // One sample of RED data
    float temperature; // Temp_Integer + Temp_Fraction
} MAX30100;

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

INIT_STATUS MAX30100_initialize(MAX30100 *sensor, I2C_HandleTypeDef *i2c_handle);

/* -------- DATA ACQUISITION -------- */

HAL_StatusTypeDef MAX30100_read_temperature(MAX30100 *sensor);
HAL_StatusTypeDef MAX30100_read_samples(MAX30100 *sensor);

/* -------- ABSTRACTION FUNCTIONS -------- */

HAL_StatusTypeDef MAX30100_read_register(MAX30100 *sensor, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MAX30100_read_registers(MAX30100 *sensor, uint8_t reg, uint8_t *data, uint8_t size);
HAL_StatusTypeDef MAX30100_write_register(MAX30100 *sensor, uint8_t reg, uint8_t *data);

#endif