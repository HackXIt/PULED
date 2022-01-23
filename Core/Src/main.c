/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// NOTE Custom Library Imports
#include "protocol.h"
// NOTE Heartrate Module Imports
#include "main.h"

// NOTE Standard Library Imports
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// NOTE I2C definitions for MAX30100
#define MAX_READ_SIZE 64
#define MAX30100_I2C_ADR 0x57
#define I2C_TIMEOUT 30
#define UART_TIMEOUT 30
#define WRITE 0x00
#define READ 0x01
#define RESET 0x40

// NOTE Registers of MAX30100
#define INT_STATUS 0x00
#define INT_ENABLE 0x01
#define FIFO_WRITE_PTR 0x02
#define OVER_FLOW_CNT 0x03
#define FIFO_READ_PTR 0x04
#define FIFO_DATA_REG 0x05
#define MODE_CONFIG 0x06
#define SPO2_CONFIG 0x07
#define LED_CONFIG 0x09
#define TEMP_INTEGER 0x16
#define TEMP_FRACTION 0x17

// NOTE Register Masks for getting specific values
#define HR_RDY 0x20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// #define PROTOCOL_EXAMPLE
#define UART_TEST
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef ret; // Return value for HAL Library functions

// Address of MAX30100 but left-shifted to make room for R/W Bit
// Since write is 0, the "| WRITE" is actually not necessary but is done here simply for being symmetric
static const uint8_t MAX30100_ADDR_W = (MAX30100_I2C_ADR << 1) | WRITE;
static const uint8_t MAX30100_ADDR_R = (MAX30100_I2C_ADR << 1) | READ;
uint8_t current_sample[4];
#ifndef CALLBACK
#define CALLBACK
typedef void (*CB)(void);
CB callback = NULL;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void _tim_timeout_nonblocking_with_callback(unsigned int ms, CB cb);
void uart_dev_log(char *log);
void led_error_blink();
void led_green_blink();
void led_blue_blink();
void led_reset();
void read_sample();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

// NOTE Example Message Initialization
#ifdef PROTOCOL_EXAMPLE
    MSG_t *my_message;
    link_t *current_item;
    uint8_t testArray[] = {
        PRINT_REQ,
        SEP,
        't',
        'e',
        's',
        't',
        '1',
        SEP,
        't',
        'e',
        's',
        't',
        '2',
        0x0};
#endif /*PROTOCOL_EXAMPLE*/

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM16_Init();
    /* USER CODE BEGIN 2 */
    led_reset(); // Turn on off RGB

#ifdef UART_TEST
    char *test_msg = "Hello World! \r\n";
    // Delays start of actual program by 5 seconds
    // Also prints Hello World 5 times.
    for (uint8_t i = 0; i < 5; i++)
    {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
        HAL_UART_Transmit(&huart2, (uint8_t *)test_msg, sizeof(char) * strlen((char *)test_msg), 50);
        HAL_Delay(1000);
    }
#endif /*UART_TEST*/

    // Setup Heartrate Module
    uint8_t reset_module = RESET;
    uint8_t module_setup = 0b00000010;
    /* module_setup
    B7 == SHDN => Shutdown not enabled
    B6 == RESET => Hardware Reset Enabled
    B3 == TEMP_ENABLE => Not enabled
    B2,B1,B0 == MODE => 010 ... Heartrate only enabled
    */
    uint8_t spo_setup = 0b00000011;
    /* spo_setup
    B7 == Not used
    B6 == SPO2_HI_RES_EN
    B5 == Reserved
    B4,B3,B2 == SPO2_SR[2:0]
    B1,B0 == LED_PW[1:0] => 11 ... Pulse Width = 1600us & ADC Resolution = 16 bit
    */
    uint8_t led_setup = 0b11111110;
    /*
    B7,B6,B5,B4 == RED_PA[3:0] (RED LED Current Control)
    B3,B2,B1,B0 == IR_PA[3:0] (IR LED Current Control)
    */
    uint8_t interrupt_setup = 0b00100000;
    /* interrupt setup
    B7 == ENB_A_FULL (FIFO Almost full interrupt)
    B6 == ENB_TEP_RDY (Temperature ready interrupt)
    B5 == ENB_HR_RDY (Heartrate ready interrupt)
    B4 == ENB_SO2_RDY (SpO2 Data ready interrupt)
    B3,B2,B1,B0 == Not used / Always 0
    */
    // Reset the device
    ret = HAL_I2C_Mem_Write(&hi2c1, MAX30100_ADDR_W, MODE_CONFIG, sizeof(uint8_t), &reset_module, sizeof(reset_module), I2C_TIMEOUT);
    // if (ret != HAL_OK)
    // {
    //     led_error_blink();
    // }
    // Write to register MODE_CONFIG using module_setup
    ret = HAL_I2C_Mem_Write(&hi2c1, MAX30100_ADDR_W, MODE_CONFIG, sizeof(uint8_t), &module_setup, sizeof(module_setup), I2C_TIMEOUT);
    // if (ret != HAL_OK)
    // {
    //     led_error_blink();
    // }
    ret = HAL_I2C_Mem_Write(&hi2c1, MAX30100_ADDR_W, LED_CONFIG, sizeof(uint8_t), &led_setup, sizeof(led_setup), I2C_TIMEOUT);
    // if (ret != HAL_OK)
    // {
    //     led_error_blink();
    // }
    ret = HAL_I2C_Mem_Write(&hi2c1, MAX30100_ADDR_W, INT_ENABLE, sizeof(uint8_t), &interrupt_setup, sizeof(spo_setup), I2C_TIMEOUT);
    // if (ret != HAL_OK)
    // {
    //     led_error_blink();
    // }
    ret = HAL_I2C_Mem_Write(&hi2c1, MAX30100_ADDR_W, SPO2_CONFIG, sizeof(uint8_t), &spo_setup, sizeof(spo_setup), I2C_TIMEOUT);
    // if (ret != HAL_OK)
    // {
    //     led_error_blink();
    // }
    uart_dev_log("Completed module setup.\r\n");
    uint8_t interrupt_status = 0x00;
    char sample_string[40] = {};
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        HAL_I2C_Mem_Read(&hi2c1, MAX30100_ADDR_R, INT_STATUS, sizeof(uint8_t), &interrupt_status, sizeof(interrupt_status), I2C_TIMEOUT);
        if (interrupt_status != 0)
        {
            read_sample();
            sprintf(sample_string, "1: %d | 2: %d | 3: %d | 4: %d\r\n",
                    current_sample[0],
                    current_sample[1],
                    current_sample[2],
                    current_sample[3]);
            uart_dev_log(sample_string);
            // HAL_Delay(5000); // Wait 5 seconds after reading samples
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

// NOTE Example message deserialization & serialization
#ifdef PROTOCOL_EXAMPLE
        my_message = deserialize_message(testArray);
        current_item = my_message->content->head;
        while (current_item != NULL)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)current_item->item, sizeof(char) * strlen((char *)current_item->item), 50);
            HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", sizeof(char) * strlen("\r\n"), 50);
            // HAL_Delay(1000);
            current_item = current_item->next;
        }

        uint8_t length = get_length_of_message(my_message);
        char str[4];
        sprintf(str, "%i", length);
        HAL_UART_Transmit(&huart2, (uint8_t *)str, sizeof(char) * strlen(str), 50);
        uint8_t *msg = serialize_message(my_message);
        size_t size = strlen((char *)msg) + 1;
        HAL_UART_Transmit(&huart2, msg, size, 50);
        HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", sizeof(char) * strlen("\r\n"), 50);
        free(msg);
#endif /*PROTOCOL_EXAMPLE*/
        /* USER CODE END 3 */
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
  */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure LSE Drive Capability
  */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
    /** Enable MSI Auto calibration
  */
    HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10909CEC;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Analogue filter
  */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Digital filter
  */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

    /* USER CODE BEGIN TIM16_Init 0 */

    /* USER CODE END TIM16_Init 0 */

    /* USER CODE BEGIN TIM16_Init 1 */

    /* USER CODE END TIM16_Init 1 */
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 32000 - 1;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 65536 - 1;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM16_Init 2 */

    /* USER CODE END TIM16_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : EXTI3_BUTTON_Pin */
    GPIO_InitStruct.Pin = EXTI3_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(EXTI3_BUTTON_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : RGB_BLUE_Pin RGB_RED_Pin RGB_GREEN_Pin */
    GPIO_InitStruct.Pin = RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BOARD_LED_Pin */
    GPIO_InitStruct.Pin = BOARD_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    __NOP();
    if (htim == &htim16)
    {
        // Stop the timer
        HAL_TIM_Base_Stop_IT(&htim16);
        // Execute global callback if not NULL
        (*callback)();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    __NOP(); // To be able to debug in here with breakpoints
    // led_green_blink();
}

void _tim_timeout_nonblocking_with_callback(unsigned int ms, CB cb)
{
    callback = cb; // Set global function pointer variable to callback
    // Activate flag for doing action in main loop
    // Not used here
    // callback_flag = 1;

    // Set autoreload to specified milliseconds
    // This assumes that the configuration is set to have 1ms per tick
    __HAL_TIM_SET_AUTORELOAD(&htim16, ms);
    // Set counter to 0 for sanity (redundant - just making sure)
    __HAL_TIM_SET_COUNTER(&htim16, 0);
    // Event generation register => Update generation (Reinitialize counter & update registers)
    TIM16->EGR = 1;
    // Status register => Reset Update Interrupt Flag (Because it was set by EGR - line above)
    // Barebone: TIM16->SR &= 0
    __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim16);
}

void uart_dev_log(char *log)
{
    if (log == NULL)
    {
        return;
    }
    HAL_StatusTypeDef ret;
    uint16_t len = (sizeof(char) * strlen(log)) + 1;

    ret = HAL_UART_Transmit_IT(&huart2, (uint8_t *)log, len);
    if (ret != HAL_OK)
    {
        // Error during transmission of log.
        // led_error_blink();
    }
}

void led_green_blink()
{
    HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, GPIO_PIN_RESET);
    _tim_timeout_nonblocking_with_callback(1000, led_reset);
}

void led_error_blink()
{
    HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, GPIO_PIN_RESET);
    _tim_timeout_nonblocking_with_callback(1000, led_reset);
}

void led_reset()
{
    HAL_GPIO_WritePin(GPIOA, RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_SET);
}
// *uint8_t read_sample()
void read_sample()
{
    // uint8_t sample[4]; // One sample consists of 4 Byte
    // We need to read from the device 4 times in order to read a whole FIFO
    for (int i = 0; i < 4; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MAX30100_ADDR_R, FIFO_DATA_REG, sizeof(uint8_t), &current_sample[i], sizeof(uint8_t), I2C_TIMEOUT);
    }
    // return sample;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
