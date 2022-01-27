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
#include "max30100.h"

// NOTE Standard Library Imports
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_BUFFER 255
#define APPROX_1_SEC 10000
#define DEBOUNCE 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* NOTE Activate protocol example or UART test sections during initialization */
// #define PROTOCOL_EXAMPLE
// #define UART_TEST

/* NOTE Configure trigger of interrupt from heartrate module
 * These macro's are dependant on the HR_ONLY_MODE and SPO2_MODE flags set in max30100.h !!!
 * TRIGGER_ON_FIFO - Trigger reacts to FIFO full and reads all samples (and prints result in main)
 * TRIGGER_ON_SAMPLE - Trigger reacts on HR_RDY or SPO2_RDY and reads 1 sample (and prints immediatly)
 * TRIGGER_ON_TEMP - Trigger reacts on TEMP_RDY and reads temperature (and prints immediatly)
 */
#define TRIGGER_ON_FIFO
// #define TRIGGER_ON_SAMPLE
/* FIXME Since temperature reading is not working, this trigger is also ineffective*/
// #define TRIGGER_ON_TEMP

/* NOTE Activate/Deactivate debugging message */
// #define DEBUG
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MAX30100 heartrate_sensor;
uint8_t init_status = 0x00;
uint8_t current_status = 0x00;
bool data_received = false;
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
void uart_transmit(char *message);
void uart_dev_log(char *log);
void check_init_error(INIT_STATUS init_status);
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

/* NOTE Testing UART communication */
#ifdef UART_TEST
    char *test_msg = "Hello World! \r\n";
    // Delays start of actual program by 5 seconds
    // Also prints Hello World 5 times & blinks led with each print.
    for (uint8_t i = 0; i < 5; i++)
    {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
        HAL_UART_Transmit(&huart2, (uint8_t *)test_msg, sizeof(char) * strlen((char *)test_msg), 50);
        HAL_Delay(1000);
    }
#endif /*UART_TEST*/

// NOTE Print example message via protocol implementation
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

    init_status = MAX30100_initialize(&heartrate_sensor, &hi2c1);
    check_init_error(init_status);
    if (init_status > 0) // Retry initialization if failed (using MAX_RETRY for amount of retries)
    {
        uart_transmit("Retrying...\r\n");
        uint8_t retry_counter = 0;
        char retry_message[24]; // Length of "Retry-Attempt xx...\r\n" +1 (or +3 ???)
        do
        {
            sprintf(retry_message, "Retry-Attempt %d...\r\n", retry_counter);
            uart_transmit(retry_message);
            init_status = MAX30100_initialize(&heartrate_sensor, &hi2c1);
            retry_counter += (init_status != HAL_OK);
            check_init_error(init_status);
        } while (init_status >= 1 && retry_counter <= MAX_RETRY);
    }
#ifdef TEMP_ONLY_MODE
    uint8_t data = MODE_TEMP_EN;
#endif
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (data_received)
        {
            char heartrate_message[MSG_BUFFER]; // 22 chars + 2 * 16 bit number + ???
            for (int i = 0; i < MAX_SAMPLES; i++)
            {
                sprintf(heartrate_message, "HR: %d - O2: %d - Temp: %f\r\n",
                        heartrate_sensor.IR_data[i],
                        heartrate_sensor.RED_data[i],
                        heartrate_sensor.temperature);
                uart_transmit(heartrate_message);
            }
            data_received = false;
        }
#ifdef TEMP_ONLY_MODE
        MAX30100_write_register(&heartrate_sensor, MODE_CONFIG, &data);
        HAL_Delay(1000);
#endif
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
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
    RCC_OscInitStruct.PLL.PLLN = 16;
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

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
    hi2c1.Init.Timing = 0x00707CBB;
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

    /*Configure GPIO pin : BUTTON_INT_Pin */
    GPIO_InitStruct.Pin = BUTTON_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : RGB_BLUE_Pin RGB_RED_Pin RGB_GREEN_Pin */
    GPIO_InitStruct.Pin = RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : HR_INT_Pin */
    GPIO_InitStruct.Pin = HR_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(HR_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BOARD_LED_Pin */
    GPIO_InitStruct.Pin = BOARD_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    __NOP();
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    __NOP();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* NOTE On button press, the samples & temperature are read once and printed in main-loop */
    if (GPIO_Pin == BUTTON_INT_Pin)
    {
        uint8_t counter = 0;
        while (HAL_GPIO_ReadPin(BUTTON_INT_GPIO_Port, BUTTON_INT_Pin) == GPIO_PIN_RESET)
        {
            counter++;
            if (counter > DEBOUNCE)
            {
                break;
            }
        }
        MAX30100_read_samples(&heartrate_sensor);
        MAX30100_read_temperature(&heartrate_sensor);
        data_received = true;
    }

    if (GPIO_Pin == HR_INT_Pin)
    {
        uint8_t interrupt_status = MAX30100_read_interrupts(&heartrate_sensor);
#ifdef DEBUG
        char interrupt_message[23]; // 15 chars + 1 * 8 bit number = 23
        sprintf(interrupt_message, "Interrupt: 0x%x\r\n", interrupt_status);
        uart_transmit(interrupt_message);
#endif
#ifdef TRIGGER_ON_FIFO
        if (interrupt_status & FIFO_FULL)
        {
            MAX30100_read_samples(&heartrate_sensor); // FIXME Ignoring HAL_Status currently
            data_received = true;
        }
#endif
#ifdef TRIGGER_ON_SAMPLE
        if ((interrupt_status & HEARTRATE_READY) || (interrupt_status & SPO2_READY))
        {
            // HAL_Status is ignored because we're reading only 1 sample and it will happen 100 times per second
            MAX30100_read_sample(&heartrate_sensor);
            char sample_message[43]; // 11 chars + 2 * 16 bit number = 43
            sprintf(sample_message, "HR: %d O2: %d\r\n",
                    heartrate_sensor.hr_sample,
                    heartrate_sensor.spo2_sample);
            uart_transmit(sample_message);
        }
#endif
#ifdef TRIGGER_ON_TEMP
        if (interrupt_status & TEMP_READY)
        {
            MAX30100_read_temperature(&heartrate_sensor); // FIXME Ignoring HAL_Status currently
            char temperature_message[MSG_BUFFER];
            sprintf(temperature_message, "Temp: %f\r\n", heartrate_sensor.temperature);
            uart_transmit(temperature_message);
        }
#endif
    }
}

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __NOP(); // To be able to debug here with breakpoints
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    __NOP(); // To be able to debug here with breakpoints
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

void uart_transmit(char *message)
{
    if (message == NULL)
    {
        return;
    }
    HAL_StatusTypeDef ret;
    uint16_t len = (sizeof(char) * strlen(message)) + 1;
    ret = HAL_UART_Transmit(&huart2, (uint8_t *)message, len, UART_TIMEOUT);
    if (ret != HAL_OK)
    {
        led_error_blink();
    }
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
        led_error_blink();
    }
}
void check_init_error(INIT_STATUS init_status)
{
    switch (init_status) // Check & Handle init_status of initialization
    {
    case REVISION_FAILED:
        uart_transmit("Communication error: Receiving revision number failed...\r\n");
        break;
    case REVISION_FALSE:
        uart_transmit("Incompatible sensor: Sensor has wrong revision number...\r\n");
        break;
    case PART_ID_FAILED:
        uart_transmit("Communication error: Receiving sensor Part-ID failed...\r\n");
        break;
    case PART_ID_FALSE:
        uart_transmit("Incompatible sensor: Sensor has wrong Part-ID...\r\n");
        break;
    case RESET_FAILED:
        uart_transmit("Communication error: Resetting sensor failed...\r\n");
        break;
    case CONFIG_FAILED:
        uart_transmit("Communication error: Configuring sensor mode failed...\r\n");
        break;
    case SPO2_CONFIG_FAILED:
        uart_transmit("Communication error: Sensor SPO2 configuration failed...\r\n");
        break;
    case INITIAL_TEMP_FAILED:
        uart_transmit("Communication error: Receiving initial temperature reading failed...\r\n");
        break;
    case LED_CONFIG_FAILED:
        uart_transmit("Communication error: Configuring sensor LEDs failed...\r\n");
        break;
    case INTERRUPT_CONFIG_FAILED:
        uart_transmit("Communication error: Configuring sensor interrupts failed...\r\n");
        break;
    default:
        uart_transmit("MAX30100 sensor completed initialization.\r\n");
        break;
    }
}

void led_blue_blink()
{
    HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, GPIO_PIN_RESET);
    _tim_timeout_nonblocking_with_callback(1000, led_reset);
}

void led_green_blink()
{
    HAL_GPIO_WritePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin, GPIO_PIN_RESET);
    _tim_timeout_nonblocking_with_callback(1000, led_reset);
}

// Error blinking is yellow - something went wrong, but rest is ok
void led_error_blink()
{
    HAL_GPIO_WritePin(GPIOA, RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET); // RED + GREEN = YELLOW
    _tim_timeout_nonblocking_with_callback(1000, led_reset);
}

void led_reset()
{
    HAL_GPIO_WritePin(GPIOA, RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_SET);
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
        led_reset();
        // Blink RED RGB 5 times on terrible error
        for (int i = 0; i < 5; i++)
        {
            HAL_GPIO_WritePin(GPIOA, RGB_RED_Pin, GPIO_PIN_RESET);
            for (int i = 0; i < APPROX_1_SEC; i++)
                ; // Approximately delaying for 1 second
            led_reset();
        }
        break;
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
