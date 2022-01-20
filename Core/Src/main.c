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
// #include "heartrate1_hal.h"
// #include "heartrate1_hw.h"

#define HANDLE (&hi2c1)

#include "heartrate1_hw.h"

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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// #define DataIsReady() (dataReady == 0)
// #define DataIsNotReady() (dataReady != 0)

// #define PROTOCOL_EXAMPLE
#define UART_TEST
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// static const uint8_t MAX30100_ADDR = MAX30100_I2C_ADR << 1; // Shift left by 1 bit to make room for R/W bit
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

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
    // uint8_t dataReady;
    /* USER CODE BEGIN 1 */
    // HAL_StatusTypeDef ret;
    // uint8_t buf[12];
    // uint16_t val;
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
    HAL_GPIO_WritePin(GPIOA, RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_SET); // Turn on off RGB

#ifdef UART_TEST
    char *test_msg = "Hello World! \r\n";
    // Delays start of actual program by 10 seconds
    // Also prints Hello World 10 times.
    for (uint8_t i = 0; i < 10; i++)
    {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
        HAL_UART_Transmit(&huart2, (uint8_t *)test_msg, sizeof(char) * strlen((char *)test_msg), 50);
        HAL_Delay(1000);
    }
#endif /*UART_TEST*/

    uint16_t ir_average;
    uint16_t red_average;
    char ir_string[20]; // red_string[20], time_string[20];
    uint16_t temp_buffer_ctr;
    uint8_t sample_num;
    unsigned long ir_buff[16] = {0}, red_buff[16] = {0};
    static bool first_measurement, measurement_continues;

    hr_init(MAX30100_I2C_ADR);

    first_measurement = true;
    measurement_continues = false;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin); // Just a signal of life
        // dataReady = hr_get_status();
        if ((hr_get_status() & 0x20) != 0)
        {
            sample_num = hr_read_diodes(ir_buff, red_buff); // Read IR and RED sensor data and store it in ir_buff and red_buff
            if (sample_num >= 1)
            {
                ir_average = 0;
                red_average = 0;
                for (temp_buffer_ctr = 0; temp_buffer_ctr < sample_num; temp_buffer_ctr++)
                {
                    ir_average += ir_buff[temp_buffer_ctr];
                    red_average += red_buff[temp_buffer_ctr];
                }
                ir_average /= sample_num; // calculate the average value for this reading
                red_average /= sample_num;
                HAL_UART_Transmit(&huart2, (uint8_t *)&ir_average, sizeof(uint8_t), HAL_MAX_DELAY);
                HAL_UART_Transmit(&huart2, (uint8_t *)&red_average, sizeof(uint8_t), HAL_MAX_DELAY);

                if (ir_average > 10000)
                {
                    if (measurement_continues == false && first_measurement == false)
                    {
                        measurement_continues = true;
                    }

                    if (first_measurement == true) // if this is our first measurement, start the timer to count miliseconds
                    {
                        HAL_UART_Transmit(&huart2, (uint8_t *)"START\r\n", sizeof(char) * 8, HAL_MAX_DELAY);
                        first_measurement = false;
                    }
                    int len = snprintf(NULL, 0, "%f", (float)ir_average);
                    snprintf(ir_string, len + 1, "%f", (float)ir_average);
                    strcat(ir_string, "\r\n");
                    len = snprintf(NULL, 0, "%s", ir_string);
                    HAL_UART_Transmit(&huart2, (uint8_t *)ir_string, len + 1, HAL_MAX_DELAY);
                }
                else
                {
                    measurement_continues = false;
                }
                // HAL_I2c
                // buf[0] = INT_STATUS; // Read Interrupt status
                // ret = HAL_I2C_Master_Transmit(&hi2c1, MAX30100_ADDR, buf, 1, HAL_MAX_DELAY);
                // if (ret != HAL_OK) // Check if everything is fine
                // {
                //     strcpy((char *)buf, "Error Tx\r\n"); // Error message in case not
                // }
                // else
                // {
                //     ret = HAL_I2C_Master_Receive(&hi2c1, MAX30100_ADDR, buf, 2, HAL_MAX_DELAY);
                //     if (ret != HAL_OK)
                //     {
                //         strcpy((char *)buf, "Error Rx\r\n");
                //     }
                //     else
                //     {
                //     }
                // }
                // HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);

                // Just wait a second
                HAL_UART_Transmit(&huart2, (uint8_t *)"Finished cycle!\r\n", sizeof(char) * 18, HAL_MAX_DELAY);
                HAL_Delay(1000);
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
        }
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
    htim16.Init.Prescaler = 0;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 65535;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
