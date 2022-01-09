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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Error LED codes
typedef enum BMS_Error_Code
{
 BMS_ERR_INIT_BQ77,
 BMS_ERR_CALS_ADC,
 BMS_ERR_EH           // Generic Handle_Error() error
} BMS_Error_Code;


// I2C slave commands (byte 1 recieved)
typedef enum BMS_I2C_Command
{
  BMS_GET_STATUS,
  BMS_GET_PACK_VOLTAGE,
  BMS_GET_CURRENT,
  // BMS_TURN_OUTPUT_ON,   // Only available in HOST control mode
  // BMS_TURN_OUTPUT_OFF   // Only available in HOST control mode
} BMS_I2C_Command;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Generic I2C stuff
#define BMS_I2C_TIMEOUT 35U   // In ms

// I2C Slave config
#define BMS_I2C_SLAVE_ADDR_BASE 0x40          // Base address of device

#define BMS_ERROR_VALUE 0xff  // Value returned if there was an error processing
                              //  a request on the slave I2C channel

// ADC polling timeout
#define BMS_ADC_TIMEOUT 35U   // In ms

// PINS
#define BMS_GPIO_0_BANK       GPIOB
#define BMS_GPIO_0_PIN        GPIO_PIN_0
#define BMS_GPIO_1_BANK       GPIOB
#define BMS_GPIO_1_PIN        GPIO_PIN_1
#define BMS_GPIO_SLOTID_BANK  GPIOB
#define BMS_GPIO_SLOTID_PIN   GPIO_PIN_2
#define BMS_GPIO_XALERT_BANK  GPIOA
#define BMS_GPIO_XALERT_PIN   GPIO_PIN_0
#define BMS_GPIO_VPACK_BANK   GPIOA
#define BMS_GPIO_VPACK_PIN    GPIO_PIN_1
#define BMS_GPIO_VOUT_BANK    GPIOA
#define BMS_GPIO_VOUT_PIN     GPIO_PIN_2
#define BMS_GPIO_IOUT_BANK    GPIOA
#define BMS_GPIO_IOUT_PIN     GPIO_PIN_3
#define BMS_GPIO_TEMP_BANK    GPIOA
#define BMS_GPIO_TEMP_PIN     GPIO_PIN_4
#define BMS_GPIO_LED0_BANK    GPIOB
#define BMS_GPIO_LED0_PIN     GPIO_PIN_12
#define BMS_GPIO_LED1_BANK    GPIOB
#define BMS_GPIO_LED1_PIN     GPIO_PIN_13
#define BMS_GPIO_LED2_BANK    GPIOB
#define BMS_GPIO_LED2_PIN     GPIO_PIN_14
#define BMS_GPIO_LED3_BANK    GPIOB
#define BMS_GPIO_LED3_PIN     GPIO_PIN_15


// ---
// Below: see Jan 2009 bq77PL900 datasheet
// ---

// I2C with bq77p900
#define BMS_BQ77_I2C_ADDR (0x10 << 1) // HEY: Does this need to be shifted?

// Registers
#define BMS_BQ77_STATUS   0x00
#define BMS_BQ77_OUTCTL   0x01
#define BMS_BQ77_STATECTL 0x02
#define BMS_BQ77_FUNCTL   0x03
#define BMS_BQ77_CELBALN  0x04
#define BMS_BQ77_CELSEL   0x05
#define BMS_BQ77_OVCFG    0x06
#define BMS_BQ77_UVCFG    0x07
#define BMS_BQ77_OUVCTL   0x08
#define BMS_BQ77_OCDCFG   0x09
#define BMS_BQ77_SCDCFG   0x0a
#define BMS_BQ77_EEPROM   0x0B

// See pg. 41
#define BMS_BQ77_STATUS_VGOOD (1 << 5)

// See pg. 42
// #define BMS_OC_DSC (1 << 1)
// #define BMS_OC_CHG (2 << 1)
#define BMS_FAST_SAMPL 0    // 0 = 50ms/cell, 1 = 100us/cell (pg. 42)


// See pg. 43
#define BMS_HOST_SHDWN 0    // In HOST mode, shutdown if PACK = 0V (disabled)
#define BMS_HOST_HMODE 0    // Set operating mode: 0 = auto, 1 = HOST
#define BMS_SNSE_CLVTG 0    // Sets cell voltage VGAIN (0.15)
#define BMS_SNSE_CLCUR 0    // Sets current monitor gain (10)

// See pg. 44
#define BMS_USES_THERM 0    // 0 = no thermistor, 1 = use thermistor

// Cell overvoltage controls
// See pg. 46
#define BMS_OV_THRSH 0x00   // The overvoltage threshold for a cell (4.15v)
#define BMS_OV_HYSTR 0x00   // Hysteresis voltage (0.1v)
#define BMS_OV_DLYTM 0x00   // Overvoltage delay time (0.5s)

// Cell undervoltage controls
// See pg. 47
#define BMS_UV_THRSH 0x00   // The overvoltage threshold for a cell (1.4v)
#define BMS_UV_HYSTR 0x00   // Hysteresis voltage (0.2v)
#define BMS_UV_SHDWN 0x00   // On undervoltage condition, automatically turn off
                            //  pack output

// Cell protection delays
// See pg. 48
#define BMS_OUV_VTRSH 0x00  // Voltage threshold?
#define BMS_OUV_DELAY 0x00  // Delay for UV condition (1s)

// Overcurrent controls
// See pg. 49
#define BMS_OCD_DELAY 0x00  // Delay for response after detection of overcurrent condition (20ms)
#define BMS_OCD_RCRMD 0     // Overcurrent/short circuit condition recovery mode (disconnected)
#define BMS_OCD_PRCRG 0     // 0-v precharge using GPOD enable (disabled)
#define BMS_OCD_BALNC 1     // Auto balance cells (enabled)

// Short-circuit controls
// See pg. 50
#define BMS_SCD_SCDVT 0x00  // Short-Circuit Discharge Voltage Threshold (60mV)
#define BMS_SCD_DELAY 0x01  // Short-circuit discharge delay (60us)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

uint16_t bms_i2c_addr;  // Address of BMS (slave interface)
uint8_t data_buf;       // Data to be sent or received over I2C
uint32_t hadc_val;      // Read ADC value

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

// Display an error code on the LEDS
static void disp_error(BMS_Error_Code code);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // Init GPIO
  HAL_GPIO_WritePin(BMS_GPIO_0_BANK, BMS_GPIO_0_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BMS_GPIO_1_BANK, BMS_GPIO_1_PIN, GPIO_PIN_RESET);

  // LEDs are on during init
  HAL_GPIO_WritePin(BMS_GPIO_LED0_BANK, BMS_GPIO_LED0_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMS_GPIO_LED1_BANK, BMS_GPIO_LED1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMS_GPIO_LED2_BANK, BMS_GPIO_LED2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMS_GPIO_LED3_BANK, BMS_GPIO_LED3_PIN, GPIO_PIN_SET);

  // Get slave address
  bms_i2c_addr = HAL_GPIO_ReadPin(BMS_GPIO_SLOTID_BANK, BMS_GPIO_SLOTID_PIN) | BMS_I2C_SLAVE_ADDR_BASE;

  // TODO: REPLACE BELOW MASTER TRANSMITS TO MEM WRITES

  // Setup the charge controller
  data_buf = (BMS_FAST_SAMPL << 7);
  if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_OUTCTL, 1, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_ERROR)
    disp_error(BMS_ERR_INIT_BQ77);

  data_buf = (BMS_SNSE_CLCUR << 7) | (BMS_SNSE_CLVTG << 6) | (BMS_HOST_HMODE << 1) | (BMS_HOST_SHDWN << 0);
  if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_STATECTL, 1, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_ERROR)
    disp_error(BMS_ERR_INIT_BQ77);

  data_buf = (BMS_USES_THERM << 5);
  if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_FUNCTL, 1, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_ERROR)
    disp_error(BMS_ERR_INIT_BQ77);

  data_buf = (BMS_OV_DLYTM << 5) | (BMS_OV_HYSTR << 3) | (BMS_OV_THRSH << 0);
  if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_OVCFG, 1, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_ERROR)
    disp_error(BMS_ERR_INIT_BQ77);

  data_buf = (BMS_UV_SHDWN << 6) | (BMS_UV_HYSTR << 4) | (BMS_UV_THRSH << 0);
  if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_UVCFG, 1, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_ERROR)
    disp_error(BMS_ERR_INIT_BQ77);

  data_buf = (BMS_OUV_DELAY << 4) | (BMS_OUV_VTRSH << 0);
  if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_OUVCTL, 1, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_ERROR)
    disp_error(BMS_ERR_INIT_BQ77);

  data_buf = (BMS_OCD_BALNC << 7) | (BMS_OCD_PRCRG << 6) | (BMS_OCD_RCRMD << 5) | (BMS_OCD_DELAY << 0);
  if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_OCDCFG, 1, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_ERROR)
    disp_error(BMS_ERR_INIT_BQ77);

  data_buf = (BMS_SCD_DELAY << 4) | (BMS_SCD_SCDVT << 0);
  if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_SCDCFG, 1, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_ERROR)
    disp_error(BMS_ERR_INIT_BQ77);
 
  

  HAL_ADC_Init(&hadc1);
  HAL_ADC_Init(&hadc2);
  
  /* Run the ADC calibration */  
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK || HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
  {
    /* Calibration Error */
    disp_error(BMS_ERR_CALS_ADC);
  }


  // Shut off LEDs
  HAL_GPIO_WritePin(BMS_GPIO_LED0_BANK, BMS_GPIO_LED0_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BMS_GPIO_LED1_BANK, BMS_GPIO_LED1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BMS_GPIO_LED2_BANK, BMS_GPIO_LED2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BMS_GPIO_LED3_BANK, BMS_GPIO_LED3_PIN, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Check if there is any incoming commands from the PMS (blocking)
    if (HAL_I2C_Slave_Receive(&hi2c1, data_buf, 1, BMS_I2C_TIMEOUT) != HAL_OK)
    {
      /* intentionally empty, left for easy error handling implementation in the future */
    }
    else
    {
      switch ((uint8_t)data_buf)  // Only care about byte 0
      {
        case BMS_GET_STATUS:

          if (HAL_I2C_Mem_Read(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_STATUS, I2C_MEMADD_SIZE_8BIT, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_OK)
          {
            data_buf &= ~(BMS_BQ77_STATUS_VGOOD); // Return STATUS reg without EEPROM voltage flag
          }
          else
          {
            data_buf = BMS_ERROR_VALUE;
          }
          
          HAL_I2C_Slave_Transmit(&hi2c1, &data_buf, 1, BMS_I2C_TIMEOUT);
          break;
        case BMS_GET_PACK_VOLTAGE:

          HAL_ADC_Start(&hadc1);

          if (HAL_ADC_PollForConversion(&hadc1, BMS_ADC_TIMEOUT) == HAL_OK)
          {
            hadc_val = HAL_ADC_GetValue(&hadc1); // NOTE: not sure how to tell this which pin to read
          }
          else
          {
            hadc_val = 0;
          }

          HAL_ADC_Stop(&hadc1);

          // SEND BACK hadc_val
          HAL_I2C_Salave_Transmit(&hi2c1, &hadc_val, 4, BMS_I2C_TIMEOUT);

          break;
        case BMS_GET_CURRENT:

            HAL_ADC_Start(&hadc2);

            if (HAL_ADC_PollForConversion(&hadc2, BMS_ADC_TIMEOUT) == HAL_OK)
            {
              hadc_val = HAL_ADC_GetValue(&hadc2); // NOTE: not sure how to tell this which pin to read
            }
            else
            {
              hadc_val = 0;
            }

            HAL_ADC_Stop(&hadc2);

            // SEND BACK hadc_val
            HAL_I2C_Salave_Transmit(&hi2c1, &hadc_val, 4, BMS_I2C_TIMEOUT);

          break; 
        // case BMS_TURN_OUTPUT_ON:   // Only available in HOST control mode

        //   if (BMS_HOST_HMODE)
        //   {
        //     data_buf = (BMS_OC_DSC) | (BMS_FAST_SAMPL << 7);
        //     if (HAL_I2C_Mem_Write(&hi2c2, BMS_BQ77_I2C_ADDR, BMS_BQ77_OUTCTL, I2C_MEMADD_SIZE_8BIT, &data_buf, 1, BMS_I2C_TIMEOUT) == HAL_OK)
        //     {
        //       data_buf = 0;
        //     }
        //     else
        //     {
        //       data_buf = BMS_ERROR_VALUE;
        //     }
        //     HAL_I2C_Slave_Transmit(&hi2c1, &data_buf, 1, BMS_I2C_TIMEOUT);
        //   }
        //   break;
        // case BMS_TURN_OUTPUT_OFF:   // Only available in HOST control mode

        //   if (BMS_HOST_HMODE)
        //   {
        //     // SOMETHING
        //   }
        //   break;
        default:

          break;
      }
    }
    
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief This function displays a given error code on the LEDs
 * @param code The code to be displayed on the LEDs
 * @return None
 */
static void disp_error(BMS_Error_Code code)
{
  __disable_irq();

  HAL_GPIO_WritePin(BMS_GPIO_LED0_BANK, BMS_GPIO_LED0_PIN, code & (1 << 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BMS_GPIO_LED1_BANK, BMS_GPIO_LED1_PIN, code & (1 << 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BMS_GPIO_LED2_BANK, BMS_GPIO_LED2_PIN, code & (1 << 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BMS_GPIO_LED3_BANK, BMS_GPIO_LED3_PIN, code & (1 << 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  while (1)
  {
	/* spin */
  }
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
    disp_error(BMS_ERR_EH);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

