/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "string.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//Del Dron
#define MPU_ADDR         (0x69 << 1)
#define MAG_ADDR         (0x0C << 1)
#define ACCEL_XOUT_H     59
#define MAG_STATUS_1     2
#define MAG_HXL          3
#define MAG_OVERFLOW_BIT 0x08
#define MAG_DATA_READY   0x01
typedef struct {
	float AccXData;
	float AccYData;
	float AccZData;
	float Temp;
	float GyroXData;
	float GyroYData;
	float GyroZData;
	float MagXData;
	float MagYData;
	float MagZData;
} IMU_Data;
typedef struct {
    float x;
    float y;
    float z;
} Offset3D;
static volatile IMU_Data lecturasIMU;
volatile Offset3D mag_offset;
volatile Offset3D mag_scale;
uint32_t medicionesParaPromedio = 100;
uint32_t vectorLecturasMagX = 0;
uint32_t vectorLecturasMagY = 0;
float filtradoLecturasMagX = 0;
float filtradoLecturasMagY = 0;
uint16_t check_flags = 0;
//Fin del Dron


extern volatile int32_t encoder3_count;
extern volatile int32_t encoder4_count;
volatile int32_t enc1 = 0;
volatile int32_t enc2 = 0;
uint8_t pwmValue = 80; //VALOR PWM
uint8_t circunferenciaLlantaCM = 21.4; //Calculado con pi * diametro = pi*68.1mm
#define ENC1_PULSOS_CM 92.3
#define ENC2_PULSOS_CM 92.5
#define ENC3_PULSOS_CM 24.2
#define ENC4_PULSOS_CM 24.9
int32_t enc1_ini = 0;
int32_t enc2_ini = 0;
int32_t enc3_ini = 0;
int32_t enc4_ini = 0;

int32_t avance1 = 0;
int32_t avance2 = 0;
int32_t avance3 = 0;
int32_t avance4 = 0;

int32_t objetivo1 = 0;
int32_t objetivo2 = 0;
int32_t objetivo3 = 0;
int32_t objetivo4 = 0;


uint8_t data = 0b00001100;
char msg[17] = "";
char num[17] = "";
float distancia_cm = 200.0; // VALOR PARA DISTANCIA DESEADA EN CM
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Init_IMU(void);
IMU_Data GetData__stMPU_9255(void);
Offset3D ComputeMagOffset(uint16_t num_samples, uint16_t delay_ms);
IMU_Data NormalizeMag(IMU_Data raw, Offset3D offset, Offset3D scale);
Offset3D ComputeMagScale(uint16_t num_samples, uint16_t delay_ms, Offset3D offset);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int32_t encoder3_count = 0;
volatile int32_t encoder4_count = 0;


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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //Dron Innit
  Init_IMU();
  HAL_Delay(100);
  mag_offset = ComputeMagOffset(500, 10);
  mag_scale = ComputeMagScale(500, 10, mag_offset);
  HAL_Delay(100);
  //Dron fin Innit

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
  // Leer posición inicial
  enc1_ini = __HAL_TIM_GET_COUNTER(&htim2); // Encoder 1
  enc2_ini = __HAL_TIM_GET_COUNTER(&htim3); // Encoder 2
  enc3_ini = encoder3_count; // Encoder 3 por interrupción
  enc4_ini = encoder4_count; // Encoder 4 por interrupción

  objetivo1 = distancia_cm * ENC1_PULSOS_CM;  //Calcular pulsos
  objetivo2 = distancia_cm * ENC2_PULSOS_CM;
  objetivo3 = distancia_cm * ENC3_PULSOS_CM;
  objetivo4 = distancia_cm * ENC4_PULSOS_CM;


  //ELIMINARRRRR DESPUESSSS LO DE ABAJO DE WRITEPINES

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    enc1 = __HAL_TIM_GET_COUNTER(&htim2);
    enc2 = __HAL_TIM_GET_COUNTER(&htim3);

    avance1 = abs(enc1 - enc1_ini);
    avance2 = abs(enc2 - enc2_ini);
    avance3 = abs(encoder3_count - enc3_ini);
    avance4 = abs(encoder4_count - enc4_ini);

    if (encoder4_count < objetivo4){
    	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwmValue);
    }
    else{
    	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
    }
    if (encoder3_count < objetivo3){
    	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwmValue);
    }
    else{
    	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
    }
    if (enc1 < objetivo1){
    	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwmValue);
    }
    else{
    	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
    }
    if (enc2 < objetivo2){
      	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwmValue);
      }
      else{
      	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
      }


    HAL_Delay(10);

    //Medimos Datos
	for (uint8_t i = 0; i < medicionesParaPromedio ;i++){
		lecturasIMU = GetData__stMPU_9255();

		lecturasIMU = NormalizeMag(GetData__stMPU_9255(), mag_offset, mag_scale);
		vectorLecturasMagX += lecturasIMU.MagXData;
		vectorLecturasMagY += lecturasIMU.MagYData;
		HAL_Delay(50);
	}
	vectorLecturasMagX = vectorLecturasMagX / medicionesParaPromedio;
	vectorLecturasMagY = vectorLecturasMagY / medicionesParaPromedio;
	HAL_Delay(50);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pins : PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void Init_IMU(void) {
	uint8_t u8destVal = 0;
	uint8_t u8sensMagVal[3] = {0};
	uint8_t data_tx[2];
	uint8_t reg;
	uint8_t dev_addr = 0x69 << 1;
	uint8_t mag_addr = 0x0C << 1;

	// 0. Reset MPU
	data_tx[0] = 0x6B; data_tx[1] = 0x80;
	if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, HAL_MAX_DELAY) == HAL_OK) {
		HAL_Delay(100);
		check_flags |= (1 << 0);
	}

	// 1. Clock source
	data_tx[0] = 0x6B; data_tx[1] = 0x01;
	if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, HAL_MAX_DELAY) == HAL_OK) {
		check_flags |= (1 << 1);
	}

	// 2. Gyro config
	data_tx[0] = 0x1B; data_tx[1] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, HAL_MAX_DELAY) == HAL_OK) {
		check_flags |= (1 << 2);
	}

	// 3. Accel config
	data_tx[0] = 0x1C; data_tx[1] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, HAL_MAX_DELAY) == HAL_OK) {
		check_flags |= (1 << 3);
	}

	// 4. Enable I2C bypass
	data_tx[0] = 0x37; data_tx[1] = 0x02;
	if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, HAL_MAX_DELAY) == HAL_OK) {
		HAL_Delay(10);
		check_flags |= (1 << 4);
	}

	// 5. Power down magnetometer
	data_tx[0] = 0x0A; data_tx[1] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, data_tx, 2, HAL_MAX_DELAY) == HAL_OK) {
		HAL_Delay(10);
		check_flags |= (1 << 5);
	}

	// 6. Enter fuse ROM mode
	data_tx[0] = 0x0A; data_tx[1] = 0x0F;
	if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, data_tx, 2, HAL_MAX_DELAY) == HAL_OK) {
		HAL_Delay(10);
		check_flags |= (1 << 6);
	}

	// 7. Read ASA registers
	reg = 0x10;
	if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, &reg, 1, HAL_MAX_DELAY) == HAL_OK &&
		HAL_I2C_Master_Receive(&hi2c2, mag_addr, u8sensMagVal, 3, HAL_MAX_DELAY) == HAL_OK) {
		check_flags |= (1 << 7);
	}

	// 8. Power down again
	data_tx[0] = 0x0A; data_tx[1] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, data_tx, 2, HAL_MAX_DELAY) == HAL_OK) {
		HAL_Delay(10);
		check_flags |= (1 << 8);
	}

	// 9. Set to continuous mode 2 (retry if needed)
	uint8_t mode_ok = 0;
	for (int retry = 0; retry < 10; retry++) {
		data_tx[0] = 0x0A; data_tx[1] = 0x16;
		HAL_I2C_Master_Transmit(&hi2c2, mag_addr, data_tx, 2, HAL_MAX_DELAY);
		HAL_Delay(10);
		reg = 0x0A;
		HAL_I2C_Master_Transmit(&hi2c2, mag_addr, &reg, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c2, mag_addr, &u8destVal, 1, HAL_MAX_DELAY);
		if (u8destVal == 0x16) {
			mode_ok = 1;
			break;
		}
	}
	if (mode_ok) check_flags |= (1 << 9);

	// 10. Confirm ST1 register responds
	reg = 0x02;
	if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, &reg, 1, HAL_MAX_DELAY) == HAL_OK &&
		HAL_I2C_Master_Receive(&hi2c2, mag_addr, &u8destVal, 1, HAL_MAX_DELAY) == HAL_OK) {
		check_flags |= (1 << 10);
	}
}

IMU_Data GetData__stMPU_9255(void) {
	uint8_t imuData[14] = {0};
	uint8_t magData[7] = {0};
	uint8_t status1 = 0;
	uint8_t reg;
	int16_t raw[10] = {0};
	IMU_Data result;

	// Leer 14 bytes: accel, temp, gyro
	reg = ACCEL_XOUT_H;
	HAL_I2C_Master_Transmit(&hi2c2, MPU_ADDR, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c2, MPU_ADDR, imuData, 14, HAL_MAX_DELAY);

	raw[0] = (imuData[0] << 8) | imuData[1];  // AccX
	raw[1] = (imuData[2] << 8) | imuData[3];  // AccY
	raw[2] = (imuData[4] << 8) | imuData[5];  // AccZ
	raw[3] = (imuData[6] << 8) | imuData[7];  // Temp
	raw[4] = (imuData[8] << 8) | imuData[9];  // GyroX
	raw[5] = (imuData[10] << 8) | imuData[11]; // GyroY
	raw[6] = (imuData[12] << 8) | imuData[13]; // GyroZ

	HAL_Delay(10);
	// Verifica si hay datos magnéticos listos
	reg = MAG_STATUS_1;
	HAL_I2C_Master_Transmit(&hi2c2, MAG_ADDR, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c2, MAG_ADDR, &status1, 1, HAL_MAX_DELAY);

	if ((status1 & MAG_DATA_READY) == MAG_DATA_READY) {
		// Leer datos magnéticos (6 + 1 bytes: HOFL)
		reg = MAG_HXL;
		HAL_I2C_Master_Transmit(&hi2c2, MAG_ADDR, &reg, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c2, MAG_ADDR, magData, 7, HAL_MAX_DELAY);

		// Si no hay overflow
		if (!(magData[6] & MAG_OVERFLOW_BIT)) {
			raw[7] = (magData[1] << 8) | magData[0]; // MagX
			raw[8] = (magData[3] << 8) | magData[2]; // MagY
			raw[9] = (magData[5] << 8) | magData[4]; // MagZ
		}
	}

	// Copiar a estructura
	result.AccXData  = (float)raw[0];
	result.AccYData  = (float)raw[1];
	result.AccZData  = (float)raw[2];
	result.Temp      = (float)raw[3];
	result.GyroXData = (float)raw[4];
	result.GyroYData = (float)raw[5];
	result.GyroZData = (float)raw[6];
	result.MagXData  = (float)raw[7];
	result.MagYData  = (float)raw[8];
	result.MagZData  = (float)raw[9];

	return result;
}

Offset3D ComputeMagOffset(uint16_t num_samples, uint16_t delay_ms) {
    float minX = 32767, maxX = -32768;
    float minY = 32767, maxY = -32768;
    float minZ = 32767, maxZ = -32768;

    for (uint16_t i = 0; i < num_samples; i++) {
        IMU_Data reading = GetData__stMPU_9255();
        if (reading.MagXData < minX) minX = reading.MagXData;
        if (reading.MagXData > maxX) maxX = reading.MagXData;
        if (reading.MagYData < minY) minY = reading.MagYData;
        if (reading.MagYData > maxY) maxY = reading.MagYData;
        if (reading.MagZData < minZ) minZ = reading.MagZData;
        if (reading.MagZData > maxZ) maxZ = reading.MagZData;
        HAL_Delay(delay_ms);
    }
    Offset3D offset;
    offset.x = (maxX + minX) / 2.0f;
    offset.y = (maxY + minY) / 2.0f;
    offset.z = (maxZ + minZ) / 2.0f;
    return offset;
}

IMU_Data NormalizeMag(IMU_Data raw, Offset3D offset, Offset3D scale) {
    raw.MagXData = (raw.MagXData - offset.x) * scale.x;
    raw.MagYData = (raw.MagYData - offset.y) * scale.y;
    raw.MagZData = (raw.MagZData - offset.z) * scale.z;
    return raw;
}

Offset3D ComputeMagScale(uint16_t num_samples, uint16_t delay_ms, Offset3D offset) {
    float minX = 32767, maxX = -32768;
    float minY = 32767, maxY = -32768;
    float minZ = 32767, maxZ = -32768;

    for (uint16_t i = 0; i < num_samples; i++) {
        IMU_Data reading = GetData__stMPU_9255();
        float x = reading.MagXData - offset.x;
        float y = reading.MagYData - offset.y;
        float z = reading.MagZData - offset.z;

        if (x < minX) { minX = x; }
        if (x > maxX) { maxX = x; }

        if (y < minY) { minY = y; }
        if (y > maxY) { maxY = y; }

        if (z < minZ) { minZ = z; }
        if (z > maxZ) { maxZ = z; }


        HAL_Delay(delay_ms);
    }

    Offset3D scale;
    scale.x = 200.0f / (maxX - minX);
    scale.y = 200.0f / (maxY - minY);
    scale.z = 200.0f / (maxZ - minZ);
    return scale;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
