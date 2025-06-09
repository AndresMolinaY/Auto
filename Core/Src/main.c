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
#include <math.h>
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
//IMU
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
    float offsetX;
    float offsetY;
    float offsetZ;
    float scaleX;
    float scaleY;
    float scaleZ;
} MagCalibration;

typedef struct {
    float offsetZ;
    float scaleZ;
} GyroCalibration;

//IMU
static volatile IMU_Data lecturasIMU;
uint16_t check_flags = 0;

volatile uint8_t mpu_data_ready;
static volatile float dt = 0.1f;

//Magnetometer
MagCalibration magCal;
uint8_t mad_calibrated = 0;
static volatile float heading;

//Girsocopio
static volatile float angle;

char msg[64];

//Giro
float heading_target = 100.0f;
uint8_t giro_completado = 0;

float kp = 1.5f, ki = 0.0f, kd = 0.0f;
float integral = 0.0f, prev_error = 0.0f;

//Fin Dron


extern volatile int32_t encoder3_count;
extern volatile int32_t encoder4_count;
volatile int32_t enc1 = 0;
volatile int32_t enc2 = 0;
uint8_t pwmValue = 80; //VALOR PWM
//uint8_t circunferenciaLlantaCM = 21.5; //Calculado con pi * diametro = pi*68.1mm
#define PULSOS_POR_VUELTA_ENC1 1968.75
#define PULSOS_POR_VUELTA_ENC2 1977
#define PULSOS_POR_VUELTA_ENC3 (492.63)
#define PULSOS_POR_VUELTA_ENC4 (512.25)
#define CIRCUNFERENCIA_LLANTA_CM 21.5  // o la que midas con regla

int32_t enc1_ini = 0;
int32_t enc2_ini = 0;
int32_t enc3_ini = 0;
int32_t enc4_ini = 0;

float avance1 = 0;
float avance2 = 0;
float avance3 = 0;
float avance4 = 0;

int32_t objetivo1 = 0;
int32_t objetivo2 = 0;
int32_t objetivo3 = 0;
int32_t objetivo4 = 0;


uint8_t data = 0b00001100;
char num[17] = "";
float distancia_cm = 100;//ALOR1bn DISTANCIA DESEADA EN CM   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
uint8_t paso_actual = 0;
uint8_t total_pasos = 4;
uint8_t en_giro = 0;
uint32_t tiempo_inicio_giro = 0;
float angulo_inicio_giro = 0.0f;
float angulo_giro_actual = 0.0f;
float angulo_objetivo_giro =65.0f;  // Puedes cambiarlo a 45.0f, 180.0f, etc.
float heading_inicial = 0;
int8_t sentido_giro = 1; // 1 = izquierda, -1 = derecha

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
void actualizarAvances(void);
float calcularPromedioTrasero(void);
void moverMotores(uint8_t pwm);
void detenerMotores(void);
void enviarDistanciaPromedioUART(float promedio);
	//IMU
void Init_IMU(void);
IMU_Data GetData__stMPU_9255(void);
MagCalibration calibrate_magnetometer(uint16_t samples);
GyroCalibration calibrate_gyroZ(uint16_t samples);
IMU_Data ReadIMU_Average(uint8_t samples);
//Fin IMU
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

  //IMU
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
  HAL_Delay(100);

  Init_IMU();
  HAL_Delay(100);
  GyroCalibration gyroCal = calibrate_gyroZ(500);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);

  magCal = calibrate_magnetometer(500);

  //Calcular Heading Inicial
  lecturasIMU = GetData__stMPU_9255();
  float mx = (lecturasIMU.MagXData - magCal.offsetX) / magCal.scaleX;
  float my = (lecturasIMU.MagYData - magCal.offsetY) / magCal.scaleY;
  float norm = sqrtf(mx * mx + my * my);
  mx /= norm;
  my /= norm;

  float heading_inicial = atan2f(my, mx) * (180.0f / M_PI);
  if (heading_inicial < 0) heading_inicial += 360.0f;

  // Objetivo: 90 grados más
  heading_target = heading_inicial + 90.0f;
  if (heading_target >= 360.0f) heading_target -= 360.0f;
  //fin imu

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

  objetivo1 = distancia_cm;  //Calcular pulsos
  objetivo2 = distancia_cm;
  objetivo3 = distancia_cm;
  objetivo4 = distancia_cm;


  //ELIMINARRRRR DESPUESSSS LO DE ABAJO DE WRITEPINES

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

enc1_ini = __HAL_TIM_GET_COUNTER(&htim2); // Usa TIM2 o el que te interese

while (1)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    lecturasIMU = ReadIMU_Average(10);  // ← 10 lecturas → dt = 0.1 s

    // GIROSCOPIO Y MAGNETÓMETRO
    float gz = (lecturasIMU.GyroZData - gyroCal.offsetZ) / gyroCal.scaleZ;
    gz *= 0.00763f; // Convierte a grados por segundo

    float mx = (lecturasIMU.MagXData - magCal.offsetX) / magCal.scaleX;
    float my = (lecturasIMU.MagYData - magCal.offsetY) / magCal.scaleY;
    float mz = (lecturasIMU.MagZData - magCal.offsetZ) / magCal.scaleZ;

    float norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm != 0.0f) {
        mx /= norm;
        my /= norm;
        mz /= norm;
    }

    // Cálculo de heading solo informativo (ya no se usa para el giro)
    heading = atan2f(my, mx) * (180.0f / M_PI);
    if (heading < 0) heading += 360.0f;

    actualizarAvances();
    float promedio_cm = calcularPromedioTrasero();

    if (!en_giro) {
        if (promedio_cm < distancia_cm) {
            moverMotores(pwmValue);
        } else {
            detenerMotores();
            en_giro = 1;
            // Fijar dirección de giro manualmente por paso
            if (paso_actual == 0 || paso_actual == 1 || paso_actual == 2 || paso_actual == 3) {
                sentido_giro = 1; // izquierda
            } else {
                sentido_giro = -1; // derecha si algún día quisieras probar
            }

            // Reiniciar ángulo
            angulo_giro_actual = 0.0f;
            tiempo_inicio_giro = HAL_GetTick();


            // Reiniciar integración del giro
            angulo_giro_actual = 0.0f;

            tiempo_inicio_giro = HAL_GetTick();
        }
    } else {
        // 1. Integrar ángulo girado con giroscopio
        angulo_giro_actual += gz * dt;

        // 2. Calcular cuánto falta para completar los 90°
        float error_giro = angulo_objetivo_giro - fabs(angulo_giro_actual);

        if (fabs(error_giro) > 3.0f) {
            // Gira en el sentido adecuado
        	if (sentido_giro == 1) {
        	    // Gira izquierda
        	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwmValue);
        	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwmValue);
        	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        	} else {
        	    // Gira derecha
        	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwmValue);
        	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwmValue);
        	}

        } else {
            // Giro completado
            detenerMotores();

            // Esperar 10 segundos
            if (HAL_GetTick() - tiempo_inicio_giro >= 10000) {
                paso_actual++;
                if (paso_actual >= total_pasos) {
                    // Terminó el cuadrado
                    while (1); // Se queda detenido
                }

                // Reiniciar variables de avance
                enc3_ini = encoder3_count;
                enc4_ini = encoder4_count;
                avance3 = 0;
                avance4 = 0;
                en_giro = 0;
            }
        }
    }

    HAL_Delay(10); // para mantener tiempo de ciclo estable
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
  huart1.Init.BaudRate = 9600;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void actualizarAvances(void) {
    enc1 = __HAL_TIM_GET_COUNTER(&htim2);
    enc2 = __HAL_TIM_GET_COUNTER(&htim3);

    avance1 = (float)(abs(enc1 - enc1_ini)) / PULSOS_POR_VUELTA_ENC1 * CIRCUNFERENCIA_LLANTA_CM;
    avance2 = (float)(abs(enc2 - enc2_ini)) / PULSOS_POR_VUELTA_ENC2 * CIRCUNFERENCIA_LLANTA_CM;
    avance3 = (float)(abs(encoder3_count - enc3_ini)) / PULSOS_POR_VUELTA_ENC3 * CIRCUNFERENCIA_LLANTA_CM;
    avance4 = (float)(abs(encoder4_count - enc4_ini)) / PULSOS_POR_VUELTA_ENC4 * CIRCUNFERENCIA_LLANTA_CM;
}


float calcularPromedioTrasero(void) {
    return (avance3 + avance4) / 2.0f;
}


void moverMotores(uint8_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);         // Delantera derecha
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm + 3.17);  // Delantera izquierda
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);         // Trasera derecha
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm + 3.17);  // Trasera izquierda
}

void detenerMotores(void) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

void Init_IMU(void) {
    uint8_t data_tx[2], reg, val;
    uint8_t asa[3] = {0};
    uint8_t dev_addr = MPU_ADDR;    // 0x69 << 1
    uint8_t mag_addr = MAG_ADDR;    // 0x0C << 1

    // 0. Reset MPU
    data_tx[0] = 0x6B; data_tx[1] = 0x80;
    if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, 100) == HAL_OK) {
        HAL_Delay(100);
        check_flags |= (1 << 0);
    }

    // 1. Clock source = PLL with X axis gyroscope
    data_tx[0] = 0x6B; data_tx[1] = 0x01;
    if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, 100) == HAL_OK)
        check_flags |= (1 << 1);

    // 2. Gyroscope config = ±250°/s
    data_tx[0] = 0x1B; data_tx[1] = 0x00;
    if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, 100) == HAL_OK)
        check_flags |= (1 << 2);

    // 3. Accelerometer config = ±2g
    data_tx[0] = 0x1C; data_tx[1] = 0x00;
    if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, 100) == HAL_OK)
        check_flags |= (1 << 3);

    // 4. Enable I2C bypass mode (talk directly to magnetometer)
    data_tx[0] = 0x37; data_tx[1] = 0x02;
    if (HAL_I2C_Master_Transmit(&hi2c2, dev_addr, data_tx, 2, 100) == HAL_OK) {
        HAL_Delay(10);
        check_flags |= (1 << 4);
    }

    // 5. Power down magnetometer
    data_tx[0] = 0x0A; data_tx[1] = 0x00;
    if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, data_tx, 2, 100) == HAL_OK) {
        HAL_Delay(10);
        check_flags |= (1 << 5);
    }

    // 6. Enter fuse ROM access mode
    data_tx[0] = 0x0A; data_tx[1] = 0x0F;
    if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, data_tx, 2, 100) == HAL_OK) {
        HAL_Delay(10);
        check_flags |= (1 << 6);
    }

    // 7. Read sensitivity adjustment values (ASA)
    reg = 0x10;
    if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, &reg, 1, 100) == HAL_OK &&
        HAL_I2C_Master_Receive(&hi2c2, mag_addr, asa, 3, 100) == HAL_OK) {
        check_flags |= (1 << 7);
    }

    // 8. Power down again
    data_tx[0] = 0x0A; data_tx[1] = 0x00;
    if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, data_tx, 2, 100) == HAL_OK) {
        HAL_Delay(10);
        check_flags |= (1 << 8);
    }

    // 9. Set magnetometer to continuous mode 2 (100Hz, 16-bit)
    uint8_t mode_ok = 0;
    for (int i = 0; i < 5; i++) {
        data_tx[0] = 0x0A; data_tx[1] = 0x16;
        HAL_I2C_Master_Transmit(&hi2c2, mag_addr, data_tx, 2, 100);
        HAL_Delay(10);
        reg = 0x0A;
        HAL_I2C_Master_Transmit(&hi2c2, mag_addr, &reg, 1, 100);
        HAL_I2C_Master_Receive(&hi2c2, mag_addr, &val, 1, 100);
        if (val == 0x16) {
            mode_ok = 1;
            break;
        }
    }
    if (mode_ok)
        check_flags |= (1 << 9);

    // 10. Confirm ST1 register responds
    reg = 0x02;
    if (HAL_I2C_Master_Transmit(&hi2c2, mag_addr, &reg, 1, 100) == HAL_OK &&
        HAL_I2C_Master_Receive(&hi2c2, mag_addr, &val, 1, 100) == HAL_OK) {
        check_flags |= (1 << 10);
    }

    // 11. Enable RAW_RDY_EN interrupt (enable data ready interrupt)
    val = 0x01;  // Bit 0 = RAW_RDY_EN
    HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x38, 1, &val, 1, 100);

    // 12. Configure INT pin: active LOW, open-drain, pulse mode (not latch)

    val = 0x12; // 0x10 (active low) + 0x02 (bypass enabled)
    HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x37, 1, &val, 1, 100);


    // 13. CONFIG (DLPF) = 3 (Accel/Gyro ~44Hz BW, 1kHz sample)
    data_tx[0] = 0x1A; data_tx[1] = 0x03;
    HAL_I2C_Master_Transmit(&hi2c2, MPU_ADDR, data_tx, 2, 100);

    // 14. SMPLRT_DIV = 9 → 1kHz / (1+9) = 100Hz output rate
    data_tx[0] = 0x19; data_tx[1] = 9;
    HAL_I2C_Master_Transmit(&hi2c2, MPU_ADDR, data_tx, 2, 100);
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


MagCalibration calibrate_magnetometer(uint16_t samples) {
    float minX =  32767, minY =  32767, minZ =  32767;
    float maxX = -32768, maxY = -32768, maxZ = -32768;

    for (uint16_t i = 0; i < samples; i++) {
        IMU_Data lectura = GetData__stMPU_9255();

        if (lectura.MagXData < minX) minX = lectura.MagXData;
        if (lectura.MagYData < minY) minY = lectura.MagYData;
        if (lectura.MagZData < minZ) minZ = lectura.MagZData;

        if (lectura.MagXData > maxX) maxX = lectura.MagXData;
        if (lectura.MagYData > maxY) maxY = lectura.MagYData;
        if (lectura.MagZData > maxZ) maxZ = lectura.MagZData;

        HAL_Delay(10);
    }

    MagCalibration result;
    result.offsetX = (maxX + minX) / 2.0f;
    result.offsetY = (maxY + minY) / 2.0f;
    result.offsetZ = (maxZ + minZ) / 2.0f;

    result.scaleX = (maxX - minX) / 2.0f;
    result.scaleY = (maxY - minY) / 2.0f;
    result.scaleZ = (maxZ - minZ) / 2.0f;

    return result;
}

GyroCalibration calibrate_gyroZ(uint16_t samples) {
    float minZ =  32767;
    float maxZ = -32768;
    float sumZ = 0;

    for (uint16_t i = 0; i < samples; i++) {
        IMU_Data lectura = GetData__stMPU_9255();
        float gyroZ = lectura.GyroZData;

        if (gyroZ < minZ) minZ = gyroZ;
        if (gyroZ > maxZ) maxZ = gyroZ;

        sumZ += gyroZ;
        HAL_Delay(2);
    }

    GyroCalibration result;
    result.offsetZ = sumZ / samples;

    //Scaling removed chat said so
    //result.scaleZ = (maxZ - minZ) / 2.0f;
    result.scaleZ = 1.0f;  // No scaling

    return result;
}



IMU_Data ReadIMU_Average(uint8_t samples) {
    IMU_Data avg = {0};

    for (uint8_t i = 0; i < samples; ) {
        if (mpu_data_ready) {
            mpu_data_ready = 0;

            IMU_Data lectura = GetData__stMPU_9255();
            avg.AccXData += lectura.AccXData;
            avg.AccYData += lectura.AccYData;
            avg.AccZData += lectura.AccZData;
            avg.GyroXData += lectura.GyroXData;
            avg.GyroYData += lectura.GyroYData;
            avg.GyroZData += lectura.GyroZData;
            avg.MagXData  += lectura.MagXData;
            avg.MagYData  += lectura.MagYData;
            avg.MagZData  += lectura.MagZData;
            i++;
        }
    }

    avg.AccXData /= samples;
    avg.AccYData /= samples;
    avg.AccZData /= samples;
    avg.GyroXData /= samples;
    avg.GyroYData /= samples;
    avg.GyroZData /= samples;
    avg.MagXData  /= samples;
    avg.MagYData  /= samples;
    avg.MagZData  /= samples;

    return avg;
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
