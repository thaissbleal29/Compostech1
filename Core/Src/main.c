/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NTC_BETA    3950.0f
#define NTC_R0      10000.0f
#define NTC_T0      298.15f
#define NTC_RREF    10000.0f
#define ADC_MAX     4095.0f

// ── Limites de acionamento ──────────────────────────────────
#define TEMP_ALTA    55.0f   // °C — aciona fan + motor
#define UMID_ALTA    70.0f   // %  — aciona somente fan
#define UMID_BAIXA   30.0f   // %  — aciona somente bomba

// ── Ciclo automático do motor ───────────────────────────────
#define INTERVALO_CICLO  30000UL  // ms — tempo entre ciclos
#define DURACAO_CICLO     8000UL  // ms — duração de cada ciclo

// ── Servo ───────────────────────────────────────────────────
#define SERVO_REPOUSO    1500     // µs — 90° (parado)
#define SERVO_ESQUERDA   1000     // µs — 0°
#define SERVO_DIREITA    2000     // µs — 180°

// ── Debounce do botão ───────────────────────────────────────
#define DEBOUNCE_MS      50       // ms — tempo mínimo entre leituras do botão

// ── Log ─────────────────────────────────────────────────────
#define LOG_MS           2000UL   // ms — intervalo de log UART

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
float     lerTemperatura(void);
float     lerUmidade(void);
uint32_t  ADC_LerCanal(uint32_t canal);
void      Servo_SetAngle(uint32_t pulso_us);
void      Fan_On(void);
void      Fan_Off(void);
void      Bomba_On(void);
void      Bomba_Off(void);
void      Motor_Ligar(void);
void      Motor_Desligar(void);
void      UART_Log(float temp, float umidade,
                   uint8_t motorAtivo, uint8_t fanAtiva,
                   uint8_t bombaAtiva, uint8_t modoManual);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ─────────────────────────────────────────────────────────────
//  ADC — lê um canal específico (alterna PA0 e PA1)
// ─────────────────────────────────────────────────────────────
uint32_t ADC_LerCanal(uint32_t canal) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = canal;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

// ─────────────────────────────────────────────────────────────
//  TEMPERATURA — NTC via Steinhart-Hart
//  PA1 → ADC_CHANNEL_1
//  Temp alta = R_ntc baixo = raw baixo
// ─────────────────────────────────────────────────────────────
float lerTemperatura(void) {
    uint32_t raw = ADC_LerCanal(ADC_CHANNEL_1);
    if (raw == 0)    return 125.0f;
    if (raw >= 4095) return -55.0f;
    float rNTC      = NTC_RREF * ((float)raw / (ADC_MAX - (float)raw));
    float steinhart = logf(rNTC / NTC_R0) / NTC_BETA + 1.0f / NTC_T0;
    return (1.0f / steinhart) - 273.15f;
}

// ─────────────────────────────────────────────────────────────
//  UMIDADE — sensor capacitivo de solo
//  PA0 → ADC_CHANNEL_0
//  Seco = raw alto (~3800), Úmido = raw baixo (~1200)
// ─────────────────────────────────────────────────────────────
float lerUmidade(void) {
    uint32_t raw = ADC_LerCanal(ADC_CHANNEL_0);
    float pct = ((float)(3800 - raw) / (3800 - 1200)) * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

// ─────────────────────────────────────────────────────────────
//  SERVO — PA8 via TIM1_CH1
// ─────────────────────────────────────────────────────────────
void Servo_SetAngle(uint32_t pulso_us) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulso_us);
}

// Oscila o servo alternando entre esquerda e direita a cada 1 s
// Chamar dentro do loop — usa agora para decidir a fase
void Motor_Ligar(void) {
    uint32_t agora = HAL_GetTick();
    if ((agora / 1000) % 2 == 0)
        Servo_SetAngle(SERVO_ESQUERDA);
    else
        Servo_SetAngle(SERVO_DIREITA);
}

void Motor_Desligar(void) {
    Servo_SetAngle(SERVO_REPOUSO);
}

// ─────────────────────────────────────────────────────────────
//  FAN — PB12
// ─────────────────────────────────────────────────────────────
void Fan_On(void)  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   }
void Fan_Off(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); }

// ─────────────────────────────────────────────────────────────
//  BOMBA — PB1
// ─────────────────────────────────────────────────────────────
void Bomba_On(void)  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   }
void Bomba_Off(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); }

// ─────────────────────────────────────────────────────────────
//  LOG UART
// ─────────────────────────────────────────────────────────────
void UART_Log(float temp, float umidade,
              uint8_t motorAtivo, uint8_t fanAtiva,
              uint8_t bombaAtiva, uint8_t modoManual) {
    char msg[160];
    int len = snprintf(msg, sizeof(msg), // @suppress("Float formatting support")
        "Temp: %.1fC | Umid: %.1f%% | Motor: %s%s | Fan: %s | Bomba: %s\r\n",
        temp, umidade,
        motorAtivo ? "ON" : "off",
        modoManual ? " (MANUAL)" : "",
        fanAtiva   ? "ON" : "off",
        bombaAtiva ? "ON" : "off");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
}
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   Motor_Desligar();
   Fan_Off();
   Bomba_Off();

   // ── Variáveis de controle ──────────────────────────────
   uint8_t  cicloAutoAtivo  = 0;      // ciclo automático do motor em andamento
   uint8_t  modoManual      = 0;      // botão pressionado = motor travado ligado
   uint8_t  motorAtivo      = 0;      // estado final do motor (auto OU manual)
   uint8_t  fanAtiva        = 0;
   uint8_t  bombaAtiva      = 0;

   // ── Temporização ──────────────────────────────────────
   uint32_t tInicioCiclo    = 0;
   uint32_t tFimCiclo       = 0;      // quando o último ciclo terminou
   uint32_t tUltimoLog      = 0;

   // ── Botão ─────────────────────────────────────────────
   uint8_t  botaoAnterior   = GPIO_PIN_SET;   // pull-up: solto = HIGH
   uint32_t tUltimoDebounce = 0;

   char boot[] = "=== COMPOSTEIRA INTELIGENTE ===\r\n"
                 "  Temp >= 55C        -> Fan + Motor\r\n"
                 "  Umidade >= 70%     -> Fan\r\n"
                 "  Umidade <= 30%     -> Bomba\r\n"
                 "  Motor automatico: 8s a cada 30s\r\n"
                 "  Botao PB3: toggle manual do motor\r\n\r\n";
   HAL_UART_Transmit(&huart2, (uint8_t*)boot, strlen(boot), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t agora   = HAL_GetTick();
		    float    temp    = lerTemperatura();
		    float    umidade = lerUmidade();

		    // ── Condições dos sensores ─────────────────────────
		    uint8_t tempAlta  = (temp    >= TEMP_ALTA);
		    uint8_t umidAlta  = (umidade >= UMID_ALTA);
		    uint8_t umidBaixa = (umidade <= UMID_BAIXA);

		    // ── Botão — toggle com debounce ────────────────────
		    // Botão com pull-up: solto = HIGH, pressionado = LOW
		    // Detecta a borda de descida (momento do clique)
		    uint8_t botaoAtual = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
		    if (botaoAtual == GPIO_PIN_RESET &&
		        botaoAnterior == GPIO_PIN_SET &&
		        (agora - tUltimoDebounce) > DEBOUNCE_MS) {

		        modoManual = !modoManual;   // inverte o estado a cada clique
		        tUltimoDebounce = agora;

		        char msg[64];
		        int len = snprintf(msg, sizeof(msg),
		            "[BOTAO] Modo manual: %s\r\n", modoManual ? "LIGADO" : "desligado");
		        HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
		    }
		    botaoAnterior = botaoAtual;

		    // ── Ciclo automático do motor ──────────────────────
		    // Roda de forma periódica independente dos sensores
		    if (cicloAutoAtivo) {
		        if (agora - tInicioCiclo >= DURACAO_CICLO) {
		            cicloAutoAtivo = 0;
		            tFimCiclo      = agora;
		            char msg[] = "[CICLO] Encerrado.\r\n";
		            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		        }
		    } else {
		        if (agora - tFimCiclo >= INTERVALO_CICLO) {
		            cicloAutoAtivo = 1;
		            tInicioCiclo   = agora;
		            char msg[] = "[CICLO] Iniciando ciclo automatico de mistura.\r\n";
		            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		        }
		    }

		    // ── Estado final do motor ──────────────────────────
		    // Liga se: ciclo automático ativo  OU  temperatura alta  OU  modo manual
		    motorAtivo = cicloAutoAtivo || tempAlta || modoManual;

		    // ── Estado final da fan ────────────────────────────
		    // Liga se: temperatura alta  OU  umidade alta
		    fanAtiva = tempAlta || umidAlta;

		    // ── Estado final da bomba ──────────────────────────
		    // Liga somente se umidade baixa
		    bombaAtiva = umidBaixa;

		    // ── Aplica os estados no hardware ─────────────────
		    if (motorAtivo) Motor_Ligar();
		    else            Motor_Desligar();

		    if (fanAtiva)   Fan_On();
		    else            Fan_Off();

		    if (bombaAtiva) Bomba_On();
		    else            Bomba_Off();

		    // ── Log periódico ──────────────────────────────────
		    if (agora - tUltimoLog >= LOG_MS) {
		        tUltimoLog = agora;
		        UART_Log(temp, umidade, motorAtivo, fanAtiva, bombaAtiva, modoManual);
		    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
