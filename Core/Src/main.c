/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Composteira Inteligente com Interrupção de Hardware (EXTI)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "dht22.h"
#include "capacitive.h"

/* Private define ------------------------------------------------------------*/
#define TEMP_ALTA        55.0f
#define UMID_AR_ALTA     70.0f
#define UMID_TERRA_BAIXA 30.0f

#define INTERVALO_CICLO  30000UL
#define DURACAO_CICLO     8000UL

#define SERVO_REPOUSO    1500
#define SERVO_ESQUERDA   1000
#define SERVO_DIREITA    2000

#define LOG_MS           2000UL

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

DHT_sensor meuDHT22;
DHT_data dadosDHT22;
CSMV2_sensor meuSensorSolo;
CSMV2_data dadosSolo;

float    dht_temp       = 0.0f;
float    dht_umid       = 0.0f;
float    umid_solo      = 0.0f;

uint8_t  cicloAutoAtivo = 0;
uint8_t  motorAtivo     = 0;
uint8_t  fanAtiva       = 0;
uint8_t  bombaAtiva     = 0;

// VARIABLES VOLATILES: Modificadas dentro da ISR (Interrupção)
volatile uint8_t  modoManual      = 0;
volatile uint32_t tUltimoDebounce = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */
// ─────────────────────────────────────────────────────────────
//  FUNÇÃO DE INTERRUPÇÃO (CALLBACK EXTI)
//  Esta função é chamada automaticamente pelo hardware no clique
// ─────────────────────────────────────────────────────────────
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Muda de GPIO_PIN_5 para GPIO_PIN_3
    if (GPIO_Pin == GPIO_PIN_3) {
        uint32_t agora = HAL_GetTick();
        if ((agora - tUltimoDebounce) > 200) {
            modoManual = !modoManual;
            tUltimoDebounce = agora;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  ATUADORES FÍSICOS
// ─────────────────────────────────────────────────────────────
void Servo_SetAngle(uint32_t pulso_us) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulso_us);
}

void Motor_Ligar(void) {
    uint32_t agora = HAL_GetTick();
    if ((agora / 1000) % 2 == 0) Servo_SetAngle(SERVO_ESQUERDA);
    else                         Servo_SetAngle(SERVO_DIREITA);
}

void Motor_Desligar(void) { Servo_SetAngle(SERVO_REPOUSO); }
void Fan_On(void)         { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   }
void Fan_Off(void)        { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); }
void Bomba_On(void)       { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    }
void Bomba_Off(void)      { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  }

void UART_Log(void) {
    char msg[160];
    int len = snprintf(msg, sizeof(msg), // @suppress("Float formatting support")
        "Temp: %.1fC | U.Ar: %.1f%% | U.Solo: %.1f%% | Mot: %s%s | Fan: %s | Bomb: %s\r\n",
        dht_temp, dht_umid, umid_solo,
        motorAtivo ? "ON" : "off", modoManual ? " (MANUAL)" : "",
        fanAtiva   ? "ON" : "off",
        bombaAtiva ? "ON" : "off");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init(); // Configura o PB5 como interrupção externa
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   Motor_Desligar();
   Fan_Off();
   Bomba_Off();

   meuDHT22.DHT_Port = GPIOA;
   meuDHT22.DHT_Pin  = GPIO_PIN_1;
   meuDHT22.type     = DHT22;
   meuDHT22.pullUp   = GPIO_PULLUP; // Ativa Pull-Up interno para estabilizar a linha

   meuSensorSolo.hadc = &hadc1;

   uint32_t tInicioCiclo    = 0;
   uint32_t tFimCiclo       = 0;
   uint32_t tUltimoLog      = 0;

   char boot[] = "\r\n=== COMPOSTEIRA COM INTERRUPCAO INICIADA ===\r\n";
   HAL_UART_Transmit(&huart2, (uint8_t*)boot, strlen(boot), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
      uint32_t agora = HAL_GetTick();

      // 1. LEITURA SENSOR DE SOLO
      dadosSolo = CSMV2_getData(&meuSensorSolo);
      umid_solo = (float)dadosSolo.moisture_percent;

      // 2. LEITURA DHT22
      dadosDHT22 = DHT_getData(&meuDHT22);
      if (dadosDHT22.temp != -128.0f) {
          dht_temp = dadosDHT22.temp;
          dht_umid = dadosDHT22.hum;
      }

      // 3. CONDIÇÕES DOS SENSORES
      uint8_t tempAlta       = (dht_temp >= TEMP_ALTA);
      uint8_t umidArAlta     = (dht_umid >= UMID_AR_ALTA);
      uint8_t umidTerraBaixa = (umid_solo <= UMID_TERRA_BAIXA);

      // Nota: Toda a lógica antiga de leitura do botão foi removida daqui!
      // O botão agora roda em paralelo via Hardware na função HAL_GPIO_EXTI_Callback.

      // 4. CICLO AUTOMÁTICO DO MOTOR
      if (cicloAutoAtivo) {
          if (agora - tInicioCiclo >= DURACAO_CICLO) {
              cicloAutoAtivo = 0;
              tFimCiclo      = agora;
          }
      } else {
          if (agora - tFimCiclo >= INTERVALO_CICLO) {
              cicloAutoAtivo = 1;
              tInicioCiclo   = agora;
          }
      }

      // 5. DECISÃO FINAL DOS ATUADORES
      // O motor liga se o ciclo automático estiver ativo, se a temperatura estiver alta OU se a interrupção ativou o modo manual.
      motorAtivo = cicloAutoAtivo || tempAlta || modoManual;
      fanAtiva   = tempAlta || umidArAlta;
      bombaAtiva = umidTerraBaixa;

      // 6. APLICAÇÃO FÍSICA NO HARDWARE
      if (motorAtivo) Motor_Ligar();
      else            Motor_Desligar();

      if (fanAtiva)   Fan_On();
      else            Fan_Off();

      if (bombaAtiva) Bomba_On();
      else            Bomba_Off();

      // 7. LOGS UART
      if (agora - tUltimoLog >= LOG_MS) {
          tUltimoLog = agora;
          UART_Log();
      }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12, GPIO_PIN_RESET);

  /* PB1 (Bomba) e PB12 (Fan) como Saídas */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configuração do PB5 como Entrada de INTERRUPÇÃO EXTERNA (EXTI) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Interrupção na borda de descida (Clique)
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // Ativa resistor de Pull-Up interno
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Ativa o vetor de interrupção no controlador de interrupções (NVIC) */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
