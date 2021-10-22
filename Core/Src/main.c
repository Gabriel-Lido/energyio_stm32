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
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "float.h"
#include "nrf24.h"
#include "messages_stm.pb.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
#include "flash_f1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ZERO_OFFSET_V 2080
#define ZERO_OFFSET_I 2088
#define ADC_TO_V_STEP 0.2030
#define ADC_TO_I_STEP 0.0113
#define SAMPLES_PER_CYCLE 128
#define ONE_SECOND 60
#define HALF_SECOND 30

#define FLASH_PAGE_ADDR 0x0801FC00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t actual_tick[128], actual_sample = 0, j = 0;
float voltage_sample[SAMPLES_PER_CYCLE], v_rms_cycle = 0, v_rms = 0, v_rms_cycles_vector[ONE_SECOND];
float current_sample[SAMPLES_PER_CYCLE], i_rms_cycle = 0, i_rms = 0, i_rms_cycles_vector[ONE_SECOND];
int pot_ativa_sample[SAMPLES_PER_CYCLE], pot_ativa_cycle = 0, pot_ativa = 0, pot_ativa_cycles_vector[ONE_SECOND];
int pot_aparente = 0, pot_aparente_cycle = 0, pot_aparente_cycles_vector[SAMPLES_PER_CYCLE];
int ready_values = 0;

float aux_v_rms, aux_i_rms = 0;
int aux_pot_ativa, aux_pot_aparente, samples = 0;

uint8_t node_address[3][6] = {"HUB01", "EA101", "12345"};
char sensor_serial[6] = "EA101";
bool pairingMode = false;
uint8_t data[32];
uint8_t length;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static bool nrf_report();
static bool nrf_pairing();
static void nrf_pairing_report(char *_serial, int _channel);
/* USER CODE BEGIN PFP */
void r_PairingMessage(uint8_t *_data, int _data_len);
void w_message(double v_rms, double i_rms, int pot_at, int pot_ap, int samples);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);

  return len;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /*Buffers flash*/
  //	uint8_t buff_write_flash[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  //	uint8_t buff_read_flash[16];
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    printf("Falha ao calibrar o ADC1");
  }
  HAL_Delay(10);

  if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
  {
    printf("Falha ao calibrar o ADC2");
  }
  HAL_Delay(10);

  HAL_TIM_Base_Start_IT(&htim2);

  /* Operações Flash*/
  //  Flash_Write_Data(FLASH_PAGE_ADDR , (uint32_t*)buff_write_flash, (sizeof(buff_write_flash)/sizeof(int)));
  //  Flash_Read_Data(FLASH_PAGE_ADDR , (uint32_t*)node_address[0], (sizeof(node_address[0])/sizeof(int)));

  NRF24_begin(GPIOB, NRF_CSN_Pin, NRF_CE_Pin, hspi1);

  NRF24_openWritingPipe(node_address[1], sizeof(node_address[1]) - 1);

  NRF24_openReadingPipe(1, node_address[0], sizeof(node_address[0]) - 1);
  //  NRF24_openReadingPipe(2, node_address[1], sizeof(node_address[1]) - 1);

  NRF24_stopListening();

  printRadioSettings();

  printf("END SETUP\n\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (pairingMode)
    {
      NRF24_startListening();
      HAL_Delay(500);
      unsigned long start = HAL_GetTick();
      bool aux = false;
      while (aux != true && (unsigned long)(HAL_GetTick()) - start <= 15000)
      {
        if (nrf_pairing())
        {
          printf("\n\n[PAIRING] report");
          nrf_pairing_report(sensor_serial, 69);
          aux = true;
        }
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
        HAL_Delay(50);
      }
      if ((unsigned long)(HAL_GetTick()) - start > 15000)
      {
        printf("\n\n[PAIRING] Timeout\n\n");
      }
      NRF24_stopListening();
      NRF24_openWritingPipe(node_address[2], sizeof(node_address[2]) - 1);
      HAL_Delay(1500);
      NRF24_stopListening();
      start = HAL_GetTick();
      while (aux == true && (unsigned long)(HAL_GetTick()) - start <= 15000)
      {
        if (nrf_report())
        {
          aux = false;
          printf("\n\n[PAIRING] Success\n\n");
        }
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
        HAL_Delay(250);
      }
      if ((unsigned long)(HAL_GetTick()) - start > 15000)
      {
        printf("\n\n[PAIRING] Timeout\n\n");
      }

      pairingMode = false;
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      NRF24_stopListening();
      NRF24_openWritingPipe(node_address[1], sizeof(node_address[1]) - 1);
      HAL_Delay(1500);
      NRF24_stopListening();
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if (v_rms <= 20)
      v_rms = 0;
    if (i_rms < 0.14)
      i_rms = 0;

    if (ready_values)
    {
      printf("Tensao: %.1f  Corrente:%.3f  "
             "Pot.Ativa:%d  Pot Aparente:%d Msgs acumuladas:%d\r\n",
             v_rms, i_rms, pot_ativa, pot_aparente, samples);
      if (samples == 0)
        w_message(v_rms, i_rms, pot_ativa, pot_aparente, samples);
      else
        w_message(aux_v_rms, aux_i_rms, aux_pot_ativa, aux_pot_aparente, samples);

      unsigned long start = HAL_GetTick();
      bool aux = false;
      while (aux == false && (unsigned long)(HAL_GetTick()) - start <= 500)
      {
        if (nrf_report())
        {
          aux_v_rms = 0;
          aux_i_rms = 0;
          aux_pot_aparente = 0;
          aux_pot_ativa = 0;
          samples = 0;
          aux = true;
          HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
        }
      }
      if ((unsigned long)(HAL_GetTick()) - start > 500)
      {
        printf("\n[TX] Timeout\n");
        aux_v_rms += v_rms;
        aux_i_rms += i_rms;
        aux_pot_aparente += pot_aparente;
        aux_pot_ativa += pot_ativa;
        samples++;
      }
      ready_values = 0;
    }

    HAL_Delay(10);

    // r_message(data, sizeof(data));

    //    NRF24_startListening();
    //    HAL_Delay(1000);
    //    listen();

    //    HAL_Delay(1000);

    //    printf("payload size %d", NRF24_getPayloadSize());

    /*

*/

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6250 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_2_Pin | LED_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin | LED_2_Pin | LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW2_Pin | SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  int count;
  //  static teste_ini = 0;

  actual_tick[actual_sample] = HAL_GetTick();

  /* Le os ADCs e converte para o valor real */
  current_sample[actual_sample] = (abs((HAL_ADC_GetValue(&hadc1) - ZERO_OFFSET_I)) * ADC_TO_I_STEP);
  voltage_sample[actual_sample] = (abs((HAL_ADC_GetValue(&hadc2) - ZERO_OFFSET_V)) * ADC_TO_V_STEP);

  /* Potencia ativa é a média ponto a ponto de V.I*/
  pot_ativa_sample[actual_sample] = current_sample[actual_sample] * voltage_sample[actual_sample];

  /* Soma quadratica dos pontos */
  v_rms_cycle += pow(voltage_sample[actual_sample], 2);
  i_rms_cycle += pow(current_sample[actual_sample], 2);
  pot_ativa_cycle += pot_ativa_sample[actual_sample];

  actual_sample++;

  /* Calcula valores no final do ciclo*/
  if (actual_sample == SAMPLES_PER_CYCLE)
  {

    v_rms_cycle = sqrt((v_rms_cycle / SAMPLES_PER_CYCLE));
    i_rms_cycle = sqrt((i_rms_cycle / SAMPLES_PER_CYCLE));
    pot_ativa_cycle = (pot_ativa_cycle / SAMPLES_PER_CYCLE);
    pot_aparente_cycle = v_rms_cycle * i_rms_cycle;

    v_rms_cycles_vector[j] = v_rms_cycle;
    i_rms_cycles_vector[j] = i_rms_cycle;
    pot_ativa_cycles_vector[j] = pot_ativa_cycle;
    pot_aparente_cycles_vector[j] = pot_aparente_cycle;
    j++;

    i_rms_cycle = v_rms_cycle = actual_sample = 0;
  }

  /* Média dos ciclos*/
  if (j == ONE_SECOND)
  {
    v_rms = i_rms = 0;
    for (count = 0; count < ONE_SECOND; count++)
    {
      v_rms += v_rms_cycles_vector[count];
      i_rms += i_rms_cycles_vector[count];
      pot_ativa += pot_ativa_cycles_vector[count];
      pot_aparente += pot_aparente_cycles_vector[count];
    }
    v_rms = v_rms / ONE_SECOND;
    i_rms = i_rms / ONE_SECOND;
    pot_ativa = pot_ativa / ONE_SECOND;
    pot_aparente = pot_aparente / ONE_SECOND;

    j = 0;

    /* Indica disponibilidade das novas médias*/
    ready_values = 1;
  }

  /*Inicia as conversoes*/
  /*ADC Corrente*/
  HAL_ADC_Start_IT(&hadc1);

  /*ADC Tensao*/
  HAL_ADC_Start_IT(&hadc2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
}

unsigned long last_micros;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  unsigned long debouncing_time = 200; // Debouncing Time in Milliseconds
  if (GPIO_Pin == SW1_Pin)
  {
    if ((HAL_GetTick() - last_micros >= debouncing_time))
    {
      printf("\n\n\nSW1\n\n\n");
      last_micros = HAL_GetTick();
    }
  }
  if (GPIO_Pin == SW2_Pin)
  {
    if ((HAL_GetTick() - last_micros >= debouncing_time))
    {
      printf("\n\n\nPAIRING MODE ON\n\n\n");
      pairingMode = true;
      last_micros = HAL_GetTick();
    }
  }
}

void w_message(double v_rms, double i_rms, int pot_at, int pot_ap, int samples)
{
  uint8_t buffer[32];
  EnergySensorReport msg = EnergySensorReport_init_zero;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  msg.v_rms = v_rms;
  msg.i_rms = i_rms;
  msg.pot_aparente = pot_ap;
  msg.pot_ativa = pot_at;
  msg.samples = samples;
  pb_encode(&stream, EnergySensorReport_fields, &msg);

  printf("MSG SERIALIZED : ");
  for (int i = 0; i < stream.bytes_written; i++)
  {
    data[i] = buffer[i];
    printf("%02X", data[i]);
  }
  printf("\n");

  data[31] = (uint8_t)stream.bytes_written;
}

bool nrf_report()
{
  unsigned long start_time = HAL_GetTick();
  bool reported = NRF24_write(&data, sizeof(data)); // transmit & save the report
  unsigned long end_time = HAL_GetTick();

  if (reported)
  {
    printf("Transmission successful!"); // payload was delivered
    printf("Tranmission time %lu ms\nSent: ", end_time - start_time);
    for (int i = 0; i < 32; i++)
    {
      printf("%02X", data[i]);
    }
    printf("\n");
  }
  return reported;
}

void nrf_pairing_report(char *_serial, int _channel)
{
  uint8_t buffer[32];
  PairingMessage msg = PairingMessage_init_zero;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  strcpy(msg.serial, _serial);
  msg.channel = _channel;
  pb_encode(&stream, PairingMessage_fields, &msg);

  printf("\nPairing Msg : ");
  for (int i = 0; i < stream.bytes_written; i++)
  {
    data[i] = buffer[i];
    printf("%02X", data[i]);
  }
  printf("\n");

  data[31] = (uint8_t)stream.bytes_written;
}

bool nrf_pairing()
{
  uint8_t pipe;
  if (NRF24_availablePipe(&pipe))
  {
    uint8_t bytes = NRF24_getPayloadSize(); // get the size of the payload
    NRF24_read(&data, bytes);               // fetch payload from FIFO
    printf("Received %d \n", bytes);        // print the size of the payload
    printf(" bytes on pipe %d \n", pipe);
    for (int i = 0; i < bytes; i++)
    {
      printf("%02X", data[i]);
    }
    printf("\n");
    printf("\n");
    r_PairingMessage(data, bytes);
    printf("\n");
    return true;
  }
  return false;
}

void r_PairingMessage(uint8_t *_data, int _data_len)
{
  PairingMessage msg = PairingMessage_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(_data, _data_len);
  pb_decode(&stream, PairingMessage_fields, &msg);

  printf("DECODED: Serial: %s  Channel: %d\r\n", msg.serial, (int)msg.channel);

  // Flash_Write_Data(FLASH_PAGE_ADDR , (uint32_t*)msg.serial, (sizeof(msg.serial)/sizeof(int)));
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
