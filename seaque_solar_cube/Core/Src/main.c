/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include <string.h>
#include "axe_init.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint8_t uart_char[MAX_SLIP_SIZE];
uint8_t pkt_buff[MAX_IPKT_SIZE];
uint8_t fdcan_tx_buff[64];
int fdcan_tx_buff_num = 0;
int transmit_buff_num = 0;
bool command_receive = false;
uint8_t can_rx_test[64];
FDCAN_TxHeaderTypeDef tx_header;
FDCAN_RxHeaderTypeDef rx_header;

const uint32_t my_address = (('Q' & 0x1F) << 24) + ('U' << 16) + ('E' << 8);
// The first multicast address is ALL, which is ASCII "ALL" preceeded by 1's over CAN.
const uint32_t all_address = (0x1F << 24) + ('A' << 16) + ('L' << 8) + 'L';
// The second multicast address is HALL, which is ASCII "ALL" preceeded by 0's over CAN.
// This makes HALL multicast messages higher priority.
const uint32_t hall_address = (0x00 << 24) + ('A' << 16) + ('L' << 8) + 'L';

// Other addresses
const uint32_t qcb_address = (('U' & 0x1F) << 24) + ('I' << 16) + ('U' << 8) + 'C';

const FDCAN_FilterTypeDef sFilterConfig1 = {
  .IdType = FDCAN_EXTENDED_ID,
  .FilterIndex = 0,
  .FilterType = FDCAN_FILTER_DUAL,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
  .FilterID1 = PLSB_ADDR,
  .FilterID2 = PLSB_ADDR
};

const FDCAN_FilterTypeDef sFilterConfig2 = {
  .IdType = FDCAN_EXTENDED_ID,
  .FilterIndex = 1,
  .FilterType = FDCAN_FILTER_DUAL,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP,
  .FilterID1 = PLSB_ADDR,
  .FilterID2 = PLSB_ADDR
};

struct TNODE table[20];
volatile int adc_conv1_complete = 0;
#define NUM_CHANNELS 8
uint16_t adc_result[NUM_CHANNELS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ICACHE_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void send_stm32_uart(ipacket_t* pkt);
int axe_process_f(ipacket_t* pkt);
void takeMeasurement();
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
  //char can_test[12] = "1234567890\r\n";
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
  MX_DMA_Init();
  MX_ICACHE_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  tx_header.Identifier = qcb_address;
  tx_header.IdType = FDCAN_EXTENDED_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_64;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.MessageMarker = 0;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;


  HAL_UART_Receive_IT(&huart4, uart_char, 1);
  memset(fdcan_tx_buff, '\0', sizeof(fdcan_tx_buff));

  // ICOMMs config
  iface_t can = {
      .active = true,
      // Should be all but your address if not RADCOM
      .addrs = {ALBIN_ADDR, DCM_ADDR, QCB_ADDR, GND_ADDR, RADCOM_ADDR},
      .mtu = CAN_MTU,
      .interface = CAN,
      .send_pkt = stm32can_send,
  };
  iface_table[0] = can;
  board_config.router = false;
  board_config.my_addr = PLSB_ADDR;
  board_config.crc32_f = crc32;
  board_config.axe_process_f = axe_process_f;
  seaque_fdcan = &hfdcan1;

  axe_config_t a_cfg = {
    .root_table = root_table,
    .root_table_len = TABLE_SIZE,
  };
  axe_set_config(a_cfg);
  init_axe();
  table_init();

  // PWM setup
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Init(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);


  // ICOMM Packet
  ipacket_t pkt = {
    .cmd = 0,
    .addr_to = GND_ADDR,
    .addr_from = board_config.my_addr,
    .n = 0,
    .m = 0,
    .data = {'\0'},
  };

  // Serial debug variables
  int return_val = 0;
  char test[50] = {'\0'};

  int tempsize = snprintf(test, sizeof(test), "Started\r\n");
  HAL_UART_Transmit(&huart5, test, tempsize, 100);

  uint16_t* packet_data;
  packet_data = (uint16_t*)(pkt.data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ----- Sweep Curve -----
    // MIKRO1_PWM
    for (int i=0; i<255; i++){
      htim3.Instance->CCR3 = i;
      takeMeasurement();
      packet_data[i] = adc_result[0];
      packet_data[i+256] = adc_result[1];
    }
    // MIKRO2_PWM
    for (int i=0; i<255; i++){
      htim1.Instance->CCR1 = i;
      takeMeasurement();
      packet_data[i+512] = adc_result[2];
      packet_data[i+512+256] = adc_result[3];
    }
    // MIKRO4_PWM
    for (int i=0; i<255; i++){
      htim3.Instance->CCR2 = i;
      takeMeasurement();
      packet_data[i+1024] = adc_result[6];
      packet_data[i+1024+256] = adc_result[7];
    }

    // a test packet
    /* for (int i=0;i<512 * 3 * 2;i++){ */
    /*   pkt.data[i] = i; */
    /* } */

    if (telemetry_en) {
      pkt.size = 512 * 3 * 2;
      send_ipacket(&pkt);
      comms_tick();
    }


    /* // send the whole packet over serial */
    /* HAL_UART_Transmit(&huart5, packet_data, pkt.size, 100); */
    /* HAL_Delay(100); */



    /* ----------------------------------- */
    /* uint16_t raw; */
    /* char msg[10]; */
    /* while (1) */
    /*   { */

    /*     // Get ADC value */
    /*     HAL_ADC_Start(&hadc1); */
    /*     HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); */
    /*     raw = HAL_ADC_GetValue(&hadc1); */

    /*     // Convert to string and print */
    /*     sprintf(msg, "%hu\r\n", raw); */
    /*     HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); */

    /*     // Pretend we have to do something else for a while */
    /*     HAL_Delay(1); */
    /*   } */
    /* ----------------------------------- */

    /* takeMeasurement(); */
    /* for (int i=0; i<NUM_CHANNELS; i++){ */
    /*     int tempsize = snprintf(test, sizeof(test), "RRank %d: %u\r\n", i+1, adc_result[i]); */
    /*     HAL_UART_Transmit(&huart5, test, tempsize, 100); */
    /*     HAL_Delay(10); */
    /* } */

    /* HAL_Delay(1000); */
    /* HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET); */
    /* HAL_Delay(1000); */
    /* HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET); */
    /* HAL_Delay(300000); */

    HAL_Delay(300000);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1) != HAL_OK){
    Error_Handler();
  }

  HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0);

  HAL_FDCAN_Start(&hfdcan1);
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
size_t place = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (uart_char[place] == 0xC0){
    deslip_data(pkt_buff, uart_char);
    ipacket_t pkt = deserialize_ipacket(pkt_buff);
    recv_ipacket(&pkt);
    place = 0;
    HAL_UART_Receive_IT(&huart4, uart_char, 1);
  } else {
    place++;
    HAL_UART_Receive_IT(&huart4, uart_char + place, 1);
  }

}
/*
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
  HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, can_rx_test);
  //can_rx_test[64] = '\n';l
  HAL_UART_Transmit(&huart4, can_rx_test, strnlen(can_rx_test, 64), 100);

}*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
  do {
    FDCAN_RxHeaderTypeDef rxh = {0};
    uint8_t rx_buffer[64];
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxh, rx_buffer);
    ipacket_t pkt = stm32can_deserialize_ipacket(&rxh, rx_buffer);
    recv_ipacket(&pkt);
  } while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0);
}
void send_stm32_uart(ipacket_t* pkt)
{
  uint8_t buff[MAX_IPKT_SIZE];
  uint8_t buffs[MAX_SLIP_SIZE];
  size_t data_len  = serialize_ipacket(pkt, buff);
  size_t sdata_len = slip_data(buffs, buff, data_len);
  HAL_UART_Transmit(&huart4, buffs, sdata_len, 100);

}

int axe_process_f(ipacket_t* pkt)
{
  ipacket_t rpkt = {
    .cmd = 0,
    .addr_to = pkt->addr_from,
    .addr_from = board_config.my_addr,
    .n = 0,
    .m = 0,
  };
  unsigned int reply_len = process_axe(pkt->data, pkt->size, rpkt.data, MAX_IPAYLOAD_SIZE);
  rpkt.size = reply_len;
  if (reply_len > 0 && reply_len < MAX_IPAYLOAD_SIZE){
    send_ipacket(&rpkt);
  }
  return 0;
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart5, (unsigned char *)&ch, 1, 100);
    return ch;
}

void takeMeasurement() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_result, NUM_CHANNELS);
    while (adc_conv1_complete == 0);
    adc_conv1_complete = 0;
}

// Signal that ADC has finished conversions
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle){
  if (AdcHandle->Instance == ADC1)
    adc_conv1_complete = 1;
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
     ex printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
