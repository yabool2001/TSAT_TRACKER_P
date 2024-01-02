/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;
#define BUFFER_SIZE 4096
/* USER CODE BEGIN PV */
char*		hello = "\nHello NEMO2SPACE TRACKER P assembly test firmware v0.0.1 started.\n" ;

// UART
uint8_t c = 0 ;
bool test = false ;
char uart_buff[250] = {0} ;

// ACC
stmdev_ctx_t my_acc_ctx ;

//TIM
uint16_t tim_seconds = 0 ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
volatile uint8_t uart5_rx_buffer[BUFFER_SIZE];
volatile uint8_t uart5_rx_index = 0;

uint8_t uart2_tx_buffer[BUFFER_SIZE];

typedef struct {
    uint8_t* buffer;
    uint16_t total_size;
    uint16_t cur_size;
    uint16_t in_index;
    uint16_t out_index;
} fifo_t;

fifo_t uart_fifo;
uint8_t 	gnss_rxd_byte = 0 ;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART5_UART_Init(void);

/* USER CODE BEGIN PFP */
void send_debug_logs ( char* ) ;
int32_t my_lis2dw12_platform_write ( void* , uint8_t , const uint8_t* , uint16_t ) ;
int32_t my_lis2dw12_platform_read ( void* , uint8_t , uint8_t* , uint16_t ) ;
void my_gnss_on ( void ) ;
void my_gnss_off ( void ) ;
void my_tim_init (void ) ;
void my_tim_start (void ) ;
void my_tim_stop (void ) ;
uint8_t fifo_init(uint8_t* , uint16_t, volatile fifo_t*);
bool fifo_is_empty(fifo_t* );
uint8_t fifo_put(fifo_t* , uint8_t);
uint8_t fifo_get(fifo_t* , uint8_t*);

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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  HAL_UART_Transmit ( HUART_DBG , (uint8_t*) hello , strlen ( hello ) , UART_TIMEOUT ) ;
  MX_USART3_UART_Init();
  MX_USART5_UART_Init();

  __enable_irq();
  /* USER CODE BEGIN 2 */
  fifo_init(uart2_tx_buffer, BUFFER_SIZE, &uart_fifo);



  my_tim_init () ;
  send_debug_logs ( "The device test started. You have max. 10 minutes to complete each steps.\n" ) ;
  send_debug_logs ( "GREEN LED ON\n" ) ;
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  send_debug_logs ( "GREEN LED OFF\n" ) ;
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);
  send_debug_logs ( "BLUE LED ON\n" ) ;
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
  HAL_Delay(2000);
  send_debug_logs ( "BLUE LED OFF\n" ) ;
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);


  // ACC TEST



  send_debug_logs ( "** Ambient Light Sensor test - 10 seconds\n" ) ;
  send_debug_logs ( "Cover light sensor -> green light should be ON.\n" ) ;
  send_debug_logs ( "Uncover light sensor -> green light should be OFF.\n" ) ;
  tim_seconds = 0 ;
  my_tim_start () ;
  while ( tim_seconds < 10 )
      {

  	  if(HAL_GPIO_ReadPin(ALS_SENS_GPIO_Port, ALS_SENS_Pin))
  		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  	  else
  		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      }
    my_tim_stop () ;


    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    send_debug_logs ( "** Accelerometer test - 10 seconds\n" ) ;
    send_debug_logs ( "** Only checks if ID of ACC is correct\n" ) ;

    tim_seconds = 0 ;
    my_tim_start () ;

  while ( tim_seconds < 10 )
  {
	  send_debug_logs ( "* IIS2DHTR test started - 10 seconds" ) ;
	  my_acc_ctx.write_reg = my_lis2dw12_platform_write ;
	  my_acc_ctx.read_reg = my_lis2dw12_platform_read ;
	  my_acc_ctx.handle = HACC ;
	  if ( my_lis2dw12_init ( &my_acc_ctx ) )
	  {
		  send_debug_logs ( "**ID is correct." ) ;
		  break;
	  }
	  else
	  {
		  send_debug_logs ( "** ID is not received or wrong" ) ;
	  }
	 //	  my_lis2dw12_int1_wu_enable ( &my_acc_ctx ) ;
	 //	  send_debug_logs ( "** LIS2DW12 wakeup int1 has been enabled. Try to wake up the device." ) ;
	 //	  while ( !test && tim_seconds < 30 )
	 //	  {
	 //		  if ( HAL_GPIO_ReadPin ( ACC_INT1_GPIO_Port , ACC_INT1_Pin ) == GPIO_PIN_SET )
	 //		  {
	 //			  test = true ;
	 //			  send_debug_logs ( "* Good! LIS2DW12 test has been accomplished." ) ;
	 //			  break ;
	 //		  }
	 //	  }
	 //	  if ( test )
	 //		  break ;
	 //	  else
	 //		  send_debug_logs ( "* Something went wrong! MCU did not received INT1." ) ;
  }
  my_tim_stop () ;


  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  // GNSS TEST

  uint8_t 	rxd_byte = 0 ;
  uint8_t 	i_nmea = 0 ;
  uint8_t 	gsv_tns = 0 ;
  uint8_t 	nmea_message[MY_NMEA_MESSAGE_MAX_SIZE] = {0} ;
  char*		nmea_gsv_label = "GSV" ;


  my_gnss_on () ;
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
  send_debug_logs ( "* Changing RF path to GNSS - CTL1 H, CTL2 L\n" ) ;
  HAL_GPIO_WritePin(RF_SW_CTL1_GPIO_Port, RF_SW_CTL1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RF_SW_CTL2_GPIO_Port, RF_SW_CTL2_Pin, GPIO_PIN_RESET);

  send_debug_logs ( "* LC76G test started. Try to complete it within 10 minutes." ) ;
  send_debug_logs ( "* GREEN LED indicate succesful 3D position fix.\n" ) ;
  HAL_UART_Receive_IT(HUART_GNSS, &rxd_byte, 1);

  tim_seconds = 0 ;
  my_tim_start () ;
  while ( tim_seconds < TIM_SECONDS_THS_SYSTEM_RESET  )
  {
//	  if (HAL_UART_Receive(HUART_GNSS, &rxd_byte, 1, UART_TIMEOUT) == HAL_OK) {
//	              // Enqueue received data into FIFO
//	              EnqueueFifo(&uartFifo, rxd_byte);
//	          }
// HAL_UART_Receive_IT(HUART_GNSS, &uartBuffer.data[uartBuffer.head], 1);
//	          // Process data in FIFO and transmit to UART2
//	          uint8_t txd_byte;
//	          if (DequeueFifo(&uartFifo, &txd_byte) == HAL_OK) {
//	              // Transmit data to UART2
//	              HAL_UART_Transmit(HUART_DBG, &txd_byte, 1, UART_TIMEOUT);
//	          }
//	  HAL_UART_Receive_IT(HUART_GNSS, &uartBuffer.data[uartBuffer.head], 1);
//	  HAL_UART_Receive ( HUART_GNSS , &rxd_byte , 2 , UART_TIMEOUT ) ;
//	  HAL_UART_Transmit ( HUART_DBG , &rxd_byte , 2 , UART_TIMEOUT ) ;
//	  send_debug_logs ( "* dumb" ) ;
	 while(!(fifo_is_empty(&uart_fifo))) {
	              // Transmit the data from UART5 to UART2
		  	  	  if(!(fifo_get(&uart_fifo, &rxd_byte)))
				  HAL_UART_Transmit(HUART_DBG, &rxd_byte, 1, UART_TIMEOUT);
//	              HAL_UART_Transmit(HUART_DBG, uart5_rx_buffer, uart5_rx_index, UART_TIMEOUT);
//	              uart5_rx_index = 0; // Reset the index after transmission

	          }
	 if(HAL_GPIO_ReadPin(GNSS_3DFIX_GPIO_Port, GNSS_3DFIX_Pin))
		 send_debug_logs ( "\n3D FIX pin HIGH\n" );
	 else
		 send_debug_logs ( "\n3D FIX pin LOW\n" );

	 if(HAL_GPIO_ReadPin(GNSS_JAM_GPIO_Port, GNSS_JAM_Pin))
	 		 send_debug_logs ( "JAM pin HIGH\n" );
	 	 else
	 		 send_debug_logs ( "JAM pin LOW\n" );

	 if(HAL_GPIO_ReadPin(GNSS_GEOF_GPIO_Port, GNSS_GEOF_Pin))
	 		 send_debug_logs ( "GEOFFENCE pin HIGH\n" );
	 	 else
	 		 send_debug_logs ( "GEOFFENCE pin LOW\n" );
	 HAL_Delay(1000);
//	  if ( rxd_byte )
//	  {
//		  if ( my_nmea_message ( &rxd_byte , nmea_message , &i_nmea ) == 2 )
//		  {
//			  if ( is_my_nmea_checksum_ok ( (char*) nmea_message ) )
//			  {
//				  if ( strstr ( (char*) nmea_message , nmea_gsv_label ) )
//				  {
//					  gsv_tns = my_nmea_get_gsv_tns ( (char*) nmea_message ) ;
//				  }
//				  if ( gsv_tns > 3 )
//					  break ;
//			  }
//		  }
//	  }
  }
  my_gnss_off ();
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
  my_tim_stop ();

  send_debug_logs ( "* Changing RF path to Astronode - CTL2 H, CTL1 L\n" );
  HAL_GPIO_WritePin(RF_SW_CTL2_GPIO_Port, RF_SW_CTL2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RF_SW_CTL1_GPIO_Port, RF_SW_CTL1_Pin, GPIO_PIN_RESET);

  if ( gsv_tns )
  {
	  sprintf ( uart_buff , "* Good! LC76G test has been accomplished. No of SV: %d" , gsv_tns ) ;
	  send_debug_logs ( uart_buff ) ;
	  uart_buff[0] = 0 ;
  }
  else
  {
	  send_debug_logs ( "\n* Something went wrong! LC76G did not find any SV." ) ;
  }

  // ASTRO TEST
  bool cfg_wr = false ;
  tim_seconds = 0 ;
  my_tim_start() ;
  while ( tim_seconds < 30 && !cfg_wr )
  {
	  reset_astronode () ;
	  HAL_Delay ( 100 ) ;
	  cfg_wr = astronode_send_cfg_wr ( true , true , true , false , true , true , true , false  ) ;
  }
  my_tim_stop() ;
  if ( cfg_wr )
  {
	  astronode_send_mpn_rr () ;
	  send_debug_logs ( "* Good! Astronode test has been accomplished." ) ;
  }
  else
  {
	  send_debug_logs ( "\n* Something went wrong! Astronode did not work fine." ) ;
  }
  send_debug_logs ( "\nThis is the end of the test." ) ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_UART_Receive	( HUART_GNSS, &c , 1 , UART_TIMEOUT ) ;
//	  HAL_UART_Transmit ( HUART_DBG, &c , 1 , UART_TIMEOUT ) ;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
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
  /* USER CODE BEGIN USART5_Init 2 */
  HAL_NVIC_SetPriority(USART3_4_5_6_LPUART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);




  /* USER CODE END USART5_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ASTRO_WKUP_Pin|ASTRO_RST_Pin|GNSS_RST_Pin|GNSS_PWR_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_SW_CTL1_Pin|RF_SW_CTL2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ACC_INT1_Pin ACC_INT2_Pin */
  GPIO_InitStruct.Pin = ACC_INT1_Pin|ACC_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_CS_Pin */
  GPIO_InitStruct.Pin = ACC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ASTRO_WKUP_Pin ASTRO_RST_Pin RF_SW_CTL1_Pin RF_SW_CTL2_Pin
                           GNSS_PWR_SW_Pin */
  GPIO_InitStruct.Pin = ASTRO_WKUP_Pin|ASTRO_RST_Pin|RF_SW_CTL1_Pin|RF_SW_CTL2_Pin
                          |GNSS_PWR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ASTRO_EVT_Pin GNSS_3DFIX_Pin GNSS_JAM_Pin */
  GPIO_InitStruct.Pin = ASTRO_EVT_Pin|GNSS_3DFIX_Pin|GNSS_JAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GNSS_RST_Pin */
  GPIO_InitStruct.Pin = GNSS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GNSS_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SATCOM_ANTN_USE_Pin GNSS_GEOF_Pin */
  GPIO_InitStruct.Pin = SATCOM_ANTN_USE_Pin|GNSS_GEOF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ALS_SENS_Pin */
  GPIO_InitStruct.Pin = ALS_SENS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALS_SENS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// FUNCTIONS REQUIRED BY astronode-stm32-example-asset library
void send_debug_logs ( char* p_tx_buffer )
{
    uint32_t length = strlen ( p_tx_buffer ) ;

    if ( length > UART_TX_MAX_BUFF_SIZE )
    {
        HAL_UART_Transmit ( HUART_DBG , ( uint8_t* ) "[ERROR] UART buffer reached max length.\n" , 42 , UART_TIMEOUT ) ;
        length = UART_TX_MAX_BUFF_SIZE;
    }

    HAL_UART_Transmit ( HUART_DBG , ( uint8_t* ) p_tx_buffer , length , UART_TIMEOUT ) ;
    HAL_UART_Transmit ( HUART_DBG , ( uint8_t* ) "\n" , 1 , UART_TIMEOUT ) ;
}

// ACC LL Function

int32_t my_lis2dw12_platform_write ( void *handle , uint8_t reg , const uint8_t *bufp , uint16_t len )
{
	HAL_GPIO_WritePin	( GPIOA , ACC_CS_Pin , GPIO_PIN_RESET ) ;
	HAL_Delay ( 20 ) ;
	HAL_SPI_Transmit	( handle , &reg , 1 , 1000 ) ;
	HAL_SPI_Transmit	( handle , (uint8_t*) bufp , len , 1000 ) ;
	HAL_GPIO_WritePin	( GPIOA , ACC_CS_Pin , GPIO_PIN_SET) ;

	return 0;
}

int32_t my_lis2dw12_platform_read ( void *handle , uint8_t reg , uint8_t *bufp , uint16_t len )
{
	reg |= 0x80;
	HAL_GPIO_WritePin ( GPIOA , ACC_CS_Pin , GPIO_PIN_RESET) ;
	HAL_Delay ( 20 ) ;
	HAL_SPI_Transmit ( handle , &reg , 1 , 1000 ) ;
	HAL_SPI_Receive ( handle , bufp , len , 1000 ) ;
	HAL_GPIO_WritePin ( GPIOA , ACC_CS_Pin , GPIO_PIN_SET) ;

	return 0;
}

// GNSS LL Function
void my_gnss_on ( void )
{
	HAL_GPIO_WritePin ( GPIOB , GNSS_PWR_SW_Pin , GPIO_PIN_SET ) ;
	HAL_GPIO_WritePin ( GPIOB , GNSS_RST_Pin , GPIO_PIN_SET ) ;
	MX_USART3_UART_Init () ;
}
void my_gnss_off ( void )
{
	HAL_GPIO_WritePin ( GPIOB , GNSS_PWR_SW_Pin , GPIO_PIN_RESET ) ;
	HAL_GPIO_WritePin ( GPIOB , GNSS_RST_Pin , GPIO_PIN_RESET ) ;
	HAL_UART_DeInit ( HUART_GNSS ) ;
}

void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
	if ( htim->Instance == TIM6 )
	{
		tim_seconds++ ;
		if ( tim_seconds > TIM_SECONDS_THS_SYSTEM_RESET )
		{
			send_debug_logs ( "Watchdog activated! System restart!" ) ;
			HAL_NVIC_SystemReset () ;
		}
	}
}

void my_tim_init (void )
{
	__HAL_TIM_CLEAR_IT ( HTIM , TIM_IT_UPDATE ) ;
}

void my_tim_start (void )
{
	tim_seconds = 0 ;
	HAL_TIM_Base_Start_IT ( HTIM ) ;
}

void my_tim_stop (void )
{
	HAL_TIM_Base_Stop_IT ( HTIM ) ;
}
void reset_astronode ( void )
{
    HAL_GPIO_WritePin ( ASTRO_RST_GPIO_Port , ASTRO_RST_Pin , GPIO_PIN_SET ) ;
    HAL_Delay ( 1 ) ;
    HAL_GPIO_WritePin ( ASTRO_RST_GPIO_Port , ASTRO_RST_Pin , GPIO_PIN_RESET ) ;
    HAL_Delay ( 250 ) ;
}
void send_astronode_request ( uint8_t* p_tx_buffer , uint32_t length )
{
    send_debug_logs ( "Message sent to the Astronode --> " ) ;
    send_debug_logs ( ( char* ) p_tx_buffer ) ;
    HAL_UART_Transmit ( HUART_ASTRO , p_tx_buffer , length , 1000 ) ;
}
uint32_t get_systick ( void )
{
    return HAL_GetTick() ;
}
bool is_systick_timeout_over ( uint32_t starting_value , uint16_t duration )
{
    return ( get_systick () - starting_value > duration ) ? true : false ;
}
bool is_astronode_character_received ( uint8_t* p_rx_char )
{
    return ( HAL_UART_Receive ( HUART_ASTRO , p_rx_char , 1 , 100 ) == HAL_OK ? true : false ) ;
}




uint8_t fifo_init(uint8_t* worker_buffer, uint16_t size, volatile fifo_t* fifo)
{
    uint8_t retval = 0;

    if (NULL == worker_buffer) {
        retval = 1;
        goto exit;
    }

    if (0 == size) {
        retval = 2;
        goto exit;
    }

    if (NULL == fifo) {
        retval = 3;
        goto exit;
    }

    fifo->buffer = worker_buffer;
    fifo->total_size = size;
    fifo->cur_size = 0;
    fifo->in_index = 0;
    fifo->out_index = 0;

exit:
    return retval;
}

bool fifo_is_empty(fifo_t* fifo)
{
    bool is_empty = true;

    if (0 != fifo->cur_size) {
        is_empty = false;
    }

    return is_empty;
}

uint8_t fifo_put(fifo_t* fifo, uint8_t byte)
{
    uint8_t retval = 0;

    if (NULL == fifo) {
        retval = 1;
        goto exit;
    }

    /* Check if fifo is not full */
    if (fifo->total_size == fifo->cur_size) {
        retval = 2;
        goto exit;
    }

    /* Enqueue the byte */
    fifo->buffer[fifo->in_index] = byte;
    /* Increase the input index */
    fifo->in_index += 1;
    /* Increase the count of the bytes placed in the fifo */
    fifo->cur_size += 1;

    /* Wrap around the buffer */
    if (fifo->in_index == fifo->total_size) {
        fifo->in_index = 0;
    }

exit:
    return retval;
}

uint8_t fifo_get(fifo_t* fifo, uint8_t* byte)
{
    uint8_t retval = 0;

    if (NULL == fifo) {
        retval = 1;
        goto exit;
    }

    if (NULL == byte) {
        retval = 2;
        goto exit;
    }

    /* Check if fifo is not full */
    if (0 == fifo->cur_size) {
        retval = 3;
        goto exit;
    }

    /* Dequeue the byte */
    *byte = fifo->buffer[fifo->out_index];
    /* Increase the output index */
    fifo->out_index += 1;
    /* Decrement the count of the bytes placed in the fifo */
    fifo->cur_size -= 1;

    /* Wrap around the buffer */
    if (fifo->out_index ==fifo->total_size) {
        fifo->out_index = 0;
    }

exit:
    return retval;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void EXTI4_15_IRQHandler (void) {
    HAL_GPIO_EXTI_IRQHandler(GNSS_3DFIX_Pin);

}
void USART3_4_5_6_LPUART1_IRQHandler(void) {
    HAL_UART_IRQHandler(HUART_GNSS);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == HUART_GNSS) {
        // Move the head of the circular buffer


    	fifo_put(&uart_fifo, gnss_rxd_byte);
//    	if (uart5_rx_index < BUFFER_SIZE) {
//    	            // Store the received data in the buffer
//    	            uart5_rx_buffer[uart5_rx_index++] = huart->Instance->RDR;
//    	        } else {
//    	            // Buffer overflow, handle the overflow condition as needed
//    	            uart5_rx_index = 0;
//    	        }
        // Start a new reception

    	        HAL_UART_Receive_IT(&huart5, &gnss_rxd_byte, 1);

        // Process the received data, e.g., transmit it to UART2

//        HAL_UART_Transmit_IT(HUART_DBG, &receivedData, 1);
//        if(HAL_UART_Transmit(HUART_DBG, &receivedData, 1, UART_TIMEOUT) != HAL_OK)
//        {
//        	while(1);
//        }
}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GNSS_3DFIX_GPIO_Port, GNSS_3DFIX_Pin))
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	send_debug_logs ( "* EXTI5 INT: Succesful 3D position fix.\n" ) ;
}
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
