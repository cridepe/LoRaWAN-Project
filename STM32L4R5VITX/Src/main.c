/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdarg.h>
#include "ov2640.h"
#include "rfm95.h" 
#include <string.h>
#include "secconfig.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
I2C_HandleTypeDef hi2c2;
LPTIM_HandleTypeDef hlptim1;
RNG_HandleTypeDef hrng;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
#define PACKET_SIZE 220 //Puoi cambiare la dimensione del pacchetto
bool uplink = false;  //booleano controllo trasmissione

rfm95_handle_t rfm95_handle = {0};    // This structure will be used to manage all the configurations for the RFM
volatile uint32_t lptim_tick_msb = 0; // Track the LPTIM msb --> Remember to clear this variable beofre initializing the LPTIM
uint32_t lse_clk = (1<<15);           // defines the LSE clock speed (in Hz)
uint16_t rfm95_status;                // handle rfm95 status to process rfm outcomes

uint8_t packet[PACKET_SIZE];
size_t current_packet_size;

//DEF ADRESS
//Inserire i dati presi da TTN
uint8_t dev_addr[4] ={};
uint8_t nw_session_keys[16] = {};
uint8_t app_session_keys[16] = {};

/**
 * Resolution selection
 */

#define RES160X120
//#define RES320X240
//#define RES640X480
//#define RES800x600
//#define RES1024x768
//#define RES1280x960

#ifdef RES160X120
enum imageResolution imgRes=RES_160X120;
uint8_t frameBuffer[RES_160X120] = { 0 };
#endif

/*
#ifdef RES320X240
enum imageResolution imgRes=RES_320X240;
uint8_t frameBuffer[RES_320X240] = { 0 };
#endif

#ifdef RES640X480
enum imageResolution imgRes=RES_640X480;
uint8_t frameBuffer[RES_640X480] = { 0 };
#endif

#ifdef RES800x600
enum imageResolution imgRes=RES_800x600;
uint8_t frameBuffer[RES_800x600] = { 0 };
#endif

#ifdef RES1024x768
enum imageResolution imgRes=RES_1024x768;
uint8_t frameBuffer[RES_1024x768] = { 0 };
#endif

#ifdef RES1280x960
enum imageResolution imgRes = RES_1280x960;
uint8_t frameBuffer[RES_1280x960] = { 0 };
#endif
*/

uint16_t bufferPointer = 0;
short headerFound = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RNG_Init(void);
static void MX_DCMI_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static uint32_t get_precision_tick();
static void precision_sleep_until(uint32_t target_ticks);
uint8_t get_random_number(uint32_t *random_number, uint16_t timeout);
static uint8_t random_int(uint8_t max);
void handle_rfm95_status(uint16_t rfm95_status);
static uint8_t get_battery_level();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vprint(const char *fmt, va_list argp) {
	char string[200];
	if (0 < vsprintf(string, fmt, argp)) // build string
			{
		HAL_UART_Transmit(&huart3, (uint8_t*) string, strlen(string), 0xffffff); // send message via UART
	}
}

void my_printf(const char *fmt, ...) // custom printf() function
{
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

//Funzione per dividere l'array di dati della foto in pacchetti e inviarli
void send_array_in_packets(const uint8_t *input_array, size_t array_length, size_t packet_length)
{
	size_t offset = 0; //offset per tenere traccia ei dati già trasmessi
	uint8_t t = 0; //conteggio pacchetti inviati

	while (offset < array_length)
	{
    	size_t bytes_remaining = array_length - offset; //byte rimanenti
    	current_packet_size = (bytes_remaining < packet_length) ? bytes_remaining : packet_length;  //calcolo lunghezza del pacchetto attuale
    	memset(packet, 0, current_packet_size);  // Pulizia buffer (opzionale)
    	memcpy(packet, &input_array[offset], current_packet_size);  //creazione pacchetto da inviare
    	offset += current_packet_size; //nuovo offset per i prossimi pacchetti
    	rfm95_status = rfm95_send_receive_cycle(&rfm95_handle, packet, current_packet_size, true);  //invio pacchetto

    	handle_rfm95_status(rfm95_status);
    	printf("%.i\n\r", t++); //numero pacchetto
    	HAL_Delay(36521); //delay calcolato sulla base di pacchetti da 220 byte e SF7
	}
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
  MX_DMA_Init();
  MX_RNG_Init();
  MX_DCMI_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_LPTIM1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_LPTIM_Counter_Start_IT(&hlptim1, 0xFFFF); // start in modalità interrupt
  HAL_Delay(1000);

  /**************************************************
  **************************************************
  *
  *       IMPOSTAZIONI MODULO LoRa:
  *
  **************************************************
  ***************************************************/

  // SET HARDWARE CONNECTIONS:
  rfm95_handle.spi_handle = &hspi1;
  rfm95_handle.nss_port   = NSS_GPIO_Port;
  rfm95_handle.nss_pin    = NSS_Pin;
  rfm95_handle.nrst_port  = RESET_RFM95_GPIO_Port;
  rfm95_handle.nrst_pin   = RESET_RFM95_Pin;

  // SET TX POWER, SF, RX2-SF:
  rfm95_handle.config.tx_sf = 7;
  rfm95_handle.config.tx_power = 14;
  rfm95_handle.config.rx2_sf = 9;

  // Set application data magic word:
  rfm95_handle.config.lora_magic = 0xABCD;
  // INIT FIELDS FOR RX MODE:
  rfm95_handle.precision_tick_frequency = lse_clk;
  rfm95_handle.precision_tick_drift_ns_per_s = 20000;
  rfm95_handle.receive_mode = RFM95_RECEIVE_MODE_RX12;

  rfm95_handle.get_precision_tick = get_precision_tick;
  rfm95_handle.precision_sleep_until = precision_sleep_until;
  rfm95_handle.random_int = random_int;
  rfm95_handle.get_battery_level = get_battery_level;

  // SETUP device address, network and application session keys:  impostazione chiavi device
  memcpy(rfm95_handle.device_address, dev_addr, sizeof(dev_addr));
  memcpy(rfm95_handle.network_session_key, nw_session_keys, sizeof(nw_session_keys));
  memcpy(rfm95_handle.application_session_key, app_session_keys, sizeof(app_session_keys));

  rfm95_status = rfm95_init(&rfm95_handle); //controllo inizializzazione modulo LoRa
  printf("Connesso\r\n");
  // Initialize RFM95 module.
  if ((rfm95_status & RFM95_STATUS_OK) == 0) {
      printf("RFM95 init failed\r\n");
      while(1);
  }

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /**************************************************
  **************************************************
  *
  *       OV2640 Initialization:
  *
  **************************************************
  *************************************************/

  OV2640_Init(&hi2c2, &hdcmi);
  HAL_Delay(10);
  OV2640_ResolutionOptions(imgRes);
  HAL_Delay(10);

  #ifdef DEBUG
  	my_printf("Finishing configuration \r\n");
  #endif

  //DEFINIZIONE VARIABILI PER LA STANDBY MODE
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

  HAL_Delay(5000);

  memset(frameBuffer, 0, sizeof frameBuffer);
  OV2640_CaptureSnapshot((uint32_t)frameBuffer, imgRes);//scatto foto

  //Una volta scattata la foto e salvato l'array di dati nel frameBuffer, si cerca l'inizio e la fine della foto in modo da
  //sapere la grandezza dell'immagine
  while (1) {
      	if (headerFound == 0 && frameBuffer[bufferPointer] == 0xFF
        		&& frameBuffer[bufferPointer + 1] == 0xD8) {
  	     			headerFound = 1;
  	      			#ifdef DEBUG
  	        			my_printf("Found header of JPEG file \r\n");
  	      			#endif
  	   	}
  	   	if (headerFound == 1 && frameBuffer[bufferPointer] == 0xFF && frameBuffer[bufferPointer + 1] == 0xD9) 
		{
  	    	bufferPointer = bufferPointer + 2;
  	    		#ifdef DEBUG
  	    			my_printf("Found EOF of JPEG file \r\n");
  	    		#endif
  	    		headerFound = 0;
  	    		break;
  	    }
 	    if (bufferPointer >= 65535) {
  	    	break;
  	    }
  	    bufferPointer++;
  }
  #ifdef DEBUG
  	  my_printf("Image size: %d bytes \r\n", bufferPointer);
  	  my_printf("Image data (hex):\r\n");
	  //convertiamo i dati dell'array in esadecimale
  	  for (int i = 0; i < bufferPointer; i++) 
	  {
  	  	  my_printf("%02X ", frameBuffer[i]);
  	  }
  	  my_printf("\r\n");
  #endif

  //HAL_UART_Transmit_DMA(&huart3, frameBuffer, bufferPointer); //Use of DMA may be necessary for larger data streams.
  send_array_in_packets(frameBuffer, sizeof(frameBuffer), PACKET_SIZE);
  send_array_in_packets(frameBuffer, bufferPointer, PACKET_SIZE);
  printf("About to enter the STANDBY MODE\r\n");
  printf("STANDBY MODE\r\n");
  HAL_PWR_EnterSTANDBYMode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

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
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  huart3.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RESET_RFM95_Pin|NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERA_PWDN_GPIO_Port, CAMERA_PWDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DIO5_Pin */
  GPIO_InitStruct.Pin = DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_RFM95_Pin */
  GPIO_InitStruct.Pin = RESET_RFM95_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_RFM95_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO1_Pin DIO0_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAMERA_RESET_Pin */
  GPIO_InitStruct.Pin = CAMERA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAMERA_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAMERA_PWDN_Pin */
  GPIO_InitStruct.Pin = CAMERA_PWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAMERA_PWDN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**************************************************
 **************************************************
 *
 *         FUNZIONI RFM95:
 *
 **************************************************
 **************************************************
 */
static uint32_t get_precision_tick() {
	    __disable_irq();
	    uint32_t precision_tick = lptim_tick_msb | HAL_LPTIM_ReadCounter(&hlptim1);
	    __enable_irq();
	    return precision_tick;
}

static void precision_sleep_until(uint32_t target_ticks) {
    while (1) {
        uint32_t curr_tick = get_precision_tick();
        if (target_ticks <= curr_tick) break;

        uint32_t sleep_ticks = target_ticks - curr_tick;

        if (sleep_ticks < 10) break;

        uint32_t delay_ms = (sleep_ticks * 1000) / 32786; // ≈ tick duration in ms
        HAL_Delay(delay_ms);
    }

    // Attesa attiva per ultimi tick
    while (get_precision_tick() < target_ticks);
}
/*
static void precision_sleep_until(uint32_t target_ticks){


  while(1){
    uint32_t curr_tick = get_precision_tick();

    // exit condition:
    if(target_ticks < curr_tick) break;

    uint32_t sleep_ticks = target_ticks - curr_tick;

    // avoid short sleep intervals:
    if(sleep_ticks < 10) break;
    else {
      // overall CMP value - some margin (needed to reset clock configurations after stop mode2)
      uint32_t compare_tick = (curr_tick & 0xFFFF) + sleep_ticks - 15;

    }
  }

  while(get_precision_tick() < target_ticks);                       // wait residue time here ( < 10 ticks)
}
*/
uint8_t get_random_number(uint32_t *random_number, uint16_t timeout){ //numero casuale generato per crare il canale

  // enable RNG peripheral:
  __HAL_RNG_ENABLE(&hrng);

  // clear clock error and seed error interrupt flags:
  __HAL_RNG_CLEAR_IT(&hrng, RNG_IT_CEI);
  __HAL_RNG_CLEAR_IT(&hrng, RNG_IT_SEI);

  // wait for data ready bit to be set:
  uint32_t start = HAL_GetTick();
  while(!__HAL_RNG_GET_FLAG(&hrng, RNG_FLAG_DRDY)){
    if (HAL_GetTick() - start > timeout) {
      __HAL_RNG_DISABLE(&hrng);
      return 1;
    }
  }

  // RNG can be switched off here:
  __HAL_RNG_DISABLE(&hrng);

  // check seed error:
  if(__HAL_RNG_GET_IT(&hrng, RNG_IT_SEI)) return 2;

  // check clock error:
  if(__HAL_RNG_GET_IT(&hrng, RNG_IT_CEI)) return 3;

  *random_number = hrng.Instance->DR;

  // check event seed error occurred while loading data:
  if(*random_number == 0) return 2;

  return 0;
}

static uint8_t random_int(uint8_t max) {
  uint32_t num;
  uint8_t rng_err = get_random_number(&num, 1);

  if (rng_err){
    // TODO handle potential rng seed error/clock error here...

    num = 0;
  }

  return num % (max + 1);
}

static uint8_t get_battery_level() {
    return 0xff; // 0xff = Unknown battery level.
}


void handle_rfm95_status(uint16_t rfm95_status) {  //lista delle possibili risposte del modulo LoRa
  printf("\n### STATUS FLAGS ###\r\n");

  // Check for critical errors first
  if (rfm95_status & RFM95_STATUS_ERROR) {
    printf("General error occurred during the operation.\r\n");
  }

  if (rfm95_status & RFM95_STATUS_SPI_ERROR) {
    printf("SPI communication error occurred.\r\n");
  }

  if (rfm95_status & RFM95_STATUS_IRQ_TIMEOUT) {
    printf("Timeout waiting for IRQ.\r\n");
  }

  if (rfm95_status & RFM95_STATUS_RX_CRC_ERR) {
    printf("CRC error in the received data.\r\n");
  }

  if (rfm95_status & RFM95_STATUS_MAC_CMD_ERR) {
    printf("Error processing MAC commands. (Wrong software implementation?)\r\n");
  }

  if (rfm95_status & RFM95_STATUS_DECODE_ERR) {
    printf("Error decoding payload.\r\n");
  }

  if (rfm95_status & RFM95_STATUS_ENCODE_ERR) {
    printf("Error encoding payload.\r\n");
  }

  // Check for non-critical flags
  if (rfm95_status & RFM95_STATUS_MAC_COMMANDS_PENDING) {
    printf("The packet was blocked to send pending MAC command answers.\r\n");
  }


  if (rfm95_status & RFM95_STATUS_RX_EMPTY) {
    printf("No data was received in the downlink.\r\n");
  }

  if (rfm95_status & RFM95_STATUS_RX_MAGIC_OK) {
    printf("Received app. data with valid magic.\r\n");
  }

  if (rfm95_status & RFM95_STATUS_RX_MAGIC_ERR) {
    printf("Received app. data with invalid magic.\r\n");
  }

  if (rfm95_status & RFM95_STATUS_ACK_RECEIVED) {
    printf("ACK received during downlink.\r\n");
  }
  else {
	rfm95_status = rfm95_send_receive_cycle(&rfm95_handle, packet, current_packet_size, true);  //invio pacchetto
	handle_rfm95_status(rfm95_status);
	HAL_Delay(5000);
  }

  // If no error flags are set, the operation was successful
  if (rfm95_status & RFM95_STATUS_OK) {
    printf("Operation completed successfully.\r\n");
  }

  // Catch-all case: If no flags are set
  if (rfm95_status == 0) {
    printf("Unknown status: no flags are set.\r\n");
  }
}

/**************************************************
 **************************************************
 *
 *         CALLBACKS IMPLEMENTED HERE:
 *
 **************************************************
 **************************************************
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){  //gestione interrupt di modulo e bottone
  if (GPIO_Pin == DIO0_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO0);
  } else if (GPIO_Pin == DIO1_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO1);
  } else if (GPIO_Pin == DIO5_Pin) {
    rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO5);
  }

}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{

        lptim_tick_msb += 0x10000;
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{

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





