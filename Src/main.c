/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"
#include "i2c_simulate.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "usbd_uvc.h"
#include "usbd_uvc_if.h"
#include "colormapping.h"
#include "tasks.h"
#include "constants.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


//uint16_t gHeader[] = {65280, 255};	//{0xFF00,0x00FF};
//uint16_t gFooter[] = {4080};					//{0x0FF0}

//ULIS_buffer gUlisFrameBuffer[4];


uint16_t gDataBufferTx[IMGSIZE] = {0};
uint16_t gDataBufferRx[IMGSIZE] = {0};

uint16_t *pDataBufferTx = NULL;
uint16_t gDataBufferBadLineDeted[IMGSIZE];
uint16_t gDataBufferComplete[IMGSIZE];
uint8_t gIndexCounter = 0;
uint8_t gSPITrigger = 1;
uint8_t gRGBBuf[FRAME_LINE_LENGTH * IMAGE_NUM_LINES * 3] = {128};
int gVigilUID1 = 0;
int gVigilUID2 = 0;
int gVigilUID3 = 0;

extern volatile uint8_t g_uvc_stream_status;
extern bool detectingBadFrame;


uint8_t rx_buffer_vigil[BUFFER_SIZE]={0}; 
volatile uint8_t recv_end_flag_vigil = 0; 
volatile uint8_t rx_len_vigil = 0;



uint8_t i2c_buffer[sizeof(float) + 1]; // add crc check byte
uint16_t adc_value[ADC_NUMBER] = {0};
static float resistance,Voltage;
const float Vref = 2.78;
volatile uint8_t I2cCheckFlag = 0;

uint16_t *framebufptr = NULL;
uint8_t g_BadLine = 0;
bool g_bCameraReset = false;

bool spi_tx_pi_Cplt = false;
bool g_bAbortFirstFrameAfterReboot = false;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;


I2C_HandleTypeDef hi2c1;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
/* USER CODE BEGIN PV */

static struct pt uvc_task_pt;
static struct pt spi_task_pt;
static struct pt detectbadline_task_pt;


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spi_tx_pi_Cplt = true;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void I2C_reset();
void ADC_Start();
void SetI2CAsSlave();
void DetectBadFrameInit();
void CheckThermalReady();

extern void DetectBadLine(uint16_t *frameBuffer,int type);
extern void DetectBadPixel(uint16_t *frameBuffer);
extern void FixBadLine(uint8_t* lineMap,uint16_t* frameBuffer);
extern void FixBadPixel(uint16_t *frameBuffer); 


//extern uint8_t lineMap[80];
extern uint8_t lineMapWhite[80];
extern uint8_t lineMapBlack[80];

//uint16_t getNeighborAvg(const int x, const int y, uint16_t *frameBuffer, uint8_t *pDiffMap);
//bool getAvailableNeighbor(const int x, const int y, const int incX, const int incY, uint16_t *frameBuffer, uint8_t *pDiffMap,uint16_t *value);
uint8_t CaculateCRC(unsigned char *ptr, unsigned char len);

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


  MX_USB_DEVICE_Init();
	
	
  MX_DMA_Init();
  
	
	MX_I2C1_Init();
  MX_USART1_UART_Init();
  //MX_USART2_UART_Init();
	
	MX_ADC1_Init();
  MX_TIM2_Init();
	MX_TIM3_Init();																					  // every 10ms interrupt
	
	MX_SPI1_Init();
  MX_SPI2_Init();
	
	SetI2CAsSlave();
	ADC_Start();
	
	
  /* USER CODE BEGIN 2 */
	uint16_t i;
	
	gDataBufferTx[0] = 0xFF00;
	gDataBufferTx[1] = 0x00FF;
	gDataBufferRx[0] = 0xFF00;
	gDataBufferRx[1] = 0x00FF;

//	for (i = 2; i < IMGSIZE - 1; i++)
//	{
//		gDataBufferTx[i] = i - 2;
//		gDataBufferRx[i] = i - 2;
//	}

	uint16_t temp;
	
	
	gDataBufferTx[IMGSIZE - 1] = 0x0FF0;
	gDataBufferRx[IMGSIZE - 1] = 0x0FF0;
	gDataBufferTx[IMGSIZE - 2] = '2';														
	gDataBufferTx[IMGSIZE - 3] = '.';
	gDataBufferTx[IMGSIZE - 4] = '0';
	gDataBufferTx[IMGSIZE - 5] = '.';
	gDataBufferTx[IMGSIZE - 6] = '1';														// version header "1".0.2
	
	for(int i = 2; i < 7; i++)
	{
		gDataBufferRx[IMGSIZE - i] = gDataBufferTx[IMGSIZE - i];
	}

	int iDX = 0;
	
	
	//execute uart command here
	uint8_t aCmdD[] = "D";
	
	
	if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)aCmdD, 1)== HAL_OK)
  {
		uint32_t tickstart = 0U;
		tickstart = HAL_GetTick();
		
    while(recv_end_flag_vigil != 1 && ((HAL_GetTick() - tickstart) < 3000))
		{
		}
		if (recv_end_flag_vigil == 1)
		{
			char szIDSegment[5] = {0};
			szIDSegment[0] = rx_buffer_vigil[2];
			szIDSegment[1] = rx_buffer_vigil[3];
			szIDSegment[2] = rx_buffer_vigil[4];
			szIDSegment[3] = rx_buffer_vigil[5];
			gVigilUID1 = atoi(szIDSegment);
			
			szIDSegment[0] = 0;
			szIDSegment[1] = 0;
			szIDSegment[2] = 0;
			szIDSegment[3] = 0;
			
			szIDSegment[0] = rx_buffer_vigil[7];
			szIDSegment[1] = rx_buffer_vigil[8];
			gVigilUID2 = atoi(szIDSegment);
			
			szIDSegment[0] = 0;
			szIDSegment[1] = 0;	
			
			szIDSegment[0] = rx_buffer_vigil[10];
			szIDSegment[1] = rx_buffer_vigil[11];
			szIDSegment[2] = rx_buffer_vigil[12];
			
			gVigilUID3 = atoi(szIDSegment);			
		}
  }
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	CheckThermalReady();
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	DetectBadFrameInit();
	
	
	//memcpy(gDataBufferTxComplete,gDataBufferComplete,IMGSIZE * sizeof(uint16_t));
	
	gDataBufferComplete[ IMGSIZE - 6] = gDataBufferComplete[ IMGSIZE - 1];
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)gDataBufferComplete, IMGSIZE - 5);
	
	gDataBufferComplete[ IMGSIZE - 6] = gDataBufferComplete[ IMGSIZE - 1];
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)gDataBufferComplete, IMGSIZE - 5);

	PT_INIT(&uvc_task_pt);
	PT_INIT(&spi_task_pt);
	PT_INIT(&detectbadline_task_pt);
	
	while(1)
	{	
		PT_SCHEDULE(uvc_task(&uvc_task_pt));
		PT_SCHEDULE(DetectBadLine_task(&detectbadline_task_pt));
		PT_SCHEDULE(SendToPi_task(&spi_task_pt));
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;                                   // normal setting 
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	/*hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;                                   // for m5s setting 
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;*/
	
	
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,rx_buffer_vigil,BUFFER_SIZE);
  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	/* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Vigil_VideoIN_Pin */
  GPIO_InitStruct.Pin = Vigil_VideoIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Vigil_VideoIN_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*GPIO_InitStruct.Pin = SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);*/
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief I2C1 Initialization Function
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  //sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.OwnAddress1 = 170;
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
  //htim2.Init.Prescaler = 47999;
	htim2.Init.Prescaler = 1400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
	//htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  //HAL_TIM_Base_Start_IT(&htim2);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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





void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	static uint32_t BusyFlag;
	
	do{

		BusyFlag = (hi2c->Instance->SR2 & 0x02);
	}while(BusyFlag);
	
	I2C_reset();

	
	/*if((hi2c->Instance->SR2 &= 0x02))
			hi2c->Instance->CR1 |= I2C_CR1_SWRST;*/
	I2cCheckFlag = 0;
	HAL_I2C_Slave_Transmit_IT(&hi2c1,i2c_buffer,5);
	
}


void I2C_reset()
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	 GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	 GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	 GPIO_InitStruct.Pull = GPIO_MODE_OUTPUT_OD;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	 GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

	 for (uint8_t i = 0; i < 10; i++)
	 {

		 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
	   {
			break;
		 }
	    
	 }

	 GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	 hi2c1.Instance->CR1 |= I2C_CR1_SWRST;
	 hi2c1.Instance->CR1 &= ~I2C_CR1_SWRST;

	 MX_I2C1_Init();
}

static void ADC_Start()
{
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_value,ADC_NUMBER);
	HAL_TIM_Base_Start(&htim2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	static float ADC_Value;
	float value_f = 0;
	uint8_t CRC_Result;
	int temp,number,ADC_Step;
	for(int i = 0; i < ADC_NUMBER; i++)
	{
		value_f += adc_value[i];
	}
	value_f /= ADC_NUMBER;

	ADC_Step = value_f+0.5;

	Voltage = Vref/4096 * ADC_Step;
	resistance = ( 10 * Voltage ) / ( Vref - Voltage);

	memcpy(i2c_buffer,&resistance,sizeof(float));
	
	CRC_Result = CaculateCRC(i2c_buffer,4);
	i2c_buffer[4] = CRC_Result;
	

	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_value,ADC_NUMBER);
	
}

void SetI2CAsSlave()
{
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

	
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_I2C_Slave_Transmit_IT(&hi2c1,i2c_buffer,5);
}



void delay_us(uint32_t udelay)
{
	uint32_t temp;
	SysTick->LOAD = 9*udelay;
	SysTick->VAL = 0x00;
	SysTick->CTRL = 0x01;
	do{
		temp = SysTick->CTRL;
	}while((temp&0x01) && (!(temp&(1<<16))));
	SysTick->CTRL = 0x00;
	SysTick->VAL = 0x00;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static int cnt = 0;
	
	if(GPIO_Pin == GPIO_PIN_8)
	{
		HAL_StatusTypeDef status;
		uint16_t *pBuf = (uint16_t*) gDataBufferRx;
		uint16_t *pBufShif = pBuf + 2;
	
		status = HAL_SPI_Receive_DMA( &hspi2, (uint8_t*)(pBufShif), (IMGSIZE - 8));

			
			
		
	}
}





/*static void DetectBadLine(uint16_t *frameBuffer)
{
	for(int i = 0; i < 80; i++)
	{
		int line = i * 80;
		for( int j = 1; j < 80 - 1; j++)
		{
			if((frameBuffer[line + j - 1] - frameBuffer[line + j]) > 30 && (frameBuffer[line + j + 1] - frameBuffer[line + j]) > 30)
			{
				m_pFrameStraightLineMap[line + j] = (m_pFrameStraightLineMap[line + j] >= BLIND_ACC_COUNT * 1.5) ? BLIND_ACC_COUNT * 1.5 : ++m_pFrameStraightLineMap[line + j];
			}
			else
				m_pFrameStraightLineMap[line + j] = (m_pFrameStraightLineMap[line + j] > 0) ? --m_pFrameStraightLineMap[line + j] : 0;
		}
	}
	for(int j = 1; j < 80 - 1; j++)
	{
		int iAcc = 0;
		for(int i = 0; i < 80; i++)
		{
			if(m_pFrameStraightLineMap[i * 80 +j] >= BLIND_ACC_COUNT)
			{
				iAcc++;
			}
		}
			
		if(iAcc > 80 * 0.4)
		{
			for(int i = 0; i < 80; i++)
			{
				frameBuffer[i * 80 + j] = (frameBuffer[i * 80 + j -1] + frameBuffer[i *80 + j + 1]) / 2;
			}
		}
	}
}*/


/*static void DetectBadLine(uint16_t *frameBuffer)
{
	for(int i = 0; i < 80; i++)
	{
		int straightline = i * 80;
		for( int j = 1; j < 80 - 1; j++)
		{
			if((frameBuffer[straightline + j - 1] - frameBuffer[straightline + j]) > 30 && (frameBuffer[straightline + j + 1] - frameBuffer[straightline + j]) > 30)
			{
				m_pFrameStraightLineMap[straightline + j] = (m_pFrameStraightLineMap[straightline + j] >= BLIND_ACC_COUNT * 1.5) ? BLIND_ACC_COUNT * 1.5 : ++m_pFrameStraightLineMap[straightline + j];
			}
			else
				m_pFrameStraightLineMap[straightline + j] = (m_pFrameStraightLineMap[straightline + j] > 0) ? --m_pFrameStraightLineMap[straightline + j] : 0;
		}
	}
	

	for(int i = 1; i < 80 - 1 ; i++)
	{
		int horizontalline = i;
		for(int j = 0; j < 80; j++)
		{
			if(((frameBuffer[(horizontalline - 1) + (80 * j)] - frameBuffer[horizontalline + (80 * j)]) > 30) && (frameBuffer[(horizontalline + 1) + (80 * j)] - frameBuffer[horizontalline + (80 * j)]) > 30 )
			{
				m_pFrameHorizontalLineMap[horizontalline + (j * 80)] = (m_pFrameHorizontalLineMap[horizontalline + (j * 80)] >= BLIND_ACC_COUNT * 1.5) ? BLIND_ACC_COUNT * 1.5 : ++m_pFrameHorizontalLineMap[horizontalline + (j * 80)];
			}
			else
				m_pFrameHorizontalLineMap[horizontalline + (j * 80)] = (m_pFrameHorizontalLineMap[horizontalline + (j * 80)] > 0) ? --m_pFrameHorizontalLineMap[horizontalline + (j *80)] : 0;
		}
	}
	
	for(int i = 0; i < 80 ; i++)
	{
		int iAcc = 0;
		for(int j = 1; j < 80 - 1; j++)
		{
			if(m_pFrameHorizontalLineMap[j*80 + i] >= BLIND_ACC_COUNT)
			{
				iAcc++;
			}
		}
		
		if(iAcc > 80 * 0.4)
		{
			for(int j = 1; j < 80 - 1 ; j++)
			{
				frameBuffer[j * 80 + i] = (frameBuffer[j * 80 + i -1] + frameBuffer[j * 80 +i + 1]) / 2;
			}
			
		}
	}
	
	
	for(int j = 1; j < 80 - 1; j++)
	{
		int iAcc = 0;
		for(int i = 0; i < 80; i++)
		{
			if(m_pFrameStraightLineMap[i * 80 +j] >= BLIND_ACC_COUNT)
			{
				iAcc++;
			}
		}
			
		if(iAcc > 80 * 0.4)
		{
			for(int i = 0; i < 80; i++)
			{
				frameBuffer[i * 80 + j] = (frameBuffer[i * 80 + j -1] + frameBuffer[i *80 + j + 1]) / 2;
			}
		}
	}
}*/



bool b_ReveiveData;
	
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	b_ReveiveData = true;
	
		memcpy(gDataBufferTx, gDataBufferRx, IMGSIZE * sizeof(uint16_t));

		if(g_BadLine == White)
		{
			DetectBadLine(&gDataBufferTx[2],White);
			DetectBadPixel(&gDataBufferTx[2]);
		}
		else if(g_BadLine == DynamicDetectBadLine)
		{
			memcpy(gDataBufferComplete, gDataBufferTx, IMGSIZE * sizeof(uint16_t));
			FixBadLine(lineMapWhite,&gDataBufferComplete[2]);
			FixBadPixel(&gDataBufferComplete[2]);
			
		}
	
	b_ReveiveData = false;
}

void DetectBadFrameInit()
{
	g_BadLine = WhiteLineAllSection;
	HAL_Delay(1000);
	g_BadLine = DynamicDetectBadLine;
}

uint8_t CaculateCRC(unsigned char *ptr, unsigned char len)
{
    unsigned char  crc = 0x00;

    while (len--)
    {
        crc = crc_table[crc ^ *ptr++];
    }
    return (crc);
}



void CheckThermalReady()
{
	while(gDataBufferRx[10] >= 0x8000 || gDataBufferRx[10] == 0)
	{
		asm("nop");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
