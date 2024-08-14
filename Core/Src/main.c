/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwgps/lwgps.h"
#include "w25q.h"
#include <math.h>
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORA_RX_BUFFER_SIZE 75
#define RX_BUFFER_SIZE 128
#define HYI_BUFFER_SIZE 78

#define TAKIM_ID 31
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
uint8_t lora_rx_buffer[LORA_RX_BUFFER_SIZE];
uint8_t RS_rx_buffer[LORA_RX_BUFFER_SIZE];
uint8_t rx_index_lora=0;
uint8_t rx_data_lora=0;

uint8_t HYI_BUFFER[HYI_BUFFER_SIZE];

uint8_t gps_rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index=0;
uint8_t rx_data=0;

uint8_t Cmd_End[3] = {0xff,0xff,0xff};


uint8_t nextion_rx_data[5];

const double PI = 4.0*atan(1.0);
const double radius_of_earth = 6378136.0474;
double _distance=0;
double _angle=0;

///////////////sustainer
uint8_t sustgpssatsinview=0;
float sustgpsaltitude=0;
float sustgpslatitude=0;
float sustgpslongitude=0;
float sustspeed=0;
float sustaltitude=0;
float susttemperature=0;
float sustaccx=0;
float sustaccy=0;
float sustaccz=0;
float sustroll=0;
float sustpitch=0;
uint8_t sustv4_battery=0;
uint8_t sustv4_mod=0;
uint8_t suststage_communication=0;

float s_distance=0;

//egu
uint8_t EGU_ARIZA=0;
uint8_t EGU_AYRILMA_TESPIT=0;
uint8_t EGU_MOTOR_ATESLEME_TALEP_IN=0;
uint8_t EGU_STAGE_DURUM=0;
uint8_t EGU_UCUS_BASLADIMI=0;
uint8_t EGU_FITIL =0;

float EGU_BATTERY=0;
float EGU_IRTIFA=0;
float EGU_ANGLE=0;

////////////BOOOSTER

uint8_t boostgpssatsinview=0;
float boostgpsaltitude=0;
float boostgpslatitude=0;
float boostgpslongitude=0;
float boostspeed=0;
float boostaltitude=0;
float boosttemperature=0;
float boostaccx=0;
float boostaccy=0;
float boostaccz=0;
float boostroll=0;
float boostpitch=0;
uint8_t boostv4_battery=0;
uint8_t boostv4_mod=0;
uint8_t booststage_communication=0;


float bs_distance=0;

typedef struct
{	uint8_t satsinview;
	float gpsaltitude;
	float gpslatitude;
	float gpslongitude;
	float speed;
	float altitude;
	float temperature;
	float accx;
	float accy;
	float accz;
	float normal;
	float pitch;
	float maxAltitude;
	uint8_t battery;
	uint8_t mod;
	uint8_t communication;

}dataTypeDef;

dataTypeDef Payload, Booster , Sustainer;


///////////////////////////////////////////////////booster ekran
uint8_t b_altitude[8];
uint8_t b_temperature[5];
uint8_t b_speed[5];
uint8_t b_roll[5];
uint8_t b_pitch[5];
uint8_t b_latitude[9];
uint8_t b_longitude[9];
uint8_t b_bat[2];
uint8_t b_sats[2];
uint8_t b_comm[2];
uint8_t b_dist[6];

uint8_t enum_bs[9];
uint8_t enum_s[9];

char s_altitude[8];
uint8_t s_temperature[5];
uint8_t s_speed[5];
uint8_t s_roll[5];
uint8_t s_pitch[5];
uint8_t s_latitude[9];
uint8_t s_longitude[9];
uint8_t s_bat[2];
uint8_t s_sats[2];
uint8_t s_comm[2];
uint8_t s_dist[6];

///////////////////////////////////////////////////
uint8_t seconds[2];
uint8_t minutes[2];
uint8_t hours[2];
/////////////////////////////////////////////////
uint8_t p_latitude[9];
uint8_t p_longitude[9];
uint8_t p_altitude[7];
uint8_t p_gpsaltitude[7];
uint8_t p_bat[2];
////////////////////////////////////////////////
uint8_t e_altitude[7];
uint8_t e_bat[2];
uint8_t e_angle[5];
uint8_t e_flight[2];
uint8_t e_stage[2];
uint8_t e_fitil[5];
uint8_t e_engine_request[3];



uint8_t test_index=0;

float adc;
float adc_pil_val;
uint8_t adc_flag;

uint8_t st_bat[2];

uint32_t tim1=0;
uint32_t takim_sayac;


uint8_t BUTTON_STATE=0;

uint8_t writeData[50] = {0,1,1,1};
uint8_t readData[50] = {0};

uint32_t crc;
uint8_t chs;
lwgps_t gps;

typedef union{
  float fVal;
  unsigned char array[4];
}float2unit8;

int counthalf=0,countfull=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void E220_CONFIG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE);
double distance_in_m(double lat1, double long1, double lat2, double long2);
double toRadians(double degree);
double calculateAngle(double lat1, double lon1, double lat2, double lon2);
void NEXTION_SendString (char *ID, char *string);
void NEXTION_SendNum (char *obj, int32_t num);
void NEXTION_SendFloat (char *obj, float num, int dp);
void HYI_BUFFER_Fill(void);
void Payload_union_converter(void);
void Enum_State_bs(void);
void Enum_State_s(void);
void Nextion_SendCommand(char* command);
void Nextion_SendFloatToTextbox(char* textbox_id, float value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
//{
//	counthalf++;
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart == &huart3){
//	if(rx_data_lora != '\n'&& rx_index_lora < LORA_RX_BUFFER_SIZE){
//	lora_rx_buffer[rx_index_lora++]=rx_data_lora;
//
//	}
//	else{
//		rx_data_lora=0;
//		rx_index_lora=0;
//
//		}
//	HAL_UART_Receive_IT(&huart3, &rx_data_lora, 1);
//	}
//
	if(huart == &huart3){
		countfull++;
		HAL_UART_Receive_DMA(&huart3, lora_rx_buffer, 75);
	}



	if(huart == &huart2) {
			if( rx_data != '\n'&& rx_index < RX_BUFFER_SIZE) {
				gps_rx_buffer[rx_index++] = rx_data;
			} else {
				lwgps_process(&gps, gps_rx_buffer, rx_index+1);
				rx_index = 0;
				rx_data = 0;
			}
			HAL_UART_Receive_IT(&huart2, &rx_data, 1);
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim==&htim11){ // 1 sn
   adc_flag=1;

	}


}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1 )
	{
		adc= HAL_ADC_GetValue(&hadc1);


		//adc_flag = 1;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM11_Init();
  MX_USB_DEVICE_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */


 // HAL_UART_Receive_IT(&huart3, &rx_data_lora, 1);
  HAL_UART_Receive_DMA(&huart3, lora_rx_buffer, 75);


  HAL_UART_Receive_IT(&huart2,&rx_data, 1);
  E220_CONFIG(0x7,0x2B,0x12,1); //0x8,0x2A,0x10,1
  lwgps_init(&gps);

  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start_IT(&htim11);

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);

  tim1=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//  HAL_UART_Receive(&huart4, nextion_rx_data, 5 , 1000);




	   if(lora_rx_buffer[3]==1 && lora_rx_buffer[50]==0x32){

		  Booster.satsinview=lora_rx_buffer[4];

			 float2unit8 f2u8_bgpsalt;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_bgpsalt.array[i]=lora_rx_buffer[i+5];
					 HYI_BUFFER[34+i]=lora_rx_buffer[i+5]; // 34 35 36 37
				 }
				 Booster.gpsaltitude=f2u8_bgpsalt.fVal;
			 float2unit8 f2u8_blatitude;

				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_blatitude.array[i]=lora_rx_buffer[i+9];
					 HYI_BUFFER[38+i]=lora_rx_buffer[i+9]; // 38 39 40 41
				 }
				 Booster.gpslatitude=f2u8_blatitude.fVal;

			 float2unit8 f2u8_blongitude;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_blongitude.array[i]=lora_rx_buffer[i+13];
					 HYI_BUFFER[42+i]=lora_rx_buffer[i+13]; // 42 43 44 45
				 }
				 Booster.gpslongitude=f2u8_blongitude.fVal;

			 float2unit8 f2u8_baltitude;
				 for(uint8_t i=0;i<4;i++)
				 {
					f2u8_baltitude.array[i]=lora_rx_buffer[i+17];
				 }
				 Booster.altitude=f2u8_baltitude.fVal;

			 float2unit8 f2u8_bspeed;

				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_bspeed.array[i]=lora_rx_buffer[i+21];
				 }
				 Booster.speed=f2u8_bspeed.fVal;

			 float2unit8 f2u8_btemp;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_btemp.array[i]=lora_rx_buffer[i+25];
				 }
				 Booster.temperature=f2u8_btemp.fVal;

			 float2unit8 f2u8_baccx;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_baccx.array[i]=lora_rx_buffer[i+29];
				 }
				 Booster.accx=f2u8_baccx.fVal;

			float2unit8 f2u8_baccy;
				 for(uint8_t i=0;i<4;i++)
				 {
					 f2u8_baccy.array[i]=lora_rx_buffer[i+33];
				 }
				 Booster.accy=f2u8_baccy.fVal;

			float2unit8 f2u8_baccz;
			      for(uint8_t i=0;i<4;i++)
				 {
			    	  f2u8_baccz.array[i]=lora_rx_buffer[i+37];
				 }
			      Booster.accz=f2u8_baccz.fVal;

			float2unit8 f2u8_broll;
				  for(uint8_t i=0;i<4;i++)
				 {
					  f2u8_broll.array[i]=lora_rx_buffer[i+41];
				 }
				  Booster.normal=f2u8_broll.fVal;

			float2unit8 f2u8_bpitch;
				  for(uint8_t i=0;i<4;i++)
				 {
					  f2u8_bpitch.array[i]=lora_rx_buffer[i+45];
				 }
				  Booster.pitch=f2u8_bpitch.fVal;

				  Booster.battery=lora_rx_buffer[49];
				  Booster.mod=lora_rx_buffer[73];
				  Booster.communication=lora_rx_buffer[51];

				  f2u8_baltitude.array[0] = lora_rx_buffer[69];
				  f2u8_baltitude.array[1] = lora_rx_buffer[70];
				  f2u8_baltitude.array[2] = lora_rx_buffer[71];
				  f2u8_baltitude.array[3] = lora_rx_buffer[72];
				  Booster.maxAltitude = f2u8_baltitude.fVal;

				  HAL_UART_Transmit(&huart6, lora_rx_buffer, 75,1000);
	  	  }

	     else if(lora_rx_buffer[3]==3 && lora_rx_buffer[50]==0x33)
		  {

		  Payload.satsinview=lora_rx_buffer[4];

		  Payload_union_converter();

		  Payload.battery=lora_rx_buffer[49];
		  Payload.mod=lora_rx_buffer[73];
		  Payload.communication=lora_rx_buffer[51];
	        // payload ekran

		  HAL_UART_Transmit(&huart6, lora_rx_buffer, 75,1000);

		  }



if(adc_flag ==1)
	  {
		  if(adc > 2476) adc = 2234;
		  if(adc < 1755) adc = 1755;
		  // 6V = 1755 adc val 1,41V
		  // 8.4V = 2476 adc val 1,99V 0,58V
		  adc_pil_val=(float)( ( ( (adc/4095)*3.3)-1.41) / (1.99-1.41) ) *100 ;
		 // adc_pil_val = (adc-1755)/(2746-1755)*100;
		  adc_flag=0;


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_12;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 16800;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 15000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Pin|BUZZER_Pin|GATE_D_Pin|GATE_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin|FN_Pin|LED2_Pin
                          |LED1_Pin|GATE_B_Pin|GATE_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin BUZZER_Pin GATE_D_Pin GATE_C_Pin */
  GPIO_InitStruct.Pin = CS_Pin|BUZZER_Pin|GATE_D_Pin|GATE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin FN_Pin LED2_Pin
                           LED1_Pin GATE_B_Pin GATE_A_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin|FN_Pin|LED2_Pin
                          |LED1_Pin|GATE_B_Pin|GATE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SECINP_Pin */
  GPIO_InitStruct.Pin = SECINP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SECINP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void E220_CONFIG(uint8_t ADDH, uint8_t ADDL, uint8_t CHN, uint8_t MODE)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
    HAL_Delay(50);

    char cfg_buff[8] = {0}; // E220 için 8 elemanlı bir dizi kullanıyoruz
    enum {Transparent, Fixed} mode;
    mode = MODE;

    cfg_buff[0] = ADDH;
    cfg_buff[1] = ADDL;
    cfg_buff[2] = 0x62;
    cfg_buff[3] = 0x00;
    cfg_buff[4] = CHN;

    switch(mode){
        case Transparent:
            cfg_buff[5] = 0x00;  // opsiyon
            break;
        case Fixed:
            cfg_buff[5] = 0x11;
            break;
        default:
            cfg_buff[5] = 0x11;
     }

     cfg_buff[6] = 0x00;
     cfg_buff[7] = 0x00;


    HAL_UART_Transmit(&huart3, (uint8_t*) cfg_buff, 8, 1000);

    HAL_Delay(25);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
    HAL_Delay(25);
}
double distance_in_m(double lat1, double long1, double lat2, double long2) {


    double dlat1=lat1*(PI/180);

    double dlong1=long1*(PI/180);
    double dlat2=lat2*(PI/180);
    double dlong2=long2*(PI/180);

    double dLong=dlong1-dlong2;
    double dLat=dlat1-dlat2;

    double aHarv= pow(sin(dLat/2.0),2.0)+cos(dlat1)*cos(dlat2)*pow(sin(dLong/2),2);
    double cHarv=2*atan2(sqrt(aHarv),sqrt(1.0-aHarv));

    double distance=radius_of_earth*cHarv;
    return (distance);
    }
double toRadians(double degree) {
    return (degree * PI / 180.0);
}

double calculateAngle(double lat1, double lon1, double lat2, double lon2){
    double lat1_rad = toRadians(lat1);
    double lat2_rad = toRadians(lat2);
    double delta_lon = toRadians(lon2 - lon1);
    double y = sin(delta_lon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);
    double angle_rad = atan2(y, x);
    double angle_deg = angle_rad * 180.0 / PI;
    return angle_deg;
}

void NEXTION_SendString (char *ID, char *string)
{
	char buf[50];
	int len = sprintf (buf, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit(&huart4, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
}


void NEXTION_SendNum (char *obj, int32_t num)
{
	uint8_t *buffer = malloc(30*sizeof (char));
	int len = sprintf ((char *)buffer, "%s.val=%ld", obj, num);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
	free(buffer);
}


void NEXTION_SendFloat (char *obj, float num, int dp)
{
	// convert to the integer
	int32_t number = num*(pow(10,dp));

	uint8_t *buffer = malloc(30*sizeof (char));
	int len = sprintf ((char *)buffer, "%s.vvs1=%d", obj, dp);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);


	len = sprintf ((char *)buffer, "%s.val=%ld", obj, number);
	HAL_UART_Transmit(&huart4, buffer, len, 1000);
	HAL_UART_Transmit(&huart4, Cmd_End, 3, 100);
	free(buffer);
}

void Nextion_SendCommand(char* command) {
    HAL_UART_Transmit(&huart4, (uint8_t*)command, strlen(command), 100);
    uint8_t end_cmd[] = {0xFF, 0xFF, 0xFF}; // Nextion end of command
    HAL_UART_Transmit(&huart4, end_cmd, 3, 100);
}

// Function to send a float value to a Nextion text box
void Nextion_SendFloatToTextbox(char* textbox_id, float value) {
    char command[50];
    char value_str[20];

    // Convert the float to a string
    snprintf(value_str, sizeof(value_str), "%.2f", value); // Adjust the format as needed

    // Format the command
    snprintf(command, sizeof(command), "%s.txt=\"%s\"", textbox_id, value_str);

    // Send the command to the Nextion display
    Nextion_SendCommand(command);
}

void HYI_BUFFER_Fill()
{
	HYI_BUFFER[0] =0xFF;
	HYI_BUFFER[1] =0xFF;
	HYI_BUFFER[2] =0x54;
	HYI_BUFFER[3] =0X52;
	HYI_BUFFER[4] =TAKIM_ID;
//	HYI_BUFFER[6] =takim_sayac;
//	HYI_BUFFER[7] =takim_sayac;
//	HYI_BUFFER[8] =takim_sayac;
//	HYI_BUFFER[9] =takim_sayac;
//	HYI_BUFFER[10] =takim_sayac;
//	HYI_BUFFER[11] =takim_sayac;
//	HYI_BUFFER[12] =takim_sayac;
//	HYI_BUFFER[13] =takim_sayac;
//	HYI_BUFFER[14] =takim_sayac;
//	HYI_BUFFER[15] =takim_sayac;
//	HYI_BUFFER[16] =takim_sayac;
//	HYI_BUFFER[17] =takim_sayac;
//	HYI_BUFFER[18] =takim_sayac;
//	HYI_BUFFER[19] =takim_sayac;
//	HYI_BUFFER[20] =takim_sayac;
//	HYI_BUFFER[21] =takim_sayac;
//	HYI_BUFFER[22] =takim_sayac;
//	HYI_BUFFER[23] =takim_sayac;
//	HYI_BUFFER[24] =takim_sayac;
//	HYI_BUFFER[25] =takim_sayac;
//	HYI_BUFFER[26] =takim_sayac;
//	HYI_BUFFER[27] =takim_sayac;
//	HYI_BUFFER[28] =takim_sayac;
//	HYI_BUFFER[29] =takim_sayac;
//	HYI_BUFFER[30] =takim_sayac;
//	HYI_BUFFER[31] =takim_sayac;
//	HYI_BUFFER[32] =takim_sayac;
//	HYI_BUFFER[33] =takim_sayac;
//	HYI_BUFFER[34] =takim_sayac;
//	HYI_BUFFER[35] =takim_sayac;
//	HYI_BUFFER[36] =takim_sayac;
//	HYI_BUFFER[37] =takim_sayac;
//	HYI_BUFFER[38] =takim_sayac;
//	HYI_BUFFER[39] =takim_sayac;
//	HYI_BUFFER[40] =takim_sayac;
//	HYI_BUFFER[41] =takim_sayac;
//	HYI_BUFFER[42] =takim_sayac;
//	HYI_BUFFER[43] =takim_sayac;
//	HYI_BUFFER[44] =takim_sayac;
//	HYI_BUFFER[45] =takim_sayac;
//	HYI_BUFFER[46] =takim_sayac;
//	HYI_BUFFER[47] =takim_sayac;
//	HYI_BUFFER[48] =takim_sayac;
//	HYI_BUFFER[49] =takim_sayac;
//	HYI_BUFFER[50] =takim_sayac;
//	HYI_BUFFER[51] =takim_sayac;
//	HYI_BUFFER[52] =takim_sayac;
//	HYI_BUFFER[53] =takim_sayac;
//	HYI_BUFFER[54] =takim_sayac;
//	HYI_BUFFER[55] =takim_sayac;
//	HYI_BUFFER[56] =takim_sayac;
//	HYI_BUFFER[57] =takim_sayac;
//	HYI_BUFFER[58] =takim_sayac;
//	HYI_BUFFER[59] =takim_sayac;
//	HYI_BUFFER[60] =takim_sayac;
//	HYI_BUFFER[61] =takim_sayac;
//	HYI_BUFFER[62] =takim_sayac;
//	HYI_BUFFER[63] =takim_sayac;
//	HYI_BUFFER[64] =takim_sayac;
//	HYI_BUFFER[65] =takim_sayac;
//	HYI_BUFFER[66] =takim_sayac;
//	HYI_BUFFER[67] =takim_sayac;
//	HYI_BUFFER[68] =takim_sayac;
//	HYI_BUFFER[69] =takim_sayac;
//	HYI_BUFFER[70] =takim_sayac;
//	HYI_BUFFER[71] =takim_sayac;
//	HYI_BUFFER[72] =takim_sayac;
//	HYI_BUFFER[73] =takim_sayac;


	HYI_BUFFER[74]= 1;//EGU_AYRILMA_TESPIT;
	//HYI_BUFFER[75]= crc; // CRC
	HYI_BUFFER[76]= 0x0D;
	HYI_BUFFER[77]= 0x0A;


}
void Payload_union_converter(void)
{
	 float2unit8 f2u8;
			 for(uint8_t i=0;i<4;i++)
			 {
				 f2u8.array[i]=lora_rx_buffer[i+5];
				 HYI_BUFFER[22+i]=lora_rx_buffer[i+5]; // 34 35 36 37
			 }
			 Payload.gpsaltitude=f2u8.fVal;


			 for(uint8_t i=0;i<4;i++)
			 {
				 f2u8.array[i]=lora_rx_buffer[i+9];
				 HYI_BUFFER[26+i]=lora_rx_buffer[i+9]; // 38 39 40 41
			 }
			 Payload.gpslatitude=f2u8.fVal;

			 for(uint8_t i=0;i<4;i++)
			 {
				 f2u8.array[i]=lora_rx_buffer[i+13];
				 HYI_BUFFER[30+i]=lora_rx_buffer[i+13]; // 42 43 44 45
			 }
			 Payload.gpslongitude=f2u8.fVal;

			 for(uint8_t i=0;i<4;i++)
			 {
				 f2u8.array[i]=lora_rx_buffer[i+17];
			 }
			 Payload.altitude=f2u8.fVal;


			 for(uint8_t i=0;i<4;i++)
			 {
				 f2u8.array[i]=lora_rx_buffer[i+21];
			 }
			 Payload.speed=f2u8.fVal;


			 for(uint8_t i=0;i<4;i++)
			 {
				 f2u8.array[i]=lora_rx_buffer[i+25];
			 }
			 Payload.temperature=f2u8.fVal;


			 for(uint8_t i=0;i<4;i++)
			 {
				 f2u8.array[i]=lora_rx_buffer[i+29];
			 }
			 Payload.accx=f2u8.fVal;


			 for(uint8_t i=0;i<4;i++)
			 {
				 f2u8.array[i]=lora_rx_buffer[i+33];
			 }
			 Payload.accy=f2u8.fVal;


		      for(uint8_t i=0;i<4;i++)
			 {
		    	  f2u8.array[i]=lora_rx_buffer[i+37];
			 }
		      Payload.accz=f2u8.fVal;


			  for(uint8_t i=0;i<4;i++)
			 {
				  f2u8.array[i]=lora_rx_buffer[i+41];
			 }
			  Payload.normal=f2u8.fVal;


			  for(uint8_t i=0;i<4;i++)
			 {
				  f2u8.array[i]=lora_rx_buffer[i+45];
			 }
			  Payload.pitch=f2u8.fVal;
}
void Enum_State_bs(void){

    switch(Booster.mod){

    case 0:
    	enum_bs[0]='O';
    	enum_bs[1]='F';
    	enum_bs[2]='F';
    	enum_bs[3]='\0';
    	enum_bs[4]='\0';
    	enum_bs[5]='\0';
    	enum_bs[6]='\0';
    	enum_bs[7]='\0';
    	enum_bs[8]='\0';
    	break;

    case 1:
    	enum_bs[0]='R';
    	enum_bs[1]='A';
    	enum_bs[2]='M';
    	enum_bs[3]='P';
    	enum_bs[4]='A';
    	enum_bs[5]='\0';
    	enum_bs[6]='\0';
    	enum_bs[7]='\0';
    	enum_bs[8]='\0';
    	break;

    case 2:
    	enum_bs[0]='U';
    	enum_bs[1]='C';
    	enum_bs[2]='U';
    	enum_bs[3]='S';
    	enum_bs[4]='\0';
    	enum_bs[5]='\0';
    	enum_bs[6]='\0';
    	enum_bs[7]='\0';
    	enum_s[8]='\0';
    	break;
    case 3:
    	enum_bs[0]='B';
    	enum_bs[1]='U';
    	enum_bs[2]='R';
    	enum_bs[3]='N';
    	enum_bs[4]='O';
    	enum_bs[5]='U';
    	enum_bs[6]='T';
    	enum_bs[7]='\0';
    	enum_bs[8]='\0';
    	break;
    case 4:
    	enum_bs[0]='A';
    	enum_bs[1]='Y';
    	enum_bs[2]='I';
    	enum_bs[3]='R';
    	enum_bs[4]='\0';
    	enum_bs[5]='\0';
    	enum_bs[6]='\0';
    	enum_bs[7]='\0';
    	enum_bs[8]='\0';
    	break;
    case 5:
    	enum_bs[0]='A';
    	enum_bs[1]='Y';
    	enum_bs[2]='R';
    	enum_bs[3]='I';
    	enum_bs[4]='L';
    	enum_bs[5]='D';
    	enum_bs[6]='I';
    	enum_bs[7]='?';
    	enum_bs[8]='\0';
    	break;
    case 6:
    	enum_bs[0]='A';
    	enum_bs[1]='Y';
    	enum_bs[2]='R';
    	enum_bs[3]='I';
    	enum_bs[4]='L';
    	enum_bs[5]='D';
    	enum_bs[6]='I';
    	enum_bs[7]='\0';
    	enum_bs[8]='\0';
    	break;
    case 7:
    	enum_bs[0]='A';
    	enum_bs[1]='Y';
    	enum_bs[2]='R';
    	enum_bs[3]='I';
    	enum_bs[4]='L';
    	enum_bs[5]='M';
    	enum_bs[6]='A';
    	enum_bs[7]='D';
    	enum_bs[8]='I';
    	break;
    case 8:
    	enum_bs[0]='F';
    	enum_bs[1]='I';
    	enum_bs[2]='N';
    	enum_bs[3]='I';
    	enum_bs[4]='S';
    	enum_bs[5]='H';
    	enum_bs[6]='\0';
    	enum_bs[7]='\0';
    	enum_bs[8]='\0';
    	break;

    	 }

    NEXTION_SendString("bs10", &enum_bs);


}
void Enum_State_s(void){

    switch(Sustainer.mod){


    case 0:
    	enum_s[0]='O';
    	enum_s[1]='F';
    	enum_s[2]='F';
    	enum_s[3]='\0';
    	enum_s[4]='\0';
    	enum_s[5]='\0';
    	enum_s[6]='\0';
    	enum_s[7]='\0';
    	enum_s[8]='\0';
    	break;
    case 1:
    	enum_s[0]='R';
    	enum_s[1]='A';
    	enum_s[2]='M';
    	enum_s[3]='P';
    	enum_s[4]='A';
    	enum_s[5]='\0';
    	enum_s[6]='\0';
    	enum_s[7]='\0';
    	enum_s[8]='\0';
    	break;

    case 2:
    	enum_s[0]='U';
    	enum_s[1]='C';
    	enum_s[2]='U';
    	enum_s[3]='S';
    	enum_s[4]='\0';
    	enum_s[5]='\0';
    	enum_s[6]='\0';
    	enum_s[7]='\0';
    	enum_s[8]='\0';
    	break;
    case 3:
    	enum_s[0]='A';
    	enum_s[1]='Y';
    	enum_s[2]='R';
    	enum_s[3]='I';
    	enum_s[4]='L';
    	enum_s[5]='D';
    	enum_s[6]='I';
    	enum_s[7]='?';
    	enum_s[8]='\0';
    	break;
    case 4:
    	enum_s[0]='A';
    	enum_s[1]='Y';
    	enum_s[2]='R';
    	enum_s[3]='I';
    	enum_s[4]='L';
    	enum_s[5]='D';
    	enum_s[6]='I';
    	enum_s[7]='\0';
    	enum_s[8]='\0';
    	break;
    case 5:
    	enum_s[0]='A';
    	enum_s[1]='P';
    	enum_s[2]='O';
    	enum_s[3]='G';
    	enum_s[4]='E';
    	enum_s[5]='E';
    	enum_s[6]='\0';
    	enum_s[7]='\0';
    	enum_s[8]='\0';
    	break;
    case 6:
    	enum_s[0]='M';
    	enum_s[1]='A';
    	enum_s[2]='I';
    	enum_s[3]='N';
    	enum_s[4]='\0';
    	enum_s[5]='\0';
    	enum_s[6]='\0';
    	enum_s[7]='\0';
    	enum_s[8]='\0';
    	break;
    case 7:
    	enum_s[0]='F';
    	enum_s[1]='I';
    	enum_s[2]='N';
    	enum_s[3]='I';
    	enum_s[4]='S';
    	enum_s[5]='H';
    	enum_s[6]='\0';
    	enum_s[7]='\0';
    	enum_s[8]='\0';
    	break;



    	 }

    NEXTION_SendString("s10", &enum_s);


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
