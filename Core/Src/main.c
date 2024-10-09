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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
//#include "dht11.h"
#include "string.h"
#include "HC_SR04.h"
#include "aes.h"
#include "loramac.h"
#include "rc522.h"
#include <stdlib.h>
#include "secrets.h"
//#include "cc20_p1305.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define TIME_SLEEP_MAX 1
#define PARKING_LOCATION_FLOOR 2
#define PARKING_LOCATION_ROW 0xA
#define PARKING_LOCATION_COLUMN 0x5
#define COUNT_EMPTY_PARKING_LOT_MAX 40

//HSCR_04 variable
uint8_t distance_sensor;
uint8_t count_empty_time;
uint8_t count_sensor_get_dirty;
uint8_t count_rfid;
uint8_t parking_lot_state = 0;
enum state_of_parkinglot {PARKING_LOT_EMTPY = 0, PARKING_LOT_IS_AVAILABLE = 1, SENSOR_IS_DIRTY = 2};

LoRa myLoRa;
uint16_t LoRa_stat = 0;


/* the assumption is that FOPTS field is absent and payload is 2 bytes max => 15 */
enum lorawan_mac_frame_offset {LORAWAN_MAC_HDR = 0, LORAWAN_DEVADDR = 1, LORAWAN_FCTRL = 5, LORAWAN_FCNT = 6, LORAWAN_FPORT = 8, LORAWAN_FRMPAYLOAD = 9, LORAWAN_MIC = 11};

//RFID varable
uint8_t rfid_status;
uint8_t str[MAX_LEN]; // Max_LEN = 16
uint8_t serial_num[5];

static uint32_t dev_addr = DEV_ADDR1;
static uint8_t nwkskey[16] = {NWKSKEY1};
static uint8_t appskey[16] = {APPSKEY1};

volatile uint8_t time_sleep = TIME_SLEEP_MAX;
uint8_t data_transmit[2] = {};
//
static struct loramac_phys_payload *loramac_payload;



static uint8_t lorawan_frame_to_calc_mic[18] = {0};

void TIM4_EnablePeripheral(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
}

void TIM2_EnablePeripheral_IT(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->CR1 |= TIM_CR1_URS;
	
	TIM2->DIER |= TIM_DIER_UIE;
}

void TIM2_Start_IT(void)
{
	// Enable counter
	TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_Disable_IT(void)
{
	// Disable counter
	TIM2->CR1 &= ~(TIM_CR1_CEN);
}

void delay_us(uint32_t time_us)
{
	if (time_us > 10) {
		uint32_t offset_in_calc = time_us - 10;
		uint32_t prev_val = offset_in_calc;
		for (offset_in_calc = time_us - 10; offset_in_calc + 4 <= time_us;)
		{
			prev_val = offset_in_calc;
			offset_in_calc += 4;
		}
		offset_in_calc = prev_val;
		time_us = offset_in_calc;
	}
	
	TIM4->CNT = 0; /*Reset counter*/
	TIM4->CR1 |= TIM_CR1_ARPE; /*Enable autoreload on update event*/
	TIM4->ARR = 10;
	TIM4->PSC = 6;
	TIM4->SR &= ~(TIM_SR_UIF); /*Clear update event flag*/
	TIM4->CR1 |= TIM_CR1_CEN; /*Enable counter*/
	
	/*Loop until timeInMs = 0*/
	while(time_us > 0)
	{
		while(!((TIM4->SR & TIM_SR_UIF) != 0));/*Wait for counter to overflow*/
		
		time_us = time_us - 1;
		TIM4->SR &= ~(TIM_SR_UIF);
	}
	
	TIM4->CR1 &= ~(TIM_CR1_CEN); /*Disable counter*/
}

void led_flashing(GPIO_TypeDef *port, uint16_t pin, uint8_t time)
{
	for (uint8_t idx = 0; idx < time * 2; idx++) {
		HAL_GPIO_TogglePin(port, pin);
		HAL_Delay(200);
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
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	TIM4_EnablePeripheral();
	TIM2_EnablePeripheral_IT();
	
	MFRC522_Init();
	myLoRa = newLoRa();

	myLoRa.CS_port         = NSS_GPIO_Port;
	myLoRa.CS_pin          = NSS_Pin;
	myLoRa.reset_port      = RST_GPIO_Port;
	myLoRa.reset_pin       = RST_Pin;
	myLoRa.DIO0_port       = DIO0_GPIO_Port;
	myLoRa.DIO0_pin        = DIO0_Pin;
	myLoRa.hSPIx           = &hspi1;
	
	myLoRa.frequency             = 921;							  // default = 433 MHz
	myLoRa.spredingFactor        = SF_7;							// default = SF_7
	myLoRa.bandWidth			       = BW_125KHz;				  // default = BW_125KHz
	myLoRa.crcRate				       = CR_4_5;						// default = CR_4_5
	myLoRa.power					       = POWER_20db;				// default = 20db
	myLoRa.overCurrentProtection = 120; 							// default = 100 mA
	myLoRa.preamble				       = 10;		  					// default = 8;
/*
	myDHT11.data_port = DHT11_GPIO_Port;
	myDHT11.data_pin = DHT11_Pin;
*/
	HAL_Delay(3000);
	LoRa_reset(&myLoRa);
	if (LoRa_init(&myLoRa) == LORA_OK) {
		LoRa_stat = 1;
	}
  if (LoRa_stat) {
		LoRa_setSyncWord(&myLoRa, 0x12);
	}
	
//	if (dht11_init(&myDHT11) == 0) {
//		led_flashing(LED_GPIO_Port, LED_Pin, 5);
//	}
	
	
	uint8_t package_count = 0;
	
	uint16_t loramac_f_cnt = 0;
	loramac_payload = loramac_init();
	loramac_fill_mac_payload(loramac_payload, 1, NULL);
	loramac_fill_phys_payload(loramac_payload, LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_UP, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (time_sleep >= TIME_SLEEP_MAX) {
			time_sleep = 0;
			if (LoRa_stat) {
				for (int i = 0; i++ < COUNT_EMPTY_PARKING_LOT_MAX;) {
						distance_sensor = HCSR04_GetDis();
						if  (distance_sensor > 40) {
								count_empty_time ++;
								HAL_Delay(200);
						} else if (distance_sensor < 5) {
								count_sensor_get_dirty++;
						}
				}
				if ( count_empty_time >= (COUNT_EMPTY_PARKING_LOT_MAX - 15)) {
						parking_lot_state =  PARKING_LOT_EMTPY	;
				} else if (count_sensor_get_dirty >= 12) {
						parking_lot_state =  SENSOR_IS_DIRTY;
				} else {
						parking_lot_state = PARKING_LOT_IS_AVAILABLE;
				}
				
				data_transmit[0] = parking_lot_state << 2 | PARKING_LOCATION_FLOOR;
				data_transmit[1] = PARKING_LOCATION_COLUMN << 4 | PARKING_LOCATION_ROW;
				
				count_empty_time = 0;
				count_sensor_get_dirty = 0;
				
				loramac_fill_fhdr(loramac_payload, dev_addr, 0, loramac_f_cnt, NULL);
  			loramac_fill_mac_payload(loramac_payload, 1, data_transmit);
				loramac_f_cnt += 1;
				uint32_t loramac_mic = 0;
				loramac_frm_payload_encryption(loramac_payload, 2, appskey);
				loramac_calculate_mic(loramac_payload, 2, nwkskey, 1, &loramac_mic); // 2 FRM_PAYLOAD + 1 MHDR + 7 FHDR + 1 FPORT
				loramac_fill_phys_payload(loramac_payload, LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_UP, loramac_mic);
				uint8_t lora_package[15] = {0}; // 2 FRM_PAYLOAD + 13 LORAWAN protocol excepts FOPTS
				loramac_serialize_data(loramac_payload, lora_package, 2);

				if (LoRa_transmit(&myLoRa, (uint8_t*)lora_package, 15, TRANSMIT_TIMEOUT)) {
					led_flashing(LED_GPIO_Port, LED_Pin, 5);
					HAL_Delay(1500);
				}
				str[4] = 0;
			}
		}
		/* Start timer interrupt */
		TIM2_Start_IT();
		/* Suspend SYSTICK to not wake up from sleep */
		HAL_SuspendTick();
		/* Enter sleep mode, will be wake up by timer*/
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		/* Start SYSTICK again */
		HAL_ResumeTick();
		/* Disable timer interrupt to process other things */
		TIM2_Disable_IT();
		time_sleep++;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own impflementation to report the HAL error return state */
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
