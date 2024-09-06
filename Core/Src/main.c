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
#include "dht11.h"
#include "aes.h"
#include "cmac.h"
#include "encrypt.h"
#include "secrets.h"
#include <stdlib.h>
#include <string.h>
//#include "secrets.h"
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

#define SLEEP_COUNT_MAX 5

/* the assumption is that FOPTS field is absent and payload is 5 bytes max => 18 */
enum lorawan_mac_frame_offset {LORAWAN_MAC_HDR = 0, LORAWAN_DEVADDR = 1, LORAWAN_FCTRL = 5, LORAWAN_FCNT = 6, LORAWAN_FPORT = 8, LORAWAN_FRMPAYLOAD = 9, LORAWAN_MIC = 14};

static LoRa myLoRa;
static uint16_t LoRa_stat = 0;

static dht11 myDHT11;

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static uint8_t NWKSKEY[] = { NWKSKEY1 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static uint8_t APPSKEY[] = { APPSKEY1 };

static const uint8_t DEVADDR[] = { DEV_ADDR1_BYTE }; // Change this address for every node

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

/* total byte = 14 */
int32_t set_msg_frame_for_mic_calc(uint8_t *msg, uint8_t *encrypted_payload, uint16_t fcounter)
{
	msg[LORAWAN_MAC_HDR] = 0x40;
	msg[LORAWAN_DEVADDR] = 0x49;
	msg[LORAWAN_DEVADDR + 1] = 0x1E;
	msg[LORAWAN_DEVADDR + 2] = 0x0D;
	msg[LORAWAN_DEVADDR + 3] = 0x26;
	msg[LORAWAN_FCTRL] = 0x00;
	msg[LORAWAN_FCNT] = fcounter & 0xFF;
	msg[LORAWAN_FCNT + 1] = fcounter >> 8;
	msg[LORAWAN_FPORT] = 0x1;
	memcpy(&msg[LORAWAN_FRMPAYLOAD], encrypted_payload, 5);
	
	return 0;
}

int32_t set_block_b_frame_for_mic_calc(uint8_t *b, uint16_t fcounter, uint8_t payload_size)
{
	b[0] = 0x49;
	memset(&b[1], 0x00, 5);
	b[6] = 0x49;
	b[7] = 0x1E;
	b[8] = 0x0D;
	b[9] = 0x26;
	b[10] = fcounter & 0xFF;
	b[11] = fcounter >> 8;
	b[12] = 0x00;
	b[13] = 0x00;
	b[14] = 0x00;
	b[15] = payload_size;
	
	return 0;
}

int32_t set_actual_data_send(uint8_t *out, uint8_t *msg, uint8_t *mic, uint8_t msg_size)
{
    out[0] = msg[LORAWAN_MAC_HDR];
    out[1] = msg[LORAWAN_DEVADDR];
    out[2] = msg[LORAWAN_DEVADDR + 1];
    out[3] = msg[LORAWAN_DEVADDR + 2];
    out[4] = msg[LORAWAN_DEVADDR + 3];
    out[5] = msg[LORAWAN_FCTRL];
    out[6] = msg[LORAWAN_FCNT];
    out[7] = msg[LORAWAN_FCNT + 1];
    out[8] = msg[LORAWAN_FPORT];
    out[9] = msg[LORAWAN_FRMPAYLOAD];
    out[10] = msg[LORAWAN_FRMPAYLOAD + 1];
    out[11] = msg[LORAWAN_FRMPAYLOAD + 2];
    out[12] = msg[LORAWAN_FRMPAYLOAD + 3];
    out[13] = msg[LORAWAN_FRMPAYLOAD + 4];
    out[14] = mic[0];
    out[15] = mic[1];
    out[16] = mic[2];
    out[17] = mic[3];

	return 0;
}

/* Size must be equals to use this func */
int32_t byte_array_xor(uint8_t *byte_a, uint8_t *byte_b, uint8_t *out, uint32_t size)
{
	for (int i = 0; i < size; i++) {
		out[i] = byte_a[i] ^ byte_b[i];
	}
	return 0;
}

uint8_t msg[14] = {0};
uint8_t b[16] = {0};
uint8_t frame_msg[30] = {0};
uint8_t cmac[16] = {0};
uint8_t mic[4] = {0};

uint8_t actual_data_send[18];
						
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
  /* USER CODE BEGIN 2 */
	TIM4_EnablePeripheral();
	TIM2_EnablePeripheral_IT();
		
	myLoRa = newLoRa();

	myLoRa.CS_port         = NSS_GPIO_Port;
	myLoRa.CS_pin          = NSS_Pin;
	myLoRa.reset_port      = RST_GPIO_Port;
	myLoRa.reset_pin       = RST_Pin;
	myLoRa.DIO0_port       = DIO0_GPIO_Port;
	myLoRa.DIO0_pin        = DIO0_Pin;
	myLoRa.hSPIx           = &hspi1;
	
	myDHT11.data_port = DHT11_GPIO_Port;
	myDHT11.data_pin = DHT11_Pin;

	HAL_Delay(3000);

	if (LoRa_init(&myLoRa) == LORA_OK) {
		LoRa_stat = 1;
	}
	if (LoRa_stat) {
		LoRa_setSyncWord(&myLoRa, 0x12);
	}
	
	if (dht11_init(&myDHT11) == 0) {
		led_flashing(LED_GPIO_Port, LED_Pin, 5);
	}

	uint8_t sleep_count = 5;
	uint8_t package_count = 0;

	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (LoRa_stat) {
			/* 
			 * total sleep time is 3 mins to comply with 500 pacakges 
			 * SF7 per day for TTN Fair Use Policy (10 bytes payload)
			 * => 30 seconds air time per device
			 * Note :we could increase it up to 1000 packages since
			 * current payload is 5 bytes only but 
			 */
			if (sleep_count >= SLEEP_COUNT_MAX) {
				
        uint8_t Ai[16] = {0x01, 0, 0, 0, 0, 0, 0x49, 0x1E, 0x0D, 0x26, 0, 0, 0, 0, 0, 1};
				Ai[13] = package_count;
				uint32_t Ai_size = 16;
				uint8_t *S = ecb_encrypt(Ai, APPSKEY, aes_128_encrypt, &Ai_size);
				
				Ai_size = 16;
        uint8_t payload[16] = {92, 0, 32, 0, 60};
				uint8_t encrypted_payload[16] = {0};
				byte_array_xor(payload, S, encrypted_payload, Ai_size);

				memset(msg, 0x00, 14);
				set_msg_frame_for_mic_calc(msg, encrypted_payload, package_count);

				memset(b, 0x00, 16);
				set_block_b_frame_for_mic_calc(b, package_count, 14);


				memset(frame_msg, 0x00, 30);
				memcpy(frame_msg, b, 16);
				memcpy(&frame_msg[16], msg, 14);
				

				memset(cmac, 0x00, 16);
				aes_cmac(frame_msg, 30, cmac, NWKSKEY);
				

				memset(mic, 0x00, 4);
				memcpy(mic, cmac, 4);
				
				memset(actual_data_send, 0x00, 18);
				set_actual_data_send(actual_data_send, msg, mic, 14);
				package_count++;
				free(S);
				S = NULL;
				sleep_count = 0;
				if (dht11_read(&myDHT11) == 0) {
					led_flashing(LED_GPIO_Port, LED_Pin, 2);
					if (LoRa_transmit(&myLoRa, actual_data_send, 18, 1000)) {
						led_flashing(LED_GPIO_Port, LED_Pin, 5);
					}
				} else {
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				}
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
		/* increment sleep count */
		sleep_count++;
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
