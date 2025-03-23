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
#include "loramac.h"
#include "secrets.h"
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

#define TIME_SLEEP_MAX 1
#define RX_BUFFER_SIZE 255

#define LE_BYTES_TO_UINT32(x) ((*(x + 3)) << 24) | ((*(x + 2)) << 16) | ((*(x + 1)) << 8) | ((*(x)))
#define LE_BYTES_TO_UINT16(x) ((*(x + 1)) << 8) | ((*(x)))

enum PROG_FSM {INIT = 0, MCU_SLEEP = 1, LORA_TX = 2, LORA_RX = 3, LORA_GPIO_INT = 4, LORA_TIMER_INT = 5, LORA_RX_PKT_RDY = 6};
volatile uint32_t prog_fsm = 0;

static LoRa myLoRa;
static uint16_t LoRa_stat = 0;

static uint32_t dev_addr = DEV_ADDR1;
static uint8_t nwkskey[16] = {NWKSKEY1};
static uint8_t appskey[16] = {APPSKEY1};

static struct loramac_phys_payload *loramac_payload;

static volatile uint8_t lorawan_rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t lorawan_rx_buf_size;
static volatile uint8_t lorawan_is_tx;
static volatile struct loramac_phys_payload lorawan_rx_phys;

dht11 myDHT11;

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

void reverse_bytes(uint8_t *bytes, size_t size)
{
    if (size <= 1) {
        return;
    }
    for (uint8_t i = 0, j = size - 1; i < size / 2; i++, j--) {
        uint8_t tmp = bytes[j];
        bytes[j] = bytes[i];
        bytes[i] = tmp;
    }
}

int32_t decrypt_lorawan_asconmac(uint8_t *lora_raw_data, uint8_t size, struct loramac_mac_payload *out)
{
	struct loramac_phys_payload *payload = loramac_init();
	uint8_t frm_payload_size = size - (1 + 4 + 1 + 2 + 1 + 4); /* [MHDR + FHDR[DevAddr + ..] + FPORT + MIC] */
	uint32_t dev_addr = LE_BYTES_TO_UINT32((&lora_raw_data[LRMAC_BYTE_OFFSET_DEVADDR]));
	uint16_t f_cnt = LE_BYTES_TO_UINT16((&lora_raw_data[LRMAC_BYTE_OFFSET_FCNT]));
	uint8_t f_ctrl = lora_raw_data[LRMAC_BYTE_OFFSET_FCTRL];
	
	loramac_fill_fhdr(payload, dev_addr, f_ctrl, f_cnt, NULL);
	
	uint8_t f_port = lora_raw_data[LRMAC_BYTE_OFFSET_FPORT];
	out->frm_payload = &lora_raw_data[LRMAC_BYTE_OFFSET_FRMPAYLOAD];
	reverse_bytes(out->frm_payload, frm_payload_size);
	loramac_fill_mac_payload(payload, f_port, out->frm_payload);

	uint8_t m_hdr = lora_raw_data[LRMAC_BYTE_OFFSET_MHDR];

	loramac_fill_phys_payload(payload, m_hdr, 0);
	
	if (f_ctrl & 0xF) {
		/* no support for FOpts */
		return -1;
	}
	uint32_t mic = 0;
	uint32_t decoded_mic = 0;
	/* Calculate MIC */
	loramac_calculate_mic(payload, frm_payload_size, nwkskey, 1, &mic);
	decoded_mic = LE_BYTES_TO_UINT32(&lora_raw_data[LRMAC_BYTE_OFFSET_FRMPAYLOAD + frm_payload_size]);
	if (mic != decoded_mic) {
		/* MIC does not match */ 
			return -2;
	}
	/* Decrypt LoRaWAN payload */
	loramac_frm_payload_encryption(payload, frm_payload_size, appskey);
	out->f_hdr.dev_addr = dev_addr;
	out->f_hdr.f_cnt = f_cnt;
	out->f_hdr.f_ctrl = f_ctrl;
	out->f_port = f_port;
	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	prog_fsm = INIT;
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
	LoRa_startReceiving(&myLoRa);
	if (dht11_init(&myDHT11) == 0) {
		led_flashing(LED_GPIO_Port, LED_Pin, 5);
	}
	
	uint8_t time_sleep = 5;
	
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
			if (prog_fsm == LORA_RX_PKT_RDY) {
				int32_t ret = decrypt_lorawan_asconmac((uint8_t *)lorawan_rx_buffer, lorawan_rx_buf_size, (struct loramac_mac_payload *)(&lorawan_rx_phys.mac_payload));
				if (!ret) {
					led_flashing(LED_GPIO_Port, LED_Pin, 4);
					// Toggle device, currently frm_payload is unused
					if (lorawan_rx_buf_size - 13 == 3) {
						LoRa_stat ^= 0x1;
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
					}
				}
			}
			if (LoRa_stat) {
				if (prog_fsm != LORA_RX_PKT_RDY && dht11_read(&myDHT11) == 0) {
					prog_fsm = LORA_TX;
					led_flashing(LED_GPIO_Port, LED_Pin, 2);
					
					loramac_fill_fhdr(loramac_payload, dev_addr, 0, loramac_f_cnt, NULL);
					loramac_fill_mac_payload(loramac_payload, 1, myDHT11.data);
					loramac_f_cnt += 1;
					uint32_t loramac_mic = 0;
					loramac_frm_payload_encryption(loramac_payload, 5, appskey);
					loramac_calculate_mic(loramac_payload, 5, nwkskey, 1, &loramac_mic); // 5 FRM_PAYLOAD + 1 MHDR + 7 FHDR + 1 FPORT
					loramac_fill_phys_payload(loramac_payload, LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_UP, loramac_mic);

					uint8_t lora_package[18] = {0}; // 5 FRM_PAYLOAD + 13 LORAWAN protocol excepts FOPTS
					loramac_serialize_data(loramac_payload, lora_package, 5);
					LoRa_gotoMode(&myLoRa, STNBY_MODE);
					LoRa_setFrequency(&myLoRa, 920400000);
					if (LoRa_transmit(&myLoRa, lora_package, 18, 1000)) {
						led_flashing(LED_GPIO_Port, LED_Pin, 5);
					}
				} else {
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				}
				LoRa_gotoMode(&myLoRa, STNBY_MODE);
				LoRa_setFrequency(&myLoRa, 921400000);
				LoRa_startReceiving(&myLoRa);
				prog_fsm = LORA_RX;
			}
		}
		/* Start timer interrupt */
		TIM2_Start_IT();
		/* Suspend SYSTICK to not wake up from sleep */
		HAL_SuspendTick();
		prog_fsm = MCU_SLEEP;
		/* Enter sleep mode, will be wake up by timer*/
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		/* Check if wake from GPIO_EXTI */
		while (prog_fsm == LORA_GPIO_INT) {
			/* Sleep again */
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		}
		/* Start SYSTICK again */
		HAL_ResumeTick();
		/* Disable timer interrupt to process other things */
		TIM2_Disable_IT();
		
		time_sleep++;
		
		led_flashing(LED_GPIO_Port, LED_Pin, 3);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		// Unused
	}
	prog_fsm = LORA_TIMER_INT;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (prog_fsm == MCU_SLEEP || prog_fsm == LORA_RX) {
		if (GPIO_Pin == DIO0_Pin) {
			lorawan_rx_buf_size = LoRa_receive(&myLoRa, (uint8_t *)lorawan_rx_buffer, RX_BUFFER_SIZE);
			prog_fsm = LORA_RX_PKT_RDY;
		}
	}
	/* This prevents overwriting state when TIMER interrupt is served first then GPIO later */
	if (prog_fsm != LORA_TIMER_INT && prog_fsm != LORA_RX_PKT_RDY) {
		prog_fsm = LORA_GPIO_INT;
	}
	lorawan_is_tx = 0;
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
