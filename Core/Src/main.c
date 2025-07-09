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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "BME280_STM32.h"
#include "FLASH_PAGE_F1.h"
#include "loramac.h"
#include "secrets.h"
#include "crypto_auth.h"
#include "crypto_aead.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
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
#define TEST_PKT 1
//#define UART_DBG 1

#define TIME_SLEEP_MAX 1
#define RX_BUFFER_SIZE 255

#define LE_BYTES_TO_UINT32(x) ((*(x + 3)) << 24) | ((*(x + 2)) << 16) | ((*(x + 1)) << 8) | ((*(x)))
#define LE_BYTES_TO_UINT16(x) ((*(x + 1)) << 8) | ((*(x)))

#define APPNONCE_STORAGE_LOCATION 0x0800C000

struct program_state {
	/* State in enum PROG_FSM */
	uint8_t fsm;
	/* Server joined flag */
	uint8_t joined;
};

enum PROG_FSM {INIT, MCU_SLEEP, LORA_TX, LORA_TX_JOIN_REQ_STARTED, LORA_RX, LORA_GPIO_INT, LORA_TIMER_INT, LORA_RX_PKT_RDY};

struct program_state prog = {0};

static LoRa myLoRa;
static uint16_t LoRa_stat = 0;

// ABP
static uint32_t dev_addr = 0; // DEV_ADDR1
static uint8_t nwkskey[16] = {0}; // NWKSKEY1
static uint8_t appskey[16] = {0}; // APPSKEY1

// OTAA
static uint8_t appeui[8] = {APP_EUI};
static uint8_t deveui[8] = {DEV_EUI};
static uint8_t appkey[16] = {APP_KEY};
static uint8_t devnonce[2] = {0};
// need to store fetch this in FLASH
static uint8_t appnonce[3] = {0, 0, 0};

static uint32_t start, end;

static struct loramac_phys_payload *loramac_payload;
static struct loramac_phys_payload_join_request *loramac_jr;
#ifdef TEST_PKT
static struct loramac_phys_payload _loramac_payload_test = {0};
static struct loramac_phys_payload *loramac_payload_test;
#endif

static volatile uint8_t lorawan_rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t lorawan_rx_buf_size;
static volatile uint8_t lorawan_is_tx;
static struct loramac_phys_payload lorawan_rx_phys;

static uint8_t myData[5] = {0};
float Temperature = 0, Pressure = 0, Humidity = 0;

void log_debug(const char *string)
{
#ifdef UART_DBG
	HAL_UART_Transmit(&huart1, (const uint8_t *)string, strlen(string), 1000);
	HAL_Delay(1);
#else
	(void)string;
#endif
}

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

void TIM4_Start(void)
{
	HAL_TIM_Base_Start(&htim4);
}

void TIM4_Disable(void)
{
	HAL_TIM_Base_Stop(&htim4);
	// Optionally disable the TIM4 clock entirely
	__HAL_RCC_TIM4_CLK_DISABLE();
}

void delay_us(uint16_t us) {
    // Ensure TIM4 clock is enabled
    __HAL_RCC_TIM4_CLK_ENABLE();
    TIM4_Start();

    // Reset the counter
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // Wait until the counter reaches the desired value
    while (__HAL_TIM_GET_COUNTER(&htim4) < us) {
        // Do nothing, just wait
    }

    // Stop the timer to save power
    TIM4_Disable();
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

int32_t lorawan_transmit(LoRa *lora, uint8_t *lora_package, uint8_t package_size, uint32_t freq)
{
	LoRa_gotoMode(lora, STNBY_MODE);
	LoRa_setFrequency(lora, freq); // 920400000
	if (LoRa_transmit(lora, lora_package, package_size, 1000)) {
		// 9 LORAWAN protocol excepts FOPTS + MAC + FRM_PAYLOAD
		return 0;
	}
	return -1;
}

static uint8_t auth_tag[16] = {0};

static struct loramac_phys_payload_join_accept ja_encrypt_out = {0};
static struct join_accept_xskey_input ja_keys_in = {0};
static uint8_t nonce[16] = {0};
static uint64_t ja_size_out = 0;

int32_t process_lorawan_join_accept(uint8_t *nwkskey_out, uint8_t *appskey_out, uint32_t *dev_addr_out, uint8_t *in)
{

	uint8_t i;
	// Fetch AppNonce from Flash
	// already fetch when init
	uint32_t appnonce_uint = (appnonce[2] << 16) | (appnonce[1] << 8) | appnonce[0];
	appnonce_uint++;
	
	appnonce[0] = appnonce_uint & 0xFF;
	appnonce[1] = (appnonce_uint >> 8) & 0xFF;
	appnonce[2] = (appnonce_uint >> 16) & 0xFF;

	// JoinAccept Noince = MHDR + AppNonce + Padding
	nonce[0] = in[0];
	nonce[1] = appnonce[0];
	nonce[2] = appnonce[1];
	nonce[3] = appnonce[2];
	// Don't count MHDR in
	uint64_t inlen = sizeof(struct loramac_phys_payload_join_accept) - 1;
	// Decrypt
	int32_t retval = crypto_aead_decrypt(ja_encrypt_out.app_nonce, &ja_size_out,
																			 NULL, &in[1],
	                                     inlen, &in[0],
																			 1, nonce,
																			 appkey);
	if (retval){
		return retval;
	}

	// get appskey
	memcpy(ja_keys_in.app_nonce, ja_encrypt_out.app_nonce, 3);
	memcpy(ja_keys_in.net_id, ja_encrypt_out.net_id, 3);
	memcpy(ja_keys_in.dev_nonce, devnonce, 2);
	ja_keys_in.byte1 = 0x02;
	crypto_auth(appskey_out, &ja_keys_in.byte1, sizeof(struct join_accept_xskey_input), appkey);

	// get devaddr
	*dev_addr_out = LE_BYTES_TO_UINT32(ja_encrypt_out.dev_addr);

	// Write back
	Flash_Write_Data(APPNONCE_STORAGE_LOCATION, &appnonce_uint, 1);

	return 0;
}

int32_t encrypt_lorawan(struct loramac_phys_payload *loramac_payload, 
												uint8_t *data, uint8_t data_size,
												uint16_t loramac_f_cnt, uint8_t f_port,
												uint8_t f_ctrl, uint8_t *lora_package,
												uint8_t *lora_package_size)
{
	if (f_ctrl != 0) {
		// Currently we don't support FOpts field so FCtrl must be 0
		return -1;
	}

	loramac_fill_fhdr(loramac_payload, dev_addr, f_ctrl, loramac_f_cnt, 0);
	loramac_fill_mac_payload(loramac_payload, f_port, data);
	loramac_fill_phys_payload(loramac_payload, LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_UP, 0);

	// fully constructed LoRaWAN package: header + frm_payload + MIC (tag)
	return loramac_enc_aead(loramac_payload, lora_package, lora_package_size, data_size, appskey);
}

int32_t decrypt_lorawan(uint8_t *lora_raw_data, uint8_t size, struct loramac_phys_payload *payload, uint8_t *out_frm_payload_size)
{
	static uint8_t out_frm_payload[255] = {0};
	
	uint8_t frm_payload_size = size - (1 + 4 + 1 + 2 + 1 + 16); /* [MHDR + FHDR[DevAddr + ..] + FPORT + MIC] */

	int res = loramac_dec_aead(payload, out_frm_payload, out_frm_payload_size, lora_raw_data, size, appskey);
	// Decryption error
	if (res){
		return res;
	}
	// Size expectation did not meet
	if (frm_payload_size != *out_frm_payload_size){
		return -1;
	}
	// no support for FOpts
	if (payload->mac_payload.f_hdr.f_ctrl & 0xF) {
		return -2;
	}
	loramac_fill_mac_payload(payload, payload->mac_payload.f_port, out_frm_payload);
	return 0;
}

int32_t enable_peripherals_clock(void)
{
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	return 0;
}

int32_t disable_peripherals_clock(void)
{
	__HAL_RCC_SPI1_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();
	TIM4_Disable();
	return 0;
}

// Sensor output is float Temperature, Humidity and Pressure. It's declared as global variables
void read_sensor()
{
	char buffer[256] = {0};
	BME280_WakeUP();
	BME280_Measure();
	myData[0] = (uint8_t)Humidity;
	myData[1] = ((uint8_t)(Humidity * 10)) % 10;
	myData[2] = (uint8_t)Temperature;
	myData[3] = ((uint8_t)(Temperature * 10)) % 10;
	myData[4] = myData[0] + myData[1] + myData[2] + myData[3];
#ifdef UART_DBG
	snprintf(buffer, sizeof(buffer), "[read_sensor] T: %.2f - H: %.2f\n\r", Temperature, Humidity);
	log_debug(buffer);
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	prog.fsm = INIT;
	prog.joined = 0;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	TIM4_EnablePeripheral();
	TIM2_EnablePeripheral_IT();

  /*Configure GPIO SDO low*/
  HAL_GPIO_WritePin(SDO_GPIO_Port, SDO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_SET);

	myLoRa = newLoRa();

	myLoRa.CS_port         = NSS_GPIO_Port;
	myLoRa.CS_pin          = NSS_Pin;
	myLoRa.reset_port      = RST_GPIO_Port;
	myLoRa.reset_pin       = RST_Pin;
	myLoRa.DIO0_port       = DIO0_GPIO_Port;
	myLoRa.DIO0_pin        = DIO0_Pin;
	myLoRa.hSPIx           = &hspi1;


	HAL_Delay(3000);

	// Read join-accept appnonce
	uint32_t dataRead = 0;
	Flash_Read_Data(APPNONCE_STORAGE_LOCATION, &dataRead, 1);
	appnonce[0] = dataRead & 0xFF;
	appnonce[1] = (dataRead >> 8) & 0xFF;
	appnonce[2] = (dataRead >> 16) & 0xFF;

	if (LoRa_init(&myLoRa) == LORA_OK) {
		LoRa_stat = 1;
		LoRa_setSyncWord(&myLoRa, 0x12);
	} else {
		//log_debug("[main] FAILED: config LoRa\n\r");
	}

	while (BME280_Config(OSRS_1, OSRS_1, OSRS_1, MODE_FORCED, T_SB_0p5, IIR_OFF)) {
		LoRa_stat = 0;
		//log_debug("[main] FAILED: config BME280\n\r");
		HAL_Delay(1000);
	}

	LoRa_startReceiving(&myLoRa);

	uint8_t time_sleep = 5;
	
	loramac_payload = loramac_init();
	uint16_t loramac_f_cnt = 0;
	uint8_t lorawan_package[30] = {0};
	uint8_t lorawan_package_length = 0;
	uint8_t lorawan_decrypted_out_size = 0;
	uint8_t data_size;
	loramac_pack_join_request(&loramac_jr, appeui, deveui, devnonce, appkey);

#ifdef TEST_PKT
	loramac_payload_test = &_loramac_payload_test;
	loramac_fill_mac_payload(loramac_payload_test, 100, NULL);
	loramac_fill_phys_payload(loramac_payload_test, LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_UP, 0);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//log_debug("[main] Done system config\n\r");
  while (1)
  {
		if (time_sleep >= TIME_SLEEP_MAX) {
			time_sleep = 0;
			// If LoRa received packet, process it
			if (prog.fsm == LORA_RX_PKT_RDY) {
				//log_debug("[main] Received downlink message\n\r");
				// Check join-accept
				if (!prog.joined) {
					uint8_t m_hdr = lorawan_rx_buffer[0];
					if (m_hdr == LORAMAC_PHYS_PAYLOAD_JOIN_ACCEPT) {
						// Encrypt the payload to get DevAddr (assigned by Network server), AppsKey and NwKsKey
						if (process_lorawan_join_accept(nwkskey, appskey, &dev_addr, (uint8_t *)lorawan_rx_buffer) == 0) {
							//log_debug("[main] Valid join-accept\n\r");
							prog.joined = 1;
						} else {
							//log_debug("[main] Invalid join-accept\n\r");
						}
					}
				} else {
					int32_t ret = decrypt_lorawan((uint8_t *)lorawan_rx_buffer, lorawan_rx_buf_size, &lorawan_rx_phys, &lorawan_decrypted_out_size);
					if (!ret) {
						//log_debug("[main] Valid downlink message\n\r");
						// Toggle device, currently frm_payload is unused
						// frm_payload is in lorawan_rx_phys
						if (lorawan_decrypted_out_size == 3) {
							LoRa_stat ^= 0x1;
						}
					} else {
						//log_debug("[main] Invalid downlink message");
					}
				}
			}
			// If LoRa is OK then proceed to next step
			if (LoRa_stat) {
				// Perform join-request at startup
				if (!prog.joined) {
					prog.fsm = LORA_TX_JOIN_REQ_STARTED;
					//log_debug("[main] Send join-request\n\r");
					// Transmit join-request message
					LoRa_transmit(&myLoRa, (uint8_t *)loramac_jr, sizeof(struct loramac_phys_payload_join_request), 1000);
					// Now wait for downlink join-accept message
				} else if (prog.joined && prog.fsm != LORA_RX_PKT_RDY) {
					// Perform sensor measurement
					read_sensor();
					prog.fsm = LORA_TX;
					data_size = sizeof(myData);
#ifdef TEST_PKT
					// Ensure TIM4 clock is enabled
					__HAL_RCC_TIM4_CLK_ENABLE();
					TIM4_Start();

					// Reset the counter
					__HAL_TIM_SET_COUNTER(&htim4, 0);
					start = __HAL_TIM_GET_COUNTER(&htim4);
#endif
					// Encrypt and package the LoRaWAN packet
					if (encrypt_lorawan(loramac_payload, myData, data_size, loramac_f_cnt, 1, 0, lorawan_package, &lorawan_package_length)) {
						//log_debug("[main] FAILED: Encrypt LoRa message\n\r");
						goto exit_tx;
					}
#ifdef TEST_PKT
					// Get the timer counter
					end = __HAL_TIM_GET_COUNTER(&htim4);
					// End time
					TIM4_Disable();
					uint32_t clk_elapsed = end - start;
					// Sanity check
					if (end >= start) {
						uint8_t data_test[5] = {0};
					  uint8_t data_size_test = sizeof(data_test);

						memcpy(data_test, (uint8_t *)&clk_elapsed, 3);
						data_test[3] = data_size;
						// Encrypt the time elapsed
						encrypt_lorawan(loramac_payload_test, data_test, data_size_test, loramac_f_cnt, 100, 0, lorawan_package, &lorawan_package_length);
						// Transmit it
						if (lorawan_transmit(&myLoRa, lorawan_package, lorawan_package_length, 920200000) == 0) {
							loramac_f_cnt += 1;
						}
					}
#else
					// Transmit using LoRa transceiver module
					//log_debug("[main] Sending LoRa message\n\r");
					if (lorawan_transmit(&myLoRa, lorawan_package, lorawan_package_length, 920200000) == 0) {
						loramac_f_cnt += 1;
					}
#endif
				}
exit_tx:
				prog.fsm = LORA_RX;
				LoRa_gotoMode(&myLoRa, STNBY_MODE);
				LoRa_setFrequency(&myLoRa, 921400000);
				LoRa_startReceiving(&myLoRa);
				// RX1 RECEIVE_DELAY1
				HAL_Delay(4000);
			}
		}
		// Sleep if didn't receive any downlink
		if (prog.fsm != LORA_RX_PKT_RDY) {
			LoRa_gotoMode(&myLoRa, SLEEP_MODE);
			//log_debug("[main] Entering sleep mode\n\r");
			/* Start timer interrupt */
			TIM2_Start_IT();
			/* Suspend SYSTICK to not wake up from sleep */
			HAL_SuspendTick();
			prog.fsm = MCU_SLEEP;
			/* Enter sleep mode, will be wake up by timer*/
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			/* Check if wake from GPIO_EXTI */
			while (prog.fsm == LORA_GPIO_INT) {
				/* Sleep again */
				HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			}
			/* Start SYSTICK again */
			HAL_ResumeTick();
			/* Enable peripherals clocks */
			/* Disable timer interrupt to process other things */
			TIM2_Disable_IT();
			//log_debug("[main] Wake from sleep mode\n\r");
		}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
	prog.fsm = LORA_TIMER_INT;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (prog.fsm == MCU_SLEEP || prog.fsm == LORA_RX) {
		if (GPIO_Pin == DIO0_Pin) {
			lorawan_rx_buf_size = LoRa_receive(&myLoRa, (uint8_t *)lorawan_rx_buffer, RX_BUFFER_SIZE);
			prog.fsm = LORA_RX_PKT_RDY;
		}
	}
	/* This prevents overwriting state when TIMER interrupt is served first then GPIO later */
	if (prog.fsm != LORA_TIMER_INT && prog.fsm != LORA_RX_PKT_RDY) {
		prog.fsm = LORA_GPIO_INT;
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
	//log_debug("[main] ERROR HANDLER");
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
