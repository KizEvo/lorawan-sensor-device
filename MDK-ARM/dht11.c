#include "main.h"
#include "dht11.h"
#include <string.h>

#define DHT11_HIGH_LOW_COUNT_INIT_ERROR_OFFSET 130
#define DHT11_BIT_HIGH_TIME_US 78
#define DHT11_DATA_MAX_BIT 40

extern void delay_us(uint32_t time_us);

int32_t dht11_init(dht11 *sensor)
{
	if (sensor == NULL) {
		return -1;
	}
	
	int32_t rc = -1;
	
	GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = sensor->data_pin;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
	
	HAL_GPIO_Init(sensor->data_port, &GPIO_InitStructPrivate); // set the pin as output

	
	HAL_GPIO_WritePin(sensor->data_port, sensor->data_pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(sensor->data_port, sensor->data_pin, GPIO_PIN_SET);
	delay_us(30);

	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(sensor->data_port, &GPIO_InitStructPrivate); // set the pin as input
	
	delay_us(40);
	
	if (!HAL_GPIO_ReadPin(sensor->data_port, sensor->data_pin)) {
		delay_us(80);
		if (HAL_GPIO_ReadPin(sensor->data_port, sensor->data_pin)) {
			rc = 0;
		}
	}

	//while (HAL_GPIO_ReadPin(sensor->data_port, sensor->data_pin));
	
	return rc;
}

int32_t dht11_read(dht11 *sensor)
{
	if (sensor == NULL) {
		return -1;
	}
	
	if (dht11_init(sensor) == 0) {
		uint8_t bit_val = 0;
		uint8_t current_byte = 0;
		uint8_t lsb_shift = 0;
		memset(sensor->data, 0x00, 5);
		for (uint8_t bit = 0; bit < DHT11_DATA_MAX_BIT; bit++) {
			while (!HAL_GPIO_ReadPin(sensor->data_port, sensor->data_pin));
			
			delay_us(40);
			
			if (HAL_GPIO_ReadPin(sensor->data_port, sensor->data_pin)) {
				bit_val =1;
			}
			
			sensor->data[current_byte] |= (bit_val << (7 - lsb_shift));
			bit_val = 0;
			lsb_shift++;

			if (lsb_shift >= 8) {
				lsb_shift = 0;
				current_byte++;
			}
			
			while (HAL_GPIO_ReadPin(sensor->data_port, sensor->data_pin));
		}
		// verify checksum
    if(sensor->data[4] == (sensor->data[0] + sensor->data[1] + sensor->data[2] + sensor->data[3]))
    {
			// convert temperature and humidity values
			sensor->humidity = (float)sensor->data[0] + (float)(sensor->data[1] / 10.0);
			sensor->temperature = (float)sensor->data[2] + (float)(sensor->data[3] / 10.0);
			return 0;
    }
	}
	return -1;
}
