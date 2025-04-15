#ifndef DHT11_H
#define DHT11_H

#include "stm32f1xx_hal.h"

typedef struct dht11 {
	/* users should set the correct port and pin of MCU to read from dht11 */
	GPIO_TypeDef* data_port;
	uint16_t data_pin;
	/* these are set by dht11_read */
	float temperature;
	float humidity;
	uint8_t data[5];
} dht11;

/**
 * @brief Init dht11 to check for response
 *
 * @params sensor dht11 sensor struct
 * @retval 0 Success
 * @retval -1 Error
 */
int32_t dht11_init(dht11 *sensor);

/**
 * @brief Read dht11 temperature and humidity
 *
 * @params sensor dht111 sensor struct
 * @retval 0 Success
 * @retval -1 Error
 */
int32_t dht11_read(dht11 *sensor);

#endif
