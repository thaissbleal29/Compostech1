#ifndef INC_CAPACITIVE_SOIL_MOISTURE_V2_H_
#define INC_CAPACITIVE_SOIL_MOISTURE_V2_H_

#include "main.h"

#define ADC_MAX_MOISTURE 1200		// Значение ADC достигнутое при максимальной влажности почвы
#define ADC_MIN_MOISTURE 2400		// Значение ADC достигнутое при минимальной влажности почвы

/* Структура объекта датчика влажности почвы*/
typedef struct {
	ADC_HandleTypeDef *hadc;	//Порт датчика
} CSMV2_sensor;

/* Структура объекта данных полученных с датчика */
typedef struct {
	uint16_t adc_value;			// Значение полученное с помощью ADC
	uint8_t moisture_percent;	// Примерное значение влажности в процентах
} CSMV2_data;

CSMV2_data CSMV2_getData(CSMV2_sensor *sensor);

#endif
