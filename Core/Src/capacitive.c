#include "capacitive.h"

/**
 * Эта функция выполняет операцию масштабирования значения из интервала [in_min, in_max] в интервал [0, 100],
 * инвертирует их и возвращает результат в процентах.
 */

uint8_t scaleToPercentage(uint16_t x, uint16_t in_min, uint16_t in_max)
{
	uint8_t out_min = 0;
	uint8_t out_max = 100;

	if (x <= in_min) {
		return 100 - out_min;
	}
	if (x >= in_max) {
		return 100 - out_max;
	}
	return 100 - ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


/* Функция для считывания данных с датчика */
CSMV2_data CSMV2_getData(CSMV2_sensor *sensor) {
	CSMV2_data data = {0, 0};

    HAL_ADC_Start(sensor->hadc);
    HAL_ADC_PollForConversion(sensor->hadc, HAL_MAX_DELAY);
    data.adc_value = HAL_ADC_GetValue(sensor->hadc);
    HAL_ADC_Stop(sensor->hadc);

    data.moisture_percent = scaleToPercentage(data.adc_value, ADC_MAX_MOISTURE, ADC_MIN_MOISTURE);

	return data;

}
