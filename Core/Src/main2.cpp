/*
 * main.cpp
 *
 *  Created on: Oct 26, 2025
 *      Author: tomwolcott
 */

#include "main2.hpp"
#include <cmath>

FreqResponse::FreqResponse(uint32_t freq) {
	data = { 0.0, 0.0, 0 };

	search_freq = freq;
	semaphore = xSemaphoreCreateMutex();
}

void FreqResponse::updateDataFromParts(uint32_t data_len, int64_t cos_part_int, int64_t sin_part_int, uint32_t time_taken_us) {
	xSemaphoreTake(semaphore, portMAX_DELAY);
	float cos_part = 2.0 * (float)cos_part_int / ((float)data_len * 2147483647.0);
	float sin_part = 2.0 * (float)sin_part_int / ((float)data_len * 2147483647.0);
	data.amplitude = sqrt(cos_part * cos_part + sin_part * sin_part);
	data.phase = atan2(sin_part, cos_part);
	data.time_taken_us = time_taken_us;
	xSemaphoreGive(semaphore);
}

const int LOOKUP_TABLE_SIZE = (1 << 12);
const int LOOKUP_MASK = LOOKUP_TABLE_SIZE - 1;
const int ADC_PICKER_PERIOD_us = 10;
static int micAvg = 0;

void FreqResponse::computeResponseFromData(uint16_t *data, uint32_t data_len) {
	int64_t cos_part_int = 0;
	int64_t sin_part_int = 0;

	int phi_cos = LOOKUP_TABLE_SIZE / 4;
	int phi_sin = 0;
	int dphi = ((ADC_PICKER_PERIOD_us * LOOKUP_TABLE_SIZE * search_freq) / 1000000) & LOOKUP_MASK;

	for (uint32_t i = 0; i < data_len; i++) {
		int16_t adc_value = micBuffer[i] - micAvg;
		phi_cos = (phi_cos + dphi) & LOOKUP_MASK;
		phi_sin = (phi_sin + dphi) & LOOKUP_MASK;

		cos_part_int += (adc_value * sineLookupTable4096[phi_cos]);
		sin_part_int += (adc_value * sineLookupTable4096[phi_sin]);
	}

	updateDataFromParts(data_len, cos_part_int, sin_part_int, 0);
}

FreqResponseData FreqResponse::get_data() {
	FreqResponseData data_copy;

	xSemaphoreTake(semaphore, portMAX_DELAY);
	data_copy = data;
	xSemaphoreGive(semaphore);

	return data_copy;
}
