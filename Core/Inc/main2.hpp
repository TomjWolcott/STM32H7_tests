/*
 * main.hpp
 *
 *  Created on: Oct 26, 2025
 *      Author: tomwolcott
 */

#ifndef INC_MAIN2_HPP_
#define INC_MAIN2_HPP_

#pragma once
#ifdef __cplusplus
extern "C" {
	#include <stdint.h>

	#include "main.h"
	#include "FreeRTOS.h"
	#include "semphr.h"
	#include "task.h"

	#include "lookups.h"
	#include "system_conf.h"
}

struct FreqResponseData {
	float amplitude, phase;
	uint32_t time_taken_us;
};

class FreqResponse {
	uint32_t search_freq;
	SemaphoreHandle_t semaphore;
	FreqResponseData data;

	void updateDataFromParts(uint32_t data_len, int64_t cos_part_int, int64_t sin_part_int, uint32_t time_taken_us);

public:
	FreqResponse(uint32_t freq);
	void computeResponseFromData(uint16_t *data, uint32_t data_len);
	FreqResponseData get_data();
};


#endif
#endif /* INC_MAIN2_HPP_ */
