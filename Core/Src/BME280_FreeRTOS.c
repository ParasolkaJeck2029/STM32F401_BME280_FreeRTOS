/*
 * BME280_FreeRTOS.c
 *
 *  Created on: Jul 19, 2022
 *      Author: ParasolkaJeck
 */

#include "BME280_FreeRTOS.h"

extern I2C_HandleTypeDef BME280_I2C_HANDLER;
extern UART_HandleTypeDef huart1;
BME280_Calibrate_parametrs BME280_Cal_par;

int32_t temp_int;

void BME280_Check_Conection(uint8_t * result){
	HAL_StatusTypeDef res;
	res = HAL_I2C_IsDeviceReady(&BME280_I2C_HANDLER, BME280_ADDR, 3, 100);
	* result = res;
}
