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

void BME280_Init(){
	LED_ON;
	HAL_I2C_Init(&BME280_I2C_HANDLER);
	uint8_t id_of_chip = BME280_GetID();
	BME280_ReadCalibration();
	BME280_SetOversampling(BME280_OVERSAMPLING_X8, BME280_OVERSAMPLING_X4, BME280_OVERSAMPLING_X4, BME280_MODE_NORMAL);
	//BME280_SetConfig(BME280_STANDBY_TIME_10, BME280_FILTER_16, BME280_3WIRE_SPI_OFF);
	//return id_of_chip;
}

void Error(){

}

void BME280_Check_Conection(uint8_t * result){
	HAL_StatusTypeDef res;
	res = HAL_I2C_IsDeviceReady(&BME280_I2C_HANDLER, BME280_ADDR, 3, 100);
	* result = res;
}
//===========Reading data from I2C==============================
uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	status = HAL_I2C_Mem_Read(&BME280_I2C_HANDLER, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
	//printf("%d\n\r", status);
	if (status != HAL_OK) Error();
	return value;
}
uint8_t BME280_ReadReg(uint8_t Reg){
	uint8_t res = 0;
	res = I2Cx_ReadData(BME280_ADDR, Reg);
	return res;
}
void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&BME280_I2C_HANDLER, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)Value, 2, 0x10000);
	if (status != HAL_OK) Error();
}
void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&BME280_I2C_HANDLER, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)Value, 3, 0x10000);
	if (status != HAL_OK) Error();
}
//==========Writing data to I2C=============================
void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value){
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&BME280_I2C_HANDLER, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,  &Value, 1, 0x10000);
	if (status != HAL_OK) Error();
}
//==========Writing register to BME280=====================
void BME280_WriteReg(uint8_t Reg, uint8_t Value){
	I2Cx_WriteData(BME280_ADDR, Reg, Value);
}

//==========Reading of different registers in BME280===========
void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value){
	I2Cx_ReadData16(BME280_ADDR,Reg, (uint16_t*) Value);
}
void BME280_ReadReg_S24(uint8_t Reg, int32_t *Value){
	I2Cx_ReadData24(BME280_ADDR,Reg, (uint32_t*) Value);
	*(int32_t *) Value &= 0x00FFFFFF;
}
void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value){
	I2Cx_ReadData16(BME280_ADDR,Reg, Value);
}
void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value){
	I2Cx_ReadData24(BME280_ADDR,Reg,  Value);
	*(uint32_t *) Value &= 0x00FFFFFF;
}
void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value)
{
  I2Cx_ReadData24(BME280_ADDR,Reg,Value);
  *(uint32_t *) Value = be24toword(*(uint32_t *) Value) & 0x00FFFFFF;
}

void BME280_ReadCalibration(){
	//function read calibration_data from sensor, needed for correct measuring of all parameters
	BME280_ReadReg_U16(CALIBRATION_T1, & BME280_Cal_par.T1);
	BME280_ReadReg_S16(CALIBRATION_T2, & BME280_Cal_par.T2);
	BME280_ReadReg_S16(CALIBRATION_T3, & BME280_Cal_par.T3);
	BME280_ReadReg_U16(CALIBRATION_P1, & BME280_Cal_par.P1);
	BME280_ReadReg_S16(CALIBRATION_P2, & BME280_Cal_par.P2);
	BME280_ReadReg_S16(CALIBRATION_P3, & BME280_Cal_par.P3);
	BME280_ReadReg_S16(CALIBRATION_P4, & BME280_Cal_par.P4);
	BME280_ReadReg_S16(CALIBRATION_P5, & BME280_Cal_par.P5);
	BME280_ReadReg_S16(CALIBRATION_P6, & BME280_Cal_par.P6);
	BME280_ReadReg_S16(CALIBRATION_P7, & BME280_Cal_par.P7);
	BME280_ReadReg_S16(CALIBRATION_P8, & BME280_Cal_par.P8);
	BME280_ReadReg_S16(CALIBRATION_P9, & BME280_Cal_par.P9);
	BME280_Cal_par.H1 = (unsigned char) BME280_ReadReg(CALIBRATION_H1);
	BME280_ReadReg_S16(CALIBRATION_H2, & BME280_Cal_par.H2);
	BME280_Cal_par.H3 = (unsigned char) BME280_ReadReg(CALIBRATION_H3);
	BME280_ReadReg_S16(CALIBRATION_H4, & BME280_Cal_par.H4);
	BME280_ReadReg_S16(CALIBRATION_H5, & BME280_Cal_par.H5);
	BME280_Cal_par.H6 = (char) BME280_ReadReg(CALIBRATION_H6);
	/* uncoment for check all calibration value
	printf("Printing calibration parameters:\n\rT1: %d\n\rT2: %d\n\rT3: %d\n\r", BME280_Cal_par.T1, BME280_Cal_par.T2, BME280_Cal_par.T3);
	printf("P1: %d\n\rP2: %d\n\rP3: %d\n\rP4: %d\n\rP5: %d\n\rP6: %d\n\rP7: %d\n\rP8: %d\n\rP9: %d\n\r", BME280_Cal_par.P1, BME280_Cal_par.P2, BME280_Cal_par.P3, BME280_Cal_par.P4, BME280_Cal_par.P5, BME280_Cal_par.P6, BME280_Cal_par.P7, BME280_Cal_par.P8, BME280_Cal_par.P9);
	printf("H1: %d\n\rH2: %d\n\rH3: %d\n\rH4: %d\n\rH5: %d\n\rH6: %d\n\r", BME280_Cal_par.H1, BME280_Cal_par.H2, BME280_Cal_par.H3, BME280_Cal_par.H4, BME280_Cal_par.H6);
	*/
}

void BME280_SetOversampling(uint8_t oversampling_temp, uint8_t oversampling_pres, uint8_t oversampling_hum, uint8_t mode)
{
	//Writing a measuring mode, use defined macros BME280_OVERSAMPLING_X... and BME280_MODE_...
	HAL_StatusTypeDef res;
	res = HAL_OK;
	if (oversampling_hum != BME280_OVERSAMPLING_X1 && oversampling_hum != BME280_OVERSAMPLING_X2 && oversampling_hum != BME280_OVERSAMPLING_X4 && oversampling_hum != BME280_OVERSAMPLING_X4 && oversampling_hum != BME280_OVERSAMPLING_X8 && oversampling_hum != BME280_OVERSAMPLING_X16){
		res = HAL_ERROR;
		return res;
	}
	if (oversampling_temp != BME280_OVERSAMPLING_X1 && oversampling_temp != BME280_OVERSAMPLING_X2 && oversampling_temp != BME280_OVERSAMPLING_X4 && oversampling_temp != BME280_OVERSAMPLING_X4 && oversampling_temp != BME280_OVERSAMPLING_X8 && oversampling_temp != BME280_OVERSAMPLING_X16){
		res = HAL_ERROR;
		return res;
	}
	if (oversampling_pres != BME280_OVERSAMPLING_X1 && oversampling_pres != BME280_OVERSAMPLING_X2 && oversampling_pres != BME280_OVERSAMPLING_X4 && oversampling_pres != BME280_OVERSAMPLING_X4 && oversampling_pres != BME280_OVERSAMPLING_X8 && oversampling_pres != BME280_OVERSAMPLING_X16){
		res = HAL_ERROR;
		return res;
	}
	if(mode != BME280_MODE_SLEEP && mode != BME280_MODE_FORCED && mode != BME280_MODE_NORMAL){
		res = HAL_ERROR;
		return res;
	}
	BME280_WriteReg(REG_CTRL_HUM, oversampling_hum);
	uint8_t reg_cntl_meas_value;
	reg_cntl_meas_value = ((oversampling_temp)<<5)|((oversampling_pres)<<2)|(mode);
	printf("ctrl_meas: %d\r\n", reg_cntl_meas_value);
	BME280_WriteReg(REG_CTRL_MEAS, reg_cntl_meas_value);
	//return res;
}
void BME280_GetOversamplingMode(uint8_t *array){
	//write array oversampling temperature, pressure, humidity, and mode
	uint8_t ovrs_hum, ovrs_temp, ovrs_pres, mode;
	uint8_t value_ctrl_meas;
	ovrs_hum = BME280_ReadReg(REG_CTRL_HUM);
	value_ctrl_meas = BME280_ReadReg(REG_CTRL_MEAS);
	ovrs_temp = (value_ctrl_meas&0b11100000)>>5;
	ovrs_pres = (value_ctrl_meas&0b00011100)>>2;
	mode = value_ctrl_meas&0b00000011;
	*array = ovrs_temp;
	*(array + sizeof(uint8_t)) = ovrs_pres;
	*(array + 2 * sizeof(uint8_t)) = ovrs_hum;
	*(array + 3 * sizeof(uint8_t)) = mode;
	//printf("temp: %d\r\npres: %d\r\nhum: %d\r\nmode: %d\r\n", ovrs_temp, ovrs_pres, ovrs_hum, mode);
	//return value_ctrl_meas;
}
