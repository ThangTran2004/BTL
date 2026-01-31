/*
 * SHT3X.c
 *
 *  Created on: Dec 24, 2025
 *      Author: THANGTRAN
 */
#include "SHT3X.h"
#define SHT31_ADDR (0x44 << 1)
extern I2C_HandleTypeDef hi2c1;
void SHT31_SendCommand(uint16_t cmd)
{
	uint8_t buffer[2];
	buffer[0] = (cmd >> 8) & 0xFF;
	buffer[1] = cmd & 0xFF;
	HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, buffer, 2, 100);
}
void SHT31_Init(void)
{
	HAL_Delay(10);
	SHT31_SendCommand(0x30A2); // Soft reset
	HAL_Delay(10);
}
void I2C_ResetBus(void)
{
    __HAL_I2C_DISABLE(&hi2c1);
    HAL_Delay(5);
    __HAL_I2C_ENABLE(&hi2c1);
}
uint8_t SHT31_ReadData(float *temperature, float *humidity)
{
	uint8_t data[6]; // Gửi lệnh đo độ chính xác cao
	SHT31_SendCommand(0x2400);
	HAL_Delay(20); // SHT31 cần 10–15ms để đo xong // Đọc 6 byte
	if (HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, data, 6, 100) != HAL_OK)
		{
		I2C_ResetBus();
		return 0;
		}
	uint16_t rawTemp = (data[0] << 8) | data[1];
	uint16_t rawHum = (data[3] << 8) | data[4];
	*temperature = -45 + (175.0 * rawTemp / 65535.0);
	*humidity = 100 * rawHum / 65535.0;
	return 1;
}
