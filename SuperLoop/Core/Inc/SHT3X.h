/*
 * SHT3X.h
 *
 *  Created on: Dec 24, 2025
 *      Author: THANGTRAN
 */

#ifndef INC_SHT3X_H_
#define INC_SHT3X_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
void SHT31_SendCommand(uint16_t cmd);
void SHT31_Init(void);
void I2C_ResetBus(void);
uint8_t SHT31_ReadData(float *temperature, float *humidity);
#ifdef __cplusplus
}
#endif
#endif /* INC_SHT3X_H_ */
