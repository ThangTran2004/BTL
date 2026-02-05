#ifndef __DHT22_H__
#define __DHT22_H__

#include "stm32f1xx_hal.h"

typedef struct {
    GPIO_TypeDef *GPIOx;       // Chân GPIO
    uint16_t GPIO_Pin;         // Pin GPIO
    TIM_HandleTypeDef *htim;   // Timer dùng để delay µs
} DHT22_HandleTypeDef;

typedef struct {
    float Temperature;
    float Humidity;
} DHT22_Data_t;

void DHT22_Init(DHT22_HandleTypeDef *dht);
int DHT22_Read(DHT22_HandleTypeDef *dht, DHT22_Data_t *data);

#endif
