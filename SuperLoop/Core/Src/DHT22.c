/*
 * DHT22.c
 *
 *  Created on: Jan 29, 2026
 *      Author: THANGTRAN
 */
#include "DHT22.h"

// Hàm delay micro giây sử dụng Timer 1
void delay_us (uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void DHT_Start (void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 0);
    HAL_Delay (1);   // DHT22 chỉ cần chờ 1ms (khác với DHT11 cần 18ms)
    HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 1);
    delay_us (30);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

uint8_t DHT_Check_Response (void) {
    uint8_t Response = 0;
    delay_us (40);
    if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) {
        delay_us (80);
        if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) Response = 1;
        else Response = 0;
    }
    while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));
    return Response;
}

uint8_t DHT_Read (void) {
    uint8_t i, j;
    for (j=0; j<8; j++) {
        while (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));
        delay_us (40);
        if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) {
            i &= ~(1<<(7-j));
        } else {
            i |= (1<<(7-j));
        }
        while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));
    }
    return i;
}

// Hàm lấy dữ liệu cho DHT22
uint8_t DHT_GetData (float *Temp, float *Hum) {
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, CheckSum;
    DHT_Start();
    if (DHT_Check_Response()) {
        Rh_byte1 = DHT_Read();
        Rh_byte2 = DHT_Read();
        Temp_byte1 = DHT_Read();
        Temp_byte2 = DHT_Read();
        CheckSum = DHT_Read();

        if (CheckSum == ((Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2) & 0xFF)) {
            // Công thức tính cho DHT22 (số 16-bit có dấu)
            *Hum = (float)((Rh_byte1 << 8) | Rh_byte2) / 10.0f;

            int16_t rawTemp = (Temp_byte1 << 8) | Temp_byte2;
            if (rawTemp & 0x8000) { // Nếu là nhiệt độ âm
                *Temp = (float)(rawTemp & 0x7FFF) / -10.0f;
            } else {
                *Temp = (float)rawTemp / 10.0f;
            }
            return 1; // OK
        }
    }
    return 0; // Error
}

