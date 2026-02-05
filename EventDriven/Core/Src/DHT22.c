#include "dht22.h"

static void DHT22_SetPinOutput(DHT22_HandleTypeDef *dht) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = dht->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(dht->GPIOx, &GPIO_InitStruct);
}

static void DHT22_SetPinInput(DHT22_HandleTypeDef *dht) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = dht->GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(dht->GPIOx, &GPIO_InitStruct);
}

static void delay_us(DHT22_HandleTypeDef *dht, uint16_t us) {
    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (__HAL_TIM_GET_COUNTER(dht->htim) < us);
}

void DHT22_Init(DHT22_HandleTypeDef *dht) {
    DHT22_SetPinOutput(dht);
    HAL_GPIO_WritePin(dht->GPIOx, dht->GPIO_Pin, GPIO_PIN_SET);
}

int DHT22_Read(DHT22_HandleTypeDef *dht, DHT22_Data_t *data) {
    uint8_t bits[5] = {0};
    uint8_t byteIndex = 0, bitIndex = 7;
    uint32_t timeout = 0;

    // Start signal
    DHT22_SetPinOutput(dht);
    HAL_GPIO_WritePin(dht->GPIOx, dht->GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);  // 1 ms – 2 ms
    HAL_GPIO_WritePin(dht->GPIOx, dht->GPIO_Pin, GPIO_PIN_SET);

    DHT22_SetPinInput(dht);
    delay_us(dht, 30);

    // Wait for DHT response (LOW)
    timeout = 0;
    while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_SET) {
        if (timeout++ > 10000) return 1;
        delay_us(dht, 1);
    }

    // Wait for HIGH
    timeout = 0;
    while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_RESET) {
        if (timeout++ > 10000) return 2;
        delay_us(dht, 1);
    }

    // Wait for LOW (start of data)
    timeout = 0;
    while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_SET) {
        if (timeout++ > 10000) return 3;
        delay_us(dht, 1);
    }

    // Read 40 bits
    for (int i = 0; i < 40; i++) {
        // Wait LOW
        while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_RESET);

        // measure HIGH time
        __HAL_TIM_SET_COUNTER(dht->htim, 0);
        while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_SET);

        uint16_t high_time = __HAL_TIM_GET_COUNTER(dht->htim);

        if (high_time > 40) bits[byteIndex] |= (1 << bitIndex);

        if (bitIndex == 0) {
            bitIndex = 7;
            byteIndex++;
        } else {
            bitIndex--;
        }
    }

    uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
    if (sum != bits[4]) return 4;  // checksum error

    data->Humidity = (bits[0] << 8 | bits[1]) * 0.1f;
    data->Temperature = (bits[2] << 8 | bits[3]) * 0.1f;

    return 0;
}
