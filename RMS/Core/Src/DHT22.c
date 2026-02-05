#include "dht22.h"
#include "cmsis_os.h"

// --- CÁC HÀM NỘI BỘ (STATIC) ---

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
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(dht->GPIOx, &GPIO_InitStruct);
}

// Hàm delay micro-giây dùng Timer phần cứng

static void delay_us(DHT22_HandleTypeDef *dht, uint16_t us) {
    __HAL_TIM_SET_COUNTER(dht->htim, 0);
    while (__HAL_TIM_GET_COUNTER(dht->htim) < us);
}

// --- CÁC HÀM PUBLIC ---

void DHT22_Init(DHT22_HandleTypeDef *dht) {
    DHT22_SetPinOutput(dht);
    HAL_GPIO_WritePin(dht->GPIOx, dht->GPIO_Pin, GPIO_PIN_SET);
}

int DHT22_Read(DHT22_HandleTypeDef *dht, DHT22_Data_t *data) {
    uint8_t bits[5] = {0};
    uint8_t byteIndex = 0, bitIndex = 7;
    uint32_t timeout = 0;

    /*  GIAI ĐOẠN 1: GỬI TÍN HIỆU START */

    DHT22_SetPinOutput(dht);
    HAL_GPIO_WritePin(dht->GPIOx, dht->GPIO_Pin, GPIO_PIN_RESET);
    osDelay(2);
    HAL_GPIO_WritePin(dht->GPIOx, dht->GPIO_Pin, GPIO_PIN_SET);
    DHT22_SetPinInput(dht);
    delay_us(dht, 30);

    taskENTER_CRITICAL();
    timeout = 0;
    while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_SET) {
        //  Nếu chờ quá lâu mà không thấy phản hồi thì thoát
        if (timeout++ > 1000) { taskEXIT_CRITICAL(); return 1; }
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_RESET) {
        if (timeout++ > 1000) { taskEXIT_CRITICAL(); return 2; }
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_SET) {
        if (timeout++ > 1000) { taskEXIT_CRITICAL(); return 3; }
    }

    for (int i = 0; i < 40; i++) {
        timeout = 0;
        while (HAL_GPIO_ReadPin(dht->GPIOx, dht->GPIO_Pin) == GPIO_PIN_RESET)
        {
             if(timeout++ > 2000) { taskEXIT_CRITICAL(); return 5; }
        }
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

    taskEXIT_CRITICAL();

    uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
    if (sum != bits[4]) return 4;  // Lỗi Checksum

    data->Humidity = (bits[0] << 8 | bits[1]) * 0.1f;
    data->Temperature = (bits[2] << 8 | bits[3]) * 0.1f;

    if (bits[2] & 0x80) {
        data->Temperature = -data->Temperature;
    }

    return 0;
}
