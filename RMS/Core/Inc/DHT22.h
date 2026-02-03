
#ifndef __DHT22_H
#define __DHT22_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h" // Hoặc dòng chip bạn đang dùng (f4xx, l4xx, etc.)

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief Cấu trúc lưu trữ dữ liệu đọc được từ DHT22
 */
typedef struct {
    float Temperature;
    float Humidity;
} DHT22_Data_t;

/**
 * @brief Cấu trúc cấu hình ngoại vi cho DHT22
 */
typedef struct {
    GPIO_TypeDef *GPIOx;    // Cổng GPIO (VD: GPIOA)
    uint16_t GPIO_Pin;      // Chân GPIO (VD: GPIO_PIN_1)
    TIM_HandleTypeDef *htim; // Timer dùng để tạo delay_us (VD: &htim1)
} DHT22_HandleTypeDef;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Khởi tạo chân GPIO cho DHT22
 * @param dht Pointer tới cấu trúc cấu hình DHT22
 */
void DHT22_Init(DHT22_HandleTypeDef *dht);

/**
 * @brief Đọc dữ liệu từ cảm biến DHT22
 * @param dht Pointer tới cấu trúc cấu hình DHT22
 * @param data Pointer tới cấu trúc lưu trữ kết quả Temperature và Humidity
 * @retval int Trả về mã trạng thái:
 * 0: Thành công
 * 1: Timeout khi chờ phản hồi (mức cao)
 * 2: Timeout khi chờ phản hồi (mức thấp)
 * 3: Timeout khi chờ phản hồi (chu kỳ chuẩn bị)
 * 4: Lỗi Checksum
 * 5: Timeout trong quá trình đọc bit dữ liệu
 */
int DHT22_Read(DHT22_HandleTypeDef *dht, DHT22_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __DHT22_H */
