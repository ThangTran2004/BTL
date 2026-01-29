/*
 * DHT22.h
 *
 *  Created on: Jan 29, 2026
 *      Author: THANGTRAN
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_

#include "stm32f1xx_hal.h" // Thư viện HAL cho dòng F1
#include "tim.h"           // Để sử dụng htim1 cho delay_us

/* Cấu hình Pin cho DHT22 - Bạn có thể thay đổi tùy theo kết nối thực tế */
#define DHT_PORT GPIOA
#define DHT_PIN  GPIO_PIN_2

/* Cấu trúc dữ liệu lưu trữ giá trị đo được */
typedef struct {
    float Temperature;
    float Humidity;
} DHT_Data_t;

/* Prototypes các hàm được định nghĩa trong DHT.c */

/**
 * @brief Tạo độ trễ micro giây sử dụng Timer
 * @param us: Số micro giây cần delay
 */
void delay_us (uint16_t us);

/**
 * @brief Gửi tín hiệu Start tới cảm biến DHT22
 */
void DHT_Start (void);

/**
 * @brief Kiểm tra phản hồi (Response) từ cảm biến
 * @retval 1 nếu có phản hồi, 0 hoặc -1 nếu lỗi
 */
uint8_t DHT_Check_Response (void);

/**
 * @brief Đọc 1 byte dữ liệu từ DHT22
 * @retval Byte dữ liệu đọc được
 */
uint8_t DHT_Read (void);

/**
 * @brief Hàm tổng hợp để lấy nhiệt độ và độ ẩm
 * @param DHT_Data: Con trỏ tới cấu trúc chứa dữ liệu trả về
 * @retval 1 nếu thành công, 0 nếu lỗi (sai CheckSum hoặc không phản hồi)
 */
uint8_t DHT_GetData (float *Temp, float *Hum);

#endif /* INC_DHT_H_ */
