

#ifndef INC_I2CLCD_H_
#define INC_I2CLCD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
void lcd_init (void);   // Khởi tạo LCD
void lcd_send_cmd (char cmd);  // Gửi lệnh
void lcd_send_data (char data);  // Gửi dữ liệu ký tự
void lcd_send_string (char *str); // Gửi chuỗi
void lcd_put_cur(int row, int col);  // Định vị con trỏ
void lcd_clear (void); // Xóa màn hình
#ifdef __cplusplus
}
#endif

#endif /* INC_I2CLCD_H_ */
