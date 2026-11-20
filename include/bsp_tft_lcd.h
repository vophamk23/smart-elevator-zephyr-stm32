/**
 * @file bsp_tft_lcd.h
 * @brief Trình điều khiển Màn hình màu Đồ họa TFT-LCD 240x320 (ILI9341 - FSMC 16-bit)
 *        Hệ thống Thang máy Thông minh BKIT ARM4 (STM32F407ZGT6).
 *
 * Chức năng:
 *  - Khởi tạo phần cứng bus song song FSMC 16-bit (Bank 1 NE1, A19-RS, PD/PE data pins).
 *  - Cung cấp các hàm vẽ đồ họa nguyên thủy (Pixel, FillRect, DrawRect, String, BigDigit, ProgressBar).
 *  - Render giao diện HMI trực quan cho 7 trạng thái thang máy:
 *      1. Đang di chuyển (Moving UP / DOWN với hiệu ứng cuộn số mượt).
 *      2. Dừng mở cửa đón khách (Door Open 100% tại tầng).
 *      3. Cảnh báo quá tải trọng lượng (Overload > 800 kg từ biến trở VR1).
 *      4. Cảnh báo kẹt cửa / vật cản rèm quang (Obstacle từ quang trở LDR).
 *      5. Báo cháy khẩn cấp & sơ tán về Tầng 1 (Fire Alarm từ nhiệt độ LM35 >= 60 C).
 *      6. Dừng khẩn cấp do người dùng (Emergency Stop qua phím E-Stop).
 *      7. Chế độ Chờ / Nghỉ tiết kiệm điện (Standby / Night Mode).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BSP_TFT_LCD_H_
#define BSP_TFT_LCD_H_

#include <stdint.h>
#include <stdbool.h>
#include "elevator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Kích thước màn hình TFT-LCD */
#define LCD_WIDTH           240
#define LCD_HEIGHT          320

/* Bảng màu RGB565 chuẩn ILI9341 */
#define LCD_COLOR_BLACK     0x0000U
#define LCD_COLOR_WHITE     0xFFFFU
#define LCD_COLOR_NAVY      0x0810U /* Nền xanh đen Dark Navy */
#define LCD_COLOR_GRAY      0x52AAU /* Xám trung bình */
#define LCD_COLOR_DARK_GRAY 0x2104U /* Xám tối làm khung */
#define LCD_COLOR_LIGHT_GRAY 0xCE79U
#define LCD_COLOR_YELLOW    0xFFE0U /* Số tầng, nút gọi */
#define LCD_COLOR_GREEN     0x07E0U /* Trạng thái an toàn, chạy UP */
#define LCD_COLOR_CYAN      0x07FFU /* Thanh cửa mở */
#define LCD_COLOR_ORANGE    0xFD20U /* Chạy DOWN, cảnh báo mức nhẹ */
#define LCD_COLOR_RED       0xF800U /* Báo cháy, quá tải, E-Stop */

/**
 * @brief Khởi tạo phần cứng GPIO, FSMC 16-bit và gửi chuỗi lệnh khởi động cho chip ILI9341.
 * @return 0 nếu thành công, mã lỗi âm nếu thất bại.
 */
int bsp_tft_lcd_init(void);

/**
 * @brief Kiểm tra xem màn hình TFT-LCD đã được khởi tạo thành công chưa.
 */
bool bsp_tft_lcd_is_ready(void);

/**
 * @brief Điều khiển độ sáng đèn nền màn hình (chân PA8 qua GPIO hoặc PWM).
 * @param brightness_pct Độ sáng từ 0% (tắt) đến 100% (sáng tối đa).
 */
void bsp_tft_lcd_set_backlight(uint8_t brightness_pct);

/**
 * @brief Xóa toàn bộ màn hình bằng một màu đơn sắc.
 */
void bsp_tft_lcd_clear(uint16_t color);

/* --- Các hàm vẽ nguyên thủy (Graphics Primitives) --- */
void bsp_tft_lcd_draw_pixel(int x, int y, uint16_t color);
void bsp_tft_lcd_fill_rect(int x, int y, int w, int h, uint16_t color);
void bsp_tft_lcd_draw_rect(int x, int y, int w, int h, uint16_t color);
void bsp_tft_lcd_draw_char(int x, int y, char c, uint16_t color, uint16_t bg_color);
void bsp_tft_lcd_draw_string(int x, int y, const char *str, uint16_t color, uint16_t bg_color);
void bsp_tft_lcd_draw_big_digit(int x, int y, char digit, uint16_t color, uint16_t bg_color);
void bsp_tft_lcd_draw_progress_bar(int x, int y, int w, int h, int pct, uint16_t bar_color, uint16_t bg_color);

/**
 * @brief Render toàn bộ Dashboard HMI theo trạng thái thời gian thực của thang máy.
 *
 * @param tel              Con trỏ gói Telemetry chứa trạng thái FSM, tầng, hướng, cờ lỗi.
 * @param call_mask        Mask 7 bit các tầng đang được gọi (bit 0 = Tầng 1 .. bit 6 = Tầng 7).
 * @param door_pct         Phần trăm mở cửa buồng thang (0% = đóng kín, 100% = mở toang).
 * @param load_kg          Tải trọng cabin đo được từ biến trở VR1 (kg).
 * @param temp_c           Nhiệt độ giếng thang đo được từ LM35 (độ C).
 * @param curtain_blocked  true nếu rèm quang LDR bị che phát hiện vật cản.
 * @param last_log_msg     Chuỗi nhật ký sự kiện ghi vào EEPROM hộp đen.
 */
void bsp_tft_lcd_render_dashboard(const elevator_telemetry_t *tel,
                                  uint8_t call_mask,
                                  uint8_t door_pct,
                                  float load_kg,
                                  float temp_c,
                                  bool curtain_blocked,
                                  const char *last_log_msg);

/**
 * @brief Kích hoạt hiệu ứng cuộn số mượt khi thang di chuyển qua các tầng.
 * @param old_floor Tầng trước đó (1..7).
 * @param new_floor Tầng mới (1..7).
 * @param dir       Hướng di chuyển (ELEV_DIR_UP hoặc ELEV_DIR_DOWN).
 */
void bsp_tft_lcd_animate_floor_rolling(uint8_t old_floor, uint8_t new_floor, elevator_dir_t dir);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TFT_LCD_H_ */
