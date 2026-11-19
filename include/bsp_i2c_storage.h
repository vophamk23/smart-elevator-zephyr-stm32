/**
 * @file bsp_i2c_storage.h
 * @brief Trình điều khiển bus I2C1 dùng chung (PB6/PB7) cho đồng hồ DS3231 và
 * hộp đen AT24C512.
 *
 * Mọi giao dịch trên I2C1 đều được tuần tự hóa qua @c g_i2c1_bus_mutex (có kế
 * thừa độ ưu tiên) nhằm tránh xung đột giữa Shell, thao tác ghi hộp đen của FSM
 * và tác vụ đọc RTC.
 */

#ifndef BSP_I2C_STORAGE_H_
#define BSP_I2C_STORAGE_H_

#include "elevator_types.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Kiểm tra trạng thái sẵn sàng của bus I2C1 và chip EEPROM AT24C512.
 * @return 0 ngay cả khi chip ngoại vi chưa gắn trên bo (chỉ ghi cảnh báo
 * LOG_WRN).
 */
int bsp_i2c_storage_init(void);

/**
 * @brief Quét toàn bộ dải địa chỉ 7-bit trên I2C1 (0x03..0x77) và in các thiết
 * bị có phản hồi ACK.
 * @param print_cb Hàm callback xuất chuỗi định dạng (ví dụ in ra Zephyr Shell).
 */
void bsp_i2c_scan_bus(void (*print_cb)(const char *fmt, ...));

/**
 * @brief Đọc ngày giờ hiện tại từ các thanh ghi 0x00..0x06 của DS3231 qua bus
 * I2C1.
 * @return 0 nếu đọc thành công, mã lỗi âm errno nếu thiết bị không trả lời
 * (NACK).
 */
int bsp_rtc_get_datetime(uint16_t *year, uint8_t *month, uint8_t *day,
                         uint8_t *hour, uint8_t *min, uint8_t *sec);

/**
 * @brief Cài đặt ngày giờ mới xuống các thanh ghi của DS3231 qua I2C1 (mã hóa
 * BCD).
 * @return 0 nếu ghi thành công, -EINVAL nếu giá trị thời gian truyền vào không
 * hợp lệ.
 */
int bsp_rtc_set_datetime(uint16_t year, uint8_t month, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec);

/**
 * @brief Ghi nối tiếp một bản ghi sự cố có dấu thời gian RTC và mã CRC16 vào
 * AT24C512.
 *
 * Tổ chức theo bộ đệm vòng 32 bản ghi từ địa chỉ 0x0080 (căn lề trang 128 byte
 * của EEPROM), đi kèm header có kiểm tra toàn vẹn CRC32 tại địa chỉ 0x0000.
 *
 * @param code        Mã lỗi hoặc lý do khởi động.
 * @param state       Trạng thái HSM tại thời điểm phát sinh sự cố.
 * @param floor       Tầng cabin hiện tại (1..7).
 * @param temp_deci_c Nhiệt độ đo được (đơn vị 0.1 độ C).
 * @return 0 nếu ghi thành công, -ENODEV nếu không tìm thấy EEPROM.
 */
int bsp_blackbox_append(blackbox_fault_code_t code, elevator_state_t state,
                        uint8_t floor, int32_t temp_deci_c);

/**
 * @brief Đọc danh sách bản ghi hộp đen hợp lệ gần nhất (xếp từ mới nhất đến cũ
 * nhất).
 * @param[out] recs      Mảng nhận các bản ghi đã kiểm tra đúng CRC16.
 * @param      max_recs  Số lượng phần tử tối đa mảng @p recs chứa được.
 * @param[out] out_count Số lượng bản ghi thực tế đọc được.
 * @return 0 nếu thành công, mã lỗi âm errno nếu lỗi bus I2C/EEPROM.
 */
int bsp_blackbox_read(elevator_blackbox_rec_t *recs, size_t max_recs,
                      size_t *out_count);

/** @brief Xóa trắng header hàng đợi vòng của hộp đen trên EEPROM AT24C512. */
int bsp_blackbox_clear(void);

#endif /* BSP_I2C_STORAGE_H_ */
