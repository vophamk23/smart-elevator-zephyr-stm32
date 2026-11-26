/**
 * @file bsp_spi_hmi.h
 * @brief Trình điều khiển bus SPI1 dùng chung (LED 7 đoạn + Bàn phím) & thăm dò Flash SPI2.
 *
 * Cơ chế chia sẻ bus & Kế thừa độ ưu tiên:
 *  - Bus SPI1 (PB3/PB4/PB5) phục vụ chung cho LED 7 đoạn 4 số (74HC595 chốt bằng PG6 LD_LATCH)
 *    và ma trận phím 4x4 (nạp song song bằng PD3 BTN_LOAD). Mọi giao dịch SPI1 đều khóa bằng
 *    @c g_spi1_bus_mutex để chống xung đột và đảo ngược ưu tiên.
 *  - Bus SPI2 (PB12..PB15) giao tiếp với chip nhớ Flash ngoài W25Q64FV trên bo mạch.
 */

#ifndef BSP_SPI_HMI_H_
#define BSP_SPI_HMI_H_

#include <stdint.h>
#include "elevator_types.h"

/**
 * @brief Khởi tạo các chân điều khiển chốt/nạp (PG6, PD3) của SPI1 và chân CS (PB12) của SPI2.
 * @return 0 nếu thành công, mã lỗi âm errno nếu GPIO điều khiển chưa sẵn sàng.
 */
int bsp_spi_hmi_init(void);

/**
 * @brief Đẩy dữ liệu mã 7 đoạn vào bộ đệm khung (framebuffer) của bộ quét ghép 500 Hz trên SPI1.
 * @param digits Mảng 4 byte mã segment Anode chung (0 = sáng, 1 = tắt) cho các ô LED 0..3.
 * @return 0 nếu cập nhật thành công, mã lỗi âm errno nếu lỗi tham số/thiết bị.
 */
int bsp_spi_7seg_write_raw(const uint8_t digits[4]);

/**
 * @brief Giữ mẫu kiểm thử cố định trên 4 LED 7 đoạn trong @p hold_ms mili-giây (phục vụ `bkit test spi1`).
 */
void bsp_spi_7seg_hold_test_pattern(const uint8_t digits[4], uint32_t hold_ms);

/**
 * @brief Hiển thị tầng hiện tại, trạng thái cửa, quá tải, hướng di chuyển và mã lỗi ('FE' / 'Err') lên 4 LED 7 đoạn.
 * @param floor      Tầng hiện tại của thang máy (1..7).
 * @param dir        Hướng chạy (LÊN / XUỐNG / DỪNG).
 * @param door_open  true nếu cửa đang mở/đang đóng đón khách ('OP N').
 * @param overload   true nếu buồng thang đang quá tải >= 800 kg (nhấp nháy 'FULL').
 * @param fire       true nếu đang kích hoạt sơ tán hỏa hoạn ('FE').
 * @param fault      true nếu đang dừng khẩn cấp E-Stop hoặc lỗi mất CAN ('Err').
 * @return 0 nếu hiển thị thành công.
 */
int bsp_spi_7seg_render_status(uint8_t floor, elevator_dir_t dir, bool door_open,
			       bool overload, bool fire, bool fault);

/**
 * @brief Kéo chốt PD3 (BTN_LOAD) và dịch đọc 16 bit trạng thái ma trận phím 4x4 qua SPI1.
 * @param[out] key_idx   Chỉ số phím đang nhấn nhỏ nhất (0..15), hoặc -1 nếu không có phím nhấn.
 * @param[out] raw_bytes Bộ đệm 2 byte dữ liệu thô đọc về phục vụ gỡ lỗi (có thể NULL).
 * @return 0 nếu quét thành công.
 */
int bsp_spi_keypad_scan(int *key_idx, uint8_t raw_bytes[2]);

/**
 * @brief Đọc mã định danh JEDEC ID (lệnh 0x9F) từ chip nhớ Flash W25Q64FV trên bus SPI2.
 * @param[out] id_out Mảng 3 byte nhận kết quả (kỳ vọng trên BKIT ARM4: 0xEF 0x40 0x17).
 * @return 0 nếu thành công, -ENODEV nếu bus SPI2 hoặc chân CS chưa sẵn sàng.
 */
int bsp_spi2_probe_jedec(uint8_t id_out[3]);

#endif /* BSP_SPI_HMI_H_ */
