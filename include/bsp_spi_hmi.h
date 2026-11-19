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
 * @brief Đẩy dữ liệu mã 7 đoạn thô lên thanh ghi dịch SPI1 để quét ghép thời gian 4 LED 7 đoạn.
 * @param digits Mảng 4 byte mã segment chung cực dương cho các chữ số 0..3.
 * @return 0 nếu truyền thành công, mã lỗi âm errno nếu lỗi bus SPI.
 */
int bsp_spi_7seg_write_raw(const uint8_t digits[4]);

/**
 * @brief Hiển thị tầng hiện tại, hướng di chuyển và mã cảnh báo ('FE' / 'Err') lên LED 7 đoạn.
 * @param floor Tầng hiện tại của thang máy (1..7).
 * @param dir   Hướng chạy (LÊN / XUỐNG / DỪNG).
 * @param fire  true nếu đang kích hoạt sơ tán hỏa hoạn ('FE').
 * @param fault true nếu đang dừng khẩn cấp E-Stop hoặc lỗi mất CAN ('Err').
 * @return 0 nếu hiển thị thành công.
 */
int bsp_spi_7seg_render_status(uint8_t floor, elevator_dir_t dir, bool fire, bool fault);

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
