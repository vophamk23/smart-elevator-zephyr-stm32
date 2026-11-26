/**
 * @file bsp_adc_sensors.h
 * @brief Trình điều khiển ADC1 12-bit đọc đa kênh (Nhiệt độ, Tải trọng, Ánh sáng & Nguồn).
 *
 * Thứ tự kênh khai báo trong app.overlay (tránh xung đột PA6 của ngắt X0 EXTI):
 *  - Ch12 (PC2): Cảm biến nhiệt độ LM35 (10 mV/độ C -> 1 mV = 0.1 độ C)
 *  - Ch11 (PC1): Biến trở giả lập tải trọng cabin (quy đổi 0..1000 kg)
 *  - Ch10 (PC0): Cảm biến quang trở ánh sáng (mV)
 *  - Ch8  (PB0): Giám sát điện áp nguồn cấp (mV)
 *  - Ch9  (PB1): Giám sát dòng tải động cơ (mV)
 */

#ifndef BSP_ADC_SENSORS_H_
#define BSP_ADC_SENSORS_H_

#include "elevator_types.h"

/**
 * @brief Khởi tạo và hiệu chuẩn toàn bộ các kênh ADC1 khai báo qua Devicetree adc_dt.
 * @return 0 nếu có ít nhất 1 kênh sẵn sàng, mã lỗi âm errno nếu thất bại.
 */
int bsp_adc_init(void);

/**
 * @brief Lấy mẫu tuần tự cả 5 kênh ADC1 và quy đổi sang đơn vị kỹ thuật (0.1 độ C, kg, mV).
 * @param[out] out Con trỏ struct nhận ảnh chụp giá trị đo được từ các cảm biến.
 * @return 0 nếu đọc thành công, -EINVAL nếu con trỏ NULL, -EIO nếu toàn bộ kênh lỗi.
 */
int bsp_adc_read_all(bkit_adc_snapshot_t *out);

/**
 * @brief Ghi đè tải trọng cabin giả lập từ PC Digital Twin / Shell HIL (`< 0` để trả về đọc biến trở VR1 thật).
 *        Nếu người dùng xoay núm biến trở VR1 thật lệch >= 40 kg so với lúc bắt đầu ghi đè, hệ thống tự động trả quyền về VR1 thật.
 */
void bsp_adc_set_hil_load_override(int32_t load_kg);

/**
 * @brief Ghi đè nhiệt độ LM35 giả lập từ PC Digital Twin / Shell HIL (đơn vị 0.1 độ C; `< 0` để trả về cảm biến thật).
 */
void bsp_adc_set_hil_temp_override(int32_t temp_deci_c);

/**
 * @brief Xóa toàn bộ giá trị ghi đè cảm biến HIL, khôi phục đọc 100% từ ADC1 phần cứng.
 */
void bsp_adc_clear_hil_overrides(void);

#endif /* BSP_ADC_SENSORS_H_ */
