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

#endif /* BSP_ADC_SENSORS_H_ */
