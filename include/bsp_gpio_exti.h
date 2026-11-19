/**
 * @file bsp_gpio_exti.h
 * @brief Trình điều khiển GPIO cách ly quang, Khóa liên động Động cơ & Ngắt ngoài EXTI.
 *
 * Ánh xạ chân theo tài liệu BKIT ARM4 Quick Start Guide:
 *  - PE4 (LED3): LED Debug / nhịp sống Heartbeat của RTOS (Mức cao sáng)
 *  - PE5 (Y0)  : Ngõ ra cách ly quang điều khiển Motor CHẠY LÊN (Active High)
 *  - PE6 (Y1)  : Ngõ ra cách ly quang điều khiển Motor CHẠY XUỐNG (Active High)
 *  - PA6 (X0)  : Cảm biến kẹt cửa dùng ngắt EXTI (Active Low, kéo lên nội)
 *  - PA7 (X1)  : Nút Dừng khẩn cấp E-Stop dùng ngắt EXTI (Active Low, kéo lên nội)
 *  - PC4 (X2)  : Ngõ vào opto cảm biến tầng / bảo trì (Active Low)
 *  - PC5 (X3)  : Ngõ vào opto cảm biến quá tải cơ khí (Active Low)
 */

#ifndef BSP_GPIO_EXTI_H_
#define BSP_GPIO_EXTI_H_

#include <stdbool.h>
#include <stdint.h>
#include "elevator_types.h"

/**
 * @brief Kiểu hàm callback được gọi ngay trong ngữ cảnh ISR khi có sườn ngắt EXTI (PA6 / PA7).
 * @param ev         Loại sự kiện an toàn tương ứng (kẹt cửa hoặc E-Stop).
 * @param isr_cycles Giá trị bộ đếm chu kỳ CPU DWT chụp ngay đầu ngắt để đo độ trễ phản hồi.
 */
typedef void (*bsp_exti_cb_t)(elevator_ev_type_t ev, uint32_t isr_cycles);

/**
 * @brief Khởi tạo các chân xuất Y0/Y1/PE4, các ngõ vào X0..X3 và đăng ký ngắt ngoài EXTI.
 * @param exti_cb Hàm callback báo sự kiện về tầng RTOS (có thể truyền NULL nếu chỉ đọc polling).
 * @return 0 nếu thành công, mã lỗi âm errno nếu thiết bị GPIO chưa sẵn sàng.
 */
int bsp_gpio_exti_init(bsp_exti_cb_t exti_cb);

/**
 * @brief Bật hoặc tắt LED Debug Heartbeat gắn tại chân PE4.
 * @param on true để bật LED, false để tắt LED.
 */
void bsp_gpio_set_led_debug(bool on);

/**
 * @brief Điều khiển hai ngõ ra Motor LÊN (PE5) và Motor XUỐNG (PE6) kèm khóa liên động cứng.
 *
 * Đảm bảo bất biến an toàn ở tầng driver: nếu vì bất kỳ lỗi nào mà cả @p up_en và @p down_en
 * cùng bằng true, hàm lập tức cắt cả hai ngõ ra về 0 để chống ngắn mạch contactor/cầu H.
 *
 * @param up_en   Cho phép xuất tín hiệu chạy Lên trên chân Y0 (PE5).
 * @param down_en Cho phép xuất tín hiệu chạy Xuống trên chân Y1 (PE6).
 */
void bsp_gpio_set_motor_outputs(bool up_en, bool down_en);

/**
 * @brief Đọc trạng thái mức logic (đã kích hoạt = true) của 4 ngõ vào cách ly quang X0..X3.
 * @param[out] x0 Trạng thái cảm biến kẹt cửa (PA6), có thể NULL nếu bỏ qua.
 * @param[out] x1 Trạng thái nút E-Stop (PA7), có thể NULL.
 * @param[out] x2 Trạng thái cảm biến tầng / bảo trì (PC4), có thể NULL.
 * @param[out] x3 Trạng thái cảm biến quá tải cơ khí (PC5), có thể NULL.
 */
void bsp_gpio_read_inputs(bool *x0, bool *x1, bool *x2, bool *x3);

#endif /* BSP_GPIO_EXTI_H_ */
