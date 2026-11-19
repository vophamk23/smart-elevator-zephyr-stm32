/**
 * @file elevator_hsm.h
 * @brief Máy trạng thái phân cấp an toàn trọng yếu (HSM) & Bộ theo dõi độ phủ chuyển trạng thái.
 *
 * Đảm bảo cứng các bất biến an toàn động cơ (cửa PHẢI đóng kín, cấm LÊN/XUỐNG cùng lúc,
 * E-Stop và Báo cháy luôn cưỡng chế ưu tiên), đồng thời ghi lại ma trận chuyển trạng thái
 * phục vụ kiểm thử độ phủ theo quy trình V-Model.
 */

#ifndef ELEVATOR_HSM_H_
#define ELEVATOR_HSM_H_

#include <stdbool.h>
#include <stdint.h>
#include "elevator_types.h"

/**
 * @brief Khởi tạo mutex, xóa sạch hàng đợi sự kiện, đưa cabin về IDLE_CLOSED và bật k_timer 100 ms.
 * @return 0 nếu khởi tạo thành công.
 */
int elevator_hsm_init(void);

/**
 * @brief Đẩy bất đồng bộ một sự kiện điều khiển, cảm biến, CAN hoặc EXTI vào @c k_msgq (không chặn).
 * @param ev Con trỏ tới gói sự kiện cần sao chép vào hàng đợi.
 * @return 0 nếu thành công, -ENOMSG/-EAGAIN nếu hàng đợi đầy.
 */
int elevator_hsm_post_event(const elevator_event_t *ev);

/**
 * @brief Chờ sự kiện (`K_FOREVER`) từ timer tick 100 ms, EXTI, CLI hoặc CAN trên @c k_msgq,
 *        cập nhật chuyển trạng thái HSM và xuất tín hiệu khóa liên động ra chân PE5/PE6.
 */
void elevator_hsm_step(void);

/**
 * @brief Đọc bộ đếm nguyên tử (`atomic_t`) thống kê số sự kiện đã gửi, đã nhận, bị rớt và đang chờ.
 */
void elevator_hsm_get_queue_stats(uint32_t *sent, uint32_t *recv,
				  uint32_t *dropped, uint32_t *in_q);

/** @brief Chụp nhanh an toàn đa luồng toàn bộ trạng thái HSM, hàng đợi, cờ lỗi và độ trễ EXTI. */
void elevator_hsm_get_telemetry(elevator_telemetry_t *out);

/** @brief Trả về mặt nạ bit các trạng thái @ref elevator_state_t đã từng đi qua trong phiên chạy. */
uint32_t elevator_hsm_get_visited_state_mask(void);

/**
 * @brief Sao chép ma trận đếm số lần chuyển giữa mọi cặp trạng thái phục vụ báo cáo độ phủ V-Model.
 * @param[out] out_matrix Mảng [HSM_STATE_COUNT][HSM_STATE_COUNT] nhận bảng đếm chuyển trạng thái.
 */
void elevator_hsm_get_transition_counts(uint16_t out_matrix[HSM_STATE_COUNT][HSM_STATE_COUNT]);

/** @brief Chuyển giá trị enum @ref elevator_state_t sang chuỗi ASCII ngắn gọn dễ đọc. */
const char *elevator_state_str(elevator_state_t st);

/** @brief Chuyển giá trị enum @ref blackbox_fault_code_t sang chuỗi mô tả sự cố để in nhật ký. */
const char *elevator_fault_str(blackbox_fault_code_t code);

#endif /* ELEVATOR_HSM_H_ */
