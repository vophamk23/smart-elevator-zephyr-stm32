/**
 * @file elevator_scheduler.h
 * @brief Bộ lập lịch gọi tầng 7 tầng theo thuật toán SCAN/LOOK độc lập phần cứng.
 *
 * Tách biệt hoàn toàn phần quản lý bitmask hàng đợi và chọn tầng kế tiếp khỏi logic HSM
 * hay trình điều khiển STM32 để dễ kiểm thử đơn vị và thẩm định thuật toán.
 */

#ifndef ELEVATOR_SCHEDULER_H_
#define ELEVATOR_SCHEDULER_H_

#include <stdbool.h>
#include <stdint.h>
#include "elevator_types.h"

/** @brief Cấu trúc lưu bitmask các cuộc gọi từ tầng 1..7 (bit N-1 ứng với tầng N). */
typedef struct {
	uint8_t cabin_mask;     /**< Nút chọn tầng bên trong cabin (Tầng 1..7). */
	uint8_t hall_up_mask;   /**< Nút gọi chiều LÊN ở hành lang (Tầng 1..6). */
	uint8_t hall_down_mask; /**< Nút gọi chiều XUỐNG ở hành lang (Tầng 2..7). */
} elev_sched_queues_t;

/** @brief Xóa toàn bộ hàng đợi gọi tầng (cả trong cabin và ngoài hành lang) về rỗng. */
void elev_sched_reset(elev_sched_queues_t *q);

/**
 * @brief Thêm một yêu cầu gọi tầng vào hàng đợi nếu tầng và loại cuộc gọi nằm trong giới hạn hợp lệ.
 * @return true nếu yêu cầu được chấp nhận, false nếu sai tầng/loại cuộc gọi.
 */
bool elev_sched_add_request(elev_sched_queues_t *q, elevator_ev_type_t type, uint8_t floor);

/** @brief Xóa mọi yêu cầu đang chờ tại tầng @p floor khi cabin đã tới nơi và mở cửa đón khách. */
void elev_sched_clear_floor(elev_sched_queues_t *q, uint8_t floor);

/**
 * @brief Chọn tầng đích kế tiếp và hướng di chuyển dựa trên thuật toán quét hai chiều LOOK/SCAN.
 * @param      q         Bitmask các yêu cầu đang chờ.
 * @param      cur_floor Tầng hiện tại của cabin (1..7).
 * @param      cur_dir   Hướng quét trước đó hoặc hiện tại của thang máy.
 * @param[out] out_dir   Hướng di chuyển mới được chọn (ELEV_DIR_IDLE nếu hết yêu cầu).
 * @return Số hiệu tầng đích kế tiếp (1..7), hoặc 0 nếu không còn yêu cầu nào.
 */
uint8_t elev_sched_pick_next_target(const elev_sched_queues_t *q, uint8_t cur_floor,
				    elevator_dir_t cur_dir, elevator_dir_t *out_dir);

/**
 * @brief Kiểm tra cabin có cần giảm tốc dừng lại ở tầng @p floor khi đang chạy qua hay không.
 * @return true nếu có khách trong cabin muốn xuống tầng này hoặc có cuộc gọi hành lang cùng chiều.
 */
bool elev_sched_should_stop(const elev_sched_queues_t *q, uint8_t floor,
			    elevator_dir_t travel_dir);

#endif /* ELEVATOR_SCHEDULER_H_ */
