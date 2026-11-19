/**
 * @file elevator_scheduler.c
 * @brief Bộ lập lịch gọi tầng 7 tầng theo thuật toán SCAN/LOOK hai chiều độc lập phần cứng.
 *
 * Chức năng tổng quát của file:
 *  - Quản lý 3 hàng đợi yêu cầu gọi tầng dưới dạng mặt nạ bit 7-bit (`cabin_mask`, `hall_up_mask`, `hall_down_mask`):
 *    bit 0 = Tầng 1, bit 1 = Tầng 2, ..., bit 6 = Tầng 7.
 *  - Quyết định chiều di chuyển và tầng đích tiếp theo theo nguyên lý gom khách cùng chiều (SCAN/LOOK):
 *    ưu tiên phục vụ hết các tầng có yêu cầu theo chiều đang đi trước khi đảo chiều.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `elev_sched_reset()`           : Xóa toàn bộ các mặt nạ gọi tầng về 0.
 *  - `elev_sched_add_request()`     : Đăng ký một yêu cầu gọi tầng mới (trong Cabin, ngoài sảnh Lên hoặc Xuống).
 *  - `elev_sched_clear_floor()`     : Xóa cờ yêu cầu tại một tầng sau khi cabin đã dừng và mở cửa đón khách.
 *  - `elev_sched_should_stop()`     : Kiểm tra xem cabin đang đi qua tầng `floor` có cần dừng lại đón/trả khách không.
 *  - `elev_sched_pick_next_target()`: Thuật toán LOOK tìm tầng đích tiếp theo và chiều đi tối ưu (`ELEV_DIR_UP`/`DOWN`/`IDLE`).
 */

#include "elevator_scheduler.h"

#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(elev_sched, LOG_LEVEL_INF);

/** @brief Quy đổi số tầng (1..7) sang mặt nạ bit tương ứng (bit 0..6); trả 0 nếu ngoài dải. */
static uint8_t floor_bit(uint8_t floor)
{
	if (floor < ELEV_MIN_FLOOR || floor > ELEV_MAX_FLOOR) {
		return 0U;
	}
	return (uint8_t)BIT(floor - 1U);
}

/** @brief Gộp cả 3 mặt nạ cuộc gọi (trong cabin + hành lang LÊN + hành lang XUỐNG). */
static uint8_t merged_requests(const elev_sched_queues_t *q)
{
	return (uint8_t)(q->cabin_mask | q->hall_up_mask | q->hall_down_mask);
}

/** @brief Kiểm tra xem còn bất kỳ yêu cầu nào ở các tầng nằm phía trên @p floor hay không. */
static bool has_request_above(const elev_sched_queues_t *q, uint8_t floor)
{
	uint8_t all = merged_requests(q);

	for (uint8_t f = (uint8_t)(floor + 1U); f <= ELEV_MAX_FLOOR; f++) {
		if ((all & floor_bit(f)) != 0U) {
			return true;
		}
	}
	return false;
}

/** @brief Kiểm tra xem còn bất kỳ yêu cầu nào ở các tầng nằm phía dưới @p floor hay không. */
static bool has_request_below(const elev_sched_queues_t *q, uint8_t floor)
{
	uint8_t all = merged_requests(q);

	for (uint8_t f = ELEV_MIN_FLOOR; f < floor; f++) {
		if ((all & floor_bit(f)) != 0U) {
			return true;
		}
	}
	return false;
}

/**
 * @brief Quét hướng LÊN: ưu tiên điểm dừng cabin/gọi Lên gần nhất; nếu hết mới chọn tầng gọi Xuống cao nhất để đảo chiều (chính sách LOOK).
 */
static uint8_t pick_upward_target(const elev_sched_queues_t *q, uint8_t cur_floor)
{
	for (uint8_t f = (uint8_t)(cur_floor + 1U); f <= ELEV_MAX_FLOOR; f++) {
		uint8_t bit = floor_bit(f);

		if (((q->cabin_mask | q->hall_up_mask) & bit) != 0U) {
			return f;
		}
	}

	/* Lên tới tầng có khách gọi Xuống cao nhất rồi mới quay đầu đi Xuống (LOOK) */
	for (uint8_t f = ELEV_MAX_FLOOR; f > cur_floor; f--) {
		if ((q->hall_down_mask & floor_bit(f)) != 0U) {
			return f;
		}
	}
	return 0U;
}

/**
 * @brief Quét hướng XUỐNG: ưu tiên điểm dừng cabin/gọi Xuống gần nhất; nếu hết mới chọn tầng gọi Lên thấp nhất để đảo chiều.
 */
static uint8_t pick_downward_target(const elev_sched_queues_t *q, uint8_t cur_floor)
{
	for (uint8_t f = (uint8_t)(cur_floor - 1U); f >= ELEV_MIN_FLOOR; f--) {
		uint8_t bit = floor_bit(f);

		if (((q->cabin_mask | q->hall_down_mask) & bit) != 0U) {
			return f;
		}
	}

	for (uint8_t f = ELEV_MIN_FLOOR; f < cur_floor; f++) {
		if ((q->hall_up_mask & floor_bit(f)) != 0U) {
			return f;
		}
	}
	return 0U;
}

/** @brief Xóa trắng cả 3 mặt nạ hàng đợi gọi tầng. */
void elev_sched_reset(elev_sched_queues_t *q)
{
	if (q != NULL) {
		memset(q, 0, sizeof(*q));
	}
}

/** @brief Ghi nhận yêu cầu gọi tầng (cabin, hành lang Lên hoặc hành lang Xuống) vào bitmask tương ứng. */
bool elev_sched_add_request(elev_sched_queues_t *q, elevator_ev_type_t type, uint8_t floor)
{
	if (q == NULL || floor_bit(floor) == 0U) {
		return false;
	}

	uint8_t bit = floor_bit(floor);

	switch (type) {
	case EV_CALL_CABIN:
		q->cabin_mask |= bit;
		return true;
	/* Không nhận gọi Lên tại tầng đỉnh (7) và không nhận gọi Xuống tại tầng đáy (1) */
	case EV_CALL_HALL_UP:
		if (floor < ELEV_MAX_FLOOR) {
			q->hall_up_mask |= bit;
			return true;
		}
		return false;
	case EV_CALL_HALL_DOWN:
		if (floor > ELEV_MIN_FLOOR) {
			q->hall_down_mask |= bit;
			return true;
		}
		return false;
	default:
		return false;
	}
}

/** @brief Xóa cờ yêu cầu tại tầng vừa đến cho cả nút cabin lẫn nút hành lang. */
void elev_sched_clear_floor(elev_sched_queues_t *q, uint8_t floor)
{
	if (q == NULL || floor_bit(floor) == 0U) {
		return;
	}

	uint8_t clear_mask = (uint8_t)~floor_bit(floor);

	q->cabin_mask     &= clear_mask;
	q->hall_up_mask   &= clear_mask;
	q->hall_down_mask &= clear_mask;
}

/**
 * @brief Thuật toán chính chọn tầng đích kế tiếp và hướng chạy tiếp theo theo chiến lược LOOK.
 *
 * Nguyên tắc của SCAN/LOOK (giống thang máy thực tế, khác FCFS ở chỗ gom cuốc theo chiều):
 *  1. Nếu ngay tại tầng hiện tại có người vừa bấm gọi -> mở cửa luôn.
 *  2. Nếu đang đi LÊN (hoặc đứng yên) mà phía trên vẫn còn người gọi -> tiếp tục quét lên.
 *  3. Khi đã hết yêu cầu phía trên -> quay đầu quét xuống nếu phía dưới có yêu cầu.
 *  4. Chỉ khi phía dưới cũng trống mới xét ngược lên -> tránh hiện tượng "chạy qua lại
 *     liên tục" hoặc bỏ đói yêu cầu ở tầng đầu/cuối (tầng 1 và tầng 7).
 */
uint8_t elev_sched_pick_next_target(const elev_sched_queues_t *q, uint8_t cur_floor,
				    elevator_dir_t cur_dir, elevator_dir_t *out_dir)
{
	if (q == NULL || out_dir == NULL || merged_requests(q) == 0U) {
		if (out_dir != NULL) {
			*out_dir = ELEV_DIR_IDLE;
		}
		return 0U;
	}

	/* Phục vụ ngay nếu có yêu cầu mở cửa ngay tại tầng cabin đang đỗ */
	if ((merged_requests(q) & floor_bit(cur_floor)) != 0U) {
		*out_dir = cur_dir;
		return cur_floor;
	}

	/* Giữ nguyên chiều quét hiện tại (ưu tiên chiều Lên nếu đang đứng yên hoặc vừa đi Lên) */
	if (cur_dir >= 0 && has_request_above(q, cur_floor)) {
		uint8_t tgt = pick_upward_target(q, cur_floor);
		if (tgt != 0U) {
			*out_dir = ELEV_DIR_UP;
			return tgt;
		}
	}

	if (has_request_below(q, cur_floor)) {
		uint8_t tgt = pick_downward_target(q, cur_floor);
		if (tgt != 0U) {
			*out_dir = ELEV_DIR_DOWN;
			return tgt;
		}
	}

	if (has_request_above(q, cur_floor)) {
		uint8_t tgt = pick_upward_target(q, cur_floor);
		if (tgt != 0U) {
			*out_dir = ELEV_DIR_UP;
			return tgt;
		}
	}

	*out_dir = ELEV_DIR_IDLE;
	return 0U;
}

/** @brief Xác định điều kiện dừng đón/trả khách khi cabin đi ngang qua @p floor theo chiều @p travel_dir. */
bool elev_sched_should_stop(const elev_sched_queues_t *q, uint8_t floor,
			    elevator_dir_t travel_dir)
{
	if (q == NULL || floor_bit(floor) == 0U) {
		return false;
	}

	uint8_t bit = floor_bit(floor);

	/* Luôn dừng nếu hành khách bên trong cabin đã bấm chọn tầng này */
	if ((q->cabin_mask & bit) != 0U) {
		return true;
	}

	/* Dừng nếu có cuộc gọi hành lang cùng chiều, hoặc cuộc gọi ngược chiều tại biên hành trình */
	if (travel_dir == ELEV_DIR_UP) {
		if ((q->hall_up_mask & bit) != 0U) {
			return true;
		}
		return !has_request_above(q, floor) && ((q->hall_down_mask & bit) != 0U);
	}

	if (travel_dir == ELEV_DIR_DOWN) {
		if ((q->hall_down_mask & bit) != 0U) {
			return true;
		}
		return !has_request_below(q, floor) && ((q->hall_up_mask & bit) != 0U);
	}

	return ((q->hall_up_mask | q->hall_down_mask) & bit) != 0U;
}
