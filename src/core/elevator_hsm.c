/**
 * @file elevator_hsm.c
 * @brief Máy trạng thái phân cấp an toàn trọng yếu 9 trạng thái (HSM) & Bộ đo độ phủ V-Model.
 *
 * Chức năng tổng quát của file:
 *  - Điều khiển toàn bộ vòng đời vận hành của thang máy 7 tầng qua 9 trạng thái:
 *    `INIT`, `IDLE_CLOSED`, `DOOR_OPENING`, `DOOR_OPEN`, `DOOR_CLOSING`, `MOVING_UP`,
 *    `MOVING_DOWN`, `FIRE_EVAC` (Sơ tán cháy về Tầng 1), và `FAIL_SAFE` (Khóa an toàn khi E-Stop/đứt CAN).
 *  - Duy trì hướng quét `scan_dir` xuyên qua các chu kỳ đóng/mở cửa trung gian để thuật toán LOOK
 *    gom khách chuẩn xác theo cả hai chiều Lên và Xuống.
 *  - Khóa cứng động cơ và tự mở cửa phát còi ngay khi phát hiện Quá tải (`>= 800 kg`), ghi Hộp đen
 *    ngoài vùng khóa Mutex để không làm nghẽn các luồng thời gian thực.
 */

#include "elevator_hsm.h"
#include "bsp_gpio_exti.h"
#include "bsp_i2c_storage.h"
#include "bsp_pwm_buzzer.h"
#include "bsp_watchdog.h"
#include "elevator_scheduler.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(elev_hsm, LOG_LEVEL_INF);

K_MSGQ_DEFINE(g_hsm_event_msgq, sizeof(elevator_event_t), 32, 4);
K_MUTEX_DEFINE(g_hsm_state_mutex);

static atomic_t g_isr_ev_sent = ATOMIC_INIT(0);
static atomic_t g_isr_ev_recv = ATOMIC_INIT(0);
static atomic_t g_isr_ev_drop = ATOMIC_INIT(0);

static void hsm_tick_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	elevator_event_t ev = {
		.type = EV_TICK_100MS,
		.isr_cycles = k_cycle_get_32(),
	};

	if (k_msgq_put(&g_hsm_event_msgq, &ev, K_NO_WAIT) == 0) {
		atomic_inc(&g_isr_ev_sent);
	} else {
		atomic_inc(&g_isr_ev_drop);
	}
}

K_TIMER_DEFINE(g_hsm_tick_timer, hsm_tick_timer_isr, NULL);

typedef struct {
	elevator_state_t state;
	uint8_t current_floor;
	uint8_t target_floor;
	elevator_dir_t direction;
	elevator_dir_t scan_dir; /* Nhớ hướng quét LOOK xuyên qua nhịp mở/đóng cửa tầng trung gian */
	door_pos_t door_pos;
	uint8_t door_pct;
	uint32_t phase_elapsed_ms;
	elev_sched_queues_t queues;
	bool fire_active;
	bool estop_active;
	bool overload_active;
	bool can_fault_active;
	uint32_t last_obstruct_ms;
	uint32_t trip_count;
	uint32_t transition_count;
	uint32_t safety_abort_count;
	uint32_t last_exti_latency_us;
	uint32_t max_exti_latency_us;
	uint32_t visited_states_mask;
	uint16_t transition_matrix[HSM_STATE_COUNT][HSM_STATE_COUNT];
} hsm_ctx_t;

static hsm_ctx_t g_hsm;

/**
 * @brief Kiểm tra khóa liên động an toàn tầng Core trước khi xuất tín hiệu điều khiển ra chân PE5/PE6.
 */
static void drive_interlocked_outputs(const hsm_ctx_t *ctx)
{
	bool move_up   = (ctx->state == HSM_STATE_MOVING_UP);
	bool move_down = (ctx->state == HSM_STATE_MOVING_DOWN) ||
			 (ctx->state == HSM_STATE_FIRE_EVAC &&
			  ctx->current_floor > ELEV_MIN_FLOOR &&
			  ctx->door_pos == DOOR_POS_CLOSED);

	/* Bất biến an toàn: cấm tuyệt đối đóng điện động cơ nếu cửa chưa đóng kín, đang E-Stop hoặc đang quá tải ở chế độ thường */
	if (ctx->door_pos != DOOR_POS_CLOSED || ctx->estop_active ||
	    (ctx->overload_active && ctx->state != HSM_STATE_FIRE_EVAC)) {
		move_up   = false;
		move_down = false;
	}

	bsp_gpio_set_motor_outputs(move_up, move_down);
}

/**
 * @brief Thực hiện chuyển trạng thái HSM, chạy hành động vào (entry action) và cập nhật ma trận V-Model.
 */
static void hsm_transition(hsm_ctx_t *ctx, elevator_state_t next)
{
	elevator_state_t prev = ctx->state;

	if (prev == next || next >= HSM_STATE_COUNT) {
		return;
	}

	LOG_INF("HSM: %s -> %s (Floor=%u)", elevator_state_str(prev), elevator_state_str(next), ctx->current_floor);

	ctx->state            = next;
	ctx->phase_elapsed_ms = 0U;
	ctx->transition_count++;
	ctx->visited_states_mask |= BIT((uint32_t)next);
	if (prev < HSM_STATE_COUNT) {
		ctx->transition_matrix[prev][next]++;
	}

	switch (next) {
	case HSM_STATE_IDLE_CLOSED:
		ctx->door_pos  = DOOR_POS_CLOSED;
		ctx->door_pct  = 0U;
		ctx->direction = ELEV_DIR_IDLE;
		break;
	case HSM_STATE_DOOR_OPENING:
		ctx->door_pos = DOOR_POS_OPENING;
		bsp_pwm_buzzer_play_tone(BUZZER_TONE_FLOOR_ARRIVE);
		break;
	case HSM_STATE_DOOR_OPEN:
		ctx->door_pos = DOOR_POS_OPEN;
		ctx->door_pct = 100U;
		elev_sched_clear_floor(&ctx->queues, ctx->current_floor, ctx->scan_dir);
		break;
	case HSM_STATE_DOOR_CLOSING:
		ctx->door_pos = DOOR_POS_CLOSING;
		break;
	case HSM_STATE_MOVING_UP:
		ctx->direction = ELEV_DIR_UP;
		ctx->scan_dir  = ELEV_DIR_UP;
		ctx->door_pos  = DOOR_POS_CLOSED;
		ctx->door_pct  = 0U;
		break;
	case HSM_STATE_MOVING_DOWN:
		ctx->direction = ELEV_DIR_DOWN;
		ctx->scan_dir  = ELEV_DIR_DOWN;
		ctx->door_pos  = DOOR_POS_CLOSED;
		ctx->door_pct  = 0U;
		break;
	case HSM_STATE_FIRE_EVAC:
		elev_sched_reset(&ctx->queues);
		ctx->target_floor = ELEV_MIN_FLOOR;
		ctx->direction    = ELEV_DIR_IDLE;
		ctx->scan_dir     = ELEV_DIR_DOWN;
		/* Nếu cửa đang mở dở trên tầng cao -> nội suy đóng tiếp từ đúng % hiện tại thay vì nhảy về 100% */
		if (ctx->current_floor > ELEV_MIN_FLOOR && ctx->door_pos != DOOR_POS_CLOSED) {
			ctx->phase_elapsed_ms = ((uint32_t)(100U - ctx->door_pct) * ELEV_DOOR_MOVE_MS) / 100U;
		}
		bsp_pwm_buzzer_play_tone(BUZZER_TONE_ALARM_SIREN);
		break;
	case HSM_STATE_FAIL_SAFE:
		ctx->direction = ELEV_DIR_IDLE;
		ctx->scan_dir  = ELEV_DIR_IDLE;
		bsp_pwm_buzzer_beep(1200U, 300U);
		break;
	default:
		break;
	}

	drive_interlocked_outputs(ctx);
}

/**
 * @brief Cập nhật lại tầng đích hiển thị khi có thêm cuộc gọi mới lúc cabin đang di chuyển.
 */
static void refresh_moving_target(hsm_ctx_t *ctx)
{
	if (ctx->state != HSM_STATE_MOVING_UP && ctx->state != HSM_STATE_MOVING_DOWN) {
		return;
	}
	elevator_dir_t nxt_dir = ELEV_DIR_IDLE;
	uint8_t nxt = elev_sched_pick_next_target(&ctx->queues, ctx->current_floor,
						  ctx->direction, &nxt_dir);
	if (nxt != 0U && nxt_dir == ctx->direction) {
		ctx->target_floor = nxt;
	}
}

static void record_exti_latency(hsm_ctx_t *ctx, uint32_t isr_cycles)
{
	if (isr_cycles == 0U) {
		return;
	}

	uint32_t delta = k_cycle_get_32() - isr_cycles;
	uint32_t us    = k_cyc_to_us_ceil32(delta);

	ctx->last_exti_latency_us = us;
	if (us > ctx->max_exti_latency_us) {
		ctx->max_exti_latency_us = us;
	}
}

static void step_fire_evac_state(hsm_ctx_t *ctx, uint32_t dt_ms)
{
	/* Phát lặp còi báo cháy chu kỳ 1.2 giây cho đến khi người vận hành giải trừ lỗi */
	if ((ctx->phase_elapsed_ms % 1200U) < dt_ms) {
		bsp_pwm_buzzer_play_tone(BUZZER_TONE_ALARM_SIREN);
	}

	if (ctx->door_pos != DOOR_POS_CLOSED) {
		if (ctx->current_floor > ELEV_MIN_FLOOR) {
			uint32_t pct  = (ctx->phase_elapsed_ms * 100U) / ELEV_DOOR_MOVE_MS;
			ctx->door_pct = (pct >= 100U) ? 0U : (uint8_t)(100U - pct);
			if (ctx->phase_elapsed_ms >= ELEV_DOOR_MOVE_MS) {
				ctx->door_pos         = DOOR_POS_CLOSED;
				ctx->door_pct         = 0U;
				ctx->phase_elapsed_ms = 0U;
				drive_interlocked_outputs(ctx);
			}
		} else {
			ctx->door_pos = DOOR_POS_OPEN;
			ctx->door_pct = 100U;
		}
		return;
	}

	if (ctx->current_floor > ELEV_MIN_FLOOR) {
		ctx->direction = ELEV_DIR_DOWN;
		drive_interlocked_outputs(ctx);
		if (ctx->phase_elapsed_ms >= ELEV_TRAVEL_FLOOR_MS) {
			ctx->phase_elapsed_ms = 0U;
			ctx->current_floor--;
			ctx->trip_count++;
		}
	} else {
		ctx->direction = ELEV_DIR_IDLE;
		ctx->door_pos  = DOOR_POS_OPEN;
		ctx->door_pct  = 100U;
		drive_interlocked_outputs(ctx);
	}
}

static void handle_periodic_tick(hsm_ctx_t *ctx, uint32_t dt_ms)
{
	ctx->phase_elapsed_ms += dt_ms;

	switch (ctx->state) {
	case HSM_STATE_IDLE_CLOSED: {
		/* Nếu đang quá tải cabin (>= 800 kg): khóa cứng mô-tơ và tự mở cửa ra báo quá tải */
		if (ctx->overload_active) {
			hsm_transition(ctx, HSM_STATE_DOOR_OPENING);
			bsp_pwm_buzzer_play_tone(BUZZER_TONE_OVERLOAD);
			break;
		}

		elevator_dir_t next_dir = ELEV_DIR_IDLE;
		uint8_t next_floor = elev_sched_pick_next_target(&ctx->queues, ctx->current_floor,
								 ctx->scan_dir, &next_dir);
		if (next_floor == 0U) {
			ctx->scan_dir     = ELEV_DIR_IDLE;
			ctx->target_floor = 0U;
		} else if (next_floor == ctx->current_floor) {
			ctx->target_floor = next_floor;
			hsm_transition(ctx, HSM_STATE_DOOR_OPENING);
		} else {
			ctx->scan_dir     = next_dir;
			ctx->target_floor = next_floor;
			hsm_transition(ctx, (next_dir == ELEV_DIR_UP) ? HSM_STATE_MOVING_UP
								      : HSM_STATE_MOVING_DOWN);
		}
		break;
	}

	case HSM_STATE_DOOR_OPENING:
		ctx->door_pct = (uint8_t)MIN(100U, (ctx->phase_elapsed_ms * 100U) / ELEV_DOOR_MOVE_MS);
		if (ctx->phase_elapsed_ms >= ELEV_DOOR_MOVE_MS) {
			hsm_transition(ctx, HSM_STATE_DOOR_OPEN);
		}
		break;

	case HSM_STATE_DOOR_OPEN:
		if (ctx->fire_active && ctx->current_floor == ELEV_MIN_FLOOR) {
			break;
		}
		if (ctx->overload_active) {
			if ((ctx->phase_elapsed_ms % 900U) < dt_ms) {
				bsp_pwm_buzzer_play_tone(BUZZER_TONE_OVERLOAD);
			}
			break;
		}
		if (ctx->phase_elapsed_ms >= ELEV_DOOR_HOLD_MS) {
			hsm_transition(ctx, HSM_STATE_DOOR_CLOSING);
		}
		break;

	case HSM_STATE_DOOR_CLOSING: {
		if (ctx->overload_active) {
			uint8_t resume_pct = ctx->door_pct;
			hsm_transition(ctx, HSM_STATE_DOOR_OPENING);
			ctx->phase_elapsed_ms = ((uint32_t)resume_pct * ELEV_DOOR_MOVE_MS) / 100U;
			ctx->door_pct         = resume_pct;
			bsp_pwm_buzzer_play_tone(BUZZER_TONE_OVERLOAD);
			break;
		}
		uint32_t pct  = (ctx->phase_elapsed_ms * 100U) / ELEV_DOOR_MOVE_MS;
		ctx->door_pct = (pct >= 100U) ? 0U : (uint8_t)(100U - pct);
		if (ctx->phase_elapsed_ms >= ELEV_DOOR_MOVE_MS) {
			if (ctx->fire_active) {
				hsm_transition(ctx, HSM_STATE_FIRE_EVAC);
			} else if (ctx->can_fault_active || ctx->estop_active) {
				hsm_transition(ctx, HSM_STATE_FAIL_SAFE);
			} else {
				hsm_transition(ctx, HSM_STATE_IDLE_CLOSED);
			}
		}
		break;
	}

	case HSM_STATE_MOVING_UP:
	case HSM_STATE_MOVING_DOWN:
		if (ctx->phase_elapsed_ms >= ELEV_TRAVEL_FLOOR_MS) {
			ctx->phase_elapsed_ms = 0U;
			if (ctx->state == HSM_STATE_MOVING_UP && ctx->current_floor < ELEV_MAX_FLOOR) {
				ctx->current_floor++;
			} else if (ctx->state == HSM_STATE_MOVING_DOWN && ctx->current_floor > ELEV_MIN_FLOOR) {
				ctx->current_floor--;
			}
			ctx->trip_count++;

			if (elev_sched_should_stop(&ctx->queues, ctx->current_floor, ctx->direction) ||
			    ctx->current_floor == ctx->target_floor) {
				hsm_transition(ctx, HSM_STATE_DOOR_OPENING);
			} else {
				refresh_moving_target(ctx);
			}
		}
		break;

	case HSM_STATE_FIRE_EVAC:
		step_fire_evac_state(ctx, dt_ms);
		break;

	case HSM_STATE_FAIL_SAFE:
		if (ctx->can_fault_active && !ctx->estop_active && ctx->door_pct < 100U) {
			ctx->door_pos = DOOR_POS_OPENING;
			ctx->door_pct = (uint8_t)MIN(100U, (ctx->phase_elapsed_ms * 100U) / ELEV_DOOR_MOVE_MS);
			if (ctx->door_pct >= 100U) {
				ctx->door_pos = DOOR_POS_OPEN;
			}
		}
		drive_interlocked_outputs(ctx);
		break;

	default:
		hsm_transition(ctx, HSM_STATE_IDLE_CLOSED);
		break;
	}
}

/**
 * @brief Đưa cabin thoát khỏi trạng thái lỗi một cách mượt mà (khép cửa từ % hiện tại nếu cửa đang mở).
 */
static void recover_from_fault(hsm_ctx_t *ctx)
{
	if (ctx->door_pct > 0U && ctx->door_pos != DOOR_POS_CLOSED) {
		uint8_t cur_pct = ctx->door_pct;
		hsm_transition(ctx, HSM_STATE_DOOR_CLOSING);
		ctx->phase_elapsed_ms = ((uint32_t)(100U - cur_pct) * ELEV_DOOR_MOVE_MS) / 100U;
		ctx->door_pct         = cur_pct;
	} else {
		hsm_transition(ctx, HSM_STATE_IDLE_CLOSED);
	}
}

int elevator_hsm_init(void)
{
	k_timer_stop(&g_hsm_tick_timer);
	k_msgq_purge(&g_hsm_event_msgq);
	atomic_set(&g_isr_ev_sent, 0);
	atomic_set(&g_isr_ev_recv, 0);
	atomic_set(&g_isr_ev_drop, 0);

	k_mutex_lock(&g_hsm_state_mutex, K_FOREVER);
	memset(&g_hsm, 0, sizeof(g_hsm));
	g_hsm.state               = HSM_STATE_INIT;
	g_hsm.current_floor       = ELEV_MIN_FLOOR;
	g_hsm.visited_states_mask = BIT((uint32_t)HSM_STATE_INIT);
	hsm_transition(&g_hsm, HSM_STATE_IDLE_CLOSED);
	k_mutex_unlock(&g_hsm_state_mutex);

	k_timer_start(&g_hsm_tick_timer, K_MSEC(100), K_MSEC(100));
	return 0;
}

int elevator_hsm_post_event(const elevator_event_t *ev)
{
	if (ev == NULL) {
		return -EINVAL;
	}

	int err = k_msgq_put(&g_hsm_event_msgq, ev, K_NO_WAIT);
	if (err == 0) {
		atomic_inc(&g_isr_ev_sent);
	} else {
		atomic_inc(&g_isr_ev_drop);
	}
	return err;
}

void elevator_hsm_step(void)
{
	elevator_event_t ev = { 0 };
	int err = k_msgq_get(&g_hsm_event_msgq, &ev, K_FOREVER);

	if (err != 0) {
		return;
	}
	atomic_inc(&g_isr_ev_recv);

	/* Biến tạm lưu bản ghi Hộp đen để ghi I2C EEPROM BÊN NGOÀI khóa Mutex */
	bool log_bb = false;
	blackbox_fault_code_t bb_code = BB_FAULT_BOOT_POR;
	elevator_state_t bb_state = HSM_STATE_IDLE_CLOSED;
	uint8_t bb_floor = 1U;
	int32_t bb_param = 0;

	k_mutex_lock(&g_hsm_state_mutex, K_FOREVER);

	switch (ev.type) {
	case EV_TICK_100MS:
		handle_periodic_tick(&g_hsm, 100U);
		break;

	case EV_CALL_CABIN:
		if (!g_hsm.fire_active && !g_hsm.estop_active) {
			if (elev_sched_add_request(&g_hsm.queues, ev.type, ev.floor)) {
				refresh_moving_target(&g_hsm);
			}
		}
		break;

	case EV_CALL_HALL_UP:
	case EV_CALL_HALL_DOWN:
		if (!g_hsm.fire_active && !g_hsm.estop_active && !g_hsm.can_fault_active) {
			if (elev_sched_add_request(&g_hsm.queues, ev.type, ev.floor)) {
				refresh_moving_target(&g_hsm);
			}
		}
		break;

	case EV_CMD_OPEN_DOOR:
		if (g_hsm.state == HSM_STATE_IDLE_CLOSED) {
			hsm_transition(&g_hsm, HSM_STATE_DOOR_OPENING);
		} else if (g_hsm.state == HSM_STATE_DOOR_CLOSING) {
			uint8_t resume_pct     = g_hsm.door_pct;
			hsm_transition(&g_hsm, HSM_STATE_DOOR_OPENING);
			g_hsm.phase_elapsed_ms = ((uint32_t)resume_pct * ELEV_DOOR_MOVE_MS) / 100U;
			g_hsm.door_pct         = resume_pct;
		} else if (g_hsm.state == HSM_STATE_DOOR_OPEN) {
			g_hsm.phase_elapsed_ms = 0U;
		}
		break;

	case EV_CMD_CLOSE_DOOR:
		if (g_hsm.state == HSM_STATE_DOOR_OPEN && !g_hsm.overload_active && !g_hsm.fire_active) {
			hsm_transition(&g_hsm, HSM_STATE_DOOR_CLOSING);
		}
		break;

	case EV_DOOR_OBSTRUCT_EXTI:
		record_exti_latency(&g_hsm, ev.isr_cycles);
		if (g_hsm.state == HSM_STATE_DOOR_CLOSING) {
			uint8_t resume_pct     = g_hsm.door_pct;
			g_hsm.last_obstruct_ms = k_uptime_get_32();
			g_hsm.safety_abort_count++;
			hsm_transition(&g_hsm, HSM_STATE_DOOR_OPENING);
			g_hsm.phase_elapsed_ms = ((uint32_t)resume_pct * ELEV_DOOR_MOVE_MS) / 100U;
			g_hsm.door_pct         = resume_pct;
			bsp_pwm_buzzer_play_tone(BUZZER_TONE_DOOR_OBSTRUCT);
			log_bb   = true;
			bb_code  = BB_FAULT_DOOR_OBSTRUCT;
			bb_state = g_hsm.state;
			bb_floor = g_hsm.current_floor;
		} else if (g_hsm.state == HSM_STATE_DOOR_OPEN || g_hsm.state == HSM_STATE_DOOR_OPENING) {
			g_hsm.last_obstruct_ms = k_uptime_get_32();
			g_hsm.safety_abort_count++;
			if (g_hsm.state == HSM_STATE_DOOR_OPEN) {
				g_hsm.phase_elapsed_ms = 0U;
			}
			bsp_pwm_buzzer_play_tone(BUZZER_TONE_DOOR_OBSTRUCT);
			log_bb   = true;
			bb_code  = BB_FAULT_DOOR_OBSTRUCT;
			bb_state = g_hsm.state;
			bb_floor = g_hsm.current_floor;
		}
		break;

	case EV_ESTOP_EXTI:
		record_exti_latency(&g_hsm, ev.isr_cycles);
		g_hsm.estop_active = true;
		g_hsm.safety_abort_count++;
		hsm_transition(&g_hsm, HSM_STATE_FAIL_SAFE);
		log_bb   = true;
		bb_code  = BB_FAULT_ESTOP;
		bb_state = g_hsm.state;
		bb_floor = g_hsm.current_floor;
		break;

	case EV_OVERHEAT_TEMP:
		if (!g_hsm.fire_active) {
			g_hsm.fire_active = true;
			g_hsm.safety_abort_count++;
			hsm_transition(&g_hsm, HSM_STATE_FIRE_EVAC);
			log_bb   = true;
			bb_code  = BB_FAULT_FIRE_OVERHEAT;
			bb_state = g_hsm.state;
			bb_floor = g_hsm.current_floor;
			bb_param = (int32_t)ev.param;
		}
		break;

	case EV_TEMP_NORMAL:
		if (g_hsm.fire_active) {
			g_hsm.fire_active = false;
			if (!g_hsm.estop_active && !g_hsm.can_fault_active) {
				recover_from_fault(&g_hsm);
			}
		}
		break;

	case EV_OVERLOAD_SET:
		if (!g_hsm.overload_active) {
			g_hsm.overload_active = true;
			bsp_pwm_buzzer_play_tone(BUZZER_TONE_OVERLOAD);
			if (g_hsm.state == HSM_STATE_IDLE_CLOSED) {
				hsm_transition(&g_hsm, HSM_STATE_DOOR_OPENING);
			} else if (g_hsm.state == HSM_STATE_DOOR_CLOSING) {
				uint8_t resume_pct     = g_hsm.door_pct;
				hsm_transition(&g_hsm, HSM_STATE_DOOR_OPENING);
				g_hsm.phase_elapsed_ms = ((uint32_t)resume_pct * ELEV_DOOR_MOVE_MS) / 100U;
				g_hsm.door_pct         = resume_pct;
			}
			drive_interlocked_outputs(&g_hsm);
			log_bb   = true;
			bb_code  = BB_FAULT_OVERLOAD;
			bb_state = g_hsm.state;
			bb_floor = g_hsm.current_floor;
			bb_param = (int32_t)ev.param;
		}
		break;

	case EV_OVERLOAD_CLEAR:
		if (g_hsm.overload_active) {
			g_hsm.overload_active = false;
			if (g_hsm.state == HSM_STATE_DOOR_OPEN) {
				g_hsm.phase_elapsed_ms = 0U;
			}
		}
		break;

	case EV_CAN_LINK_LOST:
		if (!g_hsm.can_fault_active) {
			g_hsm.can_fault_active = true;
			g_hsm.safety_abort_count++;
			if (!g_hsm.fire_active) {
				hsm_transition(&g_hsm, HSM_STATE_FAIL_SAFE);
			}
			log_bb   = true;
			bb_code  = BB_FAULT_CAN_LINK_LOST;
			bb_state = g_hsm.state;
			bb_floor = g_hsm.current_floor;
		}
		break;

	case EV_CAN_LINK_RESTORED:
		if (g_hsm.can_fault_active) {
			g_hsm.can_fault_active = false;
			if (g_hsm.state == HSM_STATE_FAIL_SAFE && !g_hsm.estop_active) {
				recover_from_fault(&g_hsm);
			}
		}
		break;

	case EV_CLEAR_FAULT:
		g_hsm.estop_active     = false;
		g_hsm.fire_active      = false;
		g_hsm.overload_active  = false;
		g_hsm.can_fault_active = false;
		g_hsm.last_obstruct_ms = 0U;
		log_bb   = true;
		bb_code  = BB_FAULT_CLEARED;
		bb_state = g_hsm.state;
		bb_floor = g_hsm.current_floor;
		recover_from_fault(&g_hsm);
		break;

	default:
		break;
	}

	k_mutex_unlock(&g_hsm_state_mutex);

	if (log_bb) {
		(void)bsp_blackbox_append(bb_code, bb_state, bb_floor, bb_param);
	}
}

bool elevator_hsm_is_obstruct_recent(uint32_t window_ms)
{
	uint32_t ts = g_hsm.last_obstruct_ms;
	if (ts == 0U) {
		return false;
	}
	return (k_uptime_get_32() - ts) <= window_ms;
}

void elevator_hsm_get_telemetry(elevator_telemetry_t *out)
{
	if (out == NULL) {
		return;
	}

	k_mutex_lock(&g_hsm_state_mutex, K_FOREVER);
	memset(out, 0, sizeof(*out));
	out->state            = (uint8_t)g_hsm.state;
	out->current_floor    = g_hsm.current_floor;
	out->target_floor     = g_hsm.target_floor;
	out->direction        = (int8_t)g_hsm.direction;
	out->door_pos         = (uint8_t)g_hsm.door_pos;
	out->door_pct         = g_hsm.door_pct;
	out->motor_up         = (g_hsm.state == HSM_STATE_MOVING_UP) &&
				(g_hsm.door_pos == DOOR_POS_CLOSED) &&
				!g_hsm.estop_active && !g_hsm.overload_active;
	out->motor_down       = ((g_hsm.state == HSM_STATE_MOVING_DOWN && !g_hsm.overload_active) ||
				 (g_hsm.state == HSM_STATE_FIRE_EVAC && g_hsm.current_floor > ELEV_MIN_FLOOR)) &&
				(g_hsm.door_pos == DOOR_POS_CLOSED) && !g_hsm.estop_active;
	out->fire_active          = g_hsm.fire_active;
	out->estop_active         = g_hsm.estop_active;
	out->overload_active      = g_hsm.overload_active;
	out->can_fault_active     = g_hsm.can_fault_active;
	out->wdt_reset_flag       = bsp_wdt_was_reset_cause();
	out->cabin_mask           = g_hsm.queues.cabin_mask;
	out->hall_up_mask         = g_hsm.queues.hall_up_mask;
	out->hall_down_mask       = g_hsm.queues.hall_down_mask;
	out->uptime_ms            = k_uptime_get_32();
	out->trip_count           = g_hsm.trip_count;
	out->transition_count     = g_hsm.transition_count;
	out->safety_abort_count   = g_hsm.safety_abort_count;
	out->last_exti_latency_us = g_hsm.last_exti_latency_us;
	out->max_exti_latency_us  = g_hsm.max_exti_latency_us;
	k_mutex_unlock(&g_hsm_state_mutex);
}

uint32_t elevator_hsm_get_visited_state_mask(void)
{
	return g_hsm.visited_states_mask;
}

void elevator_hsm_get_transition_counts(uint16_t out_matrix[HSM_STATE_COUNT][HSM_STATE_COUNT])
{
	if (out_matrix == NULL) {
		return;
	}

	k_mutex_lock(&g_hsm_state_mutex, K_FOREVER);
	memcpy(out_matrix, g_hsm.transition_matrix, sizeof(g_hsm.transition_matrix));
	k_mutex_unlock(&g_hsm_state_mutex);
}

void elevator_hsm_get_queue_stats(uint32_t *sent, uint32_t *recv, uint32_t *dropped, uint32_t *in_q)
{
	if (sent) {
		*sent = (uint32_t)atomic_get(&g_isr_ev_sent);
	}
	if (recv) {
		*recv = (uint32_t)atomic_get(&g_isr_ev_recv);
	}
	if (dropped) {
		*dropped = (uint32_t)atomic_get(&g_isr_ev_drop);
	}
	if (in_q) {
		*in_q = k_msgq_num_used_get(&g_hsm_event_msgq);
	}
}

const char *elevator_state_str(elevator_state_t st)
{
	static const char *const names[HSM_STATE_COUNT] = {
		"INIT", "IDLE_CLOSED", "DOOR_OPENING", "DOOR_OPEN", "DOOR_CLOSING",
		"MOVING_UP", "MOVING_DOWN", "FIRE_EVAC", "FAIL_SAFE",
	};

	return (st < HSM_STATE_COUNT) ? names[st] : "UNKNOWN";
}

const char *elevator_fault_str(blackbox_fault_code_t code)
{
	switch (code) {
	case BB_FAULT_BOOT_POR:
		return "BOOT_POR";
	case BB_FAULT_BOOT_WATCHDOG:
		return "BOOT_WDT";
	case BB_FAULT_DOOR_OBSTRUCT:
		return "DOOR_OBSTRUCT";
	case BB_FAULT_ESTOP:
		return "ESTOP";
	case BB_FAULT_FIRE_OVERHEAT:
		return "FIRE_OVERHEAT";
	case BB_FAULT_OVERLOAD:
		return "OVERLOAD";
	case BB_FAULT_CAN_LINK_LOST:
		return "CAN_LINK_LOST";
	case BB_FAULT_CAN_BUS_OFF:
		return "CAN_BUS_OFF";
	case BB_FAULT_CLEARED:
		return "FAULT_CLEARED";
	default:
		return "UNKNOWN";
	}
}
