/**
 * @file elevator_hsm.c
 * @brief Máy trạng thái phân cấp an toàn trọng yếu 9 trạng thái (HSM) & Bộ đo độ phủ V-Model.
 *
 * Chức năng tổng quát của file:
 *  - Điều khiển toàn bộ vòng đời vận hành của thang máy 7 tầng qua 9 trạng thái:
 *    `INIT`, `IDLE_CLOSED`, `DOOR_OPENING`, `DOOR_OPEN`, `DOOR_CLOSING`, `MOVING_UP`,
 *    `MOVING_DOWN`, `FIRE_EVAC` (Sơ tán cháy về Tầng 1), và `FAIL_SAFE` (Khóa an toàn khi E-Stop/đứt CAN).
 *  - Thực thi khóa liên động cửa - động cơ, tính độ trễ ngắt EXTI từ chu kỳ DWT và ghi ma trận độ phủ `[9][9]`.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `elevator_hsm_init()`          : Khởi tạo trạng thái ban đầu tại Tầng 1, cửa đóng và bật timer nhịp 100 ms.
 *  - `elevator_hsm_post_event()`    : Đẩy một sự kiện (gọi tầng, kẹt cửa, cháy, E-Stop...) vào hàng đợi `g_hsm_event_msgq`.
 *  - `elevator_hsm_step()`          : Hàm chính của luồng `t2_fsm`: lấy sự kiện từ hàng đợi và thực thi chuyển trạng thái.
 *  - `hsm_transition()`             : Chuyển trạng thái HSM, cập nhật ma trận độ phủ `[9][9]` và điều khiển động cơ Y0/Y1.
 *  - `elevator_hsm_get_telemetry()` : Chụp lại toàn bộ thông số trạng thái hiện thời (an toàn đa luồng qua Mutex).
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

/* Bộ đếm nguyên tử thống kê số nhịp tick đẩy từ ISR sang luồng HSM */
static atomic_t g_isr_ev_sent = ATOMIC_INIT(0);
static atomic_t g_isr_ev_recv = ATOMIC_INIT(0);
static atomic_t g_isr_ev_drop = ATOMIC_INIT(0);

/**
 * @brief Hàm ngắt k_timer chu kỳ 100 ms: đẩy sự kiện EV_TICK_100MS vào k_msgq không chặn (K_NO_WAIT).
 *        Tuân thủ kỷ luật ISR: không chặn, không log/printk, không gọi GPIO trực tiếp.
 */
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
    door_pos_t door_pos;
    uint8_t door_pct;
    uint32_t phase_elapsed_ms;
    elev_sched_queues_t queues;
    bool fire_active;
    bool estop_active;
    bool overload_active;
    bool can_fault_active;
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
                     (ctx->state == HSM_STATE_FIRE_EVAC && ctx->current_floor > ELEV_MIN_FLOOR && ctx->door_pos == DOOR_POS_CLOSED);

    /* Bất biến an toàn: cấm tuyệt đối đóng điện động cơ nếu cửa chưa đóng kín hoặc đang E-Stop */
    if (ctx->door_pos != DOOR_POS_CLOSED || ctx->estop_active) {
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
    /* Ghi nhận trạng thái và cạnh chuyển [trước][sau] phục vụ đánh giá độ phủ V-Model */
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
        elev_sched_clear_floor(&ctx->queues, ctx->current_floor);
        break;
    case HSM_STATE_DOOR_CLOSING:
        ctx->door_pos = DOOR_POS_CLOSING;
        break;
    case HSM_STATE_MOVING_UP:
        ctx->direction = ELEV_DIR_UP;
        ctx->door_pos  = DOOR_POS_CLOSED;
        ctx->door_pct  = 0U;
        break;
    case HSM_STATE_MOVING_DOWN:
        ctx->direction = ELEV_DIR_DOWN;
        ctx->door_pos  = DOOR_POS_CLOSED;
        ctx->door_pct  = 0U;
        break;
    case HSM_STATE_FIRE_EVAC:
        /* Siêu trạng thái cưỡng chế: xóa toàn bộ cuộc gọi thường và đưa cabin về Tầng 1 */
        elev_sched_reset(&ctx->queues);
        ctx->target_floor = ELEV_MIN_FLOOR;
        bsp_pwm_buzzer_play_tone(BUZZER_TONE_ALARM_SIREN);
        break;
    case HSM_STATE_FAIL_SAFE:
        ctx->direction = ELEV_DIR_IDLE;
        bsp_pwm_buzzer_beep(1200U, 300U);
        break;
    default:
        break;
    }

    drive_interlocked_outputs(ctx);
}

/**
 * @brief Tính toán độ trễ phản hồi từ lúc ngắt EXTI xảy ra tới khi luồng HSM xử lý sự kiện (micro-giây).
 */
static void record_exti_latency(hsm_ctx_t *ctx, uint32_t isr_cycles)
{
    if (isr_cycles == 0U) {
        return;
    }

    /* Phép trừ unsigned 32-bit vẫn cho độ lệch đúng ngay cả khi bộ đếm DWT bị tràn vòng */
    uint32_t delta = k_cycle_get_32() - isr_cycles;
    uint32_t us    = k_cyc_to_us_ceil32(delta);

    ctx->last_exti_latency_us = us;
    if (us > ctx->max_exti_latency_us) {
        ctx->max_exti_latency_us = us;
    }
}

/**
 * @brief Xử lý chuỗi bước sơ tán hỏa hoạn: đóng cửa an toàn, chạy thẳng về Tầng 1 rồi giữ mở cửa thoát hiểm.
 */
static void step_fire_evac_state(hsm_ctx_t *ctx, uint32_t dt_ms)
{
    ARG_UNUSED(dt_ms);

    /* Pha 1: Nếu đang kẹt giữa tầng (> F1) mà cửa còn mở -> đóng kín cửa trước khi chạy */
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

    /* Pha 2: Cửa đã đóng kín -> bật chiều XUỐNG chạy thẳng về Tầng 1 và giữ mở cửa thoát hiểm */
    if (ctx->current_floor > ELEV_MIN_FLOOR) {
        ctx->direction = ELEV_DIR_DOWN;
        drive_interlocked_outputs(ctx);
        if (ctx->phase_elapsed_ms >= ELEV_TRAVEL_FLOOR_MS) {
            ctx->phase_elapsed_ms = 0U;
            ctx->current_floor--;
        }
    } else {
        ctx->direction = ELEV_DIR_IDLE;
        ctx->door_pos  = DOOR_POS_OPEN;
        ctx->door_pct  = 100U;
        drive_interlocked_outputs(ctx);
    }
}

/**
 * @brief [HÀM 1/3: XỬ LÝ NHỊP ĐỒNG HỒ 100 MS (`handle_periodic_tick`)]
 *        Được gọi định kỳ mỗi 100 ms (khi nhận sự kiện `EV_TICK_100MS`) để cập nhật
 *        tiến trình theo thời gian của từng trạng thái thang máy:
 *        - `IDLE_CLOSED`  : Hỏi bộ lập lịch LOOK xem có tầng nào đang gọi không để mở cửa hoặc xuất phát.
 *        - `DOOR_OPENING` : Tăng độ mở cửa `door_pct` từ 0% -> 100% trong 700 ms (`ELEV_DOOR_MOVE_MS`).
 *        - `DOOR_OPEN`    : Giữ cửa mở 1800 ms (`ELEV_DOOR_HOLD_MS`); nếu đang quá tải hoặc cháy ở F1 thì giữ mở mãi.
 *        - `DOOR_CLOSING` : Giảm độ mở cửa `door_pct` từ 100% -> 0% trong 700 ms rồi chuyển về `IDLE_CLOSED`.
 *        - `MOVING_UP/DN` : Cứ đủ 1500 ms (`ELEV_TRAVEL_FLOOR_MS`) thì tăng/giảm `current_floor` 1 tầng
 *                           và kiểm tra xem tầng vừa tới có khách gọi để dừng lại mở cửa không.
 *        - `FIRE_EVAC`    : Đóng cửa (nếu đang ở tầng > 1) rồi hạ thẳng một mạch về Tầng 1 mở cửa cứu hộ.
 *        - `FAIL_SAFE`    : Giữ ngắt điện động cơ Y0/Y1 và mở cửa an toàn nếu mất kết nối CAN.
 */
static void handle_periodic_tick(hsm_ctx_t *ctx, uint32_t dt_ms)
{
    /* Cộng dồn thời gian đã trôi qua trong pha trạng thái hiện tại (đơn vị: ms) */
    ctx->phase_elapsed_ms += dt_ms;

    switch (ctx->state) {
    case HSM_STATE_IDLE_CLOSED: {
        /* Thang đang đỗ đóng cửa -> gọi thuật toán LOOK chọn tầng đích tiếp theo */
        elevator_dir_t next_dir = ELEV_DIR_IDLE;
        uint8_t next_floor = elev_sched_pick_next_target(&ctx->queues, ctx->current_floor, ctx->direction, &next_dir);
        if (next_floor == ctx->current_floor && next_floor != 0U) {
            /* Khách gọi ngay tại tầng đang đỗ -> mở cửa đón ngay */
            ctx->target_floor = next_floor;
            hsm_transition(ctx, HSM_STATE_DOOR_OPENING);
        } else if (next_floor != 0U) {
            /* Có khách ở tầng khác -> bật động cơ Y0 (LÊN) hoặc Y1 (XUỐNG) để di chuyển */
            ctx->target_floor = next_floor;
            hsm_transition(ctx, (next_dir == ELEV_DIR_UP) ? HSM_STATE_MOVING_UP : HSM_STATE_MOVING_DOWN);
        }
        break;
    }

    case HSM_STATE_DOOR_OPENING:
        /* Nội suy phần trăm độ mở cửa (0..100%) theo thời gian mở cửa 700 ms */
        ctx->door_pct = (uint8_t)MIN(100U, (ctx->phase_elapsed_ms * 100U) / ELEV_DOOR_MOVE_MS);
        if (ctx->phase_elapsed_ms >= ELEV_DOOR_MOVE_MS) {
            hsm_transition(ctx, HSM_STATE_DOOR_OPEN);
        }
        break;

    case HSM_STATE_DOOR_OPEN:
        /* Giữ mở cửa vô thời hạn nếu đang báo cháy ở Tầng 1 hoặc còn quá tải cabin */
        if (ctx->fire_active && ctx->current_floor == ELEV_MIN_FLOOR) {
            break;
        }
        if (ctx->overload_active) {
            /* Khi còn quá tải (>= 800 kg): không cho đóng cửa và phát lại còi mỗi 1000 ms */
            if ((ctx->phase_elapsed_ms % 1000U) < dt_ms) {
                bsp_pwm_buzzer_play_tone(BUZZER_TONE_OVERLOAD);
            }
            break;
        }
        /* Hết thời gian chờ khách ra/vào (1800 ms) -> bắt đầu đóng cửa */
        if (ctx->phase_elapsed_ms >= ELEV_DOOR_HOLD_MS) {
            hsm_transition(ctx, HSM_STATE_DOOR_CLOSING);
        }
        break;

    case HSM_STATE_DOOR_CLOSING: {
        /* Nội suy phần trăm cửa khép dần từ 100% về 0% trong 700 ms */
        uint32_t pct  = (ctx->phase_elapsed_ms * 100U) / ELEV_DOOR_MOVE_MS;
        ctx->door_pct = (pct >= 100U) ? 0U : (uint8_t)(100U - pct);
        if (ctx->phase_elapsed_ms >= ELEV_DOOR_MOVE_MS) {
            /* Trước khi đóng kín để chạy tiếp, kiểm tra lại các cờ an toàn đang chốt */
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
        /* Mỗi tầng di chuyển mất 1500 ms (ELEV_TRAVEL_FLOOR_MS) */
        if (ctx->phase_elapsed_ms >= ELEV_TRAVEL_FLOOR_MS) {
            ctx->phase_elapsed_ms = 0U;
            if (ctx->state == HSM_STATE_MOVING_UP && ctx->current_floor < ELEV_MAX_FLOOR) {
                ctx->current_floor++;
            } else if (ctx->state == HSM_STATE_MOVING_DOWN && ctx->current_floor > ELEV_MIN_FLOOR) {
                ctx->current_floor--;
            }
            ctx->trip_count++;

            /* Dừng mở cửa nếu có lệnh phục vụ tầng hiện tại (gom khách cùng chiều) hoặc đã chạm tầng đích */
            if (elev_sched_should_stop(&ctx->queues, ctx->current_floor, ctx->direction) || ctx->current_floor == ctx->target_floor) {
                hsm_transition(ctx, HSM_STATE_DOOR_OPENING);
            }
        }
        break;

    case HSM_STATE_FIRE_EVAC:
        /* Thực thi kịch bản sơ tán cháy: đóng cửa và hạ thẳng về Tầng 1 */
        step_fire_evac_state(ctx, dt_ms);
        break;

    case HSM_STATE_FAIL_SAFE:
        /* Trạng thái khóa an toàn: nếu lỗi do đứt CAN (không phải E-Stop) thì mở cửa tại tầng hiện tại */
        if (ctx->can_fault_active && !ctx->estop_active && ctx->door_pos == DOOR_POS_CLOSED) {
            ctx->door_pos = DOOR_POS_OPEN;
            ctx->door_pct = 100U;
        }
        drive_interlocked_outputs(ctx);
        break;

    default:
        hsm_transition(ctx, HSM_STATE_IDLE_CLOSED);
        break;
    }
}

/**
 * @brief Khởi tạo lại hàng đợi thông điệp, bộ đếm nguyên tử, đặt cabin tại Tầng 1 và bật timer 100 ms.
 */
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

    /* Khởi chạy ngắt timer định kỳ 100 ms để đẩy sự kiện điều nhịp cho HSM */
    k_timer_start(&g_hsm_tick_timer, K_MSEC(100), K_MSEC(100));
    return 0;
}

/**
 * @brief [HÀM 2/3: BỎ THƯ SỰ KIỆN VÀO HÒM THƯ HSM (`elevator_hsm_post_event`)]
 *        Hàm cửa ngõ dùng chung để MỌI NGUỒN (cả Ngắt cứng EXTI, Luồng cảm biến T1,
 *        Mạng CAN T3a, Bàn phím T4 lẫn Lệnh giả lập Shell/Python) gửi sự kiện `ev`
 *        vào hàng đợi `g_hsm_event_msgq` theo chế độ không chặn (`K_NO_WAIT`).
 */
int elevator_hsm_post_event(const elevator_event_t *ev)
{
    if (ev == NULL) {
        return -EINVAL;
    }

    /* Đẩy gói sự kiện vào hàng đợi 32 khe; K_NO_WAIT giúp an toàn khi gọi từ trong ngắt ISR */
    int err = k_msgq_put(&g_hsm_event_msgq, ev, K_NO_WAIT);
    if (err == 0) {
        atomic_inc(&g_isr_ev_sent); /* Tăng bộ đếm tổng số sự kiện đã gửi thành công */
    } else {
        atomic_inc(&g_isr_ev_drop); /* Tăng bộ đếm nếu hàng đợi đầy bị rơi gói */
    }
    return err;
}

/**
 * @brief [HÀM 3/3: VÒNG XỬ LÝ SỰ KIỆN TRUNG TÂM CỦA BỘ NÃO HSM (`elevator_hsm_step`)]
 *        Được Luồng T2 (`fsm_thread_entry`) gọi liên tục:
 *        1. Ngủ chờ lấy 1 lá thư sự kiện `ev` từ hòm thư `g_hsm_event_msgq` (`K_FOREVER`).
 *        2. Khóa Mutex `g_hsm_state_mutex` để bảo vệ biến trạng thái chung `g_hsm`.
 *        3. Rẽ nhánh `switch (ev.type)` để xử lý nhịp 100 ms, lệnh gọi tầng, đóng/mở cửa,
 *           và 5 sự kiện an toàn (Kẹt cửa X0, E-Stop X1, Báo cháy LM35, Quá tải VR1, Đứt CAN).
 */
void elevator_hsm_step(void)
{
    elevator_event_t ev = { 0 };
    /* Chờ lấy 1 sự kiện từ hòm thư (luồng T2 tự ngủ, không tốn CPU khi chưa có sự kiện mới) */
    int err = k_msgq_get(&g_hsm_event_msgq, &ev, K_FOREVER);

    if (err != 0) {
        return;
    }
    atomic_inc(&g_isr_ev_recv);

    /* Khóa Mutex trước khi đọc/ghi biến trạng thái dùng chung `g_hsm` */
    k_mutex_lock(&g_hsm_state_mutex, K_FOREVER);

    switch (ev.type) {
    case EV_TICK_100MS:
        /* Nhịp đồng hồ 100 ms từ timer -> gọi hàm 1/3 để cập nhật cửa & di chuyển tầng */
        handle_periodic_tick(&g_hsm, 100U);
        break;

    /* Nhóm lệnh gọi tầng & điều khiển cửa: tự động từ chối khi hệ thống đang báo cháy/lỗi */
    case EV_CALL_CABIN:
        if (!g_hsm.fire_active && !g_hsm.estop_active) {
            (void)elev_sched_add_request(&g_hsm.queues, ev.type, ev.floor);
        }
        break;

    case EV_CALL_HALL_UP:
    case EV_CALL_HALL_DOWN:
        if (!g_hsm.fire_active && !g_hsm.estop_active && !g_hsm.can_fault_active) {
            (void)elev_sched_add_request(&g_hsm.queues, ev.type, ev.floor);
        }
        break;

    case EV_CMD_OPEN_DOOR:
        if (g_hsm.state == HSM_STATE_IDLE_CLOSED) {
            hsm_transition(&g_hsm, HSM_STATE_DOOR_OPENING);
        } else if (g_hsm.state == HSM_STATE_DOOR_CLOSING) {
            /* Đang đóng dở mà bấm mở cửa -> mở tiếp từ % hiện tại không giật về 0% */
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

    /*
     * =========================================================================
     * [ĐIỂM HỘI TỤ HSM - XỬ LÝ CHUNG CHO CẢ NGUỒN 1 (THẬT) & NGUỒN 2 (GIẢ LẬP)]
     * Tất cả sự kiện từ cảm biến/ngắt thật (Nguồn 1) hay từ nút bấm Python/Shell/Phím
     * (Nguồn 2) đều đi vào các nhánh `case` dưới đây và cập nhật chung biến `g_hsm`.
     * =========================================================================
     */
    case EV_DOOR_OBSTRUCT_EXTI: /* Nhận từ chân X0 (PA6) thật HOẶC nút "Obstruct X0" / Phím 9 */
        record_exti_latency(&g_hsm, ev.isr_cycles);
        if (g_hsm.state == HSM_STATE_DOOR_CLOSING) {
            /* Đang đóng cửa bị kẹt -> đảo chiều mở lại từ đúng % hiện tại, tránh giật về 0% */
            uint8_t resume_pct     = g_hsm.door_pct;
            g_hsm.safety_abort_count++;
            hsm_transition(&g_hsm, HSM_STATE_DOOR_OPENING);
            g_hsm.phase_elapsed_ms = ((uint32_t)resume_pct * ELEV_DOOR_MOVE_MS) / 100U;
            g_hsm.door_pct         = resume_pct;
            bsp_pwm_buzzer_play_tone(BUZZER_TONE_DOOR_OBSTRUCT);
            (void)bsp_blackbox_append(BB_FAULT_DOOR_OBSTRUCT, g_hsm.state, g_hsm.current_floor, 0);
        } else if (g_hsm.state == HSM_STATE_DOOR_OPEN) {
            /* Cửa đã mở hết 100% -> giữ nguyên trạng thái và đặt lại bộ đếm thời gian chờ đóng */
            g_hsm.safety_abort_count++;
            g_hsm.phase_elapsed_ms = 0U;
            bsp_pwm_buzzer_play_tone(BUZZER_TONE_DOOR_OBSTRUCT);
            (void)bsp_blackbox_append(BB_FAULT_DOOR_OBSTRUCT, g_hsm.state, g_hsm.current_floor, 0);
        }
        break;

    case EV_ESTOP_EXTI: /* Nhận từ chân X1 (PA7) thật HOẶC nút "E-STOP X1" / Phím 11 */
        record_exti_latency(&g_hsm, ev.isr_cycles);
        g_hsm.estop_active = true;
        g_hsm.safety_abort_count++;
        hsm_transition(&g_hsm, HSM_STATE_FAIL_SAFE);
        (void)bsp_blackbox_append(BB_FAULT_ESTOP, g_hsm.state, g_hsm.current_floor, 0);
        break;

    case EV_OVERHEAT_TEMP: /* Nhận từ LM35 (PC2) thật >= 55°C HOẶC nút "Inject Fire" / Phím 10 */
        if (!g_hsm.fire_active) {
            g_hsm.fire_active = true;
            g_hsm.safety_abort_count++;
            hsm_transition(&g_hsm, HSM_STATE_FIRE_EVAC);
            (void)bsp_blackbox_append(BB_FAULT_FIRE_OVERHEAT, g_hsm.state, g_hsm.current_floor, (int32_t)ev.param);
        }
        break;

    case EV_TEMP_NORMAL: /* Nhận từ LM35 (PC2) thật khi nguội về <= 48°C */
        if (g_hsm.fire_active) {
            g_hsm.fire_active = false;
            /* Chỉ rời FIRE_EVAC về IDLE nếu không còn lỗi E-Stop hay mất CAN đang chốt */
            if (!g_hsm.estop_active && !g_hsm.can_fault_active) {
                hsm_transition(&g_hsm, HSM_STATE_IDLE_CLOSED);
            }
        }
        break;

    case EV_OVERLOAD_SET: /* Nhận từ biến trở VR1 (PC1) thật >= 800kg / X3 HOẶC nút "Inject Overload" */
        if (!g_hsm.overload_active) {
            g_hsm.overload_active = true;
            (void)bsp_blackbox_append(BB_FAULT_OVERLOAD, g_hsm.state, g_hsm.current_floor, (int32_t)ev.param);
        }
        break;

    case EV_OVERLOAD_CLEAR: /* Nhận từ biến trở VR1 (PC1) thật khi giảm về <= 750kg */
        g_hsm.overload_active = false;
        break;

    case EV_CAN_LINK_LOST:
        if (!g_hsm.can_fault_active) {
            g_hsm.can_fault_active = true;
            g_hsm.safety_abort_count++;
            if (!g_hsm.fire_active) {
                hsm_transition(&g_hsm, HSM_STATE_FAIL_SAFE);
            }
            (void)bsp_blackbox_append(BB_FAULT_CAN_LINK_LOST, g_hsm.state, g_hsm.current_floor, 0);
        }
        break;

    case EV_CAN_LINK_RESTORED:
        if (g_hsm.can_fault_active) {
            g_hsm.can_fault_active = false;
            if (g_hsm.state == HSM_STATE_FAIL_SAFE && !g_hsm.estop_active) {
                hsm_transition(&g_hsm, HSM_STATE_DOOR_CLOSING);
            }
        }
        break;

    case EV_CLEAR_FAULT:
        /* Chỉ gỡ chốt lỗi và đưa cabin về IDLE_CLOSED an toàn (đóng cửa về 0%) khi lệnh xóa lỗi được kích hoạt */
        g_hsm.estop_active     = false;
        g_hsm.fire_active      = false;
        g_hsm.overload_active  = false;
        g_hsm.can_fault_active = false;
        (void)bsp_blackbox_append(BB_FAULT_CLEARED, g_hsm.state, g_hsm.current_floor, 0);
        hsm_transition(&g_hsm, HSM_STATE_IDLE_CLOSED);
        break;

    default:
        break;
    }

    k_mutex_unlock(&g_hsm_state_mutex);
}

/**
 * @brief Sao chép an toàn toàn bộ thông số hiện tại của HSM ra struct telemetry cho HMI/CAN/Shell.
 */
void elevator_hsm_get_telemetry(elevator_telemetry_t *out)
{
    if (out == NULL) {
        return;
    }

    k_mutex_lock(&g_hsm_state_mutex, K_FOREVER);
    memset(out, 0, sizeof(*out));
    out->state                = (uint8_t)g_hsm.state;
    out->current_floor        = g_hsm.current_floor;
    out->target_floor         = g_hsm.target_floor;
    out->direction            = (int8_t)g_hsm.direction;
    out->door_pos             = (uint8_t)g_hsm.door_pos;
    out->door_pct             = g_hsm.door_pct;
    out->motor_up             = (g_hsm.state == HSM_STATE_MOVING_UP);
    out->motor_down           = (g_hsm.state == HSM_STATE_MOVING_DOWN) ||
                                (g_hsm.state == HSM_STATE_FIRE_EVAC && g_hsm.current_floor > ELEV_MIN_FLOOR && g_hsm.door_pos == DOOR_POS_CLOSED);
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

/**
 * @brief Lấy mặt nạ bit đánh dấu các trạng thái đã được thực thi trong quá trình vận hành.
 */
uint32_t elevator_hsm_get_visited_state_mask(void)
{
    return g_hsm.visited_states_mask;
}

/**
 * @brief Sao chép toàn bộ ma trận số lần chuyển trạng thái [từ][đến] cho lệnh kiểm tra độ phủ.
 */
void elevator_hsm_get_transition_counts(uint16_t out_matrix[HSM_STATE_COUNT][HSM_STATE_COUNT])
{
    if (out_matrix == NULL) {
        return;
    }

    k_mutex_lock(&g_hsm_state_mutex, K_FOREVER);
    memcpy(out_matrix, g_hsm.transition_matrix, sizeof(g_hsm.transition_matrix));
    k_mutex_unlock(&g_hsm_state_mutex);
}

/**
 * @brief Truy vấn các bộ đếm nguyên tử sent/recv/dropped và số phần tử đang nằm trong @c k_msgq.
 */
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

/**
 * @brief Quy đổi mã trạng thái HSM sang chuỗi tên phục vụ in Shell CLI và JSON $TWIN.
 */
const char *elevator_state_str(elevator_state_t st)
{
    static const char *const names[HSM_STATE_COUNT] = {
        "INIT", "IDLE_CLOSED", "DOOR_OPENING", "DOOR_OPEN", "DOOR_CLOSING",
        "MOVING_UP", "MOVING_DOWN", "FIRE_EVAC", "FAIL_SAFE",
    };

    return (st < HSM_STATE_COUNT) ? names[st] : "UNKNOWN";
}

/**
 * @brief Quy đổi mã lỗi ghi trong hộp đen EEPROM sang chuỗi ký tự dễ đọc.
 */
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
