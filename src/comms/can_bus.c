/**
 * @file can_bus.c
 * @brief Trình quản lý mạng truyền thông CAN1 (PB8/PB9, 500 kbps) hỗ trợ chạy 1 Kit (Loopback) và 2 Kit (Normal).
 *
 * Chức năng tổng quát của file:
 *  - Cấu hình bộ điều khiển phần cứng bxCAN1 với 3 bộ lọc chấp nhận phần cứng (Emergency `0x080`,
 *    Hall Call `0x100..0x10F`, Status/Heartbeat/Ping `0x200..0x3FF`).
 *  - Hỗ trợ chuyển đổi động giữa chế độ tự kiểm 1 bo (`CAN_MODE_LOOPBACK`) và nối mạng 2 bo (`CAN_MODE_NORMAL`),
 *    cũng như phân vai trò `CABIN` (Kit 1) hoặc `HALL` (Kit 2).
 *
 * Các hàm quan trọng nhất trong file:
 *  - `can_bus_init()`                  : Khởi tạo bxCAN1 theo đúng thứ tự `can_set_mode -> install_hw_filters -> can_start`.
 *  - `can_bus_set_mode()`              : Chuyển đổi trực tiếp giữa chế độ `LOOPBACK` (1 bo) và `NORMAL` (2 bo).
 *  - `can_bus_rx_step()`               : [Luồng T3a] Nhận khung CAN từ hàng đợi lọc phần cứng, trả lời RTR và báo sự kiện cho HSM.
 *  - `can_bus_tx_and_recover_step()`   : [Luồng T3b] Phát định kỳ Heartbeat (`0x300`) & Status (`0x200`) mỗi 250 ms, phục hồi BUS-OFF.
 *  - `can_bus_partner_watchdog_step()` : [Luồng T3c] Chờ nhịp tim đối tác với timeout 2000 ms; nếu quá hạn sẽ kích hoạt `EV_CAN_LINK_LOST`.
 *  - `can_bus_send_emergency()`        : Phát bản tin khẩn cấp ưu tiên cao nhất (`0x080`) khi có cháy hoặc dừng khẩn cấp.
 *  - `can_bus_send_hall_call()`        : Phát yêu cầu gọi tầng từ bảng hành lang (`0x100`) về bộ điều khiển cabin.
 */

#include "can_bus.h"
#include "bsp_i2c_storage.h"
#include "elevator_hsm.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(can_bus, LOG_LEVEL_INF);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

CAN_MSGQ_DEFINE(g_can_rx_msgq, 16);
K_SEM_DEFINE(g_can_partner_sem, 0, 1);
K_SEM_DEFINE(g_can_hb_sem, 0, 1);
K_MUTEX_DEFINE(g_can_state_mutex);

/**
 * @brief Hàm ngắt k_timer chu kỳ 250 ms phát nhịp gửi Heartbeat/Trạng thái CAN.
 *        Chỉ gọi k_sem_give() trong ngữ cảnh ISR để đảm bảo thời gian ngắt cực ngắn.
 */
static void can_hb_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&g_can_hb_sem);
}

K_TIMER_DEFINE(g_can_hb_timer, can_hb_timer_isr, NULL);

static can_link_mode_t g_mode = CAN_LINK_MODE_LOOPBACK;
static node_role_t g_role     = NODE_ROLE_CABIN;
static bool g_can_ready;
static bool g_link_fault_latched;
static uint32_t g_last_recover_ms;

/* Bộ đếm nguyên tử dùng chung an toàn giữa ngắt TX CAN và các luồng công việc */
static atomic_t g_hil_drop     = ATOMIC_INIT(0);
static atomic_t g_rx_ok        = ATOMIC_INIT(0);
static atomic_t g_tx_ok        = ATOMIC_INIT(0);
static atomic_t g_tx_cb_ok     = ATOMIC_INIT(0);
static atomic_t g_tx_cb_err    = ATOMIC_INIT(0);
static atomic_t g_rtr_rx_cnt   = ATOMIC_INIT(0);
static atomic_t g_hb_seq       = ATOMIC_INIT(0);
static atomic_t g_ping_cycles  = ATOMIC_INIT(0);
static atomic_t g_last_rtt_us  = ATOMIC_INIT(0);

/* Trạng thái cabin từ xa được lưu đệm khi bo mạch chạy vai trò NODE_ROLE_HALL (Kit 2) */
static uint8_t g_remote_floor     = ELEV_MIN_FLOOR;
static elevator_dir_t g_remote_dir = ELEV_DIR_IDLE;
static bool g_remote_fire;
static bool g_remote_fault;
static bool g_remote_valid;

/**
 * @brief Callback hoàn tất phát CAN bất đồng bộ (luôn gắn callback cập nhật atomic_t, không truyền NULL).
 *        Chạy trong ngữ cảnh ngắt TX CAN nên chỉ tăng bộ đếm atomic_t.
 */
static void can_tx_done_cb(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (error == 0) {
		atomic_inc(&g_tx_cb_ok);
	} else {
		atomic_inc(&g_tx_cb_err);
	}
}

/**
 * @brief Cài đặt 3 bộ lọc chấp nhận phần cứng trên bxCAN phân theo mức ưu tiên ID 11-bit.
 */
static int install_hw_filters(void)
{
	/*
	 * Quy ước cờ bộ lọc HW (Standard 11-bit ID): một ID đến khớp bộ lọc nếu (rx_id & mask) == (id & mask).
	 *  - Filter 1 (mask 0x7FF): Bắt chính xác duy nhất khung khẩn cấp 0x080.
	 *  - Filter 2 (mask 0x7F0): Bắt dải 16 ID gọi tầng hành lang 0x100..0x10F.
	 *  - Filter 3 (mask 0x600): Bắt dải trạng thái, Heartbeat & HIL PING/PONG (0x200..0x3FF).
	 */
	const struct can_filter f_estop = {
		.id = CAN_ID_EMERGENCY_STOP,
		.mask = 0x7FFU,
		.flags = 0U,
	};
	/* Bộ lọc 2: Nhóm lệnh gọi tầng hành lang (0x100..0x10F) */
	const struct can_filter f_hall = {
		.id = CAN_ID_HALL_CALL,
		.mask = 0x7F0U,
		.flags = 0U,
	};
	/* Bộ lọc 3 (mask 0x600 = bit 10..9 bằng 0b01): gom dải 0x200..0x3FF (Status, Heartbeat, PING/PONG, RTR) */
	const struct can_filter f_status_hb = {
		.id = CAN_ID_CABIN_STATUS,
		.mask = 0x600U,
		.flags = 0U,
	};

	int id1 = can_add_rx_filter_msgq(can_dev, &g_can_rx_msgq, &f_estop);
	int id2 = can_add_rx_filter_msgq(can_dev, &g_can_rx_msgq, &f_hall);
	int id3 = can_add_rx_filter_msgq(can_dev, &g_can_rx_msgq, &f_status_hb);

	if (id1 < 0 || id2 < 0 || id3 < 0) {
		LOG_ERR("CAN hardware filter alloc failed (%d, %d, %d)", id1, id2, id3);
		return -ENOSPC;
	}
	return 0;
}

/**
 * @brief Chuyển chế độ logic của ứng dụng sang cờ mode Zephyr (luôn kèm phục hồi BUS-OFF chủ động).
 */
static can_mode_t to_zephyr_mode(can_link_mode_t mode)
{
	/* Luôn bật MANUAL_RECOVERY để phần mềm giám sát và ghi hộp đen trước khi thoát BUS-OFF */
	can_mode_t base = (mode == CAN_LINK_MODE_LOOPBACK) ?
			  CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
	return base | CAN_MODE_MANUAL_RECOVERY;
}

/**
 * @brief Hàm lõi dùng chung để đóng gói khung CAN (Data hoặc RTR) và gửi không chặn kèm TX callback.
 */
static int send_frame_flags(uint32_t id, uint8_t flags, const uint8_t *data, uint8_t dlc)
{
	if (!g_can_ready || atomic_get(&g_hil_drop) != 0) {
		return -ENETDOWN;
	}

	struct can_frame frame = {
		.id = id,
		.dlc = dlc,
		.flags = flags,
	};
	if ((flags & CAN_FRAME_RTR) == 0U && data != NULL && dlc > 0U) {
		memcpy(frame.data, data, MIN((size_t)dlc, sizeof(frame.data)));
	}

	int err = can_send(can_dev, &frame, K_MSEC(20), can_tx_done_cb, NULL);
	if (err == 0) {
		atomic_inc(&g_tx_ok);
	}
	return err;
}

/**
 * @brief Hàm bọc gửi khung CAN dữ liệu chuẩn (flags = 0).
 */
static int send_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
	return send_frame_flags(id, 0U, data, dlc);
}

/**
 * @brief Khởi tạo ngoại vi CAN1 đúng trình tự driver, mở khóa watchdog đối tác và bật timer HB.
 */
int can_bus_init(can_link_mode_t mode, node_role_t role)
{
	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN1 controller not ready");
		return -ENODEV;
	}

	k_mutex_lock(&g_can_state_mutex, K_FOREVER);
	g_mode = mode;
	g_role = role;
	g_link_fault_latched = false;
	g_remote_valid = false;
	k_mutex_unlock(&g_can_state_mutex);

	/* Thứ tự khởi tạo bắt buộc của Zephyr CAN driver: set_mode -> add_rx_filter -> start */
	int err = can_set_mode(can_dev, to_zephyr_mode(mode));
	if (err != 0) {
		LOG_ERR("can_set_mode failed (%d)", err);
		return err;
	}

	err = install_hw_filters();
	if (err != 0) {
		return err;
	}

	err = can_start(can_dev);
	if (err != 0) {
		LOG_ERR("can_start failed (%d)", err);
		return err;
	}

	g_can_ready = true;
	k_sem_give(&g_can_partner_sem);
	k_timer_start(&g_can_hb_timer, K_MSEC(ELEV_CAN_HB_PERIOD_MS),
		      K_MSEC(ELEV_CAN_HB_PERIOD_MS));

	LOG_INF("CAN1 started @ 500 kbps (mode=%s, role=%s)",
		(mode == CAN_LINK_MODE_LOOPBACK) ? "LOOPBACK" : "NORMAL",
		(role == NODE_ROLE_CABIN) ? "CABIN" : "HALL");
	return 0;
}

/**
 * @brief Dừng tạm thời CAN1, chuyển đổi mode LOOPBACK/NORMAL và khởi động lại controller.
 */
int can_bus_set_mode(can_link_mode_t mode)
{
	if (!g_can_ready) {
		return -ENODEV;
	}

	(void)can_stop(can_dev);
	int err = can_set_mode(can_dev, to_zephyr_mode(mode));
	if (err == 0) {
		k_mutex_lock(&g_can_state_mutex, K_FOREVER);
		g_mode = mode;
		k_mutex_unlock(&g_can_state_mutex);
		err = can_start(can_dev);
		if (err == 0) {
			k_sem_give(&g_can_partner_sem);
		}
	}
	return err;
}

/**
 * @brief Lấy chế độ hoạt động hiện thời (LOOPBACK hoặc NORMAL) an toàn đa luồng.
 */
can_link_mode_t can_bus_get_mode(void)
{
	k_mutex_lock(&g_can_state_mutex, K_FOREVER);
	can_link_mode_t m = g_mode;
	k_mutex_unlock(&g_can_state_mutex);
	return m;
}

/**
 * @brief Chuyển vai trò nút trên bus (CABIN hoặc HALL) và xóa cờ dữ liệu cabin từ xa cũ.
 */
void can_bus_set_role(node_role_t role)
{
	k_mutex_lock(&g_can_state_mutex, K_FOREVER);
	g_role = role;
	g_remote_valid = false;
	k_mutex_unlock(&g_can_state_mutex);
	LOG_INF("Node role switched to %s", (role == NODE_ROLE_CABIN) ? "CABIN" : "HALL");
}

/**
 * @brief Truy vấn vai trò nút hiện tại của bo mạch.
 */
node_role_t can_bus_get_role(void)
{
	k_mutex_lock(&g_can_state_mutex, K_FOREVER);
	node_role_t r = g_role;
	k_mutex_unlock(&g_can_state_mutex);
	return r;
}

/**
 * @brief Đóng gói và truyền khung Trạng thái Cabin (0x200) cùng khung Heartbeat Big-Endian (0x300).
 */
int can_bus_send_status(const elevator_telemetry_t *tel)
{
	if (tel == NULL) {
		return -EINVAL;
	}

	node_role_t role = can_bus_get_role();

	/*
	 * Bố cục gói 0x200 (8 byte) cho hệ 7 tầng:
	 *  [0] state | [1] current_floor (1..7) | [2] target_floor (0..7) | [3] dir+1 (0/1/2)
	 *  [4] door_pct (0..100) | [5] cờ lỗi bit0..3 | [6] cabin_mask (bit 0..6)
	 *  [7] hall_merged_mask (bit 0..6 = tầng 1..7 đang gọi ở hành lang)
	 */
	if (role == NODE_ROLE_CABIN) {
		uint8_t payload[8] = {
			tel->state,
			tel->current_floor,
			tel->target_floor,
			(uint8_t)(tel->direction + 1),
			tel->door_pct,
			(uint8_t)((tel->fire_active ? 1U : 0U) |
				  (tel->estop_active ? 2U : 0U) |
				  (tel->overload_active ? 4U : 0U) |
				  (tel->can_fault_active ? 8U : 0U)),
			tel->cabin_mask,
			(uint8_t)(tel->hall_up_mask | tel->hall_down_mask),
		};
		(void)send_frame(CAN_ID_CABIN_STATUS, payload, sizeof(payload));
	}

	/* Đóng gói các trường đa byte chuẩn Big-Endian: [magic, state, uptime_ms(BE32), seq(BE16)] */
	uint16_t seq = (uint16_t)((uint32_t)atomic_inc(&g_hb_seq) + 1U);
	uint8_t hb[8] = {
		(uint8_t)((role == NODE_ROLE_CABIN) ? 0xA5U : 0x5AU),
		tel->state,
		0U, 0U, 0U, 0U,
		0U, 0U,
	};
	sys_put_be32(tel->uptime_ms, &hb[2]);
	sys_put_be16(seq, &hb[6]);

	return send_frame(CAN_ID_HEARTBEAT, hb, sizeof(hb));
}

/**
 * @brief Gửi lệnh gọi tầng hành lang (0x100) kèm hướng di chuyển sang nút Cabin.
 */
int can_bus_send_hall_call(uint8_t floor, bool going_up)
{
	uint8_t payload[2] = { floor, (uint8_t)(going_up ? 1U : 0U) };
	return send_frame(CAN_ID_HALL_CALL, payload, sizeof(payload));
}

/**
 * @brief Gửi khung cảnh báo khẩn cấp / báo cháy mức ưu tiên cao nhất trên bus (0x080).
 */
int can_bus_send_emergency(uint8_t reason_code, uint8_t floor)
{
	uint8_t payload[3] = { reason_code, floor, 0xFFU };
	return send_frame(CAN_ID_EMERGENCY_STOP, payload, sizeof(payload));
}

/**
 * @brief Chụp chu kỳ CPU DWT hiện tại và gửi khung HIL PING 8 byte Big-Endian (0x380) để đo RTT.
 */
int can_bus_send_ping(uint32_t seq)
{
	uint8_t payload[8] = { 0 };

	/* Ghi số thứ tự (seq) và mốc thời gian phát theo chuẩn Big-Endian 32-bit */
	sys_put_be32(seq, &payload[0]);
	sys_put_be32(k_uptime_get_32(), &payload[4]);

	(void)atomic_set(&g_ping_cycles, (atomic_val_t)k_cycle_get_32());
	return send_frame(CAN_ID_HIL_PING, payload, sizeof(payload));
}

/**
 * @brief Phát khung yêu cầu từ xa RTR (CAN_FRAME_RTR) hỏi trạng thái Cabin trên ID 0x200.
 */
int can_bus_request_cabin_rtr(void)
{
	return send_frame_flags(CAN_ID_CABIN_STATUS, CAN_FRAME_RTR, NULL, 0U);
}

/**
 * @brief Bật hoặc tắt chế độ mô phỏng đứt dây CAN; khi bật sẽ gọi k_sem_reset() để watchdog chạm -EAGAIN.
 */
void can_bus_set_hil_drop(bool drop)
{
	(void)atomic_set(&g_hil_drop, drop ? 1 : 0);
	if (drop) {
		/* Xóa token semaphore tồn đọng để watchdog đo chính xác ngưỡng timeout 2000 ms */
		k_sem_reset(&g_can_partner_sem);
	}
}

/**
 * @brief Tính thời gian trễ khứ hồi (RTT) tính bằng micro-giây khi nhận phản hồi PING/PONG.
 */
static void record_rtt_sample(void)
{
	uint32_t start_cyc = (uint32_t)atomic_get(&g_ping_cycles);
	if (start_cyc != 0U) {
		uint32_t elapsed = k_cycle_get_32() - start_cyc;
		(void)atomic_set(&g_last_rtt_us,
				 (atomic_val_t)k_cyc_to_us_ceil32(elapsed));
		(void)atomic_set(&g_ping_cycles, 0);
	}
}

/**
 * @brief Giải mã khung CAN vừa nhận: ưu tiên tách khung RTR trước khi phân loại các CAN ID dữ liệu.
 */
static void dispatch_rx_frame(const struct can_frame *rx)
{
	atomic_inc(&g_rx_ok);

	/* Phân tách dứt khoát khung truy vấn từ xa RTR khỏi khung Data thông thường */
	if ((rx->flags & CAN_FRAME_RTR) != 0U) {
		atomic_inc(&g_rtr_rx_cnt);
		if (rx->id == CAN_ID_CABIN_STATUS && can_bus_get_role() == NODE_ROLE_CABIN) {
			elevator_telemetry_t tel;
			elevator_hsm_get_telemetry(&tel);
			(void)can_bus_send_status(&tel);
		}
		return;
	}

	if (rx->id == CAN_ID_EMERGENCY_STOP && rx->dlc >= 1U) {
		elevator_event_t ev = {
			.type = (rx->data[0] == 1U) ? EV_OVERHEAT_TEMP : EV_ESTOP_EXTI,
			.isr_cycles = k_cycle_get_32(),
		};
		(void)elevator_hsm_post_event(&ev);
	} else if ((rx->id & 0x7F0U) == CAN_ID_HALL_CALL && rx->dlc >= 2U) {
		if (can_bus_get_role() == NODE_ROLE_CABIN) {
			elevator_event_t ev = {
				.type = (rx->data[1] == 1U) ? EV_CALL_HALL_UP : EV_CALL_HALL_DOWN,
				.floor = rx->data[0],
			};
			(void)elevator_hsm_post_event(&ev);
		}
	} else if (rx->id == CAN_ID_CABIN_STATUS && rx->dlc >= 6U) {
		if (can_bus_get_role() == NODE_ROLE_HALL) {
			k_mutex_lock(&g_can_state_mutex, K_FOREVER);
			g_remote_floor = rx->data[1];
			/* Giải mã độ dời +1 đã dùng khi đóng gói (0=DOWN, 1=IDLE, 2=UP) về -1/0/+1 */
			g_remote_dir   = (elevator_dir_t)((int8_t)rx->data[3] - 1);
			g_remote_fire  = (rx->data[5] & 1U) != 0U;
			g_remote_fault = (rx->data[5] & (2U | 8U)) != 0U;
			g_remote_valid = true;
			k_mutex_unlock(&g_can_state_mutex);
		}
	} else if (rx->id == CAN_ID_HIL_PING) {
		/* Loopback 1 kit đo thẳng RTT ngay khi khung quay về; 2 kit thì trả PONG (0x381) về kit gửi */
		if (can_bus_get_mode() == CAN_LINK_MODE_LOOPBACK) {
			record_rtt_sample();
		} else {
			(void)send_frame(CAN_ID_HIL_PONG, rx->data, rx->dlc);
		}
	} else if (rx->id == CAN_ID_HIL_PONG) {
		record_rtt_sample();
	}
}

/**
 * @brief Tác vụ nhận khung CAN chặn trên hàng đợi @c g_can_rx_msgq, cấp nhịp cho partner watchdog và tự khôi phục khi nối lại dây.
 */
void can_bus_rx_step(void)
{
	struct can_frame rx;
	if (k_msgq_get(&g_can_rx_msgq, &rx, K_FOREVER) != 0) {
		return;
	}

	if (!g_can_ready || atomic_get(&g_hil_drop) != 0) {
		return;
	}

	dispatch_rx_frame(&rx);
	k_sem_give(&g_can_partner_sem);

	k_mutex_lock(&g_can_state_mutex, K_FOREVER);
	bool was_fault = g_link_fault_latched;
	if (was_fault) {
		g_link_fault_latched = false;
	}
	k_mutex_unlock(&g_can_state_mutex);

	if (was_fault) {
		elevator_event_t ev = { .type = EV_CAN_LINK_RESTORED };
		(void)elevator_hsm_post_event(&ev);
		LOG_INF("CAN link restored");
	}
}

/**
 * @brief Chờ semaphore nhịp 250 ms từ @c g_can_hb_timer, giám sát phục hồi BUS-OFF và phát gói trạng thái định kỳ.
 */
void can_bus_tx_and_recover_step(const elevator_telemetry_t *tel)
{
	if (k_sem_take(&g_can_hb_sem, K_MSEC(500)) != 0) {
		return;
	}

	if (!g_can_ready) {
		return;
	}

	/* Đọc thanh ghi lỗi TEC/REC của bxCAN và thực hiện phục hồi có kiểm soát khi BUS-OFF */
	enum can_state st = CAN_STATE_ERROR_ACTIVE;
	struct can_bus_err_cnt cnt = { 0 };
	if (can_get_state(can_dev, &st, &cnt) == 0 && st == CAN_STATE_BUS_OFF) {
		uint32_t now_ms = k_uptime_get_32();
		/* Giới hạn phục hồi mỗi 1000 ms để tránh ghi hộp đen EEPROM dồn dập khi bus vẫn chập */
		if ((now_ms - g_last_recover_ms) > 1000U) {
			g_last_recover_ms = now_ms;
			LOG_ERR("CAN BUS-OFF (TEC=%u REC=%u) -> can_recover()",
				cnt.tx_err_cnt, cnt.rx_err_cnt);
			(void)bsp_blackbox_append(BB_FAULT_CAN_BUS_OFF, HSM_STATE_FAIL_SAFE, 0U, 0);
			(void)can_recover(can_dev, K_MSEC(100));
		}
	}

	if (tel != NULL && atomic_get(&g_hil_drop) == 0) {
		/* Lấy lại ảnh chụp HSM ngay sau khi thức dậy từ semaphore để dữ liệu phát đi luôn mới nhất */
		elevator_telemetry_t fresh_tel;
		elevator_hsm_get_telemetry(&fresh_tel);
		(void)can_bus_send_status(&fresh_tel);
	}
}

/**
 * @brief Giám sát nhịp sống nút đối tác qua k_sem_take(2000 ms); khi hết hạn (-EAGAIN) sẽ đẩy sự kiện EV_CAN_LINK_LOST.
 */
void can_bus_partner_watchdog_step(void)
{
	int err = k_sem_take(&g_can_partner_sem, K_MSEC(ELEV_CAN_TIMEOUT_MS));
	if (err == -EAGAIN) {
		k_mutex_lock(&g_can_state_mutex, K_FOREVER);
		/* Chỉ chốt và ghi sự kiện mất kết nối 1 lần đầu tiên cho tới khi đối tác có tín hiệu lại */
		bool first_fault = !g_link_fault_latched;
		if (first_fault) {
			g_link_fault_latched = true;
			g_remote_valid = false;
		}
		k_mutex_unlock(&g_can_state_mutex);

		if (first_fault) {
			elevator_event_t ev = { .type = EV_CAN_LINK_LOST };
			(void)elevator_hsm_post_event(&ev);
			LOG_WRN("CAN partner timeout (%u ms) -> Fail-Safe triggered",
				ELEV_CAN_TIMEOUT_MS);
		}
	}
}

/**
 * @brief Lấy ảnh chụp trạng thái phần cứng CAN1 (BUS-OFF/ERROR, TEC, REC, tổng RX/TX và RTT).
 */
void can_bus_get_diag(uint8_t *state, uint8_t *tec, uint8_t *rec,
		      uint32_t *rx_ok, uint32_t *tx_ok, uint32_t *last_rtt_us)
{
	enum can_state st = CAN_STATE_STOPPED;
	struct can_bus_err_cnt cnt = { 0 };

	if (g_can_ready) {
		(void)can_get_state(can_dev, &st, &cnt);
	}
	if (state)       *state       = (uint8_t)st;
	if (tec)         *tec         = cnt.tx_err_cnt;
	if (rec)         *rec         = cnt.rx_err_cnt;
	if (rx_ok)       *rx_ok       = (uint32_t)atomic_get(&g_rx_ok);
	if (tx_ok)       *tx_ok       = (uint32_t)atomic_get(&g_tx_ok);
	if (last_rtt_us) *last_rtt_us = (uint32_t)atomic_get(&g_last_rtt_us);
}

/**
 * @brief Đọc thống kê số lần callback TX hoàn tất/lỗi (cbOK/cbErr) và tổng khung RTR đã xử lý.
 */
void can_bus_get_ext_stats(uint32_t *tx_cb_ok, uint32_t *tx_cb_err, uint32_t *rtr_rx)
{
	if (tx_cb_ok)  *tx_cb_ok  = (uint32_t)atomic_get(&g_tx_cb_ok);
	if (tx_cb_err) *tx_cb_err = (uint32_t)atomic_get(&g_tx_cb_err);
	if (rtr_rx)    *rtr_rx    = (uint32_t)atomic_get(&g_rtr_rx_cnt);
}

/**
 * @brief Đọc trạng thái cabin phản chiếu từ mạng CAN dùng cho hiển thị 7 đoạn trên kit HALL (Kit 2).
 */
bool can_bus_get_remote_cabin_view(uint8_t *floor, elevator_dir_t *dir,
				   bool *fire, bool *fault)
{
	k_mutex_lock(&g_can_state_mutex, K_FOREVER);
	bool valid = g_remote_valid && (g_role == NODE_ROLE_HALL);
	if (valid) {
		if (floor) *floor = g_remote_floor;
		if (dir)   *dir   = g_remote_dir;
		if (fire)  *fire  = g_remote_fire;
		if (fault) *fault = g_remote_fault;
	}
	k_mutex_unlock(&g_can_state_mutex);
	return valid;
}
