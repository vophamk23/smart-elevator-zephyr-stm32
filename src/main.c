/**
 * @file main.c
 * @brief Điểm vào chính và Bộ điều phối 7 luồng RTOS tĩnh hướng sự kiện (BKIT ARM4 Smart Elevator).
 *
 * Chức năng tổng quát của file:
 *  - Khởi tạo toàn bộ trình điều khiển phần cứng (Watchdog, GPIO/EXTI, PWM, ADC, SPI1/2, I2C1, CAN1, TFT-LCD FSMC)
 *    và máy trạng thái trung tâm HSM.
 *  - Định nghĩa 7 luồng RTOS tĩnh (`K_THREAD_DEFINE`) và 3 bộ định thời (`k_timer`) điều phối toàn hệ thống.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `main()`                       : Khởi tạo toàn bộ ngoại vi (kể cả TFT-LCD ILI9341), ghi bản ghi bật nguồn vào Hộp đen và kích hoạt 3 `k_timer`.
 *  - `on_hw_exti_event()`           : Callback nhận ngắt cứng EXTI (X0 kẹt cửa, X1 E-Stop), đẩy khẩn vào HSM & đánh thức T1.
 *  - `safety_thread_entry()`        : [Luồng T1 - Prio 2] Đọc 5 kênh ADC1 & 4 chân X0..X3 mỗi 100 ms, phát hiện cháy/quá tải có trễ Hysteresis.
 *  - `fsm_thread_entry()`           : [Luồng T2 - Prio 3] Chạy vòng lặp xử lý sự kiện của Máy trạng thái HSM 9 trạng thái.
 *  - `can_rx_thread_entry()`        : [Luồng T3a - Prio 4] Nhận và giải mã khung CAN từ bộ lọc phần cứng bxCAN1.
 *  - `can_tx_thread_entry()`        : [Luồng T3b - Prio 4] Phát định kỳ Heartbeat/Status CAN mỗi 250 ms & phục hồi lỗi BUS-OFF.
 *  - `can_wdt_thread_entry()`       : [Luồng T3c - Prio 4] Giám sát mất nhịp tim CAN quá 2000 ms để kích hoạt Fail-Safe.
 *  - `hmi_thread_entry()`           : [Luồng T4 - Prio 5] Quét 4 LED 7 đoạn (100 Hz), render TFT-LCD 240x320 (10 Hz), quét bàn phím 4x4 (50 ms), tắt còi PWM & nháy LED3.
 *  - `telemetry_wdt_thread_entry()` : [Luồng T5 - Prio 8] Kiểm tra 4 luồng điểm danh để nạp IWDG và xuất chuỗi JSON `$TWIN` mỗi 200 ms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>

#include "bsp_adc_sensors.h"
#include "bsp_gpio_exti.h"
#include "bsp_i2c_storage.h"
#include "bsp_pwm_buzzer.h"
#include "bsp_spi_hmi.h"
#include "bsp_tft_lcd.h"
#include "bsp_watchdog.h"
#include "can_bus.h"
#include "elevator_hsm.h"
#include "elevator_types.h"
#include "twin_telemetry.h"

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

/** @brief Cờ kiểm thử HIL (`bkit test wdt_hang`) để tạm ngừng nạp IWDG có chủ đích. */
volatile bool g_wdt_freeze_test;

K_SEM_DEFINE(g_safety_wakeup_sem, 0, 1);
K_SEM_DEFINE(g_hmi_sem, 0, 1);
K_SEM_DEFINE(g_wdt_sem, 0, 1);

/** @brief Ngắt timer chu kỳ 100 ms đánh thức luồng T1 kiểm tra cảm biến an toàn. */
static void safety_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&g_safety_wakeup_sem);
}

/** @brief Ngắt timer chu kỳ 10 ms đánh thức luồng T4 quét ghép LED 7 đoạn và bàn phím. */
static void hmi_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&g_hmi_sem);
}

/** @brief Ngắt timer chu kỳ 100 ms đánh thức luồng T5 kiểm tra IWDG và xuất $TWIN. */
static void wdt_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&g_wdt_sem);
}

K_TIMER_DEFINE(g_safety_timer, safety_timer_isr, NULL);
K_TIMER_DEFINE(g_hmi_timer, hmi_timer_isr, NULL);
K_TIMER_DEFINE(g_wdt_timer, wdt_timer_isr, NULL);

/**
 * @brief [NGUỒN 1 - CẦU NỐI NGẮT CỨNG EXTI] Được gọi từ ISR phần cứng X0 (PA6) / X1 (PA7).
 *        Bỏ trực tiếp gói sự kiện `EV_DOOR_OBSTRUCT_EXTI` hoặc `EV_ESTOP_EXTI` vào hòm thư chung
 *        của HSM thông qua `elevator_hsm_post_event(&ev)`.
 */
static void on_hw_exti_event(elevator_ev_type_t ev_type, uint32_t isr_cycles)
{
	elevator_event_t ev = {
		.type = ev_type,
		.isr_cycles = isr_cycles,
	};
	(void)elevator_hsm_post_event(&ev);
	k_sem_give(&g_safety_wakeup_sem);
}

/* ============================================================================
 * T1: Luồng An toàn ưu tiên cao & Đọc cảm biến Analog (Priority 2)
 * ============================================================================ */

/**
 * @brief Điểm vào luồng T1: thức dậy tức thời khi có ngắt EXTI hoặc mỗi 100 ms để giám sát
 *        nhiệt độ LM35 (>= 55 độ C, có trễ hysteresis) và quá tải (>= 800 kg hoặc ngõ vào X3).
 *
 * Các hàm & file được luồng T1 gọi:
 *  - `bsp_wdt_checkin()`         <- `src/bsp/bsp_watchdog.c` (điểm danh IWDG khe `WDT_MON_SAFETY`)
 *  - `can_bus_get_role()`        <- `src/comms/can_bus.c` (chỉ giám sát cảm biến trên Kit `NODE_ROLE_CABIN`)
 *  - `bsp_adc_read_all()`        <- `src/bsp/bsp_adc_sensors.c` (đọc 5 kênh ADC1: LM35, tải, ánh sáng, nguồn)
 *  - `bsp_gpio_read_inputs()`    <- `src/bsp/bsp_gpio_exti.c` (đọc trạng thái cách ly quang `X0..X3`)
 *  - `elevator_hsm_post_event()` <- `src/core/elevator_hsm.c` (gửi sự kiện cháy/quá tải sang luồng T2)
 *  - `can_bus_send_emergency()`  <- `src/comms/can_bus.c` (phát khung CAN khẩn cấp `0x080` khi quá nhiệt)
 */
static void safety_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	bool prev_overheat = false;
	bool prev_overload = false;

	while (1) {
		/* Ngủ chờ semaphore từ timer 100 ms hoặc từ ngắt EXTI (không tốn CPU khi rảnh) */
		(void)k_sem_take(&g_safety_wakeup_sem, K_FOREVER);
		bsp_wdt_checkin(WDT_MON_SAFETY);

		/* Chỉ bo mạch giữ vai trò CABIN (Kit 1) mới giám sát cảm biến giếng thang & tải trọng */
		if (can_bus_get_role() != NODE_ROLE_CABIN) {
			continue;
		}

		/*
		 * [NGUỒN 1 - CẢM BIẾN NHIỆT LM35 THẬT TRÊN BO (PC2)]
		 * Khi hơ nóng LM35 thật >= 55.0°C (550 deci-C) -> gửi `EV_OVERHEAT_TEMP` vào hòm thư HSM.
		 * Khi LM35 nguội về <= 48.0°C (480 deci-C) -> tự động gửi `EV_TEMP_NORMAL` giải trừ cháy.
		 */
		bkit_adc_snapshot_t adc = { 0 };
		bool adc_ok = (bsp_adc_read_all(&adc) == 0 && adc.raw_ok);

		if (adc_ok) {
			if (!prev_overheat && adc.temp_deci_c >= ELEV_TEMP_FIRE_THRESH_DC) {
				prev_overheat = true;
				elevator_event_t ev = {
					.type = EV_OVERHEAT_TEMP,
					.param = (uint32_t)adc.temp_deci_c,
				};
				(void)elevator_hsm_post_event(&ev);
				(void)can_bus_send_emergency(1U, 0U);
			} else if (prev_overheat && adc.temp_deci_c <= ELEV_TEMP_CLEAR_THRESH_DC) {
				prev_overheat = false;
				elevator_event_t ev = { .type = EV_TEMP_NORMAL };
				(void)elevator_hsm_post_event(&ev);
			}
		}

		/*
		 * [NGUỒN 1 - BIẾN TRỞ TẢI VR1 THẬT (PC1) & NGÕ VÀO QUANG X3 (PC5)]
		 * Khi vặn biến trở VR1 thật >= 800 kg (hoặc kích X3) -> gửi `EV_OVERLOAD_SET` vào hòm thư HSM.
		 * Khi vặn giảm VR1 về <= 750 kg (Hysteresis) và X3 nhả -> tự động gửi `EV_OVERLOAD_CLEAR`.
		 */
		bool x0 = false, x1 = false, x2 = false, x3 = false;
		bsp_gpio_read_inputs(&x0, &x1, &x2, &x3);

		if (!prev_overload) {
			bool over_set = (adc_ok && adc.load_kg >= ELEV_LOAD_OVERLOAD_KG) || x3;
			if (over_set) {
				prev_overload = true;
				elevator_event_t ev = {
					.type = EV_OVERLOAD_SET,
					.param = adc.load_kg,
				};
				(void)elevator_hsm_post_event(&ev);
			}
		} else {
			bool over_clear = (!x3) && (!adc_ok || adc.load_kg <= ELEV_LOAD_CLEAR_KG);
			if (over_clear) {
				prev_overload = false;
				elevator_event_t ev = { .type = EV_OVERLOAD_CLEAR };
				(void)elevator_hsm_post_event(&ev);
			}
		}
	}
}

/* ============================================================================
 * T2: Luồng Điều khiển Máy trạng thái HSM & Lập lịch SCAN/LOOK (Priority 3)
 * ============================================================================ */

/**
 * @brief Điểm vào luồng T2: điểm danh watchdog rồi chờ sự kiện trên hàng đợi HSM (`K_FOREVER`).
 *
 * Các hàm & file được luồng T2 gọi:
 *  - `bsp_wdt_checkin()`   <- `src/bsp/bsp_watchdog.c` (điểm danh khe `WDT_MON_FSM`)
 *  - `elevator_hsm_step()` <- `src/core/elevator_hsm.c` (xử lý toàn bộ máy trạng thái HSM; bên trong hàm này
 *                             sẽ gọi tiếp `elevator_scheduler.c`, `bsp_gpio_exti.c`, `bsp_pwm_buzzer.c` và `bsp_i2c_storage.c`)
 */
static void fsm_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		bsp_wdt_checkin(WDT_MON_FSM);
		elevator_hsm_step();
	}
}

/* ============================================================================
 * T3a/b/c: Nhóm 3 luồng Truyền thông CAN1 hướng sự kiện (RX / TX / Partner Watchdog)
 * ============================================================================ */

/**
 * @brief Điểm vào luồng T3a: chặn nhận và giải mã mọi khung CAN hợp lệ đến từ bộ lọc phần cứng.
 *
 * Các hàm & file được luồng T3a gọi:
 *  - `can_bus_rx_step()` <- `src/comms/can_bus.c` (đọc `g_can_rx_msgq`, trả `RTR`, đẩy lệnh sang `elevator_hsm.c`
 *                           và nhả `g_can_partner_sem` cho luồng T3c)
 */
static void can_rx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		can_bus_rx_step();
	}
}

/**
 * @brief Điểm vào luồng T3b: phát định kỳ Heartbeat/Status 250 ms và giám sát phục hồi BUS-OFF.
 *
 * Các hàm & file được luồng T3b gọi:
 *  - `bsp_wdt_checkin()`              <- `src/bsp/bsp_watchdog.c` (điểm danh khe `WDT_MON_CAN`)
 *  - `elevator_hsm_get_telemetry()`   <- `src/core/elevator_hsm.c` (lấy ảnh chụp trạng thái cabin hiện tại)
 *  - `can_bus_tx_and_recover_step()`  <- `src/comms/can_bus.c` (chờ timer 250 ms, gửi CAN `0x200`/`0x300` & khôi phục BUS-OFF)
 */
static void can_tx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		bsp_wdt_checkin(WDT_MON_CAN);
		elevator_telemetry_t tel;
		elevator_hsm_get_telemetry(&tel);
		can_bus_tx_and_recover_step(&tel);
	}
}

/**
 * @brief Điểm vào luồng T3c: giám sát watchdog nút đối tác CAN ngưỡng 2000 ms (bắt timeout `-EAGAIN`).
 *
 * Các hàm & file được luồng T3c gọi:
 *  - `can_bus_partner_watchdog_step()` <- `src/comms/can_bus.c` (nếu quá 2000 ms không nhận khung CAN mới,
 *                                         tự động gửi sự kiện `EV_CAN_LINK_LOST` sang `elevator_hsm.c`)
 */
static void can_wdt_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		can_bus_partner_watchdog_step();
	}
}

/* ============================================================================
 * T4: Luồng HMI dùng chung SPI1 (7-Đoạn + Bàn phím), Còi PF8 & LED PE4 (Prio 5)
 * ============================================================================ */

/**
 * @brief Điểm vào luồng T4: quét LED 7 đoạn (100 Hz), điều nhịp tắt còi PWM, nháy LED PE4
 *        và quét phím 4x4 mỗi 50 ms để phát cuộc gọi cabin (Kit 1) hoặc gọi tầng CAN (Kit 2).
 *
 * Các hàm & file được luồng T4 gọi:
 *  - `bsp_wdt_checkin()`               <- `src/bsp/bsp_watchdog.c` (điểm danh khe `WDT_MON_HMI`)
 *  - `elevator_hsm_get_telemetry()`    <- `src/core/elevator_hsm.c` (đọc tầng/hướng/cảnh báo tại chỗ)
 *  - `can_bus_get_role()`              <- `src/comms/can_bus.c` (phân biệt Kit `CABIN` hay Kit `HALL`)
 *  - `can_bus_get_remote_cabin_view()` <- `src/comms/can_bus.c` (lấy trạng thái cabin từ xa nếu là Kit `HALL`)
 *  - `bsp_pwm_buzzer_tick()`           <- `src/bsp/bsp_pwm_buzzer.c` (đếm ngược tự tắt còi PWM không chặn)
 *  - `bsp_spi_7seg_render_status()`    <- `src/bsp/bsp_spi_hmi.c` (xuất 74HC595 ra LED 7 đoạn qua SPI1)
 *  - `bsp_gpio_set_led_debug()`        <- `src/bsp/bsp_gpio_exti.c` (nháy LED Heartbeat PE4 theo trạng thái lỗi)
 *  - `bsp_spi_keypad_scan()`           <- `src/bsp/bsp_spi_hmi.c` (quét ma trận phím 4x4 qua SPI1)
 *  - `can_bus_send_hall_call()`        <- `src/comms/can_bus.c` (Kit `HALL` gửi lệnh gọi tầng `0x100` qua CAN)
 *  - `elevator_hsm_post_event()`       <- `src/core/elevator_hsm.c` (Kit `CABIN` đẩy `EV_CALL_CABIN` vào HSM)
 */
static void hmi_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint32_t tick_div = 0U;
	int prev_key = -1;

	while (1) {
		(void)k_sem_take(&g_hmi_sem, K_FOREVER);
		bsp_wdt_checkin(WDT_MON_HMI);

		elevator_telemetry_t tel;
		elevator_hsm_get_telemetry(&tel);
		node_role_t role = can_bus_get_role();

		bsp_pwm_buzzer_tick();

		/* Hiển thị trạng thái tại chỗ trên Kit 1, hoặc dữ liệu cabin phản chiếu qua CAN trên Kit 2 */
		uint8_t disp_floor = tel.current_floor;
		elevator_dir_t disp_dir = (elevator_dir_t)tel.direction;
		bool disp_fire = tel.fire_active;
		bool disp_fault = tel.estop_active || tel.can_fault_active;

		if (role == NODE_ROLE_HALL) {
			uint8_t rem_floor = 1U;
			elevator_dir_t rem_dir = ELEV_DIR_IDLE;
			bool rem_fire = false;
			bool rem_fault = false;

			if (can_bus_get_remote_cabin_view(&rem_floor, &rem_dir, &rem_fire, &rem_fault)) {
				disp_floor = rem_floor;
				disp_dir = rem_dir;
				disp_fire = rem_fire;
				disp_fault = rem_fault || tel.can_fault_active;
			}
		}

		(void)bsp_spi_7seg_render_status(disp_floor, disp_dir, disp_fire, disp_fault);

		/* Cập nhật Màn hình đồ họa màu TFT-LCD 240x320 (ILI9341 - FSMC 16-bit) mỗi 100 ms */
		if ((tick_div % 10U) == 0U) {
			bkit_adc_snapshot_t adc_snap;
			(void)bsp_adc_read_all(&adc_snap);
			float load_kg = (float)adc_snap.load_kg;
			float temp_c = (float)adc_snap.temp_deci_c / 10.0f;
			bool x0_door = false;
			bsp_gpio_read_inputs(&x0_door, NULL, NULL, NULL);
			bool curtain_blocked = x0_door || (adc_snap.light_mv > 3000U);
			uint8_t call_mask = tel.cabin_mask | tel.hall_up_mask | tel.hall_down_mask;

			bsp_tft_lcd_render_dashboard(&tel, call_mask, tel.door_pct,
						     load_kg, temp_c, curtain_blocked,
						     elevator_state_str((elevator_state_t)tel.state));
		}

		/* Nháy nhanh LED PE4 (chu kỳ 200 ms) khi có cháy/lỗi, hoặc nháy chậm 800 ms khi bình thường */
		uint32_t blink_period = (disp_fire || disp_fault) ? 10U : 40U;
		if ((tick_div % blink_period) == 0U) {
			bsp_gpio_set_led_debug(((tick_div / blink_period) & 1U) == 0U);
		}

		/*
		 * Quét ma trận phím 4x4 (16 phím: key 0..15) qua SPI1 mỗi 50 ms (tick_div % 5 == 0)
		 * và khử nhấn lặp. Ánh xạ đầy đủ cả 16 phím trên bo BKIT - Arm4 để có thể test
		 * độc lập toàn bộ tính năng ngay trên bo mạch:
		 *  - Phím 0..6   : Gọi Tầng 1..7 (Cabin hoặc Hall CAN tùy vai trò bo mạch)
		 *  - Phím 7      : Mở cửa nhanh (`EV_CMD_OPEN_DOOR`)
		 *  - Phím 8      : Đóng cửa nhanh (`EV_CMD_CLOSE_DOOR`)
		 *  - Phím 9      : Giả lập Kẹt cửa X0 (`EV_DOOR_OBSTRUCT_EXTI`)
		 *  - Phím 10     : Giả lập Báo cháy quá nhiệt (`EV_OVERHEAT_TEMP`)
		 *  - Phím 11     : Kích hoạt Dừng khẩn cấp E-Stop X1 (`EV_ESTOP_EXTI`)
		 *  - Phím 12..15 : Xóa chốt lỗi & khôi phục vận hành (`EV_CLEAR_FAULT`)
		 */
		if ((tick_div % 5U) == 0U) {
			int key = -1;
			if (bsp_spi_keypad_scan(&key, NULL) == 0 && key >= 0 && key != prev_key) {
				LOG_INF("Keypad 4x4 pressed: key_idx=%d", key);
				if (key < ELEV_NUM_FLOORS) {
					uint8_t floor = (uint8_t)(key + 1);
					bsp_pwm_buzzer_beep(2400U, 35U);
					if (role == NODE_ROLE_HALL) {
						/* Tầng đỉnh (F7) chỉ có chiều gọi Xuống; các tầng 1..6 mặc định gọi Lên */
						bool going_up = (floor < ELEV_MAX_FLOOR);
						(void)can_bus_send_hall_call(floor, going_up);
					} else {
						elevator_event_t ev = {
							.type = EV_CALL_CABIN,
							.floor = floor,
						};
						(void)elevator_hsm_post_event(&ev);
					}
				} else if (key == 7) {
					bsp_pwm_buzzer_beep(2200U, 40U);
					elevator_event_t ev = { .type = EV_CMD_OPEN_DOOR };
					(void)elevator_hsm_post_event(&ev);
				} else if (key == 8) {
					bsp_pwm_buzzer_beep(1800U, 40U);
					elevator_event_t ev = { .type = EV_CMD_CLOSE_DOOR };
					(void)elevator_hsm_post_event(&ev);
				} else if (key == 9) {
					elevator_event_t ev = {
						.type = EV_DOOR_OBSTRUCT_EXTI,
						.isr_cycles = k_cycle_get_32(),
					};
					(void)elevator_hsm_post_event(&ev);
				} else if (key == 10) {
					elevator_event_t ev = {
						.type = EV_OVERHEAT_TEMP,
						.param = 620U,
					};
					(void)elevator_hsm_post_event(&ev);
					(void)can_bus_send_emergency(1U, 0U);
				} else if (key == 11) {
					elevator_event_t ev = {
						.type = EV_ESTOP_EXTI,
						.isr_cycles = k_cycle_get_32(),
					};
					(void)elevator_hsm_post_event(&ev);
				} else {
					bsp_pwm_buzzer_beep(2600U, 45U);
					can_bus_set_hil_drop(false);
					elevator_event_t ev = { .type = EV_CLEAR_FAULT };
					(void)elevator_hsm_post_event(&ev);
				}
			}
			prev_key = key;
		}

		tick_div++;
	}
}

/* ============================================================================
 * T5: Luồng giám sát IWDG đa luồng & Xuất gói $TWIN JSON (Priority 8 - Thấp nhất)
 * ============================================================================ */

/**
 * @brief Điểm vào luồng T5: kiểm tra check-in sống của các luồng để nạp IWDG và phát `$TWIN` mỗi 200 ms.
 *
 * Các hàm & file được luồng T5 gọi:
 *  - `bsp_wdt_service()`          <- `src/bsp/bsp_watchdog.c` (chỉ nạp IWDG khi mọi luồng chưa treo > 1500 ms)
 *  - `twin_telemetry_is_enabled()`<- `src/comms/twin_telemetry.c` (kiểm tra cờ cho phép xuất chuỗi `$TWIN`)
 *  - `twin_telemetry_emit_now()`  <- `src/comms/twin_telemetry.c` (đóng gói JSON trạng thái gửi UART cho PC Twin)
 */
static void telemetry_wdt_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint32_t loop_cnt = 0U;

	while (1) {
		(void)k_sem_take(&g_wdt_sem, K_FOREVER);

		/* Chỉ nạp IWDG khi mọi luồng giám sát còn điểm danh hợp lệ trong ngưỡng 1500 ms */
		(void)bsp_wdt_service(g_wdt_freeze_test);

		if (twin_telemetry_is_enabled() && (loop_cnt % 2U) == 0U) {
			twin_telemetry_emit_now();
		}

		loop_cnt++;
	}
}

/* Khởi tạo tĩnh toàn bộ 7 luồng RTOS lúc biên dịch (không cấp phát bộ nhớ động heap) */
K_THREAD_DEFINE(t1_safety,  1536, safety_thread_entry,        NULL, NULL, NULL, 2, 0, 500);
K_THREAD_DEFINE(t2_fsm,     2048, fsm_thread_entry,           NULL, NULL, NULL, 3, 0, 500);
K_THREAD_DEFINE(t3_can_rx,  1536, can_rx_thread_entry,        NULL, NULL, NULL, 4, 0, 500);
K_THREAD_DEFINE(t3_can_tx,  1536, can_tx_thread_entry,        NULL, NULL, NULL, 4, 0, 500);
K_THREAD_DEFINE(t3_can_wdt, 1024, can_wdt_thread_entry,       NULL, NULL, NULL, 4, 0, 500);
K_THREAD_DEFINE(t4_hmi,     1536, hmi_thread_entry,           NULL, NULL, NULL, 5, 0, 500);
K_THREAD_DEFINE(t5_wdt,     1536, telemetry_wdt_thread_entry, NULL, NULL, NULL, 8, 0, 500);

/**
 * @brief Hàm khởi động chính của hệ thống: khởi tạo toàn bộ ngoại vi, lõi HSM/CAN, ghi hộp đen lúc bật nguồn và chạy các `k_timer`.
 *
 * Thứ tự gọi khởi tạo sang các file:
 *  - `bsp_watchdog_init()`, `bsp_wdt_was_reset_cause()` <- `src/bsp/bsp_watchdog.c`
 *  - `bsp_gpio_exti_init()`                             <- `src/bsp/bsp_gpio_exti.c`
 *  - `bsp_pwm_buzzer_init()`, `bsp_pwm_buzzer_play_tone()` <- `src/bsp/bsp_pwm_buzzer.c`
 *  - `bsp_adc_init()`                                   <- `src/bsp/bsp_adc_sensors.c`
 *  - `bsp_spi_hmi_init()`                               <- `src/bsp/bsp_spi_hmi.c`
 *  - `bsp_i2c_storage_init()`, `bsp_blackbox_append()`  <- `src/bsp/bsp_i2c_storage.c`
 *  - `elevator_hsm_init()`                              <- `src/core/elevator_hsm.c`
 *  - `can_bus_init()`                                   <- `src/comms/can_bus.c`
 */
int main(void)
{
	LOG_INF("Booting HCMUT BKIT ARM4 Smart Elevator Firmware (Zephyr %s)",
		KERNEL_VERSION_STRING);

	int err = bsp_watchdog_init();
	if (err < 0) {
		LOG_WRN("IWDG watchdog init returned %d (continuing without IWDG)", err);
	}

	err = bsp_gpio_exti_init(on_hw_exti_event);
	if (err < 0) {
		LOG_ERR("Critical GPIO/EXTI init failed (%d)", err);
		return err;
	}

	(void)bsp_pwm_buzzer_init();
	(void)bsp_adc_init();
	(void)bsp_spi_hmi_init();
	(void)bsp_tft_lcd_init();
	(void)bsp_i2c_storage_init();

	(void)elevator_hsm_init();
	(void)can_bus_init(CAN_LINK_MODE_LOOPBACK, NODE_ROLE_CABIN);

	if (bsp_wdt_was_reset_cause()) {
		LOG_WRN("Previous boot ended via IWDG Watchdog reset -> logging black-box entry");
		(void)bsp_blackbox_append(BB_FAULT_BOOT_WATCHDOG, HSM_STATE_INIT, 1U, 0);
	} else {
		(void)bsp_blackbox_append(BB_FAULT_BOOT_POR, HSM_STATE_IDLE_CLOSED, 1U, 0);
	}

	/* Kích hoạt các k_timer định kỳ sau khi phần cứng và hàng đợi IPC đã sẵn sàng */
	k_timer_start(&g_safety_timer, K_MSEC(100), K_MSEC(100));
	k_timer_start(&g_hmi_timer, K_MSEC(10), K_MSEC(10));
	k_timer_start(&g_wdt_timer, K_MSEC(100), K_MSEC(100));

	bsp_pwm_buzzer_play_tone(BUZZER_TONE_BOOT);

	printk("\n====================================================\n");
	printk(" BKIT ARM4 Smart Elevator Ready (Type 'help' in Shell)\n");
	printk("====================================================\n");
	return 0;
}
