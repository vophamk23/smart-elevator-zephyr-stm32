/**
 * @file main.c
 * @brief Điểm vào chính và Bộ điều phối 7 luồng RTOS tĩnh hướng sự kiện (BKIT ARM4 Smart Elevator).
 *
 * Chức năng tổng quát của file:
 *  - Khởi tạo toàn bộ trình điều khiển phần cứng (Watchdog, GPIO/EXTI, PWM, ADC, SPI1/2, I2C1, CAN1, TFT-LCD FSMC)
 *    và máy trạng thái trung tâm HSM.
 *  - Định nghĩa 7 luồng RTOS tĩnh (`K_THREAD_DEFINE`) và các bộ định thời (`k_timer`) điều phối toàn hệ thống.
 *  - Tự động hiệu chuẩn mức nền ánh sáng phòng cho cảm biến rèm quang LDR (`PC0`) lúc khởi động để chống
 *    báo giả trong phòng tối, đồng thời kích hoạt mở cửa ngược dưới 50 ms khi tay che rèm quang.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
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

/** @brief Trạng thái rèm quang LDR (PC0) sau khi đã hiệu chuẩn nền ánh sáng môi trường. */
static volatile bool g_ldr_curtain_blocked;

K_SEM_DEFINE(g_safety_wakeup_sem, 0, 1);
K_SEM_DEFINE(g_hmi_sem, 0, 1);
K_SEM_DEFINE(g_wdt_sem, 0, 1);

static void safety_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&g_safety_wakeup_sem);
}

static void hmi_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&g_hmi_sem);
}

static void wdt_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&g_wdt_sem);
}

K_TIMER_DEFINE(g_safety_timer, safety_timer_isr, NULL);
K_TIMER_DEFINE(g_hmi_timer, hmi_timer_isr, NULL);
K_TIMER_DEFINE(g_wdt_timer, wdt_timer_isr, NULL);

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
 * T1: Luồng An toàn ưu tiên cao & Đọc cảm biến Analog 50 ms (Priority 2)
 * ============================================================================ */
static void safety_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	bool prev_overheat = false;
	bool prev_overload = false;

	/* Bộ hiệu chuẩn nền tự động cho quang trở LDR (PC0) tránh báo giả khi phòng tối/sáng khác nhau */
	int32_t ldr_baseline_mv = -1;
	uint8_t ldr_init_cnt = 0U;
	int32_t ldr_accum_mv = 0;
	uint32_t ldr_blocked_ticks = 0U;

	while (1) {
		(void)k_sem_take(&g_safety_wakeup_sem, K_FOREVER);
		bsp_wdt_checkin(WDT_MON_SAFETY);

		if (can_bus_get_role() != NODE_ROLE_CABIN) {
			continue;
		}

		elevator_telemetry_t tel;
		elevator_hsm_get_telemetry(&tel);

		/* Đồng bộ trạng thái trễ với cờ thực tế của HSM (phòng trường hợp vừa bấm Xóa lỗi / Tiêm lỗi) */
		prev_overheat = tel.fire_active;
		prev_overload = tel.overload_active;

		bkit_adc_snapshot_t adc = { 0 };
		bool adc_ok = (bsp_adc_read_all(&adc) == 0 && adc.raw_ok);

		/* 1. Giám sát nhiệt độ giếng thang LM35 (PC2) có trễ Hysteresis (>= 55.0 C bật, <= 48.0 C tắt) */
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

		/* 2. Giám sát quá tải cabin VR1 (PC1) & Opto X3 (PC5) có trễ Hysteresis (>= 800 kg / <= 750 kg) */
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

		/* 3. Giám sát Rèm quang chống kẹp cửa LDR (PC0) tự hiệu chuẩn nền môi trường (< 50 ms) */
		if (adc_ok) {
			int32_t cur_mv = (int32_t)adc.light_mv;
			if (ldr_baseline_mv < 0) {
				ldr_accum_mv += cur_mv;
				ldr_init_cnt++;
				if (ldr_init_cnt >= 4U) {
					ldr_baseline_mv = ldr_accum_mv / 4;
				}
			} else {
				int32_t diff = abs(cur_mv - ldr_baseline_mv);
				if (!g_ldr_curtain_blocked) {
					if (diff >= 380) {
						g_ldr_curtain_blocked = true;
						ldr_blocked_ticks = 0U;
					} else if (diff < 120) {
						/* Bám chậm theo độ trôi ánh sáng tự nhiên của phòng */
						ldr_baseline_mv = (ldr_baseline_mv * 31 + cur_mv) / 32;
					}
				} else {
					ldr_blocked_ticks++;
					if (diff <= 220) {
						g_ldr_curtain_blocked = false;
						ldr_blocked_ticks = 0U;
					} else if (ldr_blocked_ticks >= 160U) {
						/* Nếu môi trường đổi độ sáng cố định > 8 giây -> cập nhật lại nền mới */
						ldr_baseline_mv = cur_mv;
						g_ldr_curtain_blocked = false;
						ldr_blocked_ticks = 0U;
					}
				}
			}

			/* Nếu cửa đang khép mà rèm quang LDR hoặc X0 bị che -> mở cửa ngược lại ngay lập tức! */
			if ((g_ldr_curtain_blocked || x0) &&
			    (tel.state == HSM_STATE_DOOR_CLOSING || tel.state == HSM_STATE_DOOR_OPEN)) {
				elevator_event_t ev = {
					.type = EV_DOOR_OBSTRUCT_EXTI,
					.isr_cycles = k_cycle_get_32(),
				};
				(void)elevator_hsm_post_event(&ev);
			}
		}
	}
}

/* ============================================================================
 * T2: Luồng Điều khiển Máy trạng thái HSM & Lập lịch SCAN/LOOK (Priority 3)
 * ============================================================================ */
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
static void can_rx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		can_bus_rx_step();
	}
}

static void can_tx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		elevator_telemetry_t tel;
		elevator_hsm_get_telemetry(&tel);
		bsp_wdt_checkin(WDT_MON_CAN);
		can_bus_tx_and_recover_step(&tel);
	}
}

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
 * T4: Luồng Giao diện HMI (LED 7 đoạn + TFT-LCD + Bàn phím 4x4 + Còi PWM + LED3) (Priority 5)
 * ============================================================================ */
static void hmi_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint32_t tick_div = 0U;
	int prev_stable_key = -1;
	int candidate_key = -1;

	while (1) {
		(void)k_sem_take(&g_hmi_sem, K_FOREVER);
		bsp_wdt_checkin(WDT_MON_HMI);

		bsp_pwm_buzzer_tick();

		elevator_telemetry_t tel;
		elevator_hsm_get_telemetry(&tel);

		node_role_t role = can_bus_get_role();
		uint8_t disp_floor = tel.current_floor;
		elevator_dir_t disp_dir = (elevator_dir_t)tel.direction;
		bool disp_door_open = (tel.door_pos != DOOR_POS_CLOSED);
		bool disp_overload = tel.overload_active;
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
				disp_door_open = false;
				disp_overload = false;
				disp_fire = rem_fire;
				disp_fault = rem_fault || tel.can_fault_active;
			}
		}

		/* Cập nhật nội dung bộ đệm 4 LED 7 đoạn mỗi 20 ms (quét phần cứng chạy riêng ở 500 Hz) */
		if ((tick_div % 2U) == 0U) {
			(void)bsp_spi_7seg_render_status(disp_floor, disp_dir, disp_door_open,
							 disp_overload, disp_fire, disp_fault);
		}

		/* Cập nhật Màn hình màu TFT-LCD 240x320 (ILI9341 - FSMC 16-bit) mỗi 100 ms */
		if ((tick_div % 10U) == 0U) {
			bkit_adc_snapshot_t adc_snap = { 0 };
			(void)bsp_adc_read_all(&adc_snap);
			float load_kg = (float)adc_snap.load_kg;
			float temp_c = (float)adc_snap.temp_deci_c / 10.0f;
			bool x0_door = false;
			bsp_gpio_read_inputs(&x0_door, NULL, NULL, NULL);
			bool curtain_blocked = x0_door || g_ldr_curtain_blocked ||
					       elevator_hsm_is_obstruct_recent(1400U);
			uint8_t call_mask = tel.cabin_mask | tel.hall_up_mask | tel.hall_down_mask;

			bsp_tft_lcd_render_dashboard(&tel, call_mask, tel.door_pct,
						     load_kg, temp_c, curtain_blocked,
						     elevator_state_str((elevator_state_t)tel.state));
		}

		/* Nháy nhanh LED PE4 (chu kỳ 200 ms) khi có cháy/lỗi/quá tải, hoặc nháy chậm 800 ms khi bình thường */
		uint32_t blink_period = (disp_fire || disp_fault || disp_overload) ? 10U : 40U;
		if ((tick_div % blink_period) == 0U) {
			bsp_gpio_set_led_debug(((tick_div / blink_period) & 1U) == 0U);
		}

		/*
		 * Quét ma trận phím 4x4 (16 phím: key 0..15) qua SPI1 kèm lọc chống dội 2 mẫu liên tiếp,
		 * ánh xạ ĐÚNG 100% theo chữ in lụa (Silkscreen) trên mặt bo mạch BKIT ARM4 thật:
		 *   Hàng 0: [1](0)->Tầng 1 | [2](1)->Tầng 2 | [3](2)->Tầng 3 | [▲](3)->Mở cửa nhanh
		 *   Hàng 1: [4](4)->Tầng 4 | [5](5)->Tầng 5 | [6](6)->Tầng 6 | [▼](7)->Đóng cửa nhanh
		 *   Hàng 2: [7](8)->Tầng 7 | [8](9)->Kẹt cửa | [9](10)->Báo cháy | [▶](11)->E-Stop
		 *   Hàng 3: [E](12), [0](13), [B](14), [◀](15) -> Giải trừ lỗi / Khôi phục vận hành
		 */
		if ((tick_div % 3U) == 0U) {
			static const int8_t k_key_to_floor[16] = {
				1,  2,  3, -1, /* Hàng 0: [1], [2], [3], [▲] */
				4,  5,  6, -1, /* Hàng 1: [4], [5], [6], [▼] */
				7, -1, -1, -1, /* Hàng 2: [7], [8], [9], [▶] */
			       -1, -1, -1, -1  /* Hàng 3: [E], [0], [B], [◀] */
			};
			int raw_key = -1;
			if (bsp_spi_keypad_scan(&raw_key, NULL) == 0) {
				if (raw_key == candidate_key) {
					if (raw_key >= 0 && raw_key < 16 && raw_key != prev_stable_key) {
						int8_t mapped_floor = k_key_to_floor[raw_key];
						LOG_INF("Keypad 4x4 pressed: key_idx=%d (floor=%d)",
							raw_key, mapped_floor);
						if (mapped_floor >= ELEV_MIN_FLOOR &&
						    mapped_floor <= ELEV_MAX_FLOOR) {
							uint8_t floor = (uint8_t)mapped_floor;
							bsp_pwm_buzzer_beep(2400U, 35U);
							if (role == NODE_ROLE_HALL) {
								bool going_up = (floor < ELEV_MAX_FLOOR);
								(void)can_bus_send_hall_call(floor, going_up);
							} else {
								elevator_event_t ev = {
									.type = EV_CALL_CABIN,
									.floor = floor,
								};
								(void)elevator_hsm_post_event(&ev);
							}
						} else if (raw_key == 3) { /* Nút [▲]: Mở cửa nhanh */
							bsp_pwm_buzzer_beep(2200U, 40U);
							elevator_event_t ev = { .type = EV_CMD_OPEN_DOOR };
							(void)elevator_hsm_post_event(&ev);
						} else if (raw_key == 7) { /* Nút [▼]: Đóng cửa nhanh */
							bsp_pwm_buzzer_beep(1800U, 40U);
							elevator_event_t ev = { .type = EV_CMD_CLOSE_DOOR };
							(void)elevator_hsm_post_event(&ev);
						} else if (raw_key == 9) { /* Nút [8]: Giả lập Kẹt cửa */
							elevator_event_t ev = {
								.type = EV_DOOR_OBSTRUCT_EXTI,
								.isr_cycles = k_cycle_get_32(),
							};
							(void)elevator_hsm_post_event(&ev);
						} else if (raw_key == 10) { /* Nút [9]: Giả lập Báo cháy 62°C */
							bsp_adc_set_hil_temp_override(620);
							elevator_event_t ev = {
								.type = EV_OVERHEAT_TEMP,
								.param = 620U,
							};
							(void)elevator_hsm_post_event(&ev);
							(void)can_bus_send_emergency(1U, 0U);
						} else if (raw_key == 11) { /* Nút [▶]: Dừng khẩn cấp E-Stop */
							bsp_gpio_set_motor_outputs(false, false);
							elevator_event_t ev = {
								.type = EV_ESTOP_EXTI,
								.isr_cycles = k_cycle_get_32(),
							};
							(void)elevator_hsm_post_event(&ev);
						} else { /* Hàng 3 [E], [0], [B], [◀]: Xóa lỗi */
							bsp_pwm_buzzer_beep(2600U, 45U);
							can_bus_set_hil_drop(false);
							bsp_adc_clear_hil_overrides();
							elevator_event_t ev = { .type = EV_CLEAR_FAULT };
							(void)elevator_hsm_post_event(&ev);
						}
					}
					prev_stable_key = raw_key;
				} else {
					candidate_key = raw_key;
				}
			}
		}

		tick_div++;
	}
}

/* ============================================================================
 * T5: Luồng giám sát IWDG đa luồng & Xuất gói $TWIN JSON (Priority 8)
 * ============================================================================ */
static void telemetry_wdt_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint32_t loop_cnt = 0U;

	while (1) {
		(void)k_sem_take(&g_wdt_sem, K_FOREVER);

		(void)bsp_wdt_service(g_wdt_freeze_test);

		if (twin_telemetry_is_enabled() && (loop_cnt % 2U) == 0U) {
			twin_telemetry_emit_now();
		}

		loop_cnt++;
	}
}

/* Khởi tạo tĩnh toàn bộ 7 luồng RTOS với dung lượng Stack dư dả an toàn (< 45% tải đỉnh) */
K_THREAD_DEFINE(t1_safety,  2048, safety_thread_entry,        NULL, NULL, NULL, 2, 0, 500);
K_THREAD_DEFINE(t2_fsm,     2560, fsm_thread_entry,           NULL, NULL, NULL, 3, 0, 500);
K_THREAD_DEFINE(t3_can_rx,  1536, can_rx_thread_entry,        NULL, NULL, NULL, 4, 0, 500);
K_THREAD_DEFINE(t3_can_tx,  1536, can_tx_thread_entry,        NULL, NULL, NULL, 4, 0, 500);
K_THREAD_DEFINE(t3_can_wdt, 1024, can_wdt_thread_entry,       NULL, NULL, NULL, 4, 0, 500);
K_THREAD_DEFINE(t4_hmi,     4096, hmi_thread_entry,           NULL, NULL, NULL, 5, 0, 500);
K_THREAD_DEFINE(t5_wdt,     2048, telemetry_wdt_thread_entry, NULL, NULL, NULL, 8, 0, 500);

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

	/* Chạy giám sát an toàn mỗi 50 ms (< 50 ms đáp ứng rèm quang), HMI mỗi 10 ms và WDT/$TWIN mỗi 100 ms */
	k_timer_start(&g_safety_timer, K_MSEC(50), K_MSEC(50));
	k_timer_start(&g_hmi_timer, K_MSEC(10), K_MSEC(10));
	k_timer_start(&g_wdt_timer, K_MSEC(100), K_MSEC(100));

	bsp_pwm_buzzer_play_tone(BUZZER_TONE_BOOT);

	printk("\n====================================================\n");
	printk(" BKIT ARM4 Smart Elevator Ready (Type 'help' in Shell)\n");
	printk("====================================================\n");
	return 0;
}
