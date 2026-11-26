/**
 * @file cli_elev.c
 * @brief Tập lệnh Zephyr Shell (`elev`) điều khiển vận hành thang máy, tiêm lỗi HIL & kiểm tra độ phủ V-Model.
 *
 * Chức năng tổng quát của file:
 *  - Cung cấp giao diện dòng lệnh `elev ...` qua cổng nối tiếp USART1 cho phép cả người vận hành lẫn
 *    phần mềm Python Digital Twin gửi lệnh gọi tầng, điều khiển cửa, bơm lỗi thử nghiệm và đọc Hộp đen EEPROM.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `cmd_elev_status()`   : Lệnh `elev status` — In toàn bộ trạng thái HSM, hàng đợi, X0..X3, ADC1 và chẩn đoán CAN1.
 *  - `cmd_elev_call()`     : Lệnh `elev call <1..7> [cabin|up|down]` — Gửi yêu cầu gọi tầng vào bộ lập lịch LOOK hoặc qua CAN.
 *  - `cmd_elev_door()`     : Lệnh `elev door <open|close|obstruct>` — Mở/đóng cửa hoặc giả lập ngắt kẹt cửa X0.
 *  - `cmd_elev_fault()`    : Lệnh `elev fault <fire|overload|can_drop|estop|clear>` — Tiêm lỗi phần mềm HIL hoặc xóa lỗi.
 *  - `cmd_elev_mode()`     : Lệnh `elev mode <loopback|normal>` — Chuyển mạng CAN1 giữa chế độ 1 bo và 2 bo.
 *  - `cmd_elev_role()`     : Lệnh `elev role <cabin|hall>` — Đổi vai trò bo mạch giữa Tủ Cabin và Bảng gọi tầng Sảnh.
 *  - `cmd_elev_log()`      : Lệnh `elev log [clear]` — Đọc 32 bản ghi sự cố từ Hộp đen AT24C512 kèm kiểm tra CRC32.
 *  - `cmd_elev_coverage()` : Lệnh `elev coverage` — In bảng ma trận độ phủ chuyển trạng thái `[9][9]` và thống kê CAN.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "bsp_adc_sensors.h"
#include "bsp_gpio_exti.h"
#include "bsp_i2c_storage.h"
#include "can_bus.h"
#include "elevator_hsm.h"
#include "elevator_types.h"
#include "twin_telemetry.h"

/* Nhãn rút gọn 3 ký tự khớp đúng thứ tự enum elevator_state_t để in bảng 80 cột */
static const char *const k_short_state[HSM_STATE_COUNT] = {
	"INI", "IDL", "OPN", "OPD", "CLS", "MUP", "MDN", "FIR", "FSF"
};

/**
 * @brief Lệnh `elev status`: in tổng quan trạng thái HSM, hàng đợi MsgQ, I/O, ADC1 và chẩn đoán CAN1.
 */
static int cmd_elev_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	elevator_telemetry_t tel;
	bkit_adc_snapshot_t adc = { 0 };
	uint8_t can_st = 0U, tec = 0U, rec = 0U;
	uint32_t rx_ok = 0U, tx_ok = 0U, rtt_us = 0U;
	uint32_t tx_cb_ok = 0U, tx_cb_err = 0U, rtr_rx = 0U;
	uint32_t ev_sent = 0U, ev_recv = 0U, ev_drop = 0U, ev_inq = 0U;
	bool x0 = false, x1 = false, x2 = false, x3 = false;

	elevator_hsm_get_telemetry(&tel);
	elevator_hsm_get_queue_stats(&ev_sent, &ev_recv, &ev_drop, &ev_inq);
	(void)bsp_adc_read_all(&adc);
	bsp_gpio_read_inputs(&x0, &x1, &x2, &x3);
	can_bus_get_diag(&can_st, &tec, &rec, &rx_ok, &tx_ok, &rtt_us);
	can_bus_get_ext_stats(&tx_cb_ok, &tx_cb_err, &rtr_rx);

	shell_print(sh, "=== BKIT ARM4 Smart Elevator Status ===");
	shell_print(sh, "Role / Twin : %s | $TWIN=%s",
		    (can_bus_get_role() == NODE_ROLE_HALL) ? "HALL (Kit2)" : "CABIN (Kit1)",
		    twin_telemetry_is_enabled() ? "ON" : "OFF");
	shell_print(sh, "State       : %s (transitions=%u, visited_mask=0x%03X)",
		    elevator_state_str((elevator_state_t)tel.state),
		    tel.transition_count, elevator_hsm_get_visited_state_mask());
	shell_print(sh, "Floor/Target: F%u -> F%u (dir=%d, trips=%u)",
		    tel.current_floor, tel.target_floor, tel.direction, tel.trip_count);
	shell_print(sh, "Door        : pos=%u (%u%%), Motor Y0(UP)=%d Y1(DOWN)=%d",
		    tel.door_pos, tel.door_pct, tel.motor_up, tel.motor_down);
	shell_print(sh, "Queues      : Cabin=0x%X HallUp=0x%X HallDn=0x%X",
		    tel.cabin_mask, tel.hall_up_mask, tel.hall_down_mask);
	shell_print(sh, "HSM MsgQ    : sent=%u recv=%u drop=%u pending=%u",
		    ev_sent, ev_recv, ev_drop, ev_inq);
	shell_print(sh, "Safety Flags: Fire=%d EStop=%d Overload=%d CanFault=%d WdtReset=%d",
		    tel.fire_active, tel.estop_active, tel.overload_active,
		    tel.can_fault_active, tel.wdt_reset_flag);
	shell_print(sh, "EXTI Latency: last=%u us, max=%u us, aborts=%u",
		    tel.last_exti_latency_us, tel.max_exti_latency_us, tel.safety_abort_count);
	shell_print(sh, "Opto Inputs : X0(PA6)=%d X1(PA7)=%d X2(PC4)=%d X3(PC5)=%d",
		    x0, x1, x2, x3);
	shell_print(sh, "ADC Sensors : Temp=%d.%d C | Load=%u kg | Light=%u mV",
		    adc.temp_deci_c / 10, abs(adc.temp_deci_c % 10), adc.load_kg, adc.light_mv);
	shell_print(sh, "CAN1 500kbps: mode=%s st=%u TEC=%u REC=%u TX=%u RX=%u cbOK=%u cbErr=%u RTR=%u RTT=%u us",
		    (can_bus_get_mode() == CAN_LINK_MODE_LOOPBACK) ? "LOOPBACK" : "NORMAL",
		    can_st, tec, rec, tx_ok, rx_ok, tx_cb_ok, tx_cb_err, rtr_rx, rtt_us);
	return 0;
}

/**
 * @brief Lệnh `elev call`: gửi yêu cầu gọi tầng (cabin hoặc hành lang Lên/Xuống) vào HSM hoặc truyền qua CAN khi ở vai trò HALL.
 */
static int cmd_elev_call(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: elev call <1..7> [cabin|up|down]");
		return -EINVAL;
	}

	int floor = atoi(argv[1]);
	if (floor < ELEV_MIN_FLOOR || floor > ELEV_MAX_FLOOR) {
		shell_error(sh, "Invalid floor %d (must be 1..%d)", floor, ELEV_MAX_FLOOR);
		return -EINVAL;
	}

	elevator_event_t ev = { .type = EV_CALL_CABIN, .floor = (uint8_t)floor };
	if (argc >= 3) {
		if (strcmp(argv[2], "up") == 0) {
			ev.type = EV_CALL_HALL_UP;
		} else if (strcmp(argv[2], "down") == 0) {
			ev.type = EV_CALL_HALL_DOWN;
		} else if (strcmp(argv[2], "cabin") != 0) {
			shell_error(sh, "Unknown call type '%s' (use cabin|up|down)", argv[2]);
			return -EINVAL;
		}
	}

	/* Khi bo mạch đang đóng vai trò bảng gọi tầng HALL, gửi khung gọi tầng qua CAN ID 0x100 */
	if (can_bus_get_role() == NODE_ROLE_HALL && ev.type != EV_CALL_CABIN) {
		int err = can_bus_send_hall_call((uint8_t)floor, (ev.type == EV_CALL_HALL_UP));
		shell_print(sh, "HALL->CAN(0x100) TX floor %d %s (err=%d)",
			    floor, (ev.type == EV_CALL_HALL_UP) ? "UP" : "DOWN", err);
		return err;
	}

	int err = elevator_hsm_post_event(&ev);
	shell_print(sh, "Posted floor %d request (err=%d)", floor, err);
	return err;
}

/**
 * @brief [NGUỒN 2 - NÚT GIẢ LẬP CỬA / KẸT CỬA TỪ PYTHON TWIN & SHELL]
 *        Khi bấm nút "Obstruct (X0)" trên Python (hoặc gõ `elev door obstruct`), hàm này tạo
 *        sự kiện `EV_DOOR_OBSTRUCT_EXTI` bỏ vào cùng hòm thư `elevator_hsm_post_event(&ev)`.
 */
static int cmd_elev_door(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: elev door <open|close|obstruct>");
		return -EINVAL;
	}

	elevator_event_t ev = { .isr_cycles = k_cycle_get_32() };
	if (strcmp(argv[1], "open") == 0) {
		ev.type = EV_CMD_OPEN_DOOR;
	} else if (strcmp(argv[1], "close") == 0) {
		ev.type = EV_CMD_CLOSE_DOOR;
	} else if (strcmp(argv[1], "obstruct") == 0) {
		ev.type = EV_DOOR_OBSTRUCT_EXTI; /* Giả lập ngắt kẹt cửa X0 */
	} else {
		shell_error(sh, "Unknown door subcommand: %s", argv[1]);
		return -EINVAL;
	}

	return elevator_hsm_post_event(&ev);
}

/**
 * @brief [NGUỒN 2 - NÚT GIẢ LẬP SỰ CỐ TỪ PYTHON TWIN & SHELL]
 *        Khi bấm "Inject Fire", "Inject Overload", "E-STOP (X1)" trên Python (hoặc gõ `elev fault ...`),
 *        hàm này tạo gói sự kiện y hệt cảm biến thật rồi bỏ vào chung hòm thư `elevator_hsm_post_event(&ev)`.
 */
static int cmd_elev_fault(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: elev fault <fire|overload|can_drop|estop|clear>");
		return -EINVAL;
	}

	elevator_event_t ev = { .isr_cycles = k_cycle_get_32() };

	if (strcmp(argv[1], "fire") == 0) {
		bsp_adc_set_hil_temp_override(620);
		ev.type = EV_OVERHEAT_TEMP;      /* Giả lập LM35 nóng 62.0°C (620 deci-C) */
		ev.param = 620U;
		(void)can_bus_send_emergency(1U, 0U);
	} else if (strcmp(argv[1], "overload") == 0) {
		bsp_adc_set_hil_load_override(920);
		ev.type = EV_OVERLOAD_SET;       /* Giả lập biến trở VR1 nặng 920 kg */
		ev.param = 920U;
	} else if (strcmp(argv[1], "can_drop") == 0) {
		can_bus_set_hil_drop(true);      /* Giả lập đứt dây CAN -> chờ 2000 ms kích Fail-Safe */
		shell_print(sh, "HIL: CAN link drop active (Fail-Safe triggers in 2000 ms)");
		return 0;
	} else if (strcmp(argv[1], "estop") == 0) {
		bsp_gpio_set_motor_outputs(false, false);
		ev.type = EV_ESTOP_EXTI;         /* Giả lập ngắt nút dừng khẩn cấp X1 */
	} else if (strcmp(argv[1], "clear") == 0) {
		can_bus_set_hil_drop(false);
		bsp_adc_clear_hil_overrides();
		ev.type = EV_CLEAR_FAULT;        /* Xóa toàn bộ cờ lỗi đang chốt trong g_hsm */
	} else {
		shell_error(sh, "Unknown fault type: %s", argv[1]);
		return -EINVAL;
	}

	return elevator_hsm_post_event(&ev);
}

/**
 * @brief Lệnh `elev mode`: chuyển đổi bus CAN1 giữa tự kiểm 1 kit (LOOPBACK) và mạng 2 kit (NORMAL).
 */
static int cmd_elev_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: elev mode <loopback|normal>");
		return -EINVAL;
	}

	can_link_mode_t mode;
	if (strcmp(argv[1], "normal") == 0) {
		mode = CAN_LINK_MODE_NORMAL;
	} else if (strcmp(argv[1], "loopback") == 0) {
		mode = CAN_LINK_MODE_LOOPBACK;
	} else {
		shell_error(sh, "Invalid mode '%s' (use loopback|normal)", argv[1]);
		return -EINVAL;
	}

	int err = can_bus_set_mode(mode);
	shell_print(sh, "CAN mode set to %s (err=%d)",
		    (mode == CAN_LINK_MODE_NORMAL) ? "NORMAL" : "LOOPBACK", err);
	return err;
}

/**
 * @brief Lệnh `elev role`: đổi vai trò bo mạch giữa bộ điều khiển CABIN (Kit 1) và bảng HALL (Kit 2).
 */
static int cmd_elev_role(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: elev role <cabin|hall>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "cabin") == 0) {
		can_bus_set_role(NODE_ROLE_CABIN);
	} else if (strcmp(argv[1], "hall") == 0) {
		can_bus_set_role(NODE_ROLE_HALL);
	} else {
		shell_error(sh, "Invalid role '%s' (use cabin|hall)", argv[1]);
		return -EINVAL;
	}

	shell_print(sh, "Node role set to %s",
		    (can_bus_get_role() == NODE_ROLE_HALL) ? "HALL (Kit 2)" : "CABIN (Kit 1)");
	return 0;
}

/**
 * @brief Lệnh `elev sensor <load|temp|auto> [val]`: đồng bộ thanh kéo cảm biến từ PC Digital Twin xuống STM32.
 */
static int cmd_elev_sensor(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: elev sensor <load <0..1000>|temp <deci_c>|auto>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "auto") == 0) {
		bsp_adc_clear_hil_overrides();
		shell_print(sh, "HIL sensor overrides cleared (using real VR1 & LM35)");
		return 0;
	}

	if (argc < 3) {
		shell_error(sh, "Usage: elev sensor <load|temp> <value|auto>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "load") == 0) {
		if (strcmp(argv[2], "auto") == 0) {
			bsp_adc_set_hil_load_override(-1);
		} else {
			int kg = atoi(argv[2]);
			bsp_adc_set_hil_load_override(kg);
		}
		return 0;
	}

	if (strcmp(argv[1], "temp") == 0) {
		if (strcmp(argv[2], "auto") == 0) {
			bsp_adc_set_hil_temp_override(-1);
		} else {
			int dc = atoi(argv[2]);
			bsp_adc_set_hil_temp_override(dc);
		}
		return 0;
	}

	shell_error(sh, "Unknown sensor '%s' (use load|temp|auto)", argv[1]);
	return -EINVAL;
}

/**
 * @brief Lệnh `elev twin`: bật/tắt xuất dòng bản tin telemetry $TWIN JSON định kỳ mỗi 200 ms.
 */
static int cmd_elev_twin(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: elev twin <on|off>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "on") == 0) {
		twin_telemetry_set_enabled(true);
	} else if (strcmp(argv[1], "off") == 0) {
		twin_telemetry_set_enabled(false);
	} else {
		shell_error(sh, "Usage: elev twin <on|off>");
		return -EINVAL;
	}

	shell_print(sh, "Digital Twin $TWIN stream: %s",
		    twin_telemetry_is_enabled() ? "ON" : "OFF");
	return 0;
}

/**
 * @brief Lệnh `elev log`: đọc toàn bộ bản ghi hộp đen sự cố hoặc xóa trắng bộ đệm vòng trên AT24C512.
 */
static int cmd_elev_log(const struct shell *sh, size_t argc, char **argv)
{
	if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
		int err = bsp_blackbox_clear();
		shell_print(sh, "AT24C512 blackbox cleared (err=%d)", err);
		return err;
	}

	elevator_blackbox_rec_t recs[16];
	size_t count = 0U;
	int err = bsp_blackbox_read(recs, ARRAY_SIZE(recs), &count);
	if (err < 0) {
		shell_error(sh, "AT24C512 blackbox read failed (%d)", err);
		return err;
	}

	shell_print(sh, "=== AT24C512 Black-Box Records (%zu entries, newest first) ===", count);
	for (size_t i = 0; i < count; i++) {
		shell_print(sh, "  #%u [%04u-%02u-%02u %02u:%02u:%02u | up=%u ms] %-13s State=%s F%u T=%d.%dC",
			    recs[i].seq, recs[i].year, recs[i].month, recs[i].day,
			    recs[i].hour, recs[i].min, recs[i].sec, recs[i].uptime_ms,
			    elevator_fault_str((blackbox_fault_code_t)recs[i].fault_code),
			    elevator_state_str((elevator_state_t)recs[i].state),
			    recs[i].floor, recs[i].temp_deci_c / 10, abs(recs[i].temp_deci_c % 10));
	}
	return 0;
}

/**
 * @brief Lệnh `elev coverage`: xuất danh sách trạng thái đã đi qua và ma trận chuyển trạng thái V-Model.
 */
static int cmd_elev_coverage(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint16_t mat[HSM_STATE_COUNT][HSM_STATE_COUNT];
	uint32_t mask = elevator_hsm_get_visited_state_mask();
	elevator_hsm_get_transition_counts(mat);

	uint32_t visited = 0U;
	uint32_t distinct_edges = 0U;
	uint32_t total_edges = 0U;

	shell_print(sh, "=== V-Model HSM State & Transition Coverage ===");
	shell_print(sh, "Visited states mask=0x%03X:", mask);
	for (size_t i = 0; i < HSM_STATE_COUNT; i++) {
		bool seen = (mask & BIT(i)) != 0U;
		if (seen) {
			visited++;
		}
		shell_print(sh, "  [%zu] %-12s : %s", i, elevator_state_str((elevator_state_t)i),
			    seen ? "VISITED" : "not yet");
	}

	shell_fprintf(sh, SHELL_NORMAL, "\nFrom\\To ");
	for (size_t j = 0; j < HSM_STATE_COUNT; j++) {
		shell_fprintf(sh, SHELL_NORMAL, " %4s", k_short_state[j]);
	}
	shell_print(sh, "");

	for (size_t i = 0; i < HSM_STATE_COUNT; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%-7s ", k_short_state[i]);
		for (size_t j = 0; j < HSM_STATE_COUNT; j++) {
			uint16_t c = mat[i][j];
			if (c > 0U) {
				distinct_edges++;
				total_edges += c;
			}
			shell_fprintf(sh, SHELL_NORMAL, " %4u", c);
		}
		shell_print(sh, "");
	}

	shell_print(sh, "Summary: %u/%d states visited, %u distinct transitions, %u total steps",
		    visited, HSM_STATE_COUNT, distinct_edges, total_edges);
	return 0;
}

/*
 * Đăng ký cây lệnh Zephyr Shell: `elev <status|call|door|fault|mode|role|twin|log|coverage>`.
 * Tham số (required, optional) trong SHELL_CMD_ARG tính luôn cả tên lệnh con ở argv[0].
 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_elev,
	SHELL_CMD(status, NULL, "Print full elevator HSM, IO, ADC & CAN status", cmd_elev_status),
	SHELL_CMD_ARG(call, NULL, "elev call <1..7> [cabin|up|down]", cmd_elev_call, 2, 1),
	SHELL_CMD_ARG(door, NULL, "elev door <open|close|obstruct>", cmd_elev_door, 2, 0),
	SHELL_CMD_ARG(fault, NULL, "elev fault <fire|overload|can_drop|estop|clear>", cmd_elev_fault, 2, 0),
	SHELL_CMD_ARG(sensor, NULL, "elev sensor <load <kg>|temp <deci_c>|auto>", cmd_elev_sensor, 2, 1),
	SHELL_CMD_ARG(mode, NULL, "elev mode <loopback|normal>", cmd_elev_mode, 2, 0),
	SHELL_CMD_ARG(role, NULL, "elev role <cabin|hall>", cmd_elev_role, 2, 0),
	SHELL_CMD_ARG(twin, NULL, "elev twin <on|off> (PC Digital Twin telemetry)", cmd_elev_twin, 2, 0),
	SHELL_CMD_ARG(log, NULL, "elev log [read|clear] (AT24C512 black-box)", cmd_elev_log, 1, 1),
	SHELL_CMD(coverage, NULL, "Print V-Model state & transition coverage matrix", cmd_elev_coverage),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(elev, &sub_elev, "Smart Elevator control, HIL & coverage commands", NULL);
