/**
 * @file twin_telemetry.c
 * @brief Bộ đóng gói dữ liệu trạng thái thời gian thực dạng JSON (`$TWIN`) gửi lên PC Digital Twin qua USART1.
 *
 * Chức năng tổng quát của file:
 *  - Thu thập ảnh chụp trạng thái từ Máy trạng thái HSM, 5 kênh cảm biến ADC1 và thống kê mạng CAN1,
 *    sau đó xuất thành dòng chuỗi JSON `$TWIN,{...}` mỗi 200 ms để giao diện Python Digital Twin hiển thị.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `twin_telemetry_set_enabled()` : Bật hoặc tắt luồng xuất dữ liệu JSON `$TWIN` ra cổng UART.
 *  - `twin_telemetry_is_enabled()`  : Kiểm tra trạng thái bật/tắt hiện tại của luồng `$TWIN`.
 *  - `twin_telemetry_emit_now()`    : Đóng gói toàn bộ thông số (tầng, cửa, Y0/Y1, nhiệt độ, tải, CAN, độ trễ EXTI) và in ra UART.
 */

#include "twin_telemetry.h"
#include "bsp_adc_sensors.h"
#include "can_bus.h"
#include "elevator_hsm.h"
#include "elevator_types.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(twin_tx, LOG_LEVEL_INF);

static volatile bool g_twin_enabled;

/**
 * @brief Bật hoặc tắt chế độ xuất chuỗi telemetry JSON $TWIN ra cổng nối tiếp.
 */
void twin_telemetry_set_enabled(bool en)
{
	g_twin_enabled = en;
	LOG_INF("Digital Twin JSON telemetry stream %s", en ? "ENABLED" : "DISABLED");
}

/**
 * @brief Kiểm tra xem luồng xuất dữ liệu $TWIN có đang được bật hay không.
 */
bool twin_telemetry_is_enabled(void)
{
	return g_twin_enabled;
}

/**
 * @brief Thu thập ảnh chụp từ HSM, ADC1, CAN1 và xuất một bản tin JSON $TWIN qua printk().
 */
void twin_telemetry_emit_now(void)
{
	elevator_telemetry_t tel;
	bkit_adc_snapshot_t adc = { 0 };
	uint8_t can_st = 0U, tec = 0U, rec = 0U;
	uint32_t rx_ok = 0U, tx_ok = 0U, rtt_us = 0U;

	elevator_hsm_get_telemetry(&tel);
	(void)bsp_adc_read_all(&adc);
	can_bus_get_diag(&can_st, &tec, &rec, &rx_ok, &tx_ok, &rtt_us);

	/*
	 * Quy ước khung truyền lên PC Digital Twin:
	 *  - Bắt đầu bằng tiền tố '$TWIN,' để phần mềm Python lọc riêng dòng JSON, bỏ qua log chữ.
	 *  - Bổ sung cm/um/dm (mặt nạ bit 7 tầng) và trips để giao diện bật đèn nút đang chờ.
	 */
	printk("$TWIN,{\"ms\":%u,\"st\":\"%s\",\"fl\":%u,\"tgt\":%u,"
	       "\"dir\":%d,\"door\":%u,\"y0\":%d,\"y1\":%d,"
	       "\"fire\":%d,\"estop\":%d,\"ovl\":%d,\"canf\":%d,"
	       "\"temp\":%d,\"load\":%u,\"tec\":%u,\"rec\":%u,\"lat_us\":%u,"
	       "\"cm\":%u,\"um\":%u,\"dm\":%u,\"trips\":%u}\n",
	       tel.uptime_ms, elevator_state_str((elevator_state_t)tel.state),
	       tel.current_floor, tel.target_floor, tel.direction, tel.door_pct,
	       tel.motor_up, tel.motor_down, tel.fire_active, tel.estop_active,
	       tel.overload_active, tel.can_fault_active,
	       adc.temp_deci_c, adc.load_kg, tec, rec, tel.last_exti_latency_us,
	       tel.cabin_mask, tel.hall_up_mask, tel.hall_down_mask, tel.trip_count);
}
