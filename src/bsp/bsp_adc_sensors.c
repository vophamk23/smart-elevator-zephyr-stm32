/**
 * @file bsp_adc_sensors.c
 * @brief Trình điều khiển ADC1 12-bit đa kênh đọc cảm biến nhiệt độ LM35, biến trở tải VR1, quang trở LDR và nguồn.
 *
 * Chức năng tổng quát của file:
 *  - Tự động lấy cấu hình 5 kênh ADC1 từ Devicetree (`app.overlay`: PC2, PC1, PC0, PB0, PB1).
 *  - Đọc giá trị mẫu thô 12-bit (0..4095), đổi sang millivolt (0..3300 mV) và quy đổi sang đơn vị kỹ thuật.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `bsp_adc_init()`     : Khởi tạo và nạp cấu hình kênh cho cả 5 ngõ vào tương tự của bộ ADC1.
 *  - `bsp_adc_read_all()` : Quét tuần tự 5 kênh ADC1, đổi sang mV rồi tính ra Nhiệt độ LM35 (0.1°C),
 *                           Tải trọng cabin từ biến trở VR1 (0..1000 kg) và Ánh sáng quang trở LDR (mV).
 */

#include "bsp_adc_sensors.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(bsp_adc, LOG_LEVEL_INF);

/*
 * Macro tự động quét thuộc tính `io-channels` trong app.overlay để sinh mảng
 * `adc_channels[0..4]` theo đúng thứ tự 5 chân cảm biến Analog trên bo BKIT - Arm4:
 *  - adc_channels[0]: Chân PC2 (ADC1_IN12) -> Cảm biến nhiệt độ LM35 (U22)
 *  - adc_channels[1]: Chân PC1 (ADC1_IN11) -> Biến trở xoay tải trọng VR1
 *  - adc_channels[2]: Chân PC0 (ADC1_IN10) -> Quang trở cảm biến ánh sáng (LDR)
 *  - adc_channels[3]: Chân PB0 (ADC1_IN8)  -> Mạch đo điện áp nguồn nuôi (Bus Voltage)
 *  - adc_channels[4]: Chân PB1 (ADC1_IN9)  -> Mạch đo dòng điện tiêu thụ (Motor Current)
 */
#define DT_SPEC_AND_COMMA_FOR_INPUTS(node_id, prop, idx) \
	COND_CODE_1(DT_PHA_HAS_CELL_AT_IDX(node_id, prop, idx, input), \
		    (ADC_DT_SPEC_GET_BY_IDX(node_id, idx),), ())

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA_FOR_INPUTS)
};

/**
 * @brief Khởi tạo và cấu hình độ phân giải/thời gian lấy mẫu cho cả 5 kênh ADC1.
 */
int bsp_adc_init(void)
{
	int ready_cnt = 0;

	for (size_t i = 0; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			LOG_WRN("ADC channel idx=%zu not ready", i);
			continue;
		}
		int err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_WRN("ADC channel idx=%zu setup failed (%d)", i, err);
		} else {
			ready_cnt++;
		}
	}

	LOG_INF("ADC1 configured (%d/%zu channels ready)", ready_cnt, ARRAY_SIZE(adc_channels));
	return (ready_cnt > 0) ? 0 : -ENODEV;
}

/**
 * @brief Đọc tuần tự 5 kênh ADC1, đổi sang mV và quy đổi ra nhiệt độ (0.1 độ C) cùng tải trọng (kg).
 */
int bsp_adc_read_all(bkit_adc_snapshot_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	int32_t mv_vals[5] = { 0 };
	bool any_ok = false;

	for (size_t i = 0; i < ARRAY_SIZE(adc_channels) && i < ARRAY_SIZE(mv_vals); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			continue;
		}

		uint16_t raw_sample = 0U;
		struct adc_sequence seq = {
			.buffer = &raw_sample,
			.buffer_size = sizeof(raw_sample),
		};

		/* Nạp lại cấu hình adc_sequence cho từng kênh trước khi đọc đơn kênh 12-bit */
		(void)adc_sequence_init_dt(&adc_channels[i], &seq);
		if (adc_read_dt(&adc_channels[i], &seq) == 0) {
			int32_t mv = (int32_t)raw_sample;
			/* Quy đổi giá trị raw 0..4095 sang millivolt dựa theo Vref trong Devicetree */
			if (adc_raw_to_millivolts_dt(&adc_channels[i], &mv) == 0) {
				mv_vals[i] = mv;
				any_ok = true;
			}
		}
	}

	/*
	 * Bước quy đổi từ điện áp gốc `mv_vals[0..4]` (đơn vị: mV, dải 0..3300 mV)
	 * sang đơn vị vật lý của từng cảm biến tương ứng:
	 */

	/*
	 * [0] Cảm biến nhiệt độ LM35 (Chân PC2 - ADC1_IN12, linh kiện U22):
	 *     Theo đặc tuyến LM35: cứ tăng 1 độ C thì điện áp tăng 10 mV (1 mV = 0.1 độ C).
	 *     Do đó giá trị `mv_vals[0]` (mV) chính bằng nhiệt độ tính theo đơn vị 0.1 độ C (deci-C).
	 *     Ví dụ: 285 mV -> 285 (tức 28.5 độ C); >= 550 (55.0 độ C) -> Kích hoạt Báo cháy.
	 */
	out->temp_deci_c = mv_vals[0];

	/*
	 * [1] Núm vặn Biến trở tải trọng VR1 (Chân PC1 - ADC1_IN11):
	 *     Khi vặn biến trở từ min -> max, điện áp `mv_vals[1]` biến thiên từ 0 -> 3300 mV.
	 *     Áp dụng quy tắc tam suất tuyến tính để quy đổi dải 0..3300 mV sang tải trọng 0..1000 kg:
	 *         load_kg = (mv_vals[1] * 1000 kg) / 3300 mV
	 *     Khi load_kg >= 800 kg -> Kích hoạt cảnh báo Quá tải (OVERLOAD).
	 */
	out->load_kg     = (uint16_t)CLAMP((mv_vals[1] * (int32_t)ELEV_LOAD_MAX_KG) / 3300,
					   0, (int32_t)ELEV_LOAD_MAX_KG);

	/*
	 * [2] Quang trở cảm biến ánh sáng LDR (Chân PC0 - ADC1_IN10):
	 *     Giữ nguyên đơn vị điện áp millivolt (0..3300 mV) phản ánh cường độ sáng buồng thang.
	 */
	out->light_mv    = (uint16_t)CLAMP(mv_vals[2], 0, 3300);

	/*
	 * [3] Điện áp nguồn Bus (Chân PB0 - ADC1_IN8):
	 *     Giữ nguyên đơn vị millivolt (0..3300 mV) để giám sát mức áp nguồn trên bo mạch.
	 */
	out->bus_mv      = (uint16_t)CLAMP(mv_vals[3], 0, 3300);

	/*
	 * [4] Dòng điện tiêu thụ (Chân PB1 - ADC1_IN9):
	 *     Giữ nguyên đơn vị millivolt (0..3300 mV) tỉ lệ với dòng tải tiêu thụ trên bo mạch.
	 */
	out->current_mv  = (uint16_t)CLAMP(mv_vals[4], 0, 3300);
	out->raw_ok      = any_ok;

	return any_ok ? 0 : -EIO;
}
