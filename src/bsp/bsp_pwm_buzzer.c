/**
 * @file bsp_pwm_buzzer.c
 * @brief Trình điều khiển còi Buzzer PWM không chặn trên chân PF8 (TIM13_CH1) cho bo mạch BKIT ARM4.
 *
 * Chức năng tổng quát của file:
 *  - Phát 5 mẫu âm thanh cảnh báo chuẩn (Khởi động, Đến tầng, Kẹt cửa, Quá tải, Báo cháy/E-Stop)
 *    bằng xung PWM phần cứng sóng vuông 50% duty cycle mà không làm nghẽn (block) luồng gọi.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `bsp_pwm_buzzer_init()`      : Kiểm tra thiết bị PWM13 (PF8) và đưa còi về trạng thái tắt an toàn.
 *  - `bsp_pwm_buzzer_play_tone()` : Phát một trong 5 âm cảnh báo định sẵn theo bảng tần số và thời lượng.
 *  - `bsp_pwm_buzzer_beep()`      : Phát xung PWM ở tần số (Hz) và thời lượng (ms) tùy chọn, đặt mốc giờ tự ngắt.
 *  - `bsp_pwm_buzzer_tick()`      : Được luồng `t4_hmi` gọi mỗi 10 ms để tự động ngắt xung PWM khi hết thời lượng kêu.
 */

#include "bsp_pwm_buzzer.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(bsp_pwm, LOG_LEVEL_INF);

static const struct pwm_dt_spec buzzer_pwm = PWM_DT_SPEC_GET(DT_ALIAS(pwm_buzzer));

/* Mutex bảo vệ trạng thái còi và thanh ghi TIM13 tránh xung đột khi nhiều luồng cùng phát âm */
K_MUTEX_DEFINE(g_buzzer_mutex);

static int64_t g_buzzer_stop_ms;
static bool g_buzzer_active;

/**
 * @brief Khởi tạo ngoại vi PWM PF8 (TIM13_CH1), ngắt tiếng mặc định và kêu 1 nhịp boot.
 */
int bsp_pwm_buzzer_init(void)
{
	if (!pwm_is_ready_dt(&buzzer_pwm)) {
		LOG_WRN("PWM buzzer PF8 (TIM13_CH1) not ready");
		return -ENODEV;
	}

	bsp_pwm_buzzer_mute();
	bsp_pwm_buzzer_play_tone(BUZZER_TONE_BOOT);
	LOG_INF("PWM buzzer PF8 initialized");
	return 0;
}

/**
 * @brief Xuất sóng vuông PWM 50% duty-cycle ở tần số @p freq_hz và đặt mốc thời gian tự tắt.
 */
void bsp_pwm_buzzer_beep(uint32_t freq_hz, uint32_t duration_ms)
{
	if (!pwm_is_ready_dt(&buzzer_pwm) || freq_hz == 0U || duration_ms == 0U) {
		return;
	}

	/*
	 * Cơ chế phát còi KHÔNG CHẶN (non-blocking):
	 *  - Thay vì dùng k_msleep(duration_ms) làm đứng luồng gọi còi, ta chỉ đặt xung PWM
	 *    (chu kỳ T = 1 giây / tần số, duty 50% giúp còi áp điện kêu rõ nhất) rồi lưu mốc
	 *    thời gian `g_buzzer_stop_ms`.
	 *  - Luồng HMI (`t4_hmi`) sẽ tự tắt còi khi gọi `bsp_pwm_buzzer_tick()` mỗi 10 ms.
	 */
	uint32_t period_ns = NSEC_PER_SEC / freq_hz;

	k_mutex_lock(&g_buzzer_mutex, K_FOREVER);
	(void)pwm_set_dt(&buzzer_pwm, period_ns, period_ns / 2U);
	g_buzzer_stop_ms = k_uptime_get() + (int64_t)duration_ms;
	g_buzzer_active = true;
	k_mutex_unlock(&g_buzzer_mutex);
}

/* Bảng tra cấu hình âm mẫu lưu cố định trong Flash (table-driven, không cấp phát động) */
struct buzzer_tone_cfg {
	uint32_t freq_hz;
	uint32_t duration_ms;
};

static const struct buzzer_tone_cfg k_tone_table[] = {
	[BUZZER_TONE_BOOT]          = { 1800U,  80U },
	[BUZZER_TONE_FLOOR_ARRIVE]  = { 2200U, 100U },
	[BUZZER_TONE_DOOR_OBSTRUCT] = { 2800U, 250U },
	[BUZZER_TONE_OVERLOAD]      = { 1500U, 160U },
	[BUZZER_TONE_ALARM_SIREN]   = { 3000U, 400U },
};

/**
 * @brief Tra bảng @c k_tone_table[] trong Flash để phát âm báo tương ứng với sự kiện thang máy.
 */
void bsp_pwm_buzzer_play_tone(bsp_buzzer_tone_t tone)
{
	if ((size_t)tone >= ARRAY_SIZE(k_tone_table)) {
		return;
	}
	bsp_pwm_buzzer_beep(k_tone_table[tone].freq_hz,
			    k_tone_table[tone].duration_ms);
}

/**
 * @brief Đưa độ rộng xung PWM về 0 ngay lập tức để tắt tiếng còi.
 */
void bsp_pwm_buzzer_mute(void)
{
	if (!pwm_is_ready_dt(&buzzer_pwm)) {
		return;
	}

	k_mutex_lock(&g_buzzer_mutex, K_FOREVER);
	(void)pwm_set_dt(&buzzer_pwm, PWM_HZ(2000U), 0U);
	g_buzzer_active = false;
	k_mutex_unlock(&g_buzzer_mutex);
}

/**
 * @brief Hàm định kỳ (~10 ms ở luồng HMI) kiểm tra hết thời gian phát để tự động ngắt còi.
 */
void bsp_pwm_buzzer_tick(void)
{
	if (!g_buzzer_active || !pwm_is_ready_dt(&buzzer_pwm)) {
		return;
	}

	k_mutex_lock(&g_buzzer_mutex, K_FOREVER);
	if (g_buzzer_active && k_uptime_get() >= g_buzzer_stop_ms) {
		(void)pwm_set_dt(&buzzer_pwm, PWM_HZ(2000U), 0U);
		g_buzzer_active = false;
	}
	k_mutex_unlock(&g_buzzer_mutex);
}
