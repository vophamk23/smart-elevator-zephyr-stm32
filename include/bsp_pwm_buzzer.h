/**
 * @file bsp_pwm_buzzer.h
 * @brief Trình điều khiển còi PWM không chặn trên chân PF8 (TIM13_CH1) cho kit BKIT ARM4.
 */

#ifndef BSP_PWM_BUZZER_H_
#define BSP_PWM_BUZZER_H_

#include <stdint.h>

/** @brief Tập các âm báo chuẩn cho từng tình huống vận hành của thang máy. */
typedef enum {
	BUZZER_TONE_BOOT = 0,
	BUZZER_TONE_FLOOR_ARRIVE,
	BUZZER_TONE_DOOR_OBSTRUCT,
	BUZZER_TONE_OVERLOAD,
	BUZZER_TONE_ALARM_SIREN
} bsp_buzzer_tone_t;

/**
 * @brief Khởi tạo ngoại vi PWM PF8 (TIM13_CH1), tắt tiếng ban đầu và phát âm khởi động.
 * @return 0 nếu thành công, -ENODEV nếu kênh PWM chưa sẵn sàng.
 */
int bsp_pwm_buzzer_init(void);

/**
 * @brief Kích phát một xung âm tần số xác định không gây nghẽn luồng (bảo vệ bằng k_mutex).
 * @param freq_hz     Tần số âm thanh tính bằng Hz (thường từ 800..4000 Hz).
 * @param duration_ms Thời gian duy trì phát âm (ms) trước khi tự tắt.
 */
void bsp_pwm_buzzer_beep(uint32_t freq_hz, uint32_t duration_ms);

/** @brief Tra bảng âm mẫu trong Flash (table-driven) và phát âm báo theo sự kiện. */
void bsp_pwm_buzzer_play_tone(bsp_buzzer_tone_t tone);

/** @brief Ngắt tín hiệu xuất xung ra còi PWM ngay lập tức. */
void bsp_pwm_buzzer_mute(void);

/**
 * @brief Kiểm tra hết hạn thời gian phát âm để tự tắt còi; gọi định kỳ (~10 ms) từ luồng HMI.
 */
void bsp_pwm_buzzer_tick(void);

#endif /* BSP_PWM_BUZZER_H_ */
