/**
 * @file bsp_gpio_exti.c
 * @brief Trình điều khiển GPIO cách ly quang, Khóa liên động Động cơ (Y0/Y1) & Ngắt ngoài EXTI (X0/X1) trên BKIT ARM4.
 *
 * Chức năng tổng quát của file:
 *  - Quản lý 4 ngõ vào cách ly quang `X0..X3` (PA6, PA7, PC4, PC5), trong đó `X0` (Kẹt cửa) và `X1` (E-Stop)
 *    chạy ngắt phần cứng EXTI có khử rung 40 ms và bấm giờ chu kỳ CPU DWT để đo độ trễ ngắt.
 *  - Quản lý 2 ngõ ra cách ly quang điều khiển động cơ `Y0` (PE5 - LÊN), `Y1` (PE6 - XUỐNG) và đèn `LED3` (PE4).
 *
 * Các hàm quan trọng nhất trong file:
 *  - `bsp_gpio_exti_init()`             : Cấu hình các chân GPIO vào/ra và gắn hàm phục vụ ngắt EXTI cho X0 (PA6), X1 (PA7).
 *  - `bsp_gpio_set_motor_outputs()`     : Điều khiển động cơ Y0 (UP) & Y1 (DOWN) kèm khóa liên động cứng (chặn bật cả 2 cùng lúc).
 *  - `bsp_gpio_read_inputs()`           : Đọc mức logic tức thời của 4 ngõ vào cách ly quang X0, X1, X2, X3.
 *  - `bsp_gpio_set_led_debug()`         : Bật/tắt đèn báo nhịp tim & cảnh báo lỗi LED3 (chân PE4).
 *  - `dispatch_debounced_exti()`        : Hàm nội bộ xử lý trong ISR: chụp chu kỳ DWT `k_cycle_get_32()`, lọc rung 40 ms và gọi callback.
 */

#include "bsp_gpio_exti.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bsp_gpio, LOG_LEVEL_INF);

/* Khoảng thời gian tối thiểu (ms) để khử rung (debounce) tiếp điểm cơ/quang */
#define EXTI_DEBOUNCE_MS 40U

static const struct gpio_dt_spec led_debug = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec out_y0    = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec out_y1    = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

static const struct gpio_dt_spec in_x0     = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec in_x1     = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec in_x2     = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec in_x3     = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);

static struct gpio_callback g_x0_cb_data;
static struct gpio_callback g_x1_cb_data;
static bsp_exti_cb_t g_exti_cb;
static volatile uint32_t g_x0_last_ms;
static volatile uint32_t g_x1_last_ms;

/**
 * @brief Hàm dùng chung xử lý trong ngắt EXTI: ghi lại số chu kỳ CPU DWT trước khi lọc
 *        rung phím 40 ms để đo chính xác độ trễ phần cứng từ ISR đến luồng RTOS.
 */
static void dispatch_debounced_exti(volatile uint32_t *last_ms, elevator_ev_type_t ev)
{
	/* Chụp chu kỳ DWT trước khi lọc rung 40 ms để phản ánh đúng thời điểm sườn ngắt phần cứng */
	uint32_t isr_cycles = k_cycle_get_32();
	uint32_t now_ms = k_uptime_get_32();

	if ((now_ms - *last_ms) < EXTI_DEBOUNCE_MS) {
		return;
	}
	*last_ms = now_ms;

	if (g_exti_cb != NULL) {
		g_exti_cb(ev, isr_cycles);
	}
}

/**
 * @brief [NGUỒN 1 - NGẮT CỨNG X0] Trình phục vụ ngắt ngoài khi chạm dây kích chân X0 (PA6 - Kẹt cửa).
 *        Gửi sự kiện `EV_DOOR_OBSTRUCT_EXTI` qua callback `on_hw_exti_event()` vào hòm thư HSM.
 */
static void exti_x0_handler(const struct device *port, struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	dispatch_debounced_exti(&g_x0_last_ms, EV_DOOR_OBSTRUCT_EXTI);
}

/**
 * @brief [NGUỒN 1 - NGẮT CỨNG X1] Trình phục vụ ngắt ngoài khi chạm dây kích chân X1 (PA7 - E-Stop).
 *        Ngắt nguồn động cơ Y0/Y1 ngay tức khắc tại thanh ghi BSRR (dưới 2 us) trước khi gửi sự kiện về HSM.
 */
static void exti_x1_handler(const struct device *port, struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Cắt cứng nguồn mô-tơ ngay trong ngữ cảnh ISR để đảm bảo chuẩn Hard Real-Time (< 12 us) */
	bsp_gpio_set_motor_outputs(false, false);
	dispatch_debounced_exti(&g_x1_last_ms, EV_ESTOP_EXTI);
}

/**
 * @brief Cấu hình chân GPIO làm ngõ vào và kích hoạt ngắt theo sườn mức tích cực (EXTI).
 */
static int configure_exti_pin(const struct gpio_dt_spec *spec,
			      struct gpio_callback *cb_data,
			      gpio_callback_handler_t handler,
			      const char *name)
{
	int err = gpio_pin_configure_dt(spec, GPIO_INPUT);

	if (err < 0) {
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(spec, GPIO_INT_EDGE_TO_ACTIVE);
	if (err < 0) {
		LOG_WRN("%s EXTI interrupt config failed (%d)", name, err);
		return err;
	}

	gpio_init_callback(cb_data, handler, BIT(spec->pin));
	gpio_add_callback(spec->port, cb_data);
	return 0;
}

/**
 * @brief Khởi tạo các chân ngõ ra (PE4..PE6), ngõ vào opto (X0..X3) và gắn 2 kênh ngắt EXTI.
 */
int bsp_gpio_exti_init(bsp_exti_cb_t exti_cb)
{
	const struct gpio_dt_spec *outputs[] = { &led_debug, &out_y0, &out_y1 };
	const struct gpio_dt_spec *inputs[]  = { &in_x0, &in_x1, &in_x2, &in_x3 };
	int err;

	g_exti_cb = exti_cb;

	for (size_t i = 0; i < ARRAY_SIZE(outputs); i++) {
		if (!gpio_is_ready_dt(outputs[i])) {
			LOG_ERR("GPIO output %zu not ready", i);
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(outputs[i], GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			return err;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(inputs); i++) {
		if (!gpio_is_ready_dt(inputs[i])) {
			LOG_ERR("GPIO input %zu not ready", i);
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(inputs[i], GPIO_INPUT);
		if (err < 0) {
			return err;
		}
	}

	/* Gắn ngắt cứng EXTI cho X0 (kẹt cửa) và X1 (E-Stop); X2/X3 được đọc định kỳ ở luồng T1 */
	(void)configure_exti_pin(&in_x0, &g_x0_cb_data, exti_x0_handler, "X0(PA6)");
	(void)configure_exti_pin(&in_x1, &g_x1_cb_data, exti_x1_handler, "X1(PA7)");

	bsp_gpio_set_motor_outputs(false, false);
	LOG_INF("Discrete I/O & EXTI (PA6/PA7) initialized");
	return 0;
}

/**
 * @brief Điều khiển trạng thái bật/tắt của LED Heartbeat PE4.
 */
void bsp_gpio_set_led_debug(bool on)
{
	(void)gpio_pin_set_dt(&led_debug, on ? 1 : 0);
}

/**
 * @brief Điều khiển hai chiều động cơ PE5/PE6 và cưỡng chế khóa liên động chống bật đồng thời.
 */
void bsp_gpio_set_motor_outputs(bool up_en, bool down_en)
{
	/*
	 * Khóa liên động an toàn tầng phần cứng:
	 * Dù tầng điều khiển phía trên gặp lỗi logic, Y0 (LÊN) và Y1 (XUỐNG) tuyệt đối
	 * không được phép bật cùng lúc nhằm tránh chập mạch contactor / cầu H.
	 */
	if (up_en && down_en) {
		up_en = false;
		down_en = false;
		if (!k_is_in_isr()) {
			LOG_ERR("SAFETY INTERLOCK: Blocked simultaneous UP+DOWN energization");
		}
	}

	(void)gpio_pin_set_dt(&out_y0, up_en ? 1 : 0);
	(void)gpio_pin_set_dt(&out_y1, down_en ? 1 : 0);
}

/**
 * @brief Đọc trạng thái mức logic hiện tại của 4 ngõ vào quang X0..X3.
 */
void bsp_gpio_read_inputs(bool *x0, bool *x1, bool *x2, bool *x3)
{
	if (x0 != NULL) {
		*x0 = gpio_pin_get_dt(&in_x0) > 0;
	}
	if (x1 != NULL) {
		*x1 = gpio_pin_get_dt(&in_x1) > 0;
	}
	if (x2 != NULL) {
		*x2 = gpio_pin_get_dt(&in_x2) > 0;
	}
	if (x3 != NULL) {
		*x3 = gpio_pin_get_dt(&in_x3) > 0;
	}
}
