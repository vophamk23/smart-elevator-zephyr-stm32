/**
 * @file bsp_spi_hmi.c
 * @brief Trình điều khiển bus SPI1 dùng chung (4 LED 7 đoạn + Ma trận phím 4x4) & Flash W25Q64FV trên SPI2.
 *
 * Chức năng tổng quát của file:
 *  - Dùng Mutex `g_spi1_bus_mutex` để chia sẻ an toàn bus SPI1 giữa 2 IC dịch nối tầng `74HC595`
 *    (U14 chọn vị trí LED tích cực mức thấp + U15 xuất mã đoạn a..g,dp; chốt tại chân PG6 `LD_LATCH`)
 *    và 2 IC dịch `74HC165` (U16 + U17 đọc 16 nút bàn phím ma trận 4x4; nạp song song tại chân PD3 `BTN_LOAD`).
 *  - Quét ghép kênh 4 LED 7 đoạn độc lập ở tần số 500 Hz (2 ms/chữ số -> 125 Hz toàn khung) qua `k_timer` +
 *    `k_work` hệ thống, đảm bảo LED sáng đều tuyệt đối không nhấp nháy kể cả khi màn hình TFT-LCD đang vẽ nặng.
 *  - Giao tiếp với chip nhớ Flash ngoài `W25Q64FV` qua bus SPI2 (chân chọn chip PB12).
 */

#include "bsp_spi_hmi.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bsp_spi, LOG_LEVEL_INF);

static const struct device *const spi1_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct device *const spi2_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));

static const struct gpio_dt_spec spi1_ld_latch = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), ld_latch_gpios);
static const struct gpio_dt_spec spi1_btn_load = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), btn_load_gpios);
static const struct gpio_dt_spec spi2_w25q_cs  = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), w25q_cs_gpios);

/* Mutex hỗ trợ kế thừa độ ưu tiên, tuần tự hóa việc xuất LED 7 đoạn & đọc phím trên SPI1 */
K_MUTEX_DEFINE(g_spi1_bus_mutex);

/* Bảng mã LED 7 đoạn Anode chung (bit = 0 là SÁNG, bit = 1 là TẮT; bit 7 = dp luôn = 1 tắt dấu chấm) */
static const uint8_t seg7_digit_lut[10] = {
	0xC0U, 0xF9U, 0xA4U, 0xB0U, 0x99U, 0x92U, 0x82U, 0xF8U, 0x80U, 0x90U
};

#define SEG7_BLANK      0xFFU
#define SEG7_CHAR_E     0x86U
#define SEG7_CHAR_F     0x8EU
#define SEG7_CHAR_L     0xC7U
#define SEG7_CHAR_O     0xC0U
#define SEG7_CHAR_U     0xC1U
#define SEG7_CHAR_P     0x8CU
#define SEG7_CHAR_D     0xA1U
#define SEG7_CHAR_N     0xABU
#define SEG7_CHAR_R     0xAFU
#define SEG7_CHAR_DASH  0xBFU

/*
 * Mã 3 thanh ngang trên 1 ô LED 7 đoạn Anode chung (bit 0 = Sáng) dùng làm hiệu ứng
 * thang máy trượt từ dưới lên trên hoặc từ trên xuống dưới ở ô LED số 2 (digits[2]):
 *  - SEG7_BAR_BOT (0xF7): Thanh ngang đáy  (đoạn d: '_')
 *  - SEG7_BAR_MID (0xBF): Thanh ngang giữa (đoạn g: '-')
 *  - SEG7_BAR_TOP (0xFE): Thanh ngang đỉnh (đoạn a: '¯')
 */
#define SEG7_BAR_BOT    0xF7U
#define SEG7_BAR_MID    0xBFU
#define SEG7_BAR_TOP    0xFEU

static const uint8_t seg7_scroll_up[3]   = { SEG7_BAR_BOT, SEG7_BAR_MID, SEG7_BAR_TOP };
static const uint8_t seg7_scroll_down[3] = { SEG7_BAR_TOP, SEG7_BAR_MID, SEG7_BAR_BOT };

static const struct spi_config g_spi1_cfg = {
	.frequency = 2000000U,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	.slave = 0U,
};

/* Bộ đệm khung 4 chữ số của cụm LED 7 đoạn (mặc định 0xFF = tắt sạch) */
static uint8_t g_seg7_fb[4] = { SEG7_BLANK, SEG7_BLANK, SEG7_BLANK, SEG7_BLANK };
static uint8_t g_seg7_scan_idx;
static uint32_t g_seg7_test_hold_until_ms;

/**
 * @brief Đẩy 1 khung 16-bit ra chuỗi 2 IC 74HC595 trên bo BKIT ARM4:
 *        - Byte 0 (dịch sang IC U14 xa hơn): Mã chọn vị trí ô LED 0..3 tích cực mức THẤP `~(1 << idx)`;
 *          các bit 4..7 giữ mức 1 để tắt dấu hai chấm ':' và cụm LED6, LED7, LED8.
 *        - Byte 1 (nằm lại IC U15 gần hơn): Mã thanh đoạn `a..g, dp` Anode chung (0 = Sáng, 1 = Tắt).
 */
static int seg7_shift_one_digit_locked(uint8_t idx, uint8_t seg_code)
{
	uint8_t pos_select = (uint8_t)(~(1U << (idx & 0x03U)));
	uint8_t tx_buf[2] = { pos_select, seg_code };
	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	int err = spi_write(spi1_dev, &g_spi1_cfg, &tx_set);
	if (err == 0) {
		gpio_pin_set_dt(&spi1_ld_latch, 0);
		k_busy_wait(1U);
		gpio_pin_set_dt(&spi1_ld_latch, 1);
		k_busy_wait(1U);
		gpio_pin_set_dt(&spi1_ld_latch, 0);
	}
	return err;
}

/**
 * @brief Tác vụ nền quét 1 ô LED mỗi 2 ms (500 Hz -> 125 Hz cho cả 4 chữ số), không chặn nếu SPI1 đang đọc phím.
 */
static void seg7_scan_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!device_is_ready(spi1_dev)) {
		return;
	}

	if (k_mutex_lock(&g_spi1_bus_mutex, K_NO_WAIT) != 0) {
		return;
	}

	uint8_t idx = (uint8_t)(g_seg7_scan_idx & 0x03U);
	g_seg7_scan_idx = (uint8_t)((idx + 1U) & 0x03U);
	(void)seg7_shift_one_digit_locked(idx, g_seg7_fb[idx]);

	k_mutex_unlock(&g_spi1_bus_mutex);
}

K_WORK_DEFINE(g_seg7_scan_work, seg7_scan_work_handler);

static void seg7_scan_timer_isr(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	(void)k_work_submit(&g_seg7_scan_work);
}

K_TIMER_DEFINE(g_seg7_scan_timer, seg7_scan_timer_isr, NULL);

/**
 * @brief Khởi tạo các chân điều khiển chốt/nạp GPIO (PG6, PD3, PB12), tắt sạch 74HC595 và bật timer quét 2 ms.
 */
int bsp_spi_hmi_init(void)
{
	if (!gpio_is_ready_dt(&spi1_ld_latch) ||
	    !gpio_is_ready_dt(&spi1_btn_load) ||
	    !gpio_is_ready_dt(&spi2_w25q_cs)) {
		LOG_WRN("SPI control GPIO not ready");
		return -ENODEV;
	}

	/* PG6 (LD_LATCH) nghỉ mức thấp; PD3 (BTN_LOAD / SH_LD# của 74HC165) nghỉ mức CAO (1 = chế độ dịch) */
	int err = gpio_pin_configure_dt(&spi1_ld_latch, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return err;
	}
	err = gpio_pin_configure_dt(&spi1_btn_load, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		return err;
	}
	err = gpio_pin_configure_dt(&spi2_w25q_cs, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return err;
	}

	if (!device_is_ready(spi1_dev)) {
		LOG_WRN("SPI1 peripheral not ready");
		return -ENODEV;
	}
	if (!device_is_ready(spi2_dev)) {
		LOG_WRN("SPI2 peripheral not ready");
	}

	/* Đẩy ngay 0xFFFF vào 2 IC 74HC595 để tắt sạch cả 4 ô LED, dấu ':' và LED6..8 ngay khi cấp nguồn */
	k_mutex_lock(&g_spi1_bus_mutex, K_FOREVER);
	uint8_t all_off[2] = { 0xFFU, 0xFFU };
	const struct spi_buf tx = { .buf = all_off, .len = sizeof(all_off) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	if (spi_write(spi1_dev, &g_spi1_cfg, &tx_set) == 0) {
		gpio_pin_set_dt(&spi1_ld_latch, 1);
		k_busy_wait(2U);
		gpio_pin_set_dt(&spi1_ld_latch, 0);
	}
	k_mutex_unlock(&g_spi1_bus_mutex);

	/* Khởi chạy bộ quét ghép kênh 2 ms/chữ số (125 Hz toàn khung, mượt tuyệt đối) */
	k_timer_start(&g_seg7_scan_timer, K_MSEC(2), K_MSEC(2));
	return 0;
}

/**
 * @brief Cập nhật bộ đệm 4 chữ số LED 7 đoạn và xuất ngay 1 nhịp quét để phản hồi tức thời.
 */
int bsp_spi_7seg_write_raw(const uint8_t digits[4])
{
	if (!device_is_ready(spi1_dev) || digits == NULL) {
		return -ENODEV;
	}

	k_mutex_lock(&g_spi1_bus_mutex, K_FOREVER);
	memcpy(g_seg7_fb, digits, sizeof(g_seg7_fb));
	uint8_t idx = (uint8_t)(g_seg7_scan_idx & 0x03U);
	g_seg7_scan_idx = (uint8_t)((idx + 1U) & 0x03U);
	int err = seg7_shift_one_digit_locked(idx, g_seg7_fb[idx]);
	k_mutex_unlock(&g_spi1_bus_mutex);

	return err;
}

/**
 * @brief Giữ cố định mẫu kiểm thử trên 4 LED 7 đoạn trong `hold_ms` để lệnh `bkit test spi1` quan sát rõ bằng mắt.
 */
void bsp_spi_7seg_hold_test_pattern(const uint8_t digits[4], uint32_t hold_ms)
{
	if (digits == NULL) {
		return;
	}
	g_seg7_test_hold_until_ms = k_uptime_get_32() + hold_ms;
	(void)bsp_spi_7seg_write_raw(digits);
}

/**
 * @brief Mã hóa toàn diện trạng thái thang máy ra 4 ô LED 7 đoạn:
 *        - Báo cháy (`fire`)         -> "FE_4" (đang hạ) hoặc nhấp nháy "FE 1" (tại F1)
 *        - Lỗi khẩn (`fault`)        -> "ErrN"
 *        - Quá tải (`overload`)      -> Nhấp nháy "FULL" mỗi 250 ms
 *        - Đang đi LÊN               -> "UP" + vạch cuộn lên + số tầng
 *        - Đang đi XUỐNG             -> "dn" + vạch cuộn xuống + số tầng
 *        - Cửa đang mở/đóng          -> "OP N"
 *        - Đang đỗ đóng cửa          -> "F- N"
 */
int bsp_spi_7seg_render_status(uint8_t floor, elevator_dir_t dir, bool door_open,
			       bool overload, bool fire, bool fault)
{
	uint32_t now_ms = k_uptime_get_32();

	/* Tạm nhường quyền hiển thị nếu người dùng vừa gõ `bkit test spi1` */
	if ((int32_t)(g_seg7_test_hold_until_ms - now_ms) > 0) {
		return 0;
	}

	uint8_t digits[4] = { SEG7_BLANK, SEG7_BLANK, SEG7_BLANK, SEG7_BLANK };
	uint32_t anim_step = (now_ms / 180U) % 3U;
	bool blink_on = ((now_ms / 250U) & 1U) == 0U;
	uint8_t floor_seg = (floor <= 9U) ? seg7_digit_lut[floor] : SEG7_CHAR_DASH;

	if (fire) {
		if (floor > ELEV_MIN_FLOOR) {
			digits[0] = SEG7_CHAR_F;
			digits[1] = SEG7_CHAR_E;
			digits[2] = seg7_scroll_down[anim_step];
			digits[3] = floor_seg;
		} else if (blink_on) {
			digits[0] = SEG7_CHAR_F;
			digits[1] = SEG7_CHAR_E;
			digits[2] = SEG7_BLANK;
			digits[3] = seg7_digit_lut[1];
		}
		return bsp_spi_7seg_write_raw(digits);
	}

	if (fault) {
		digits[0] = SEG7_CHAR_E;
		digits[1] = SEG7_CHAR_R;
		digits[2] = SEG7_CHAR_R;
		digits[3] = floor_seg;
		return bsp_spi_7seg_write_raw(digits);
	}

	if (overload) {
		if (blink_on) {
			digits[0] = SEG7_CHAR_F;
			digits[1] = SEG7_CHAR_U;
			digits[2] = SEG7_CHAR_L;
			digits[3] = SEG7_CHAR_L;
		}
		return bsp_spi_7seg_write_raw(digits);
	}

	if (dir == ELEV_DIR_UP) {
		digits[0] = SEG7_CHAR_U;
		digits[1] = SEG7_CHAR_P;
		digits[2] = seg7_scroll_up[anim_step];
	} else if (dir == ELEV_DIR_DOWN) {
		digits[0] = SEG7_CHAR_D;
		digits[1] = SEG7_CHAR_N;
		digits[2] = seg7_scroll_down[anim_step];
	} else if (door_open) {
		digits[0] = SEG7_CHAR_O;
		digits[1] = SEG7_CHAR_P;
		digits[2] = SEG7_BLANK;
	} else {
		digits[0] = SEG7_CHAR_F;
		digits[1] = SEG7_CHAR_DASH;
		digits[2] = SEG7_BLANK;
	}

	digits[3] = floor_seg;
	return bsp_spi_7seg_write_raw(digits);
}

/**
 * @brief Tạo xung chốt PD3 (BTN_LOAD: 1 -> 0 -> 1) nạp song song 2 IC 74HC165 và đọc 16 bit phím qua SPI1.
 *        Lưu ý: Trong lúc phát xung SCK để đọc MISO từ 74HC165, chân PG6 (LD_LATCH của 74HC595) giữ nguyên mức 0
 *        nên ngõ ra chốt hiển thị của LED 7 đoạn hoàn toàn không bị ảnh hưởng.
 */
int bsp_spi_keypad_scan(int *key_idx, uint8_t raw_bytes[2])
{
	if (!device_is_ready(spi1_dev)) {
		return -ENODEV;
	}

	uint8_t tx_dummy[2] = { 0xFFU, 0xFFU };
	uint8_t rx_buf[2]   = { 0xFFU, 0xFFU };

	const struct spi_buf tx = { .buf = tx_dummy, .len = sizeof(tx_dummy) };
	const struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	k_mutex_lock(&g_spi1_bus_mutex, K_FOREVER);
	gpio_pin_set_dt(&spi1_btn_load, 1);
	k_busy_wait(2U);
	gpio_pin_set_dt(&spi1_btn_load, 0);
	k_busy_wait(5U);
	gpio_pin_set_dt(&spi1_btn_load, 1);
	k_busy_wait(5U);
	int err = spi_transceive(spi1_dev, &g_spi1_cfg, &tx_set, &rx_set);
	k_mutex_unlock(&g_spi1_bus_mutex);

	if (raw_bytes != NULL) {
		raw_bytes[0] = rx_buf[0];
		raw_bytes[1] = rx_buf[1];
	}

	if (key_idx != NULL) {
		*key_idx = -1;
		uint16_t word = (uint16_t)(((uint16_t)rx_buf[0] << 8) | rx_buf[1]);

		/* Phím tích cực mức thấp (0): bỏ qua 0xFFFF (nhả hết) và 0x0000 (hở mạch) */
		if (err == 0 && word != 0xFFFFU && word != 0x0000U) {
			for (int bit = 0; bit < 16; bit++) {
				if ((word & BIT(bit)) == 0U) {
					*key_idx = bit;
					break;
				}
			}
		}
	}

	return err;
}

/**
 * @brief Giao tiếp SPI2 gửi lệnh 0x9F đọc 3 byte JEDEC ID của chip Flash W25Q64FV.
 */
int bsp_spi2_probe_jedec(uint8_t id_out[3])
{
	if (!device_is_ready(spi2_dev) || !gpio_is_ready_dt(&spi2_w25q_cs)) {
		return -ENODEV;
	}

	const struct spi_config cfg = {
		.frequency = 2000000U,
		.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
		.slave = 0U,
	};

	uint8_t tx_buf[4] = { 0x9FU, 0x00U, 0x00U, 0x00U };
	uint8_t rx_buf[4] = { 0 };
	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	gpio_pin_set_dt(&spi2_w25q_cs, 1);
	k_busy_wait(2U);
	int err = spi_transceive(spi2_dev, &cfg, &tx_set, &rx_set);
	gpio_pin_set_dt(&spi2_w25q_cs, 0);

	if (err == 0 && id_out != NULL) {
		id_out[0] = rx_buf[1];
		id_out[1] = rx_buf[2];
		id_out[2] = rx_buf[3];
	}
	return err;
}
