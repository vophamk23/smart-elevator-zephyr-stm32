/**
 * @file bsp_spi_hmi.c
 * @brief Trình điều khiển bus SPI1 dùng chung (4 LED 7 đoạn + Ma trận phím 4x4) & Flash W25Q64FV trên SPI2.
 *
 * Chức năng tổng quát của file:
 *  - Dùng Mutex `g_spi1_bus_mutex` để chia sẻ an toàn bus SPI1 giữa 2 IC dịch 74HC595 (quét 4 LED 7 đoạn,
 *    chân chốt PG6) và 2 IC dịch 74HC165 (đọc 16 nút bàn phím ma trận 4x4, chân nạp PD3).
 *  - Giao tiếp với chip nhớ Flash ngoài `W25Q64FV` qua bus SPI2 (chân chọn chip PB12).
 *
 * Các hàm quan trọng nhất trong file:
 *  - `bsp_spi_hmi_init()`           : Cấu hình các chân GPIO chốt/nạp (PG6, PD3, PB12) và kiểm tra bus SPI1/SPI2.
 *  - `bsp_spi_7seg_write_raw()`     : Quét ghép thời gian từng ô trong 4 LED 7 đoạn ở tần số 100 Hz qua SPI1.
 *  - `bsp_spi_7seg_render_status()` : Mã hóa chiều đi ('UP'/'dn' + vạch cuộn), số tầng (1..7) hoặc mã lỗi ('FE'/'Err') ra 4 LED.
 *  - `bsp_spi_keypad_scan()`        : Chốt chân PD3 và đọc 16 bit trạng thái bàn phím 4x4 qua SPI1 để lấy mã phím (0..15).
 *  - `bsp_spi2_probe_jedec()`       : Đọc mã định danh nhà sản xuất JEDEC ID (3 byte: EF 40 17) của chip Flash W25Q64FV.
 */

#include "bsp_spi_hmi.h"

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

/* Bảng mã LED 7 đoạn Anode chung (0..9 kèm các ký tự trạng thái thang máy) */
static const uint8_t seg7_digit_lut[10] = {
	0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90
};

#define SEG7_BLANK      0xFFU
#define SEG7_CHAR_E     0x86U
#define SEG7_CHAR_F     0x8EU
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

/* Bảng 3 pha cuộn vạch ngang: đi LÊN (đáy -> giữa -> đỉnh) và đi XUỐNG (đỉnh -> giữa -> đáy) */
static const uint8_t seg7_scroll_up[3]   = { SEG7_BAR_BOT, SEG7_BAR_MID, SEG7_BAR_TOP };
static const uint8_t seg7_scroll_down[3] = { SEG7_BAR_TOP, SEG7_BAR_MID, SEG7_BAR_BOT };

static const struct spi_config g_spi1_cfg = {
	.frequency = 1000000U,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	.slave = 0U,
};

/**
 * @brief Khởi tạo các chân điều khiển chốt/nạp GPIO (PG6, PD3, PB12) và kiểm tra SPI1/SPI2.
 */
int bsp_spi_hmi_init(void)
{
	const struct gpio_dt_spec *pins[] = { &spi1_ld_latch, &spi1_btn_load, &spi2_w25q_cs };

	for (size_t i = 0; i < ARRAY_SIZE(pins); i++) {
		if (!gpio_is_ready_dt(pins[i])) {
			LOG_WRN("SPI control GPIO %zu not ready", i);
			return -ENODEV;
		}
		int err = gpio_pin_configure_dt(pins[i], GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			return err;
		}
	}

	if (!device_is_ready(spi1_dev)) {
		LOG_WRN("SPI1 peripheral not ready");
	}
	if (!device_is_ready(spi2_dev)) {
		LOG_WRN("SPI2 peripheral not ready");
	}
	return 0;
}

/**
 * @brief Quét ghép thời gian 1 chữ số LED 7 đoạn mỗi lần gọi qua SPI1 và tạo xung chốt PG6.
 */
int bsp_spi_7seg_write_raw(const uint8_t digits[4])
{
	if (!device_is_ready(spi1_dev) || digits == NULL) {
		return -ENODEV;
	}

	/* Quét luân phiên từng digit (chu kỳ HMI 10 ms -> tần số quét toàn khung 100 Hz) */
	static uint8_t scan_idx;
	/* Byte 0: mã đoạn a..g của ký tự; Byte 1: mặt nạ BIT(0..3) chọn 1 trong 4 chữ số qua 2 IC 74HC595 */
	uint8_t tx_buf[2] = {
		digits[scan_idx & 0x03U],
		(uint8_t)BIT(scan_idx & 0x03U),
	};
	scan_idx = (uint8_t)((scan_idx + 1U) & 0x03U);

	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	k_mutex_lock(&g_spi1_bus_mutex, K_FOREVER);
	int err = spi_write(spi1_dev, &g_spi1_cfg, &tx_set);
	if (err == 0) {
		/* Tạo sườn lên tại PG6 (LD_LATCH / RCLK) để chốt dữ liệu ra ngõ ra 74HC595 */
		gpio_pin_set_dt(&spi1_ld_latch, 0);
		k_busy_wait(2U);
		gpio_pin_set_dt(&spi1_ld_latch, 1);
		k_busy_wait(2U);
		gpio_pin_set_dt(&spi1_ld_latch, 0);
	}
	k_mutex_unlock(&g_spi1_bus_mutex);

	return err;
}

/**
 * @brief Mã hóa trạng thái thang máy ra đủ 4 ô LED 7 đoạn (tránh nhầm hình chữ U khi vẽ mũi tên):
 *        - Đứng yên ở Tầng 1          -> "[F][-][ ][1]" ("F- 1" = Floor 1)
 *        - Đang đi LÊN qua Tầng 3     -> "[U][P][_/-/\xAF][3]" ("UP" + vạch ngang leo từ đáy lên đỉnh + tầng 3)
 *        - Đang đi XUỐNG qua Tầng 5   -> "[d][n][\xAF/-/_][5]" ("dn" + vạch ngang tụt từ đỉnh xuống đáy + tầng 5)
 *        - Báo cháy Sơ tán ở Tầng 4   -> "[F][E][\xAF/-/_][4]" ("FE" + vạch hạ xuống + tầng 4)
 *        - Lỗi E-Stop / Đứt CAN Tầng 2 -> "[E][r][r][2]" ("Err2" = Error tại Tầng 2)
 */
int bsp_spi_7seg_render_status(uint8_t floor, elevator_dir_t dir, bool fire, bool fault)
{
	/* Mặc định tắt cả 4 ô LED (Anode chung: 0xFF = tắt toàn bộ các đoạn) */
	uint8_t digits[4] = { SEG7_BLANK, SEG7_BLANK, SEG7_BLANK, SEG7_BLANK };
	/* Chia thời gian thực theo mỗi 180 ms để tạo nhịp cuộn 3 nấc mượt mắt ở ô LED số 2 */
	uint32_t anim_step = (k_uptime_get_32() / 180U) % 3U;

	if (fire) {
		/* Báo cháy: Hiện chữ "FE" + vạch cuộn xuống + số tầng */
		digits[0] = SEG7_CHAR_F;
		digits[1] = SEG7_CHAR_E;
		digits[2] = seg7_scroll_down[anim_step];
	} else if (fault) {
		/* Lỗi khẩn cấp (E-Stop / Đứt CAN): Hiện rõ chữ "Err" ở 3 ô đầu + số tầng ở ô cuối */
		digits[0] = SEG7_CHAR_E;
		digits[1] = SEG7_CHAR_R;
		digits[2] = SEG7_CHAR_R;
	} else if (dir == ELEV_DIR_UP) {
		/* Đang đi LÊN: Hiện chữ "UP" ở 2 ô đầu + vạch ngang leo từ dưới lên ở ô thứ 3 */
		digits[0] = SEG7_CHAR_U;
		digits[1] = SEG7_CHAR_P;
		digits[2] = seg7_scroll_up[anim_step];
	} else if (dir == ELEV_DIR_DOWN) {
		/* Đang đi XUỐNG: Hiện chữ "dn" (Down) ở 2 ô đầu + vạch ngang rơi từ trên xuống ở ô thứ 3 */
		digits[0] = SEG7_CHAR_D;
		digits[1] = SEG7_CHAR_N;
		digits[2] = seg7_scroll_down[anim_step];
	} else {
		/* Đang đứng yên: Hiện "F-" (Floor) ở 2 ô đầu */
		digits[0] = SEG7_CHAR_F;
		digits[1] = SEG7_CHAR_DASH;
	}

	/* Ô số 3 ngoài cùng bên phải (digits[3]): Luôn hiển thị số tầng hiện tại (1..7) */
	digits[3] = (floor <= 9U) ? seg7_digit_lut[floor] : SEG7_CHAR_DASH;
	return bsp_spi_7seg_write_raw(digits);
}

/**
 * @brief Tạo xung chốt PD3 (BTN_LOAD) để nạp song song và đọc về 16 bit trạng thái phím qua SPI1.
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
	/* Nạp song song trạng thái phím bằng PD3 (BTN_LOAD) trước khi dịch 16 bit qua MISO */
	gpio_pin_set_dt(&spi1_btn_load, 0);
	k_busy_wait(2U);
	gpio_pin_set_dt(&spi1_btn_load, 1);
	k_busy_wait(2U);
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
			/* Quét 16 bit tìm phím đầu tiên bị kéo xuống mức 0 (cấu hình pull-up, tích cực thấp) */
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

	/* Byte 0 gửi opcode JEDEC 0x9F; 3 byte 0x00 tiếp theo tạo clock đẩy 3 byte ID về rx_buf[1..3] */
	uint8_t tx_buf[4] = { 0x9FU, 0x00U, 0x00U, 0x00U };
	uint8_t rx_buf[4] = { 0 };
	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	/* Chân w25q-cs khai báo GPIO_ACTIVE_LOW trong DTS: mức logic 1 kéo chân PB12 xuống LOW */
	gpio_pin_set_dt(&spi2_w25q_cs, 1);
	k_busy_wait(2U);
	int err = spi_transceive(spi2_dev, &cfg, &tx_set, &rx_set);
	k_busy_wait(2U);
	gpio_pin_set_dt(&spi2_w25q_cs, 0);

	if (err == 0 && id_out != NULL) {
		id_out[0] = rx_buf[1];
		id_out[1] = rx_buf[2];
		id_out[2] = rx_buf[3];
	}
	return err;
}
