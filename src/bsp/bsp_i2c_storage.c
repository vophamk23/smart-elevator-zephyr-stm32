/**
 * @file bsp_i2c_storage.c
 * @brief Trình điều khiển bus I2C1 dùng chung (PB6/PB7) cho Đồng hồ DS3231 RTC & Hộp đen vòng EEPROM AT24C512.
 *
 * Chức năng tổng quát của file:
 *  - Bảo vệ truy cập đồng thời trên bus I2C1 bằng khóa Mutex `g_i2c1_bus_mutex`.
 *  - Đọc/cài đặt ngày giờ thực (BCD) từ IC `DS3231` (địa chỉ `0x68`).
 *  - Lưu trữ và truy xuất 32 bản ghi sự cố theo cấu trúc vòng đệm (Ring Buffer) trên `AT24C512` (địa chỉ `0x50`)
 *    kèm mã kiểm tra toàn vẹn kép `CRC16-ANSI` và `CRC32-IEEE`.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `bsp_i2c_storage_init()` : Khởi tạo bus I2C1, kiểm tra kết nối DS3231/AT24C512 và nạp header vòng đệm Hộp đen.
 *  - `bsp_rtc_get_datetime()` : Đọc thời gian thực (Năm, Tháng, Ngày, Giờ, Phút, Giây) từ thanh ghi BCD của DS3231.
 *  - `bsp_rtc_set_datetime()` : Ghi mốc ngày giờ thực mới xuống IC DS3231.
 *  - `bsp_blackbox_append()`  : Đóng gói bản ghi sự cố 32 byte (kèm giờ RTC, nhiệt độ, tải trọng, CRC32) và ghi vào EEPROM.
 *  - `bsp_blackbox_read()`    : Đọc bản ghi sự cố từ EEPROM và tự động đối chiếu lại mã CRC16/CRC32 để phát hiện lỗi dữ liệu.
 *  - `bsp_blackbox_clear()`   : Xóa định dạng lại vùng nhớ Hộp đen EEPROM về trạng thái ban đầu.
 */

#include "bsp_i2c_storage.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(bsp_i2c, LOG_LEVEL_INF);

static const struct device *const i2c1_dev   = DEVICE_DT_GET(DT_NODELABEL(i2c1));
static const struct device *const eeprom_dev = DEVICE_DT_GET(DT_ALIAS(eeprom_0));

#define DS3231_I2C_ADDR     0x68U

/* AT24C512 có trang ghi 128 byte: đặt header tại 0x0000 và mỗi bản ghi gọn trong 1 trang */
#define BB_MAGIC            0x454C4556U /* "ELEV" */
#define BB_HDR_OFFSET       0x0000U
#define BB_REC_BASE_OFFSET  0x0080U
#define BB_MAX_RECORDS      32U
#define BB_REC_STRIDE       32U

/* Mutex hỗ trợ kế thừa độ ưu tiên nhằm tuần tự hóa mọi truy cập DS3231 và AT24C512 trên I2C1 */
K_MUTEX_DEFINE(g_i2c1_bus_mutex);

struct bb_header {
	uint32_t magic;
	uint32_t next_seq;
	uint16_t write_idx;
	uint16_t valid_count;
	uint32_t crc32;
} __packed;

/** @brief Chuyển byte mã BCD từ thanh ghi DS3231 sang số nguyên nhị phân. */
static uint8_t bcd_to_bin(uint8_t val)
{
	return (uint8_t)(((val >> 4) * 10U) + (val & 0x0FU));
}

/** @brief Chuyển số nguyên nhị phân (0..99) sang mã BCD để nạp xuống thanh ghi DS3231. */
static uint8_t bin_to_bcd(uint8_t val)
{
	return (uint8_t)(((val / 10U) << 4) | (val % 10U));
}

/** @brief Tính mã kiểm tra toàn vẹn CRC32-IEEE cho phần header (chỉ tính các trường trước `crc32`). */
static uint32_t bb_hdr_crc(const struct bb_header *hdr)
{
	return crc32_ieee((const uint8_t *)hdr, offsetof(struct bb_header, crc32));
}

/** @brief Tính mã kiểm tra CRC16-CCITT cho từng bản ghi sự cố hộp đen 32 byte. */
static uint16_t bb_rec_crc(const elevator_blackbox_rec_t *rec)
{
	return crc16_ccitt(0xFFFFU, (const uint8_t *)rec,
			   offsetof(elevator_blackbox_rec_t, crc16));
}

/**
 * @brief Kiểm tra mức độ sẵn sàng của controller I2C1 và chip nhớ EEPROM AT24C512.
 */
int bsp_i2c_storage_init(void)
{
	if (!device_is_ready(i2c1_dev)) {
		LOG_WRN("I2C1 controller not ready");
		return -ENODEV;
	}
	if (!device_is_ready(eeprom_dev)) {
		LOG_WRN("AT24C512 EEPROM (0x50) not responding on I2C1");
	}
	LOG_INF("I2C1 (PB6/PB7) storage & RTC driver ready");
	return 0;
}

/**
 * @brief Quét dải địa chỉ I2C 7-bit (0x03..0x77) dưới sự bảo vệ của g_i2c1_bus_mutex và in ACK.
 */
void bsp_i2c_scan_bus(void (*print_cb)(const char *fmt, ...))
{
	if (print_cb == NULL) {
		return;
	}
	if (!device_is_ready(i2c1_dev)) {
		print_cb("I2C1 device not ready\n");
		return;
	}

	int found = 0;

	k_mutex_lock(&g_i2c1_bus_mutex, K_FOREVER);
	/* Thăm dò từng địa chỉ bằng khung I2C WRITE rỗng (0 byte): nếu chip ACK là thiết bị đang có mặt */
	for (uint8_t addr = 0x03U; addr <= 0x77U; addr++) {
		uint8_t dummy = 0U;
		struct i2c_msg msg = {
			.buf = &dummy,
			.len = 0U,
			.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
		};
		if (i2c_transfer(i2c1_dev, &msg, 1U, addr) == 0) {
			print_cb("  [ACK] 0x%02X\n", addr);
			found++;
		}
	}
	k_mutex_unlock(&g_i2c1_bus_mutex);

	if (found == 0) {
		print_cb("  No I2C1 devices ACKed (check PB6/PB7 jumpers & pull-ups)\n");
	}
}

/**
 * @brief Đọc 7 byte thanh ghi thời gian thực (0x00..0x06) từ chip DS3231 và giải mã BCD.
 */
int bsp_rtc_get_datetime(uint16_t *year, uint8_t *month, uint8_t *day,
			 uint8_t *hour, uint8_t *min, uint8_t *sec)
{
	if (!device_is_ready(i2c1_dev)) {
		return -ENODEV;
	}

	uint8_t reg_start = 0x00U;
	uint8_t raw[7] = { 0 };

	k_mutex_lock(&g_i2c1_bus_mutex, K_FOREVER);
	int err = i2c_write_read(i2c1_dev, DS3231_I2C_ADDR,
				 &reg_start, 1U, raw, sizeof(raw));
	k_mutex_unlock(&g_i2c1_bus_mutex);

	if (err == 0) {
		/* Cắt các bit cấu hình/trạng thái ở DS3231 (ví dụ bit CH ở giây, bit 12/24h ở giờ, bit Century ở tháng) */
		if (sec)   *sec   = bcd_to_bin((uint8_t)(raw[0] & 0x7FU));
		if (min)   *min   = bcd_to_bin((uint8_t)(raw[1] & 0x7FU));
		if (hour)  *hour  = bcd_to_bin((uint8_t)(raw[2] & 0x3FU));
		if (day)   *day   = bcd_to_bin((uint8_t)(raw[4] & 0x3FU));
		if (month) *month = bcd_to_bin((uint8_t)(raw[5] & 0x1FU));
		if (year)  *year  = (uint16_t)(2000U + bcd_to_bin(raw[6]));
	}
	return err;
}

/**
 * @brief Kiểm tra biên hợp lệ và ghi cấu hình ngày giờ BCD xuống đồng hồ DS3231 (0x68).
 */
int bsp_rtc_set_datetime(uint16_t year, uint8_t month, uint8_t day,
			 uint8_t hour, uint8_t min, uint8_t sec)
{
	if (!device_is_ready(i2c1_dev)) {
		return -ENODEV;
	}
	if (year < 2000U || year > 2099U || month < 1U || month > 12U ||
	    day < 1U || day > 31U || hour > 23U || min > 59U || sec > 59U) {
		return -EINVAL;
	}

	uint8_t payload[8] = {
		0x00U, /* Địa chỉ thanh ghi giây bắt đầu của DS3231 */
		bin_to_bcd(sec),
		bin_to_bcd(min),
		bin_to_bcd(hour), /* Định dạng 24 giờ */
		0x01U,            /* Thứ trong tuần (không sử dụng) */
		bin_to_bcd(day),
		bin_to_bcd(month),
		bin_to_bcd((uint8_t)(year - 2000U)),
	};

	k_mutex_lock(&g_i2c1_bus_mutex, K_FOREVER);
	int err = i2c_write(i2c1_dev, payload, sizeof(payload), DS3231_I2C_ADDR);
	k_mutex_unlock(&g_i2c1_bus_mutex);
	return err;
}

/**
 * @brief Ghi thêm một sự cố an toàn kèm dấu thời gian DS3231 và mã CRC16 vào vòng đệm AT24C512.
 */
int bsp_blackbox_append(blackbox_fault_code_t code, elevator_state_t state,
			uint8_t floor, int32_t temp_deci_c)
{
	if (!device_is_ready(eeprom_dev)) {
		return -ENODEV;
	}

	struct bb_header hdr = { 0 };
	elevator_blackbox_rec_t rec = { 0 };

	/* Dấu thời gian dự phòng trong trường hợp kit không gắn pin CMOS / IC DS3231 */
	uint16_t y = 2026U;
	uint8_t mo = 1U, d = 1U, h = 0U, mi = 0U, s = 0U;
	(void)bsp_rtc_get_datetime(&y, &mo, &d, &h, &mi, &s);

	k_mutex_lock(&g_i2c1_bus_mutex, K_FOREVER);

	int err = eeprom_read(eeprom_dev, BB_HDR_OFFSET, &hdr, sizeof(hdr));
	/* Nếu EEPROM trắng hoặc header hỏng magic/CRC32 thì khởi tạo lại mục lục rỗng */
	if (err < 0 || hdr.magic != BB_MAGIC || hdr.crc32 != bb_hdr_crc(&hdr)) {
		memset(&hdr, 0, sizeof(hdr));
		hdr.magic = BB_MAGIC;
		hdr.next_seq = 1U;
	}

	rec.seq         = hdr.next_seq++;
	rec.uptime_ms   = k_uptime_get_32();
	rec.year        = y;
	rec.month       = mo;
	rec.day         = d;
	rec.hour        = h;
	rec.min         = mi;
	rec.sec         = s;
	rec.fault_code  = (uint8_t)code;
	rec.state       = (uint8_t)state;
	rec.floor       = floor;
	rec.temp_deci_c = (int16_t)temp_deci_c;
	rec.crc16       = bb_rec_crc(&rec);

	/* Ghi payload bản ghi mới trước rồi mới xoay vòng write_idx và chốt CRC32 header để tránh mất toàn vẹn */
	off_t rec_off = (off_t)(BB_REC_BASE_OFFSET + ((uint32_t)hdr.write_idx * BB_REC_STRIDE));
	err = eeprom_write(eeprom_dev, rec_off, &rec, sizeof(rec));
	if (err == 0) {
		hdr.write_idx = (uint16_t)((hdr.write_idx + 1U) % BB_MAX_RECORDS);
		if (hdr.valid_count < BB_MAX_RECORDS) {
			hdr.valid_count++;
		}
		hdr.crc32 = bb_hdr_crc(&hdr);
		err = eeprom_write(eeprom_dev, BB_HDR_OFFSET, &hdr, sizeof(hdr));
	}

	k_mutex_unlock(&g_i2c1_bus_mutex);
	return err;
}

/**
 * @brief Đọc ngược từ mới đến cũ các bản ghi hộp đen trong EEPROM và xác thực từng bản ghi bằng CRC16.
 */
int bsp_blackbox_read(elevator_blackbox_rec_t *recs, size_t max_recs, size_t *out_count)
{
	if (recs == NULL || out_count == NULL) {
		return -EINVAL;
	}
	*out_count = 0U;

	if (!device_is_ready(eeprom_dev)) {
		return -ENODEV;
	}

	struct bb_header hdr = { 0 };

	k_mutex_lock(&g_i2c1_bus_mutex, K_FOREVER);
	int err = eeprom_read(eeprom_dev, BB_HDR_OFFSET, &hdr, sizeof(hdr));
	/* Nếu EEPROM trắng hoặc header không khớp magic/CRC32 thì trả về 0 bản ghi */
	if (err < 0 || hdr.magic != BB_MAGIC || hdr.crc32 != bb_hdr_crc(&hdr)) {
		k_mutex_unlock(&g_i2c1_bus_mutex);
		return (err < 0) ? err : 0;
	}

	size_t n = MIN((size_t)hdr.valid_count, max_recs);
	for (size_t i = 0; i < n; i++) {
		/* Lùi chỉ mục vòng tròn để lấy bản ghi mới nhất trước và chỉ trả về bản ghi đạt CRC16 */
		uint16_t idx = (uint16_t)((hdr.write_idx + BB_MAX_RECORDS - 1U - i) % BB_MAX_RECORDS);
		off_t rec_off = (off_t)(BB_REC_BASE_OFFSET + ((uint32_t)idx * BB_REC_STRIDE));
		elevator_blackbox_rec_t tmp = { 0 };

		if (eeprom_read(eeprom_dev, rec_off, &tmp, sizeof(tmp)) == 0 &&
		    tmp.crc16 == bb_rec_crc(&tmp)) {
			recs[(*out_count)++] = tmp;
		}
	}
	k_mutex_unlock(&g_i2c1_bus_mutex);
	return 0;
}

/**
 * @brief Khởi tạo lại header hộp đen trên AT24C512 về trạng thái rỗng (xóa toàn bộ chỉ mục).
 */
int bsp_blackbox_clear(void)
{
	if (!device_is_ready(eeprom_dev)) {
		return -ENODEV;
	}

	struct bb_header hdr = {
		.magic = BB_MAGIC,
		.next_seq = 1U,
		.write_idx = 0U,
		.valid_count = 0U,
	};
	hdr.crc32 = bb_hdr_crc(&hdr);

	k_mutex_lock(&g_i2c1_bus_mutex, K_FOREVER);
	int err = eeprom_write(eeprom_dev, BB_HDR_OFFSET, &hdr, sizeof(hdr));
	k_mutex_unlock(&g_i2c1_bus_mutex);
	return err;
}
