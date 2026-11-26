/**
 * @file cli_bkit_test.c
 * @brief Tập lệnh Zephyr Shell (`bkit`) kiểm thử độc lập từng ngoại vi trên bo mạch BKIT ARM4 & thống kê luồng RTOS.
 *
 * Chức năng tổng quát của file:
 *  - Cho phép kiểm tra phần cứng từng khối ngoại vi riêng lẻ trên bo mạch BKIT ARM4 (ADC, GPIO/Opto,
 *    LED 7 đoạn, Bàn phím 4x4, Còi PWM, RTC DS3231, EEPROM AT24C512, Flash W25Q64, CAN Ping) khi chấm thực hành.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `cmd_bkit_test_gpio()`    : Lệnh `bkit test gpio` — Đọc trạng thái 4 ngõ vào quang X0..X3 và thử bật/tắt Y0, Y1, LED3.
 *  - `cmd_bkit_test_pwm()`     : Lệnh `bkit test pwm` — Phát thử còi PWM trên chân PF8 ở tần số tùy chọn.
 *  - `cmd_bkit_test_adc()`     : Lệnh `bkit test adc` — Đọc và in giá trị tức thời của 5 kênh ADC1 (LM35, VR1, LDR, Áp, Dòng).
 *  - `cmd_bkit_test_i2c()`     : Lệnh `bkit test i2c` — Quét bus I2C1 (AT24C512/DS3231) và đọc giờ từ DS3231.
 *  - `cmd_bkit_test_rtc_set()` : Lệnh `bkit test rtc_set` — Cài đặt ngày giờ thực cho chip DS3231 (I2C1).
 *  - `cmd_bkit_test_spi1()`    : Lệnh `bkit test spi1` — Thử hiển thị 4 LED 7 đoạn & quét phím ma trận 4x4 (74HC165) trên SPI1.
 *  - `cmd_bkit_test_spi2()`    : Lệnh `bkit test spi2` — Đọc mã JEDEC ID (`EF 40 17`) của chip Flash ngoài W25Q64FV (SPI2).
 *  - `cmd_bkit_test_can()`     : Lệnh `bkit test can` — Bắn khung CAN PING (`0x380`) hoặc RTR (`0x200`) và đo RTT (us).
 *  - `cmd_bkit_test_lcd()`     : Lệnh `bkit test lcd` — Vẽ thanh màu thử nghiệm và kiểm tra màn hình màu TFT-LCD 240x320 (FSMC 16-bit).
 *  - `cmd_bkit_test_wdt_hang()`: Lệnh `bkit test wdt_hang` — Ngừng nạp IWDG để kiểm chứng tự reset MCU sau ~2500 ms.
 *  - `cmd_bkit_stats()`        : Lệnh `bkit stats` — In nguyên nhân reset, UID MCU và mức tiêu thụ Stack của toàn bộ luồng RTOS.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* kernel.h phải đặt trước thread_analyzer.h vì thread_analyzer phụ thuộc các kiểu k_tid_t của kernel */
#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/shell/shell.h>

#include "bsp_adc_sensors.h"
#include "bsp_gpio_exti.h"
#include "bsp_i2c_storage.h"
#include "bsp_pwm_buzzer.h"
#include "bsp_spi_hmi.h"
#include "bsp_tft_lcd.h"
#include "bsp_watchdog.h"
#include "can_bus.h"

extern volatile bool g_wdt_freeze_test;

static const struct shell *g_scan_sh;

/**
 * @brief Hàm chuyển tiếp chuỗi in từ bộ quét bus I2C1 sang giao diện Zephyr Shell đang hoạt động.
 */
static void shell_scan_printer(const char *fmt, ...)
{
	va_list ap;
	char buf[128];

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	if (g_scan_sh != NULL) {
		shell_fprintf(g_scan_sh, SHELL_NORMAL, "%s", buf);
	}
}

/**
 * @brief Lệnh `bkit test gpio`: bật tắt thử PE4, PE5 (Y0), PE6 (Y1) và đọc trạng thái X0..X3.
 */
static int cmd_bkit_test_gpio(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	bool x0 = false, x1 = false, x2 = false, x3 = false;

	shell_print(sh, "Cycling PE4 (LED3), PE5 (Y0 UP), PE6 (Y1 DOWN)...");
	bsp_gpio_set_led_debug(true);
	bsp_gpio_set_motor_outputs(true, false);
	k_msleep(350);
	bsp_gpio_set_motor_outputs(false, true);
	k_msleep(350);
	bsp_gpio_set_motor_outputs(false, false);
	bsp_gpio_set_led_debug(false);

	bsp_gpio_read_inputs(&x0, &x1, &x2, &x3);
	shell_print(sh, "Inputs: X0(PA6)=%d X1(PA7)=%d X2(PC4)=%d X3(PC5)=%d", x0, x1, x2, x3);
	return 0;
}

/**
 * @brief Lệnh `bkit test pwm`: phát thử 1 tiếng còi 250 ms trên chân PF8 ở tần số tùy chọn.
 */
static int cmd_bkit_test_pwm(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t freq = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 2400U;
	bsp_pwm_buzzer_beep(freq, 250U);
	shell_print(sh, "PF8 TIM13_CH1 buzzer beep triggered at %u Hz (250 ms)", freq);
	return 0;
}

/**
 * @brief Lệnh `bkit test adc`: lấy mẫu cả 5 kênh ADC1 (nhiệt độ LM35, tải, ánh sáng, áp, dòng).
 */
static int cmd_bkit_test_adc(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	bkit_adc_snapshot_t adc = { 0 };
	int err = bsp_adc_read_all(&adc);
	shell_print(sh, "ADC1 read (err=%d):", err);
	shell_print(sh, "  PC2 (LM35 Temp)   : %d.%d degC",
		    adc.temp_deci_c / 10, abs(adc.temp_deci_c % 10));
	shell_print(sh, "  PC1 (Pot LoadSim) : %u kg", adc.load_kg);
	shell_print(sh, "  PC0 (Light)       : %u mV", adc.light_mv);
	shell_print(sh, "  PB0 (Voltage)     : %u mV", adc.bus_mv);
	shell_print(sh, "  PB1 (Current)     : %u mV", adc.current_mv);
	return err;
}

/**
 * @brief Lệnh `bkit test i2c`: quét toàn bộ thiết bị trên bus I2C1 và đọc thử giờ DS3231 RTC.
 */
static int cmd_bkit_test_i2c(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Scanning I2C1 (PB6/PB7) [DS3231=0x68, AT24C512=0x50]...");
	g_scan_sh = sh;
	bsp_i2c_scan_bus(shell_scan_printer);
	g_scan_sh = NULL;

	uint16_t y = 0U;
	uint8_t mo = 0U, d = 0U, h = 0U, mi = 0U, s = 0U;
	int rtc_err = bsp_rtc_get_datetime(&y, &mo, &d, &h, &mi, &s);
	if (rtc_err == 0) {
		shell_print(sh, "DS3231 RTC: %04u-%02u-%02u %02u:%02u:%02u",
			    y, mo, d, h, mi, s);
	} else {
		shell_print(sh, "DS3231 RTC read returned %d", rtc_err);
	}
	return 0;
}

/**
 * @brief Lệnh `bkit test rtc_set`: cài đặt ngày giờ thực tế xuống đồng hồ DS3231 trên I2C1.
 */
static int cmd_bkit_test_rtc_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 7) {
		shell_error(sh, "Usage: bkit test rtc_set <YYYY> <MM> <DD> <hh> <mm> <ss>");
		return -EINVAL;
	}

	int y = atoi(argv[1]);
	int mo = atoi(argv[2]);
	int d = atoi(argv[3]);
	int h = atoi(argv[4]);
	int mi = atoi(argv[5]);
	int s = atoi(argv[6]);

	if (y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
	    h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59) {
		shell_error(sh, "Out-of-range date/time fields");
		return -EINVAL;
	}

	int err = bsp_rtc_set_datetime((uint16_t)y, (uint8_t)mo, (uint8_t)d,
				       (uint8_t)h, (uint8_t)mi, (uint8_t)s);
	shell_print(sh, "DS3231 RTC set %04d-%02d-%02d %02d:%02d:%02d (err=%d)",
		    y, mo, d, h, mi, s, err);
	return err;
}

/**
 * @brief Lệnh `bkit test spi1`: kiểm thử xuất mẫu LED 7 đoạn và quét thử ma trận phím 4x4 dùng chung SPI1.
 */
static int cmd_bkit_test_spi1(const struct shell *sh, size_t argc, char **argv)
{
	/* 0x01 = sáng đủ 7 đoạn '8' (tắt dấu chấm dp ở bit 0) theo sơ đồ U15 BKIT ARM4 */
	uint8_t pattern[4] = { 0x01U, 0x01U, 0x01U, 0x01U };

	if (argc >= 2) {
		uint8_t val = (uint8_t)strtoul(argv[1], NULL, 16);
		memset(pattern, val, sizeof(pattern));
	}

	bsp_spi_7seg_hold_test_pattern(pattern, 1500U);
	int key = -1;
	uint8_t rx[2] = { 0 };
	int err_key = bsp_spi_keypad_scan(&key, rx);

	shell_print(sh, "SPI1 7-Seg pattern=0x%02X held for 1500 ms across 4 digits", pattern[0]);
	shell_print(sh, "SPI1 Keypad scan err=%d raw=[0x%02X, 0x%02X] key=%d",
		    err_key, rx[0], rx[1], key);
	return 0;
}

/**
 * @brief Lệnh `bkit test spi2`: đọc 3 byte mã định danh JEDEC ID từ chip Flash W25Q64FV trên SPI2.
 */
static int cmd_bkit_test_spi2(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint8_t jedec[3] = { 0 };
	int err = bsp_spi2_probe_jedec(jedec);
	shell_print(sh, "SPI2 W25Q64FV JEDEC ID probe (err=%d): %02X %02X %02X (expected EF 40 17)",
		    err, jedec[0], jedec[1], jedec[2]);
	return err;
}

/**
 * @brief Lệnh `bkit test can [rtr]`: phát khung HIL PING (0x380) hoặc RTR (0x200) rồi đọc TEC/REC/cbOK/RTT.
 */
static int cmd_bkit_test_can(const struct shell *sh, size_t argc, char **argv)
{
	static uint32_t ping_seq = 1U;
	bool send_rtr = (argc >= 2 && strcmp(argv[1], "rtr") == 0);
	int tx_err = send_rtr ? can_bus_request_cabin_rtr() : can_bus_send_ping(ping_seq++);

	/* Nhường CPU ngắn 20 ms để luồng t3_can_rx kịp xử lý phản hồi từ g_can_rx_msgq */
	k_msleep(20);

	uint8_t st = 0U, tec = 0U, rec = 0U;
	uint32_t rx_ok = 0U, tx_ok = 0U, rtt_us = 0U;
	uint32_t tx_cb_ok = 0U, tx_cb_err = 0U, rtr_rx = 0U;

	can_bus_get_diag(&st, &tec, &rec, &rx_ok, &tx_ok, &rtt_us);
	can_bus_get_ext_stats(&tx_cb_ok, &tx_cb_err, &rtr_rx);

	shell_print(sh, "CAN1 %s (err=%d) -> st=%u TEC=%u REC=%u TX=%u RX=%u cbOK=%u cbErr=%u RTR=%u RTT=%u us",
		    send_rtr ? "RTR(0x200)" : "PING",
		    tx_err, st, tec, rec, tx_ok, rx_ok, tx_cb_ok, tx_cb_err, rtr_rx, rtt_us);
	return tx_err;
}

/**
 * @brief Lệnh `bkit test wdt_hang`: bật cờ đóng băng nạp IWDG để kiểm chứng tự reset phần cứng sau ~2.5 giây.
 */
static int cmd_bkit_test_wdt_hang(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_warn(sh, "Pausing IWDG feeding! STM32 will hardware-reset in ~2500 ms...");
	g_wdt_freeze_test = true;
	return 0;
}

/**
 * @brief Lệnh `bkit test lcd`: kiểm tra hiển thị màn hình màu TFT-LCD 240x320 qua FSMC 16-bit.
 */
static int cmd_bkit_test_lcd(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Testing ILI9341 FSMC 16-bit TFT-LCD (240x320)...");
	int err = bsp_tft_lcd_run_self_test(1800U);
	if (err < 0) {
		shell_error(sh, "LCD self-test failed (%d)", err);
		return err;
	}

	shell_print(sh, "TFT-LCD self-test PASS (holding color bars for 1800 ms, then auto-restoring Dashboard)!");
	return 0;
}

/**
 * @brief Lệnh `bkit stats`: in lý do reset từ HWINFO, UID chip và mức tiêu thụ stack/CPU của toàn bộ luồng RTOS.
 */
static int cmd_bkit_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t cause = 0U;
	uint8_t uid[12] = { 0 };
	int uid_len = 0;

	bsp_wdt_get_hwinfo(&cause, uid, &uid_len);
	shell_print(sh, "Reset Cause=0x%08X (WDT=%d) | MCU UID (%d bytes): %02X%02X%02X%02X...",
		    cause, bsp_wdt_was_reset_cause(), uid_len, uid[0], uid[1], uid[2], uid[3]);
	shell_print(sh, "Printing Zephyr Thread Analyzer summary to console:");
	thread_analyzer_print(0);
	return 0;
}

/*
 * Macro Zephyr Shell đăng ký hai tầng lệnh con:
 *  - `bkit test <gpio|pwm|adc|i2c|rtc_set|spi1|spi2|can|wdt_hang>`: kiểm tra từng ngoại vi.
 *  - `bkit stats`: xem nguyên nhân reset (HWINFO) + mức chiếm dụng RAM stack của 7 luồng.
 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_bkit_test,
	SHELL_CMD(gpio, NULL, "Cycle PE4/PE5/PE6 & read PA6/PA7/PC4/PC5", cmd_bkit_test_gpio),
	SHELL_CMD_ARG(pwm, NULL, "Beep PF8 buzzer [freq_hz]", cmd_bkit_test_pwm, 1, 1),
	SHELL_CMD(adc, NULL, "Read all 5 ADC1 channels (LM35, Pot, Light, Volt, Curr)", cmd_bkit_test_adc),
	SHELL_CMD(i2c, NULL, "Scan I2C1 & read DS3231 RTC + AT24C512", cmd_bkit_test_i2c),
	SHELL_CMD_ARG(rtc_set, NULL, "Set DS3231 RTC <YYYY> <MM> <DD> <hh> <mm> <ss>", cmd_bkit_test_rtc_set, 7, 0),
	SHELL_CMD_ARG(spi1, NULL, "Test SPI1 7-seg + keypad [hex_pattern]", cmd_bkit_test_spi1, 1, 1),
	SHELL_CMD(spi2, NULL, "Read W25Q64FV JEDEC ID on SPI2", cmd_bkit_test_spi2),
	SHELL_CMD_ARG(can, NULL, "Send CAN1 PING or RTR [rtr] & inspect TEC/REC/RTT", cmd_bkit_test_can, 1, 1),
	SHELL_CMD(lcd, NULL, "Test ILI9341 FSMC 16-bit TFT-LCD 240x320", cmd_bkit_test_lcd),
	SHELL_CMD(wdt_hang, NULL, "Stop feeding IWDG to verify hardware watchdog reset", cmd_bkit_test_wdt_hang),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_bkit,
	SHELL_CMD(test, &sub_bkit_test, "Onboard peripheral hardware self-tests", NULL),
	SHELL_CMD(stats, NULL, "Print HWINFO reset cause & Zephyr thread stack/CPU stats", cmd_bkit_stats),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(bkit, &sub_bkit, "BKIT ARM4 board hardware diagnostics", NULL);
