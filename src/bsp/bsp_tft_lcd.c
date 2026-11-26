/**
 * @file bsp_tft_lcd.c
 * @brief Hiện thực Trình điều khiển Màn hình màu TFT-LCD 240x320 (ILI9341 - FSMC 16-bit)
 *        Hệ thống Thang máy Thông minh BKIT ARM4 (STM32F407ZGT6).
 *
 * Chức năng tổng quát của file:
 *  - Cấu hình phần cứng FSMC 16-bit (Bank 1 NOR/PSRAM 1) trên STM32F407ZGT6 nối chip ILI9341,
 *    đồng thời kéo cao chân `PG10 (FSMC_NE3)` để cô lập chip SRAM ngoài `IS61LV51216` tránh xung đột bus D0..D15.
 *  - Quản lý máy trạng thái khung hình (`lcd_ui_mode_t`) đảm bảo xóa sạch nền khi chuyển đổi giữa
 *    Dashboard 4 khối và các màn hình khẩn cấp (Overload / Fire Alarm / E-Stop / Self-Test), không bao giờ đè chữ.
 *  - Chỉ vẽ lại các phần tử thay đổi (Dirty-Region Rendering), đệm khoảng trắng cố định chống sót ký tự đuôi,
 *    xóa sạch viền ngoài `CALL QUEUE` theo cả hai chiều Lên/Xuống, vẽ mũi tên đồ họa sắc nét và đồng bộ giờ DS3231 RTC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_tft_lcd.h"
#include "bsp_i2c_storage.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <soc.h>

LOG_MODULE_REGISTER(bsp_lcd, LOG_LEVEL_INF);

/*
 * Ánh xạ bộ nhớ FSMC Bank 1 (NOR/PSRAM 1):
 *  - RS nối chân PE3 (FSMC_A19) -> dịch 1 bit ở chế độ 16-bit bus tương ứng bit 20 địa chỉ AHB.
 *  - Ghi Lệnh (RS = 0): 0x60000000
 *  - Ghi Dữ liệu (RS = 1): 0x60100000
 */
#define LCD_CMD_ADDR    (*((volatile uint16_t *)0x60000000U))
#define LCD_DATA_ADDR   (*((volatile uint16_t *)0x60100000U))

typedef enum {
	LCD_UI_NONE = 0,
	LCD_UI_NORMAL,
	LCD_UI_OVERLOAD,
	LCD_UI_FIRE,
	LCD_UI_ESTOP,
} lcd_ui_mode_t;

static bool g_lcd_ready;
static lcd_ui_mode_t g_ui_mode = LCD_UI_NONE;
static bool g_hdr_drawn;
static uint8_t g_prev_floor = 0xFFU;
static uint8_t g_prev_call_mask = 0xFFU;
static uint8_t g_prev_queue_floor = 0xFFU;
static uint32_t g_last_render_uptime;

/* Bộ đệm đồng hồ thực DS3231 đọc mỗi 1000 ms (tự động dự phòng bằng uptime nếu không gắn DS3231) */
static uint32_t g_last_rtc_poll_ms;
static bool g_rtc_valid;
static uint8_t g_rtc_h, g_rtc_m, g_rtc_s;

/* ============================================================================
 * BẢNG FONT CHỮ ASCII 6x8 (0x20..0x7A)
 * ============================================================================ */
static const uint8_t font_6x8[][6] = {
	[' ' - ' '] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	['!' - ' '] = { 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00 },
	['"' - ' '] = { 0x00, 0x07, 0x00, 0x07, 0x00, 0x00 },
	['#' - ' '] = { 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00 },
	['$' - ' '] = { 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00 },
	['%' - ' '] = { 0x23, 0x13, 0x08, 0x64, 0x62, 0x00 },
	['&' - ' '] = { 0x36, 0x49, 0x55, 0x22, 0x50, 0x00 },
	['\'' - ' '] = { 0x00, 0x05, 0x03, 0x00, 0x00, 0x00 },
	['(' - ' '] = { 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00 },
	[')' - ' '] = { 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00 },
	['*' - ' '] = { 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00 },
	['+' - ' '] = { 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00 },
	[',' - ' '] = { 0x00, 0x50, 0x30, 0x00, 0x00, 0x00 },
	['-' - ' '] = { 0x08, 0x08, 0x08, 0x08, 0x08, 0x00 },
	['.' - ' '] = { 0x00, 0x60, 0x60, 0x00, 0x00, 0x00 },
	['/' - ' '] = { 0x20, 0x10, 0x08, 0x04, 0x02, 0x00 },
	['0' - ' '] = { 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00 },
	['1' - ' '] = { 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00 },
	['2' - ' '] = { 0x42, 0x61, 0x51, 0x49, 0x46, 0x00 },
	['3' - ' '] = { 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00 },
	['4' - ' '] = { 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00 },
	['5' - ' '] = { 0x27, 0x45, 0x45, 0x45, 0x39, 0x00 },
	['6' - ' '] = { 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00 },
	['7' - ' '] = { 0x01, 0x71, 0x09, 0x05, 0x03, 0x00 },
	['8' - ' '] = { 0x36, 0x49, 0x49, 0x49, 0x36, 0x00 },
	['9' - ' '] = { 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00 },
	[':' - ' '] = { 0x00, 0x36, 0x36, 0x00, 0x00, 0x00 },
	[';' - ' '] = { 0x00, 0x56, 0x36, 0x00, 0x00, 0x00 },
	['<' - ' '] = { 0x08, 0x14, 0x22, 0x41, 0x00, 0x00 },
	['=' - ' '] = { 0x14, 0x14, 0x14, 0x14, 0x14, 0x00 },
	['>' - ' '] = { 0x00, 0x41, 0x22, 0x14, 0x08, 0x00 },
	['?' - ' '] = { 0x02, 0x01, 0x51, 0x09, 0x06, 0x00 },
	['@' - ' '] = { 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00 },
	['A' - ' '] = { 0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00 },
	['B' - ' '] = { 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00 },
	['C' - ' '] = { 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00 },
	['D' - ' '] = { 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00 },
	['E' - ' '] = { 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00 },
	['F' - ' '] = { 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00 },
	['G' - ' '] = { 0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00 },
	['H' - ' '] = { 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00 },
	['I' - ' '] = { 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00 },
	['J' - ' '] = { 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00 },
	['K' - ' '] = { 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00 },
	['L' - ' '] = { 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00 },
	['M' - ' '] = { 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00 },
	['N' - ' '] = { 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00 },
	['O' - ' '] = { 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00 },
	['P' - ' '] = { 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00 },
	['Q' - ' '] = { 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00 },
	['R' - ' '] = { 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00 },
	['S' - ' '] = { 0x46, 0x49, 0x49, 0x49, 0x31, 0x00 },
	['T' - ' '] = { 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00 },
	['U' - ' '] = { 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00 },
	['V' - ' '] = { 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00 },
	['W' - ' '] = { 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00 },
	['X' - ' '] = { 0x63, 0x14, 0x08, 0x14, 0x63, 0x00 },
	['Y' - ' '] = { 0x07, 0x08, 0x70, 0x08, 0x07, 0x00 },
	['Z' - ' '] = { 0x61, 0x51, 0x49, 0x45, 0x43, 0x00 },
	['[' - ' '] = { 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00 },
	['\\' - ' '] = { 0x02, 0x04, 0x08, 0x10, 0x20, 0x00 },
	[']' - ' '] = { 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00 },
	['^' - ' '] = { 0x04, 0x02, 0x01, 0x02, 0x04, 0x00 },
	['_' - ' '] = { 0x40, 0x40, 0x40, 0x40, 0x40, 0x00 },
	['`' - ' '] = { 0x00, 0x01, 0x02, 0x04, 0x00, 0x00 },
	['a' - ' '] = { 0x20, 0x54, 0x54, 0x54, 0x78, 0x00 },
	['b' - ' '] = { 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00 },
	['c' - ' '] = { 0x38, 0x44, 0x44, 0x44, 0x20, 0x00 },
	['d' - ' '] = { 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00 },
	['e' - ' '] = { 0x38, 0x54, 0x54, 0x54, 0x18, 0x00 },
	['f' - ' '] = { 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00 },
	['g' - ' '] = { 0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00 },
	['h' - ' '] = { 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00 },
	['i' - ' '] = { 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00 },
	['j' - ' '] = { 0x20, 0x40, 0x44, 0x3D, 0x00, 0x00 },
	['k' - ' '] = { 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00 },
	['l' - ' '] = { 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00 },
	['m' - ' '] = { 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00 },
	['n' - ' '] = { 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00 },
	['o' - ' '] = { 0x38, 0x44, 0x44, 0x44, 0x38, 0x00 },
	['p' - ' '] = { 0x7C, 0x14, 0x14, 0x14, 0x08, 0x00 },
	['q' - ' '] = { 0x08, 0x14, 0x14, 0x18, 0x7C, 0x00 },
	['r' - ' '] = { 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00 },
	['s' - ' '] = { 0x48, 0x54, 0x54, 0x54, 0x20, 0x00 },
	['t' - ' '] = { 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00 },
	['u' - ' '] = { 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00 },
	['v' - ' '] = { 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00 },
	['w' - ' '] = { 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00 },
	['x' - ' '] = { 0x44, 0x28, 0x10, 0x28, 0x44, 0x00 },
	['y' - ' '] = { 0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00 },
	['z' - ' '] = { 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00 },
	['{' - ' '] = { 0x00, 0x08, 0x36, 0x41, 0x00, 0x00 },
	['|' - ' '] = { 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00 },
	['}' - ' '] = { 0x00, 0x41, 0x36, 0x08, 0x00, 0x00 },
	['~' - ' '] = { 0x08, 0x04, 0x08, 0x10, 0x08, 0x00 },
};

static inline void lcd_write_cmd(uint16_t cmd)
{
	LCD_CMD_ADDR = cmd;
}

static inline void lcd_write_data(uint16_t data)
{
	LCD_DATA_ADDR = data;
}

static void lcd_set_window(int x0, int y0, int x1, int y1)
{
	if (x0 < 0) {
		x0 = 0;
	}
	if (y0 < 0) {
		y0 = 0;
	}
	if (x1 >= LCD_WIDTH) {
		x1 = LCD_WIDTH - 1;
	}
	if (y1 >= LCD_HEIGHT) {
		y1 = LCD_HEIGHT - 1;
	}

	lcd_write_cmd(0x2AU);
	lcd_write_data((uint16_t)((x0 >> 8) & 0xFFU));
	lcd_write_data((uint16_t)(x0 & 0xFFU));
	lcd_write_data((uint16_t)((x1 >> 8) & 0xFFU));
	lcd_write_data((uint16_t)(x1 & 0xFFU));

	lcd_write_cmd(0x2BU);
	lcd_write_data((uint16_t)((y0 >> 8) & 0xFFU));
	lcd_write_data((uint16_t)(y0 & 0xFFU));
	lcd_write_data((uint16_t)((y1 >> 8) & 0xFFU));
	lcd_write_data((uint16_t)(y1 & 0xFFU));

	lcd_write_cmd(0x2CU);
}

void bsp_tft_lcd_invalidate(void)
{
	g_ui_mode = LCD_UI_NONE;
	g_hdr_drawn = false;
	g_prev_floor = 0xFFU;
	g_prev_call_mask = 0xFFU;
	g_prev_queue_floor = 0xFFU;
}

int bsp_tft_lcd_init(void)
{
#if defined(CONFIG_SOC_SERIES_STM32F4X)
	/* 1. Bật clock GPIO A, C, D, E, G và bộ điều khiển FSMC */
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN |
			 RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN |
			 RCC_AHB1ENR_GPIOGEN);
	RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;

	/*
	 * Cô lập chip SRAM ngoài IS61LV51216 (dùng chung bus FSMC_D0..D15) bằng cách
	 * cấu hình chân chọn chip PG10 (FSMC_NE3) làm GPIO Output kéo lên mức CAO (1 = Deselect).
	 */
	GPIOG->MODER &= ~(3U << (10 * 2));
	GPIOG->MODER |= (1U << (10 * 2));
	GPIOG->ODR |= (1U << 10);

	/* 2. Cấu hình chân Đèn nền PA8 (GPIO Output Push-pull) */
	GPIOA->MODER &= ~(3U << (8 * 2));
	GPIOA->MODER |= (1U << (8 * 2));
	GPIOA->ODR |= (1U << 8);

	/* 3. Cấu hình và tạo xung Reset cứng trên chân PC13 */
	GPIOC->MODER &= ~(3U << (13 * 2));
	GPIOC->MODER |= (1U << (13 * 2));
	GPIOC->ODR |= (1U << 13);
	k_busy_wait(10000U);
	GPIOC->ODR &= ~(1U << 13);
	k_busy_wait(20000U);
	GPIOC->ODR |= (1U << 13);
	k_busy_wait(50000U);

	/* 4. Cấu hình AF12 (FSMC) cho các chân dữ liệu/điều khiển trên Port D và Port E */
	uint32_t pd_pins = (1U << 0) | (1U << 1) | (1U << 4) | (1U << 5) |
			   (1U << 7) | (1U << 8) | (1U << 9) | (1U << 10) |
			   (1U << 14) | (1U << 15);
	for (int i = 0; i < 16; i++) {
		if (pd_pins & (1U << i)) {
			GPIOD->MODER &= ~(3U << (i * 2));
			GPIOD->MODER |= (2U << (i * 2));
			GPIOD->OSPEEDR |= (3U << (i * 2));
			if (i < 8) {
				GPIOD->AFR[0] &= ~(0xFU << (i * 4));
				GPIOD->AFR[0] |= (12U << (i * 4));
			} else {
				GPIOD->AFR[1] &= ~(0xFU << ((i - 8) * 4));
				GPIOD->AFR[1] |= (12U << ((i - 8) * 4));
			}
		}
	}

	uint32_t pe_pins = (1U << 3) | (1U << 7) | (1U << 8) | (1U << 9) |
			   (1U << 10) | (1U << 11) | (1U << 12) | (1U << 13) |
			   (1U << 14) | (1U << 15);
	for (int i = 0; i < 16; i++) {
		if (pe_pins & (1U << i)) {
			GPIOE->MODER &= ~(3U << (i * 2));
			GPIOE->MODER |= (2U << (i * 2));
			GPIOE->OSPEEDR |= (3U << (i * 2));
			if (i < 8) {
				GPIOE->AFR[0] &= ~(0xFU << (i * 4));
				GPIOE->AFR[0] |= (12U << (i * 4));
			} else {
				GPIOE->AFR[1] &= ~(0xFU << ((i - 8) * 4));
				GPIOE->AFR[1] |= (12U << ((i - 8) * 4));
			}
		}
	}

	/*
	 * 5. Cấu hình FSMC Bank1 Subbank 1 (NE1):
	 *    - BCR1 = 0x00001011 (Async SRAM 16-bit, WREN=1, MBKEN=1, tắt FACCEN NOR Flash)
	 *    - BTR1 = ADDSET=3, DATAST=10 chu kỳ HCLK (168 MHz -> chu kỳ ghi ~77 ns > 66 ns chuẩn ILI9341)
	 */
	FSMC_Bank1->BTCR[0] = 0x00001011U;
	FSMC_Bank1->BTCR[1] = 0x00000A03U;
#endif

	/* 6. Khởi tạo tập lệnh chuẩn cho bộ điều khiển đồ họa ILI9341 */
	lcd_write_cmd(0x01U);
	k_msleep(50);

	lcd_write_cmd(0xC0U);
	lcd_write_data(0x26U);

	lcd_write_cmd(0xC1U);
	lcd_write_data(0x11U);

	lcd_write_cmd(0xC5U);
	lcd_write_data(0x35U);
	lcd_write_data(0x3EU);

	lcd_write_cmd(0x36U);
	lcd_write_data(0x48U);

	lcd_write_cmd(0x3AU);
	lcd_write_data(0x55U);

	lcd_write_cmd(0xB1U);
	lcd_write_data(0x00U);
	lcd_write_data(0x1BU);

	lcd_write_cmd(0x11U);
	k_msleep(120);

	lcd_write_cmd(0x29U);
	k_msleep(20);

	g_lcd_ready = true;
	bsp_tft_lcd_clear(LCD_COLOR_NAVY);
	LOG_INF("ILI9341 FSMC 16-bit LCD initialized (240x320, RGB565)");
	return 0;
}

bool bsp_tft_lcd_is_ready(void)
{
	return g_lcd_ready;
}

void bsp_tft_lcd_set_backlight(uint8_t brightness_pct)
{
#if defined(CONFIG_SOC_SERIES_STM32F4X)
	if (brightness_pct > 0U) {
		GPIOA->ODR |= (1U << 8);
	} else {
		GPIOA->ODR &= ~(1U << 8);
	}
#else
	ARG_UNUSED(brightness_pct);
#endif
}

void bsp_tft_lcd_clear(uint16_t color)
{
	bsp_tft_lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
	bsp_tft_lcd_invalidate();
}

void bsp_tft_lcd_draw_pixel(int x, int y, uint16_t color)
{
	if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) {
		return;
	}
	lcd_set_window(x, y, x, y);
	lcd_write_data(color);
}

void bsp_tft_lcd_fill_rect(int x, int y, int w, int h, uint16_t color)
{
	if (w <= 0 || h <= 0 || x >= LCD_WIDTH || y >= LCD_HEIGHT) {
		return;
	}
	int x1 = x + w - 1;
	int y1 = y + h - 1;
	if (x < 0) {
		x = 0;
	}
	if (y < 0) {
		y = 0;
	}
	if (x1 >= LCD_WIDTH) {
		x1 = LCD_WIDTH - 1;
	}
	if (y1 >= LCD_HEIGHT) {
		y1 = LCD_HEIGHT - 1;
	}

	lcd_set_window(x, y, x1, y1);
	uint32_t total = (uint32_t)(x1 - x + 1) * (uint32_t)(y1 - y + 1);
	for (uint32_t i = 0; i < total; i++) {
		lcd_write_data(color);
	}
}

void bsp_tft_lcd_draw_rect(int x, int y, int w, int h, uint16_t color)
{
	bsp_tft_lcd_fill_rect(x, y, w, 1, color);
	bsp_tft_lcd_fill_rect(x, y + h - 1, w, 1, color);
	bsp_tft_lcd_fill_rect(x, y, 1, h, color);
	bsp_tft_lcd_fill_rect(x + w - 1, y, 1, h, color);
}

void bsp_tft_lcd_draw_char(int x, int y, char c, uint16_t color, uint16_t bg_color)
{
	if (x < 0 || y < 0 || x + 5 >= LCD_WIDTH || y + 7 >= LCD_HEIGHT) {
		return;
	}
	unsigned char uc = (unsigned char)c;
	if (uc < ' ' || uc > '~') {
		uc = ' ';
	}
	const uint8_t *glyph = font_6x8[uc - ' '];
	lcd_set_window(x, y, x + 5, y + 7);

	for (int row = 0; row < 8; row++) {
		for (int col = 0; col < 6; col++) {
			lcd_write_data((glyph[col] & (1U << row)) ? color : bg_color);
		}
	}
}

void bsp_tft_lcd_draw_string(int x, int y, const char *str, uint16_t color, uint16_t bg_color)
{
	if (str == NULL) {
		return;
	}
	int cur_x = x;
	while (*str && cur_x <= LCD_WIDTH - 6) {
		bsp_tft_lcd_draw_char(cur_x, y, *str, color, bg_color);
		cur_x += 6;
		str++;
	}
}

/**
 * @brief In chuỗi độ rộng cố định `fixed_chars` (tự động bù khoảng trắng xóa sạch đuôi chữ cũ, ví dụ chống lỗi `IDLE_CLOSEDG`).
 */
static void lcd_draw_string_padded(int x, int y, const char *str, int fixed_chars,
				   uint16_t color, uint16_t bg_color)
{
	int cur_x = x;
	bool ended = (str == NULL);

	for (int i = 0; i < fixed_chars && cur_x <= LCD_WIDTH - 6; i++) {
		char ch = ' ';
		if (!ended) {
			if (str[i] == '\0') {
				ended = true;
			} else {
				ch = str[i];
			}
		}
		bsp_tft_lcd_draw_char(cur_x, y, ch, color, bg_color);
		cur_x += 6;
	}
}

/**
 * @brief Vẽ số tầng lớn 24x40 px có cắt biên dọc `[clip_y0..clip_y1]` phục vụ cuộn trượt mượt không tràn khung.
 */
static void lcd_draw_big_digit_clipped(int x, int y, char digit,
				       int clip_y0, int clip_y1,
				       uint16_t color, uint16_t bg_color)
{
	if (digit < '0' || digit > '9') {
		digit = '-';
	}
	const uint8_t *glyph = font_6x8[digit - ' '];
	const int scale_x = 4;
	const int scale_y = 5;

	for (int col = 0; col < 6; col++) {
		uint8_t bits = glyph[col];
		for (int row = 0; row < 8; row++) {
			int py0 = y + row * scale_y;
			int py1 = py0 + scale_y - 1;
			if (py1 < clip_y0 || py0 > clip_y1) {
				continue;
			}
			if (py0 < clip_y0) {
				py0 = clip_y0;
			}
			if (py1 > clip_y1) {
				py1 = clip_y1;
			}
			uint16_t c = (bits & (1U << row)) ? color : bg_color;
			bsp_tft_lcd_fill_rect(x + col * scale_x, py0, scale_x, py1 - py0 + 1, c);
		}
	}
}

void bsp_tft_lcd_draw_big_digit(int x, int y, char digit, uint16_t color, uint16_t bg_color)
{
	lcd_draw_big_digit_clipped(x, y, digit, 0, LCD_HEIGHT - 1, color, bg_color);
}

void bsp_tft_lcd_draw_progress_bar(int x, int y, int w, int h, int pct,
				   uint16_t bar_color, uint16_t bg_color)
{
	if (pct < 0) {
		pct = 0;
	}
	if (pct > 100) {
		pct = 100;
	}

	bsp_tft_lcd_draw_rect(x, y, w, h, LCD_COLOR_GRAY);
	int fill_w = ((w - 2) * pct) / 100;
	int empty_w = (w - 2) - fill_w;

	if (fill_w > 0) {
		bsp_tft_lcd_fill_rect(x + 1, y + 1, fill_w, h - 2, bar_color);
	}
	if (empty_w > 0) {
		bsp_tft_lcd_fill_rect(x + 1 + fill_w, y + 1, empty_w, h - 2, bg_color);
	}
}

/**
 * @brief Vẽ biểu tượng mũi tên tam giác đồ họa sắc nét (thay vì ký tự UTF-8 gây lỗi mất hình trên font ASCII).
 */
static void lcd_draw_dir_badge(int x, int y, elevator_dir_t dir)
{
	bsp_tft_lcd_fill_rect(x, y, 11, 9, LCD_COLOR_NAVY);

	if (dir == ELEV_DIR_UP) {
		for (int r = 0; r < 5; r++) {
			bsp_tft_lcd_fill_rect(x + 5 - r, y + 1 + r, 2 * r + 1, 1, LCD_COLOR_GREEN);
		}
		bsp_tft_lcd_fill_rect(x + 4, y + 6, 3, 2, LCD_COLOR_GREEN);
		lcd_draw_string_padded(x + 14, y + 1, "UP", 6, LCD_COLOR_GREEN, LCD_COLOR_NAVY);
	} else if (dir == ELEV_DIR_DOWN) {
		bsp_tft_lcd_fill_rect(x + 4, y + 1, 3, 2, LCD_COLOR_ORANGE);
		for (int r = 0; r < 5; r++) {
			bsp_tft_lcd_fill_rect(x + 1 + r, y + 3 + r, 9 - 2 * r, 1, LCD_COLOR_ORANGE);
		}
		lcd_draw_string_padded(x + 14, y + 1, "DOWN", 6, LCD_COLOR_ORANGE, LCD_COLOR_NAVY);
	} else {
		bsp_tft_lcd_fill_rect(x + 1, y + 4, 9, 2, LCD_COLOR_CYAN);
		lcd_draw_string_padded(x + 14, y + 1, "IDLE", 6, LCD_COLOR_WHITE, LCD_COLOR_NAVY);
	}
}

/**
 * @brief Vẽ 7 ô nút gọi tầng căn giữa (chỉ vẽ lại khi `call_mask` hoặc `current_floor` thay đổi,
 *        và xóa triệt để khung viền ngoài `bx - 1, y - 1` khi cabin rời tầng theo cả chiều Lên lẫn Xuống).
 */
static void draw_call_queue_row(int y, uint8_t call_mask, uint8_t current_floor, bool force_redraw)
{
	if (!force_redraw && call_mask == g_prev_call_mask && current_floor == g_prev_queue_floor) {
		return;
	}

	if (force_redraw) {
		bsp_tft_lcd_draw_string(90, y - 11, "CALL QUEUE", LCD_COLOR_LIGHT_GRAY, LCD_COLOR_NAVY);
	}

	const int start_x = 18;
	const int box_w = 24;
	const int box_h = 24;
	const int gap = 6;

	for (int i = 0; i < 7; i++) {
		int bx = start_x + i * (box_w + gap);
		uint8_t floor_num = (uint8_t)(i + 1);
		bool is_called = (call_mask & (1U << i)) != 0U;
		bool is_current = (floor_num == current_floor);

		bool prev_called = (g_prev_call_mask & (1U << i)) != 0U;
		bool prev_current = (floor_num == g_prev_queue_floor);
		if (!force_redraw && is_called == prev_called && is_current == prev_current) {
			continue;
		}

		uint16_t bg = is_called ? LCD_COLOR_YELLOW : LCD_COLOR_DARK_GRAY;
		uint16_t text_c = is_called ? LCD_COLOR_BLACK : LCD_COLOR_WHITE;

		bsp_tft_lcd_fill_rect(bx, y, box_w, box_h, bg);
		bsp_tft_lcd_draw_char(bx + 9, y + 8, (char)('0' + floor_num), text_c, bg);

		if (is_current) {
			/* Đánh dấu tầng hiện tại bằng viền đôi màu xanh lá rực rỡ */
			bsp_tft_lcd_draw_rect(bx - 1, y - 1, box_w + 2, box_h + 2, LCD_COLOR_GREEN);
			bsp_tft_lcd_draw_rect(bx, y, box_w, box_h, LCD_COLOR_GREEN);
		} else {
			/* Xóa sạch viền xanh bên ngoài (bx - 1, y - 1) về màu nền NAVY khi thang rời tầng! */
			bsp_tft_lcd_draw_rect(bx - 1, y - 1, box_w + 2, box_h + 2, LCD_COLOR_NAVY);
			bsp_tft_lcd_draw_rect(bx, y, box_w, box_h,
					      is_called ? LCD_COLOR_WHITE : LCD_COLOR_GRAY);
		}
	}

	g_prev_call_mask = call_mask;
	g_prev_queue_floor = current_floor;
}

/**
 * @brief Cập nhật thanh Header trên cùng (0..25 px): Tên hệ thống + Giờ thực DS3231 RTC + Trạng thái mạng CAN.
 */
static void render_top_header(uint32_t now_ms, bool can_fault)
{
	if (!g_hdr_drawn) {
		bsp_tft_lcd_fill_rect(0, 0, LCD_WIDTH, 26, LCD_COLOR_DARK_GRAY);
		bsp_tft_lcd_draw_string(8, 9, "BKIT ELEVATOR", LCD_COLOR_WHITE, LCD_COLOR_DARK_GRAY);
		g_hdr_drawn = true;
	}

	if (now_ms - g_last_rtc_poll_ms >= 1000U || g_last_rtc_poll_ms == 0U) {
		g_last_rtc_poll_ms = now_ms;
		uint16_t y = 0U;
		uint8_t mo = 0U, d = 0U, h = 0U, mi = 0U, s = 0U;
		if (bsp_rtc_get_datetime(&y, &mo, &d, &h, &mi, &s) == 0) {
			g_rtc_valid = true;
			g_rtc_h = h;
			g_rtc_m = mi;
			g_rtc_s = s;
		} else {
			g_rtc_valid = false;
		}
	}

	uint32_t hh, mm, ss;
	if (g_rtc_valid) {
		hh = g_rtc_h;
		mm = g_rtc_m;
		ss = g_rtc_s;
	} else {
		uint32_t sec = now_ms / 1000U;
		hh = (sec / 3600U) % 24U;
		mm = (sec / 60U) % 60U;
		ss = sec % 60U;
	}

	char time_str[12];
	snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u", hh, mm, ss);
	lcd_draw_string_padded(104, 9, time_str, 8, LCD_COLOR_LIGHT_GRAY, LCD_COLOR_DARK_GRAY);

	if (can_fault) {
		lcd_draw_string_padded(178, 9, "[CAN:ERR]", 9, LCD_COLOR_RED, LCD_COLOR_DARK_GRAY);
	} else {
		lcd_draw_string_padded(178, 9, "[CAN:OK]", 9, LCD_COLOR_GREEN, LCD_COLOR_DARK_GRAY);
	}
}

void bsp_tft_lcd_animate_floor_rolling(uint8_t old_floor, uint8_t new_floor, elevator_dir_t dir)
{
	if (!g_lcd_ready || old_floor == new_floor || old_floor < 1U || old_floor > 9U) {
		return;
	}

	const int base_y = 52;
	const int h = 40;
	const int steps = 4;

	for (int s = 1; s <= steps; s++) {
		int dy = (s * h) / steps;
		bsp_tft_lcd_fill_rect(104, base_y, 32, h, LCD_COLOR_NAVY);

		if (dir >= 0) {
			lcd_draw_big_digit_clipped(108, base_y - dy, (char)('0' + old_floor),
						   base_y, base_y + h - 1, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
			lcd_draw_big_digit_clipped(108, base_y + h - dy, (char)('0' + new_floor),
						   base_y, base_y + h - 1, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
		} else {
			lcd_draw_big_digit_clipped(108, base_y + dy, (char)('0' + old_floor),
						   base_y, base_y + h - 1, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
			lcd_draw_big_digit_clipped(108, base_y - h + dy, (char)('0' + new_floor),
						   base_y, base_y + h - 1, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
		}
		k_msleep(10);
	}
}

void bsp_tft_lcd_render_dashboard(const elevator_telemetry_t *tel,
				  uint8_t call_mask,
				  uint8_t door_pct,
				  float load_kg,
				  float temp_c,
				  bool curtain_blocked,
				  const char *last_log_msg)
{
	if (!g_lcd_ready || tel == NULL) {
		return;
	}

	/* Xác định chế độ màn hình ưu tiên an toàn: E-Stop > Báo cháy > Quá tải > Bình thường */
	lcd_ui_mode_t target_mode = LCD_UI_NORMAL;
	if (tel->estop_active) {
		target_mode = LCD_UI_ESTOP;
	} else if (tel->fire_active) {
		target_mode = LCD_UI_FIRE;
	} else if (tel->overload_active) {
		target_mode = LCD_UI_OVERLOAD;
	}

	uint32_t now = k_uptime_get_32();
	bool mode_changed = (target_mode != g_ui_mode);

	if (!mode_changed && (now - g_last_render_uptime < 80U) &&
	    tel->current_floor == g_prev_floor && call_mask == g_prev_call_mask) {
		return;
	}
	g_last_render_uptime = now;

	/* Quy đổi sang số nguyên để định dạng nhanh, chính xác tuyệt đối, không bao giờ lỗi `*float*` */
	int load_i = (int)(load_kg + 0.5f);
	if (load_i < 0) {
		load_i = 0;
	}
	int temp_dc = (int)(temp_c * 10.0f + ((temp_c >= 0.0f) ? 0.5f : -0.5f));
	int temp_int = temp_dc / 10;
	int temp_frac = abs(temp_dc % 10);

	render_top_header(now, tel->can_fault_active);

	/* Khi chuyển qua lại giữa Dashboard và Popup sự cố -> Xóa sạch toàn bộ vùng thân màn hình! */
	if (mode_changed) {
		bsp_tft_lcd_fill_rect(0, 26, LCD_WIDTH, LCD_HEIGHT - 26, LCD_COLOR_NAVY);
		g_ui_mode = target_mode;
		g_prev_floor = 0xFFU;
		g_prev_call_mask = 0xFFU;
		g_prev_queue_floor = 0xFFU;

		if (target_mode == LCD_UI_NORMAL) {
			bsp_tft_lcd_draw_rect(10, 160, 220, 64, LCD_COLOR_DARK_GRAY);
			bsp_tft_lcd_draw_rect(10, 230, 220, 84, LCD_COLOR_DARK_GRAY);
		} else if (target_mode == LCD_UI_OVERLOAD) {
			bsp_tft_lcd_fill_rect(10, 38, 220, 104, LCD_COLOR_RED);
			bsp_tft_lcd_draw_rect(12, 40, 216, 100, LCD_COLOR_WHITE);
			bsp_tft_lcd_draw_string(36, 54, "!! OVERLOAD DETECTED !!", LCD_COLOR_WHITE, LCD_COLOR_RED);
			bsp_tft_lcd_draw_string(48, 76, "PLEASE STEP OUT NOW!", LCD_COLOR_YELLOW, LCD_COLOR_RED);
			bsp_tft_lcd_draw_string(36, 98, "MOTOR & DOOR INTERLOCKED", LCD_COLOR_WHITE, LCD_COLOR_RED);
			bsp_tft_lcd_draw_rect(10, 156, 220, 148, LCD_COLOR_RED);
		} else if (target_mode == LCD_UI_FIRE) {
			bsp_tft_lcd_fill_rect(10, 38, 220, 104, LCD_COLOR_RED);
			bsp_tft_lcd_draw_rect(12, 40, 216, 100, LCD_COLOR_WHITE);
			bsp_tft_lcd_draw_string(36, 54, "!! FIRE ALARM ACTIVE !!", LCD_COLOR_WHITE, LCD_COLOR_RED);
			bsp_tft_lcd_draw_string(27, 98, "EVACUATING CABIN TO FLOOR 1", LCD_COLOR_WHITE, LCD_COLOR_RED);
			bsp_tft_lcd_draw_rect(10, 156, 220, 148, LCD_COLOR_RED);
		} else if (target_mode == LCD_UI_ESTOP) {
			bsp_tft_lcd_fill_rect(10, 38, 220, 104, LCD_COLOR_ORANGE);
			bsp_tft_lcd_draw_rect(12, 40, 216, 100, LCD_COLOR_BLACK);
			bsp_tft_lcd_draw_string(33, 54, "EMERGENCY STOP ACTIVE!", LCD_COLOR_BLACK, LCD_COLOR_ORANGE);
			bsp_tft_lcd_draw_string(33, 76, "TRACTION MOTOR CUT OFF", LCD_COLOR_RED, LCD_COLOR_ORANGE);
			bsp_tft_lcd_draw_string(24, 98, "PRESS KEY [E] / CLI TO RESET", LCD_COLOR_BLACK, LCD_COLOR_ORANGE);
			bsp_tft_lcd_draw_rect(10, 156, 220, 148, LCD_COLOR_ORANGE);
		}
	}

	char buf[44];

	/* ================= MÀN HÌNH KHẨN CẤP 1: QUÁ TẢI CABIN ================= */
	if (target_mode == LCD_UI_OVERLOAD) {
		snprintf(buf, sizeof(buf), "CABIN FLOOR : F%u (DOOR %u%%)", tel->current_floor, door_pct);
		lcd_draw_string_padded(20, 170, buf, 32, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 186, 200, 12, door_pct, LCD_COLOR_ORANGE, LCD_COLOR_DARK_GRAY);

		snprintf(buf, sizeof(buf), "LOAD: %d / 800 kg [OVERLOAD]", load_i);
		lcd_draw_string_padded(20, 216, buf, 32, LCD_COLOR_RED, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 232, 200, 12, 100, LCD_COLOR_RED, LCD_COLOR_DARK_GRAY);

		lcd_draw_string_padded(20, 264, "REDUCE LOAD <= 750 kg TO CLEAR", 32,
				       LCD_COLOR_LIGHT_GRAY, LCD_COLOR_NAVY);
		return;
	}

	/* ================= MÀN HÌNH KHẨN CẤP 2: BÁO CHÁY SƠ TÁN ================= */
	if (target_mode == LCD_UI_FIRE) {
		snprintf(buf, sizeof(buf), "WELL TEMP: %d.%d C (>= 55.0 C)", temp_int, temp_frac);
		lcd_draw_string_padded(36, 76, buf, 28, LCD_COLOR_YELLOW, LCD_COLOR_RED);

		if (tel->current_floor > ELEV_MIN_FLOOR) {
			snprintf(buf, sizeof(buf), "EVACUATION: F%u -> F1 (DESCEND)", tel->current_floor);
		} else {
			snprintf(buf, sizeof(buf), "ARRIVED F1: SAFE EXIT OPEN %u%%", door_pct);
		}
		lcd_draw_string_padded(20, 170, buf, 32, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 188, 200, 12, door_pct, LCD_COLOR_RED, LCD_COLOR_DARK_GRAY);

		bsp_tft_lcd_draw_big_digit(108, 214, (char)('0' + tel->current_floor),
					   LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
		lcd_draw_string_padded(20, 272, "PRESS KEY [E] / CLEAR WHEN SAFE", 32,
				       LCD_COLOR_LIGHT_GRAY, LCD_COLOR_NAVY);
		return;
	}

	/* ================= MÀN HÌNH KHẨN CẤP 3: PHANH E-STOP ================= */
	if (target_mode == LCD_UI_ESTOP) {
		snprintf(buf, sizeof(buf), "LOCKED AT FLOOR: F%u (Y0=0 Y1=0)", tel->current_floor);
		lcd_draw_string_padded(20, 172, buf, 32, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);

		snprintf(buf, sizeof(buf), "EXTI LATENCY   : %u us (MAX %u)",
			 tel->last_exti_latency_us, tel->max_exti_latency_us);
		lcd_draw_string_padded(20, 198, buf, 32, LCD_COLOR_CYAN, LCD_COLOR_NAVY);

		snprintf(buf, sizeof(buf), "LOAD: %d kg | TEMP: %d.%d C", load_i, temp_int, temp_frac);
		lcd_draw_string_padded(20, 224, buf, 32, LCD_COLOR_WHITE, LCD_COLOR_NAVY);

		lcd_draw_string_padded(20, 264, "STATUS: FAIL_SAFE INTERLOCKED", 32,
				       LCD_COLOR_ORANGE, LCD_COLOR_NAVY);
		return;
	}

	/* ================= MÀN HÌNH DASHBOARD VẬN HÀNH 4 KHỐI ================= */
	lcd_draw_dir_badge(92, 34, (elevator_dir_t)tel->direction);

	if (g_prev_floor != tel->current_floor) {
		if (g_prev_floor != 0xFFU) {
			bsp_tft_lcd_animate_floor_rolling(g_prev_floor, tel->current_floor,
							  (elevator_dir_t)tel->direction);
		}
		bsp_tft_lcd_draw_big_digit(108, 52, (char)('0' + tel->current_floor),
					   LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
		g_prev_floor = tel->current_floor;
	}

	if (tel->target_floor > 0U) {
		snprintf(buf, sizeof(buf), "Target Floor: F%u", tel->target_floor);
	} else {
		snprintf(buf, sizeof(buf), "Target Floor: --");
	}
	lcd_draw_string_padded(72, 100, buf, 18, LCD_COLOR_LIGHT_GRAY, LCD_COLOR_NAVY);

	/* Hàng 7 ô gọi tầng căn giữa (xóa sạch viền xanh tầng cũ theo cả chiều Lên & Xuống) */
	draw_call_queue_row(128, call_mask, tel->current_floor, mode_changed);

	/* KHỐI 3: CỬA & RÈM QUANG CHỐNG KẸP */
	bool door_reopening_alert = curtain_blocked &&
				    (tel->state == HSM_STATE_DOOR_OPENING ||
				     tel->state == HSM_STATE_DOOR_OPEN ||
				     tel->state == HSM_STATE_DOOR_CLOSING);

	if (door_reopening_alert) {
		lcd_draw_string_padded(20, 168, "DOOR: OBSTACLE! RE-OPENING", 28,
				       LCD_COLOR_ORANGE, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 184, 200, 12, door_pct,
					      LCD_COLOR_ORANGE, LCD_COLOR_DARK_GRAY);
	} else {
		if (door_pct == 0U) {
			snprintf(buf, sizeof(buf), "DOOR: CLOSED            [0%%]");
		} else if (door_pct >= 100U) {
			snprintf(buf, sizeof(buf), "DOOR: OPEN            [100%%]");
		} else if (tel->state == HSM_STATE_DOOR_OPENING) {
			snprintf(buf, sizeof(buf), "DOOR: OPENING         [%3u%%]", door_pct);
		} else {
			snprintf(buf, sizeof(buf), "DOOR: CLOSING         [%3u%%]", door_pct);
		}
		lcd_draw_string_padded(20, 168, buf, 28, LCD_COLOR_CYAN, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 184, 200, 12, door_pct,
					      LCD_COLOR_CYAN, LCD_COLOR_DARK_GRAY);
	}

	if (curtain_blocked) {
		lcd_draw_string_padded(20, 204, "Light Curtain: BEAM BLOCKED", 28,
				       LCD_COLOR_RED, LCD_COLOR_NAVY);
	} else {
		lcd_draw_string_padded(20, 204, "Light Curtain: CLEAR (SAFE)", 28,
				       LCD_COLOR_GREEN, LCD_COLOR_NAVY);
	}

	/* KHỐI 4: TẢI TRỌNG, NHIỆT ĐỘ & NHẬT KÝ TRẠNG THÁI */
	int load_pct = (load_i * 100) / (int)ELEV_LOAD_OVERLOAD_KG;
	int bar_pct = (load_pct > 100) ? 100 : load_pct;
	snprintf(buf, sizeof(buf), "LOAD: %d/800 kg (%d%%)", load_i, load_pct);
	lcd_draw_string_padded(20, 238, buf, 28, LCD_COLOR_WHITE, LCD_COLOR_NAVY);
	bsp_tft_lcd_draw_progress_bar(20, 252, 200, 10, bar_pct,
				      (load_pct >= 85) ? LCD_COLOR_YELLOW : LCD_COLOR_GREEN,
				      LCD_COLOR_DARK_GRAY);

	snprintf(buf, sizeof(buf), "TEMP: %d.%d C (%s)",
		 temp_int, temp_frac, (temp_dc >= 480) ? "WARM" : "SAFE");
	lcd_draw_string_padded(20, 270, buf, 28, LCD_COLOR_WHITE, LCD_COLOR_NAVY);

	if (tel->can_fault_active) {
		lcd_draw_string_padded(20, 290, "STATE: FAIL_SAFE (CAN LOST)", 28,
				       LCD_COLOR_ORANGE, LCD_COLOR_NAVY);
	} else {
		snprintf(buf, sizeof(buf), "STATE: %.21s",
			 (last_log_msg != NULL && last_log_msg[0] != '\0') ? last_log_msg : "IDLE_CLOSED");
		lcd_draw_string_padded(20, 290, buf, 28, LCD_COLOR_LIGHT_GRAY, LCD_COLOR_NAVY);
	}
}
