/**
 * @file bsp_tft_lcd.c
 * @brief Hiện thực Trình điều khiển Màn hình màu TFT-LCD 240x320 (ILI9341 - FSMC 16-bit)
 *        Hệ thống Thang máy Thông minh BKIT ARM4 (STM32F407ZGT6).
 *
 * Chức năng tổng quát của file:
 *  - Cấu hình phần cứng FSMC 16-bit (Bank 1 NOR/PSRAM 1) trên STM32F407ZGT6 nối chip ILI9341.
 *  - Cung cấp các hàm vẽ đồ họa 2D cơ bản (điểm, khối chữ nhật, chuỗi ký tự ASCII, số lớn, thanh tiến trình).
 *  - Quản lý giao diện HMI hiển thị thông tin buồng thang 240x320 pixel trực quan thời gian thực.
 *  - Thực hiện hiệu ứng hoạt họa cuộn số tầng dọc (Vertical Rolling Animation) mượt mà 30-50 FPS.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `bsp_tft_lcd_init()`               : Cấu hình GPIO Port A/C/D/E, thanh ghi FSMC BCR1/BTR1 và gửi chuỗi lệnh khởi động ILI9341.
 *  - `bsp_tft_lcd_clear()`              : Xóa sạch màn hình theo màu sắc tùy chọn.
 *  - `bsp_tft_lcd_fill_rect()`          : Tô màu khối chữ nhật qua giao tiếp nạp vùng cửa sổ FSMC.
 *  - `bsp_tft_lcd_draw_string()`        : Vẽ chuỗi ký tự theo font ASCII 6x8 tối ưu không tràn viền.
 *  - `bsp_tft_lcd_draw_big_digit()`     : Vẽ chữ số lớn 24x32 pixel phục vụ hiển thị số tầng trung tâm.
 *  - `bsp_tft_lcd_draw_progress_bar()`  : Vẽ thanh tiến trình đóng/mở cửa và cân tải trọng kèm viền phân cách.
 *  - `bsp_tft_lcd_render_dashboard()`   : Render toàn diện 4 khối giao diện (Header, Số tầng/Hàng đợi, Cửa/Rèm quang, Cảm biến/Hộp đen) và các popup sự cố (Quá tải, Báo cháy, E-Stop).
 *  - `bsp_tft_lcd_animate_floor_rolling()` : Hoạt họa cuộn trượt số tầng dọc khi cabin di chuyển qua các tầng.
 *
 * Sơ đồ chân phần cứng (Mục 5.3 QuickStartGuide BKIT ARM4):
 *  - Data Bus 16-bit: PD14(D0), PD15(D1), PD0(D2), PD1(D3), PE7..PE15(D4..D12), PD8..PD10(D13..D15)
 *  - Điều khiển: RS=PE3 (FSMC_A19), CS=PD7 (FSMC_NE1), WR=PD5 (FSMC_NWE), RD=PD4 (FSMC_NOE)
 *  - Tín hiệu phụ: RESET=PC13, BLK=PA8 (Đèn nền)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_tft_lcd.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <soc.h>

LOG_MODULE_REGISTER(bsp_lcd, LOG_LEVEL_INF);

/* 
 * Ánh xạ bộ nhớ FSMC Bank 1 (NOR/PSRAM 1)
 * RS kết nối PE3 (FSMC_A19):
 *  - Trong chế độ 16-bit bus, đường địa chỉ ngoài A19 tương ứng với bit địa chỉ trong Cortex-M là bit 20.
 *  - Địa chỉ ghi Lệnh (RS = 0): 0x60000000
 *  - Địa chỉ ghi Dữ liệu (RS = 1): 0x60000000 | (1 << 20) = 0x60100000
 */
#define LCD_CMD_ADDR    (*((volatile uint16_t *)0x60000000U))
#define LCD_DATA_ADDR   (*((volatile uint16_t *)0x60100000U))

static bool g_lcd_ready = false;
static uint8_t g_last_rendered_floor = 0xFFU;
static uint32_t g_last_render_uptime = 0U;

/* ============================================================================
 * BẢNG FONT CHỮ ASCII 6x8 (Tối ưu hiển thị nhãn, thông số trên màn hình 240x320)
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
};

/* Ghi lệnh tới ILI9341 */
static inline void lcd_write_cmd(uint16_t cmd)
{
	LCD_CMD_ADDR = cmd;
}

/* Ghi dữ liệu tới ILI9341 */
static inline void lcd_write_data(uint16_t data)
{
	LCD_DATA_ADDR = data;
}

/**
 * @brief Thiết lập cửa sổ vẽ (Active Window) trên ILI9341
 */
static void lcd_set_window(int x0, int y0, int x1, int y1)
{
	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= LCD_WIDTH) x1 = LCD_WIDTH - 1;
	if (y1 >= LCD_HEIGHT) y1 = LCD_HEIGHT - 1;

	lcd_write_cmd(0x2AU); /* Column Address Set */
	lcd_write_data((uint16_t)((x0 >> 8) & 0xFFU));
	lcd_write_data((uint16_t)(x0 & 0xFFU));
	lcd_write_data((uint16_t)((x1 >> 8) & 0xFFU));
	lcd_write_data((uint16_t)(x1 & 0xFFU));

	lcd_write_cmd(0x2BU); /* Page Address Set */
	lcd_write_data((uint16_t)((y0 >> 8) & 0xFFU));
	lcd_write_data((uint16_t)(y0 & 0xFFU));
	lcd_write_data((uint16_t)((y1 >> 8) & 0xFFU));
	lcd_write_data((uint16_t)(y1 & 0xFFU));

	lcd_write_cmd(0x2CU); /* Memory Write */
}

/**
 * @brief Khởi tạo phần cứng FSMC 16-bit và gửi tập lệnh chuẩn cho ILI9341
 */
int bsp_tft_lcd_init(void)
{
#if defined(CONFIG_SOC_SERIES_STM32F4X)
	/* 1. Bật clock cấp cho các GPIO Port A, C, D, E và bộ nhớ FSMC */
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN |
			 RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN);
	RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;

	/* 2. Cấu hình chân Đèn nền PA8 (GPIO Output Push-pull) */
	GPIOA->MODER &= ~(3U << (8 * 2));
	GPIOA->MODER |= (1U << (8 * 2));
	GPIOA->ODR |= (1U << 8); /* Bật sáng đèn nền */

	/* 3. Cấu hình chân RESET PC13 */
	GPIOC->MODER &= ~(3U << (13 * 2));
	GPIOC->MODER |= (1U << (13 * 2));
	GPIOC->ODR |= (1U << 13);
	k_busy_wait(10000U);
	GPIOC->ODR &= ~(1U << 13); /* Kéo Reset thấp */
	k_busy_wait(20000U);
	GPIOC->ODR |= (1U << 13);  /* Thả Reset cao */
	k_busy_wait(50000U);

	/* 
	 * 4. Cấu hình AF12 (FSMC) cho các chân Port D và Port E:
	 *  - PD0, PD1, PD4, PD5, PD7, PD8, PD9, PD10, PD14, PD15
	 *  - PE3, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15
	 */
	uint32_t pd_pins = (1U << 0) | (1U << 1) | (1U << 4) | (1U << 5) |
			   (1U << 7) | (1U << 8) | (1U << 9) | (1U << 10) |
			   (1U << 14) | (1U << 15);
	for (int i = 0; i < 16; i++) {
		if (pd_pins & (1U << i)) {
			GPIOD->MODER &= ~(3U << (i * 2));
			GPIOD->MODER |= (2U << (i * 2)); /* Alternate Function */
			GPIOD->OSPEEDR |= (3U << (i * 2)); /* Very High Speed */
			if (i < 8) {
				GPIOD->AFR[0] &= ~(0xFU << (i * 4));
				GPIOD->AFR[0] |= (12U << (i * 4)); /* AF12: FSMC */
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

	/* 5. Cấu hình thanh ghi FSMC Bank 1 (BCR1 & BTR1 cho SRAM/NOR 16-bit) */
	FSMC_Bank1->BTCR[0] = 0x00001059U; /* 16-bit, Write Enable, Async */
	FSMC_Bank1->BTCR[1] = 0x10000502U; /* Timing chu kỳ đọc/ghi tối ưu */
#endif

	/* 6. Chuỗi lệnh khởi động vi mạch ILI9341 */
	lcd_write_cmd(0x01U); /* Software Reset */
	k_msleep(50);

	lcd_write_cmd(0xC0U); /* Power Control 1 */
	lcd_write_data(0x26U);

	lcd_write_cmd(0xC1U); /* Power Control 2 */
	lcd_write_data(0x11U);

	lcd_write_cmd(0xC5U); /* VCOM Control 1 */
	lcd_write_data(0x35U);
	lcd_write_data(0x3EU);

	lcd_write_cmd(0x36U); /* Memory Access Control: Portrait 240x320 RGB */
	lcd_write_data(0x48U);

	lcd_write_cmd(0x3AU); /* Pixel Format Set: 16-bit RGB565 */
	lcd_write_data(0x55U);

	lcd_write_cmd(0xB1U); /* Frame Rate: 70Hz */
	lcd_write_data(0x00U);
	lcd_write_data(0x1BU);

	lcd_write_cmd(0x11U); /* Sleep Out */
	k_msleep(120);

	lcd_write_cmd(0x29U); /* Display ON */
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
	if (brightness_pct > 0) {
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
	if (x < 0) x = 0;
	if (y < 0) y = 0;
	if (x1 >= LCD_WIDTH) x1 = LCD_WIDTH - 1;
	if (y1 >= LCD_HEIGHT) y1 = LCD_HEIGHT - 1;

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
	if (c < ' ' || c > 'z') {
		c = ' ';
	}
	const uint8_t *glyph = font_6x8[c - ' '];
	lcd_set_window(x, y, x + 5, y + 7);

	for (int row = 0; row < 8; row++) {
		for (int col = 0; col < 6; col++) {
			if (glyph[col] & (1U << row)) {
				lcd_write_data(color);
			} else {
				lcd_write_data(bg_color);
			}
		}
	}
}

void bsp_tft_lcd_draw_string(int x, int y, const char *str, uint16_t color, uint16_t bg_color)
{
	if (str == NULL) return;
	int cur_x = x;
	while (*str && cur_x <= LCD_WIDTH - 6) {
		bsp_tft_lcd_draw_char(cur_x, y, *str, color, bg_color);
		cur_x += 6;
		str++;
	}
}

void bsp_tft_lcd_draw_big_digit(int x, int y, char digit, uint16_t color, uint16_t bg_color)
{
	if (digit < '0' || digit > '9') {
		digit = '-';
	}
	const uint8_t *glyph = font_6x8[digit - ' '];

	/* Phóng to font 6x8 lên gấp 4 lần (24x32 pixel) cực kỳ sắc nét */
	int scale_x = 4;
	int scale_y = 5;

	for (int col = 0; col < 6; col++) {
		uint8_t bits = glyph[col];
		for (int row = 0; row < 8; row++) {
			uint16_t c = (bits & (1U << row)) ? color : bg_color;
			bsp_tft_lcd_fill_rect(x + col * scale_x, y + row * scale_y, scale_x, scale_y, c);
		}
	}
}

void bsp_tft_lcd_draw_progress_bar(int x, int y, int w, int h, int pct, uint16_t bar_color, uint16_t bg_color)
{
	if (pct < 0) pct = 0;
	if (pct > 100) pct = 100;

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
 * @brief Vẽ 7 ô nút gọi tầng căn giữa hoàn hảo không tràn lề:
 *        Mỗi ô 24x24 px, khoảng cách 6 px. Tổng 204 px, lề mỗi bên 18 px.
 */
static void draw_call_queue_row(int y, uint8_t call_mask, uint8_t current_floor)
{
	bsp_tft_lcd_draw_string(84, y - 10, "CALL QUEUE", LCD_COLOR_LIGHT_GRAY, LCD_COLOR_NAVY);

	const int start_x = 18;
	const int box_w = 24;
	const int box_h = 24;
	const int gap = 6;

	for (int i = 0; i < 7; i++) {
		int bx = start_x + i * (box_w + gap);
		int floor_num = i + 1;
		bool is_called = (call_mask & (1U << i)) != 0U;
		bool is_current = (floor_num == current_floor);

		uint16_t bg = is_called ? LCD_COLOR_YELLOW : LCD_COLOR_DARK_GRAY;
		uint16_t text_c = is_called ? LCD_COLOR_BLACK : LCD_COLOR_WHITE;

		bsp_tft_lcd_fill_rect(bx, y, box_w, box_h, bg);
		bsp_tft_lcd_draw_char(bx + 9, y + 8, '0' + floor_num, text_c, bg);

		if (is_current) {
			bsp_tft_lcd_draw_rect(bx - 1, y - 1, box_w + 2, box_h + 2, LCD_COLOR_GREEN);
		} else {
			bsp_tft_lcd_draw_rect(bx, y, box_w, box_h, LCD_COLOR_GRAY);
		}
	}
}

/**
 * @brief Render toàn bộ Dashboard HMI theo trạng thái thời gian thực
 */
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

	uint32_t now = k_uptime_get_32();
	if (now - g_last_render_uptime < 80U && tel->current_floor == g_last_rendered_floor) {
		return; /* Giới hạn tốc độ quét tránh chiếm bus quá mức */
	}
	g_last_render_uptime = now;
	g_last_rendered_floor = tel->current_floor;

	/* ========================================================================
	 * TRƯỜNG HỢP 3: BÁO ĐỘNG QUÁ TẢI (OVERLOAD HAZARD > 800 KG)
	 * ======================================================================== */
	if (tel->overload_active || (double)load_kg > 800.0) {
		bsp_tft_lcd_fill_rect(10, 40, 220, 100, LCD_COLOR_RED);
		bsp_tft_lcd_draw_rect(12, 42, 216, 96, LCD_COLOR_WHITE);
		bsp_tft_lcd_draw_string(30, 55, "!! OVERLOAD DETECTED !!", LCD_COLOR_WHITE, LCD_COLOR_RED);
		bsp_tft_lcd_draw_string(45, 75, "PLEASE STEP OUT!", LCD_COLOR_YELLOW, LCD_COLOR_RED);
		bsp_tft_lcd_draw_string(35, 95, "DOOR CANNOT CLOSE", LCD_COLOR_WHITE, LCD_COLOR_RED);

		bsp_tft_lcd_draw_string(20, 160, "DOOR: LOCKED OPEN  [100%]", LCD_COLOR_RED, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 175, 200, 12, 100, LCD_COLOR_RED, LCD_COLOR_DARK_GRAY);

		char buf[40];
		snprintf(buf, sizeof(buf), "LOAD: %.0f kg / 800 kg [OVER]", (double)load_kg);
		bsp_tft_lcd_draw_string(20, 235, buf, LCD_COLOR_RED, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 250, 200, 12, 100, LCD_COLOR_RED, LCD_COLOR_DARK_GRAY);
		return;
	}

	/* ========================================================================
	 * TRƯỜNG HỢP 5: BÁO CHÁY KHẨN CẤP (FIRE ALARM SƠ TÁN VỀ TẦNG 1)
	 * ======================================================================== */
	if (tel->fire_active || (double)temp_c >= 60.0) {
		bsp_tft_lcd_fill_rect(10, 40, 220, 100, LCD_COLOR_RED);
		bsp_tft_lcd_draw_rect(12, 42, 216, 96, LCD_COLOR_WHITE);
		bsp_tft_lcd_draw_string(35, 55, "FIRE ALARM DETECTED!", LCD_COLOR_WHITE, LCD_COLOR_RED);
		char tbuf[32];
		snprintf(tbuf, sizeof(tbuf), "WELL TEMP: %.1f C", (double)temp_c);
		bsp_tft_lcd_draw_string(55, 75, tbuf, LCD_COLOR_YELLOW, LCD_COLOR_RED);
		bsp_tft_lcd_draw_string(28, 95, "EVACUATING TO FLOOR 1", LCD_COLOR_WHITE, LCD_COLOR_RED);

		bsp_tft_lcd_draw_string(20, 160, "MOTION: DIRECT TO F1", LCD_COLOR_RED, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 175, 200, 12, 100, LCD_COLOR_RED, LCD_COLOR_DARK_GRAY);
		return;
	}

	/* ========================================================================
	 * TRƯỜNG HỢP 6: DỪNG KHẨN CẤP (EMERGENCY STOP E-STOP)
	 * ======================================================================== */
	if (tel->estop_active) {
		bsp_tft_lcd_fill_rect(10, 40, 220, 100, LCD_COLOR_ORANGE);
		bsp_tft_lcd_draw_rect(12, 42, 216, 96, LCD_COLOR_BLACK);
		bsp_tft_lcd_draw_string(30, 55, "EMERGENCY STOP ACTIVE", LCD_COLOR_BLACK, LCD_COLOR_ORANGE);
		bsp_tft_lcd_draw_string(35, 75, "SYSTEM LOCKED BY USER", LCD_COLOR_RED, LCD_COLOR_ORANGE);
		bsp_tft_lcd_draw_string(25, 95, "CONTACT SERVICE OPERATOR", LCD_COLOR_BLACK, LCD_COLOR_ORANGE);
		return;
	}

	/* ========================================================================
	 * KHỐI 1: HEADER (0 -> 28 px)
	 * ======================================================================== */
	bsp_tft_lcd_fill_rect(0, 0, LCD_WIDTH, 26, LCD_COLOR_DARK_GRAY);
	bsp_tft_lcd_draw_string(10, 8, "BKIT ELEVATOR", LCD_COLOR_WHITE, LCD_COLOR_DARK_GRAY);

	uint32_t sec = now / 1000U;
	char time_str[16];
	snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u", (sec / 3600U) % 24U, (sec / 60U) % 60U, sec % 60U);
	bsp_tft_lcd_draw_string(115, 8, time_str, LCD_COLOR_LIGHT_GRAY, LCD_COLOR_DARK_GRAY);

	if (tel->can_fault_active) {
		bsp_tft_lcd_draw_string(185, 8, "[CAN:ERR]", LCD_COLOR_RED, LCD_COLOR_DARK_GRAY);
	} else {
		bsp_tft_lcd_draw_string(185, 8, "[CAN:OK]", LCD_COLOR_GREEN, LCD_COLOR_DARK_GRAY);
	}

	/* ========================================================================
	 * KHỐI 2: SỐ TẦNG & HÀNG NÚT GỌI TẦNG (30 -> 155 px)
	 * ======================================================================== */
	if (tel->direction == ELEV_DIR_UP) {
		bsp_tft_lcd_draw_string(100, 36, "▲  UP  ", LCD_COLOR_GREEN, LCD_COLOR_NAVY);
	} else if (tel->direction == ELEV_DIR_DOWN) {
		bsp_tft_lcd_draw_string(100, 36, "▼  DOWN", LCD_COLOR_ORANGE, LCD_COLOR_NAVY);
	} else {
		bsp_tft_lcd_draw_string(100, 36, "—  IDLE", LCD_COLOR_WHITE, LCD_COLOR_NAVY);
	}

	/* Vẽ số tầng lớn 24x40 pixel tại tọa độ X=108, Y=54 */
	bsp_tft_lcd_draw_big_digit(108, 54, '0' + tel->current_floor, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);

	char target_str[32];
	snprintf(target_str, sizeof(target_str), "Target Floor: %u", tel->target_floor);
	bsp_tft_lcd_draw_string(72, 102, target_str, LCD_COLOR_LIGHT_GRAY, LCD_COLOR_NAVY);

	/* 7 ô gọi tầng */
	draw_call_queue_row(128, call_mask, tel->current_floor);

	/* ========================================================================
	 * KHỐI 3: THANH TIẾN TRÌNH CỬA & RÈM QUANG (160 -> 225 px)
	 * ======================================================================== */
	bsp_tft_lcd_draw_rect(10, 160, 220, 65, LCD_COLOR_DARK_GRAY);

	if (curtain_blocked) {
		bsp_tft_lcd_draw_string(20, 168, "DOOR: OBSTACLE! RE-OPENING", LCD_COLOR_ORANGE, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 185, 200, 12, door_pct, LCD_COLOR_ORANGE, LCD_COLOR_DARK_GRAY);
		bsp_tft_lcd_draw_string(20, 205, "Light Curtain: BEAM BLOCKED", LCD_COLOR_RED, LCD_COLOR_NAVY);
	} else {
		char door_str[36];
		if (door_pct == 0) {
			snprintf(door_str, sizeof(door_str), "DOOR: CLOSED           [0%%]");
		} else if (door_pct >= 100) {
			snprintf(door_str, sizeof(door_str), "DOOR: OPEN           [100%%]");
		} else {
			snprintf(door_str, sizeof(door_str), "DOOR: MOVING          [%2d%%]", door_pct);
		}
		bsp_tft_lcd_draw_string(20, 168, door_str, LCD_COLOR_CYAN, LCD_COLOR_NAVY);
		bsp_tft_lcd_draw_progress_bar(20, 185, 200, 12, door_pct, LCD_COLOR_CYAN, LCD_COLOR_DARK_GRAY);
		bsp_tft_lcd_draw_string(20, 205, "Light Curtain: CLEAR (SAFE)", LCD_COLOR_GREEN, LCD_COLOR_NAVY);
	}

	/* ========================================================================
	 * KHỐI 4: CẢM BIẾN TẢI, NHIỆT ĐỘ & HỘP ĐEN (230 -> 315 px)
	 * ======================================================================== */
	bsp_tft_lcd_draw_rect(10, 230, 220, 85, LCD_COLOR_DARK_GRAY);

	char load_str[40];
	int load_pct = (int)(((double)load_kg * 100.0) / 800.0);
	snprintf(load_str, sizeof(load_str), "LOAD: %.0f/800 kg (%d%%)", (double)load_kg, load_pct);
	bsp_tft_lcd_draw_string(20, 238, load_str, LCD_COLOR_WHITE, LCD_COLOR_NAVY);
	bsp_tft_lcd_draw_progress_bar(20, 252, 200, 10, load_pct, (load_pct > 75) ? LCD_COLOR_YELLOW : LCD_COLOR_GREEN, LCD_COLOR_DARK_GRAY);

	char temp_str[36];
	snprintf(temp_str, sizeof(temp_str), "TEMP: %.1f C (SAFE)", (double)temp_c);
	bsp_tft_lcd_draw_string(20, 270, temp_str, LCD_COLOR_WHITE, LCD_COLOR_NAVY);

	if (last_log_msg != NULL && last_log_msg[0] != '\0') {
		char log_disp[40];
		snprintf(log_disp, sizeof(log_disp), "LOG : %.30s", last_log_msg);
		bsp_tft_lcd_draw_string(20, 290, log_disp, LCD_COLOR_LIGHT_GRAY, LCD_COLOR_NAVY);
	}
}

/**
 * @brief Hiệu ứng cuộn số mượt khi chuyển tầng (Vertical Rolling)
 */
void bsp_tft_lcd_animate_floor_rolling(uint8_t old_floor, uint8_t new_floor, elevator_dir_t dir)
{
	if (!g_lcd_ready || old_floor == new_floor) {
		return;
	}
	int base_y = 54;
	int h = 40;
	int steps = 6;

	for (int s = 1; s <= steps; s++) {
		int dy = (s * h) / steps;
		bsp_tft_lcd_fill_rect(104, base_y - 10, 32, h + 20, LCD_COLOR_NAVY);

		if (dir == ELEV_DIR_UP) {
			bsp_tft_lcd_draw_big_digit(108, base_y - dy, '0' + old_floor, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
			bsp_tft_lcd_draw_big_digit(108, base_y + h - dy, '0' + new_floor, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
		} else {
			bsp_tft_lcd_draw_big_digit(108, base_y + dy, '0' + old_floor, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
			bsp_tft_lcd_draw_big_digit(108, base_y - h + dy, '0' + new_floor, LCD_COLOR_YELLOW, LCD_COLOR_NAVY);
		}
		k_msleep(20);
	}
}
