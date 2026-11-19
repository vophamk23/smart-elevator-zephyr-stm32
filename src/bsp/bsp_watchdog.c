/**
 * @file bsp_watchdog.c
 * @brief Trình điều khiển Watchdog phần cứng (IWDG) giám sát độ sống đa luồng & đọc định danh chip HWINFO.
 *
 * Chức năng tổng quát của file:
 *  - Kiểm tra nguyên nhân khởi động lại chip (do bật nguồn bình thường hay do bị Watchdog IWDG reset).
 *  - Thực thi cơ chế Watchdog đa luồng: chỉ nạp lại bộ đếm phần cứng IWDG (ngưỡng 2500 ms) khi tất cả
 *    4 luồng trọng yếu (`t1_safety`, `t2_fsm`, `t3_can_tx`, `t4_hmi`) đều báo cáo còn sống trong vòng 1500 ms.
 *
 * Các hàm quan trọng nhất trong file:
 *  - `bsp_watchdog_init()`       : Đọc cờ nguyên nhân reset từ HWINFO, cấu hình cửa sổ thời gian 2500 ms và bật IWDG.
 *  - `bsp_wdt_checkin()`         : Để từng luồng RTOS gọi điểm danh (cập nhật mốc `k_uptime_get()` của luồng đó).
 *  - `bsp_wdt_service()`         : Kiểm tra cả 4 luồng; nếu không có luồng nào bị treo quá 1500 ms thì mới nạp `wdt_feed()`.
 *  - `bsp_wdt_was_reset_cause()` : Trả về `true` nếu lần khởi động vừa rồi là do lỗi treo máy bị Watchdog reset.
 */

#include "bsp_watchdog.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bsp_wdt, LOG_LEVEL_INF);

/* Ngưỡng thời gian tối đa các luồng giám sát phải điểm danh trước khi cắt nạp IWDG (2500 ms) */
#define THREAD_STALL_TIMEOUT_MS 1500LL

static const struct device *const wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));

static int g_wdt_channel_id = -1;
static bool g_wdt_ready;
static bool g_was_wdt_reset;
static uint32_t g_reset_cause;

static int64_t g_last_checkin_ms[WDT_MON_COUNT];
static struct k_spinlock g_wdt_lock;

/**
 * @brief Đọc và xóa nguyên nhân reset trong HWINFO, đặt mốc checkin ban đầu và kích hoạt IWDG.
 */
int bsp_watchdog_init(void)
{
	/* Đọc nguyên nhân reset cũ và xóa ngay thanh ghi cờ để lần bật nguồn sau không nhận nhầm */
	if (hwinfo_get_reset_cause(&g_reset_cause) == 0) {
		g_was_wdt_reset = (g_reset_cause & RESET_WATCHDOG) != 0U;
		(void)hwinfo_clear_reset_cause();
	}

	int64_t now = k_uptime_get();
	for (size_t i = 0; i < WDT_MON_COUNT; i++) {
		g_last_checkin_ms[i] = now;
	}

	if (!device_is_ready(wdt_dev)) {
		LOG_WRN("IWDG watchdog device not ready");
		return -ENODEV;
	}

	/* IWDG của STM32 reset trực tiếp SoC khi hết hạn (không hỗ trợ callback trước reset) */
	struct wdt_timeout_cfg cfg = {
		.flags = WDT_FLAG_RESET_SOC,
		.window.min = 0U,
		.window.max = ELEV_WDT_TIMEOUT_MS,
		.callback = NULL,
	};

	g_wdt_channel_id = wdt_install_timeout(wdt_dev, &cfg);
	if (g_wdt_channel_id < 0) {
		LOG_WRN("IWDG timeout install failed (%d)", g_wdt_channel_id);
		return g_wdt_channel_id;
	}

	/* Tạm dừng đếm IWDG khi CPU đang bị chặn ở breakpoint SWD để không tự reset lúc debug */
	int err = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (err < 0) {
		LOG_WRN("IWDG setup failed (%d)", err);
		return err;
	}

	g_wdt_ready = true;
	(void)wdt_feed(wdt_dev, g_wdt_channel_id);
	LOG_INF("IWDG armed (%u ms, WDT_RESET=%d)", ELEV_WDT_TIMEOUT_MS, g_was_wdt_reset);
	return 0;
}

/**
 * @brief Cập nhật dấu thời gian sống mới nhất cho một khe luồng RTOS (dùng spinlock an toàn).
 */
void bsp_wdt_checkin(wdt_mon_id_t id)
{
	if ((size_t)id >= WDT_MON_COUNT) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&g_wdt_lock);
	g_last_checkin_ms[id] = k_uptime_get();
	k_spin_unlock(&g_wdt_lock, key);
}

/**
 * @brief Quét tất cả khe luồng; chỉ cho phép gọi wdt_feed() nếu không có luồng nào treo > 1500 ms.
 */
bool bsp_wdt_service(bool force_freeze)
{
	if (force_freeze || !g_wdt_ready || g_wdt_channel_id < 0) {
		return false;
	}

	/*
	 * Watchdog phần cứng STM32 (IWDG) chạy bằng xung độc lập LSI:
	 *  - Chỉ được phép nạp lại (`wdt_feed`) khi TẤT CẢ các luồng t1..t4 đều báo "còn sống"
	 *    trong vòng 1500 ms gần nhất (THREAD_STALL_TIMEOUT_MS).
	 *  - Dùng `k_spinlock` thay mutex để thao tác đọc/ghi mảng timestamp diễn ra tức thời.
	 */
	int64_t now = k_uptime_get();
	int stalled_id = -1;

	k_spinlock_key_t key = k_spin_lock(&g_wdt_lock);
	for (int i = 0; i < (int)WDT_MON_COUNT; i++) {
		if ((now - g_last_checkin_ms[i]) > THREAD_STALL_TIMEOUT_MS) {
			stalled_id = i;
			break;
		}
	}
	k_spin_unlock(&g_wdt_lock, key);

	if (stalled_id >= 0) {
		LOG_ERR("Thread stall detected in %s -> suppressing IWDG feed!",
			bsp_wdt_slot_name((wdt_mon_id_t)stalled_id));
		return false;
	}

	(void)wdt_feed(wdt_dev, g_wdt_channel_id);
	return true;
}

/**
 * @brief Kiểm tra cờ đã lưu để biết lần khởi động hiện tại có phải do Watchdog reset hay không.
 */
bool bsp_wdt_was_reset_cause(void)
{
	return g_was_wdt_reset;
}

/**
 * @brief Trích xuất thanh ghi nguyên nhân reset đã chốt và số định danh silicon UID 96-bit.
 */
void bsp_wdt_get_hwinfo(uint32_t *cause, uint8_t uid[12], int *uid_len)
{
	if (cause != NULL) {
		*cause = g_reset_cause;
	}
	if (uid != NULL && uid_len != NULL) {
		memset(uid, 0, 12U);
		*uid_len = (int)hwinfo_get_device_id(uid, 12U);
	}
}

/**
 * @brief Ánh xạ mã khe giám sát watchdog sang tên chuỗi của luồng RTOS tương ứng.
 */
const char *bsp_wdt_slot_name(wdt_mon_id_t id)
{
	switch (id) {
	case WDT_MON_SAFETY: return "t1_safety";
	case WDT_MON_FSM:    return "t2_fsm";
	case WDT_MON_CAN:    return "t3_can";
	case WDT_MON_HMI:    return "t4_hmi";
	default:             return "unknown";
	}
}
