/**
 * @file bsp_watchdog.h
 * @brief Trình điều khiển Watchdog phần cứng (IWDG) giám sát độ sống đa luồng & đọc HWINFO.
 *
 * Thay vì nạp IWDG một cách mù quáng ở một tác vụ đơn lẻ, mỗi luồng RTOS trọng yếu đều phải
 * điểm danh qua @ref bsp_wdt_checkin(). Hàm @ref bsp_wdt_service() chỉ cho phép reset bộ đếm
 * IWDG khi toàn bộ các luồng giám sát còn phản hồi trong ngưỡng thời gian quy định.
 */

#ifndef BSP_WATCHDOG_H_
#define BSP_WATCHDOG_H_

#include <stdbool.h>
#include <stdint.h>
#include "elevator_types.h"

/**
 * @brief Đọc nguyên nhân reset trước đó qua HWINFO, xóa cờ phần cứng và kích hoạt IWDG.
 * @return 0 nếu kích hoạt thành công (hoặc -ENODEV nếu phần cứng IWDG không sẵn sàng).
 */
int bsp_watchdog_init(void);

/**
 * @brief Ghi nhận dấu thời gian sống (checkin timestamp) từ từng luồng RTOS được giám sát.
 * @param id Định danh khe giám sát của luồng (@c WDT_MON_SAFETY .. @c WDT_MON_HMI).
 */
void bsp_wdt_checkin(wdt_mon_id_t id);

/**
 * @brief Kiểm tra mọi luồng giám sát đều còn hoạt động trong ngưỡng cho phép rồi mới nạp IWDG.
 * @param force_freeze Khi bằng true (chế độ test HIL), cố tình ngừng nạp IWDG để kiểm chứng tự reset MCU.
 * @return true nếu đã nạp IWDG hợp lệ, false nếu đang đóng băng test hoặc phát hiện có luồng bị treo.
 */
bool bsp_wdt_service(bool force_freeze);

/** @brief Trả về true nếu lần khởi động trước đó của MCU là do hết hạn Hardware Watchdog. */
bool bsp_wdt_was_reset_cause(void);

/**
 * @brief Lấy mặt nạ nguyên nhân reset đã lưu lúc khởi động và chuỗi UID 96-bit của STM32.
 * @param[out] cause   Mặt nạ bit cờ @c RESET_* trả về từ @c hwinfo_get_reset_cause().
 * @param[out] uid     Bộ đệm 12 byte chứa số định danh duy nhất của vi điều khiển.
 * @param[out] uid_len Số byte UID hợp lệ đã sao chép vào bộ đệm.
 */
void bsp_wdt_get_hwinfo(uint32_t *cause, uint8_t uid[12], int *uid_len);

/** @brief Quy đổi mã định danh khe giám sát watchdog sang tên luồng dễ đọc. */
const char *bsp_wdt_slot_name(wdt_mon_id_t id);

#endif /* BSP_WATCHDOG_H_ */
