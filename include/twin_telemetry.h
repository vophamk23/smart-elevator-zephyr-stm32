/**
 * @file twin_telemetry.h
 * @brief Bộ đóng gói dữ liệu telemetry dạng JSON ($TWIN) gửi PC Digital Twin & HIL Logger.
 */

#ifndef TWIN_TELEMETRY_H_
#define TWIN_TELEMETRY_H_

#include <stdbool.h>

/** @brief Bật hoặc tắt luồng xuất bản tin JSON $TWIN định kỳ qua cổng USART1. */
void twin_telemetry_set_enabled(bool en);

/** @brief Truy vấn trạng thái bật/tắt hiện tại của luồng dữ liệu Digital Twin. */
bool twin_telemetry_is_enabled(void);

/**
 * @brief Gom ảnh chụp trạng thái từ HSM, ADC1 và CAN1 để xuất ngay 1 dòng JSON $TWIN.
 */
void twin_telemetry_emit_now(void);

#endif /* TWIN_TELEMETRY_H_ */
