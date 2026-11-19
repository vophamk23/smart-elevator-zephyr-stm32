/**
 * @file can_bus.h
 * @brief Trình quản lý mạng CAN1 (PB8/PB9, 500 kbps) hỗ trợ chạy Loopback 1 Kit hoặc mạng 2 Kit.
 */

#ifndef CAN_BUS_H_
#define CAN_BUS_H_

#include <stdbool.h>
#include <stdint.h>
#include "elevator_types.h"

/* Cấp phát CAN ID chuẩn 11-bit (ID càng nhỏ thì độ ưu tiên phân xử bus càng cao) */
#define CAN_ID_EMERGENCY_STOP   0x080U
#define CAN_ID_HALL_CALL        0x100U
#define CAN_ID_CABIN_STATUS     0x200U
#define CAN_ID_HEARTBEAT        0x300U
#define CAN_ID_HIL_PING         0x380U
#define CAN_ID_HIL_PONG         0x381U

/**
 * @brief Cấu hình CAN1 tốc độ 500 kbps, gắn 3 bộ lọc phần cứng theo đúng trình tự driver và bật bus.
 * @param mode CAN_LINK_MODE_LOOPBACK (tự kiểm 1 kit) hoặc CAN_LINK_MODE_NORMAL (mạng 2 kit).
 * @param role NODE_ROLE_CABIN (kit điều khiển chính) hoặc NODE_ROLE_HALL (kit bảng gọi tầng).
 * @return 0 nếu thành công, mã lỗi âm errno nếu bộ điều khiển CAN chưa sẵn sàng.
 */
int can_bus_init(can_link_mode_t mode, node_role_t role);

/** @brief Chuyển đổi qua lại giữa chế độ LOOPBACK và NORMAL trong lúc hệ thống đang chạy. */
int can_bus_set_mode(can_link_mode_t mode);

/** @brief Lấy chế độ hoạt động hiện tại của bộ điều khiển CAN1. */
can_link_mode_t can_bus_get_mode(void);

/** @brief Chuyển đổi vai trò nút phân tán giữa CABIN (Kit 1) và HALL (Kit 2). */
void can_bus_set_role(node_role_t role);

/** @brief Lấy vai trò nút phân tán hiện tại của bo mạch. */
node_role_t can_bus_get_role(void);

/** @brief Phát gói trạng thái cabin (0x200, nếu là CABIN) và gói Heartbeat Big-Endian (0x300). */
int can_bus_send_status(const elevator_telemetry_t *tel);

/** @brief Gửi lệnh gọi tầng hành lang (0x100) từ nút HALL sang nút CABIN qua bus CAN. */
int can_bus_send_hall_call(uint8_t floor, bool going_up);

/** @brief Phát khung cảnh báo khẩn cấp / báo cháy mức ưu tiên cao nhất (0x080). */
int can_bus_send_emergency(uint8_t reason_code, uint8_t floor);

/** @brief Gửi khung HIL PING (0x380) đóng gói Big-Endian để đo thời gian vòng RTT. */
int can_bus_send_ping(uint32_t seq);

/** @brief Gửi khung truy vấn từ xa CAN RTR hỏi trạng thái Cabin trên ID 0x200. */
int can_bus_request_cabin_rtr(void);

/**
 * @brief Bước nhận khung CAN chặn vô thời hạn (`k_msgq_get(..., K_FOREVER)`): phân loại RTR/Data
 *        và cấp token cho semaphore giám sát đối tác `g_can_partner_sem`.
 */
void can_bus_rx_step(void);

/**
 * @brief Chờ semaphore nhịp 250 ms từ `k_timer` ISR, phát gói status/HB và kiểm tra phục hồi BUS-OFF.
 * @param tel Ảnh chụp trạng thái cabin mới nhất dùng để đóng gói truyền đi.
 */
void can_bus_tx_and_recover_step(const elevator_telemetry_t *tel);

/**
 * @brief Tác vụ giám sát kết nối nút đối tác CAN: chặn tối đa 2000 ms trên `k_sem_take`,
 *        lập tức kích hoạt Fail-Safe (`EV_CAN_LINK_LOST`) khi gặp timeout `-EAGAIN`.
 */
void can_bus_partner_watchdog_step(void);

/** @brief Lấy thông số chẩn đoán phần cứng CAN (trạng thái, TEC/REC, tổng TX/RX và RTT micro-giây). */
void can_bus_get_diag(uint8_t *state, uint8_t *tec, uint8_t *rec,
		      uint32_t *rx_ok, uint32_t *tx_ok, uint32_t *last_rtt_us);

/** @brief Đọc bộ đếm nguyên tử callback TX không NULL (cbOK/cbErr) và số khung RTR đã nhận. */
void can_bus_get_ext_stats(uint32_t *tx_cb_ok, uint32_t *tx_cb_err, uint32_t *rtr_rx);

/**
 * @brief Đọc thông tin trạng thái cabin từ xa được phản chiếu trên kit chạy vai trò NODE_ROLE_HALL.
 * @return true nếu có dữ liệu hợp lệ mới nhận từ nút CABIN, false nếu chưa có hoặc sai vai trò.
 */
bool can_bus_get_remote_cabin_view(uint8_t *floor, elevator_dir_t *dir,
				   bool *fire, bool *fault);

/** @brief Bật hoặc tắt chế độ giả lập đứt dây CAN (gọi `k_sem_reset`) phục vụ kiểm thử Fail-Safe HIL. */
void can_bus_set_hil_drop(bool drop);

#endif /* CAN_BUS_H_ */
