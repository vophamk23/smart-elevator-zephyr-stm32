/**
 * @file elevator_types.h
 * @brief Định nghĩa các kiểu dữ liệu, enum trạng thái, sự kiện, lỗi và hằng số cốt lõi
 *        cho hệ thống thang máy thông minh 7 tầng trên bo mạch BKIT ARM4.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ELEVATOR_TYPES_H_
#define ELEVATOR_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#define ELEV_MIN_FLOOR          1
#define ELEV_MAX_FLOOR          7
#define ELEV_NUM_FLOORS         7

/* Timing constants (milliseconds) calibrated for real-time demo & HIL verification */
#define ELEV_TRAVEL_FLOOR_MS    1500U
#define ELEV_DOOR_MOVE_MS       700U
#define ELEV_DOOR_HOLD_MS       1800U
#define ELEV_CAN_HB_PERIOD_MS   250U
#define ELEV_CAN_TIMEOUT_MS     2000U
#define ELEV_WDT_TIMEOUT_MS     2500U

/* Temperature thresholds in decidegrees Celsius (0.1 degC) */
#define ELEV_TEMP_FIRE_THRESH_DC  550   /* 55.0 degC triggers Fire Evacuation */
#define ELEV_TEMP_CLEAR_THRESH_DC 480   /* Hysteresis reset threshold: 48.0 degC */

/* Ngưỡng tải trọng quá tải có trễ chống chập chờn (Hysteresis: bật >= 800 kg, tắt <= 750 kg) */
#define ELEV_LOAD_MAX_KG          1000U
#define ELEV_LOAD_OVERLOAD_KG     800U
#define ELEV_LOAD_CLEAR_KG        750U

typedef enum {
	HSM_STATE_INIT = 0,
	HSM_STATE_IDLE_CLOSED,
	HSM_STATE_DOOR_OPENING,
	HSM_STATE_DOOR_OPEN,
	HSM_STATE_DOOR_CLOSING,
	HSM_STATE_MOVING_UP,
	HSM_STATE_MOVING_DOWN,
	HSM_STATE_FIRE_EVAC,
	HSM_STATE_FAIL_SAFE,
	HSM_STATE_COUNT
} elevator_state_t;

typedef enum {
	ELEV_DIR_IDLE = 0,
	ELEV_DIR_UP   = 1,
	ELEV_DIR_DOWN = -1
} elevator_dir_t;

typedef enum {
	DOOR_POS_CLOSED = 0,
	DOOR_POS_OPENING,
	DOOR_POS_OPEN,
	DOOR_POS_CLOSING
} door_pos_t;

typedef enum {
	EV_NONE = 0,
	EV_TICK_100MS,
	EV_CALL_CABIN,
	EV_CALL_HALL_UP,
	EV_CALL_HALL_DOWN,
	EV_CMD_OPEN_DOOR,
	EV_CMD_CLOSE_DOOR,
	EV_DOOR_OBSTRUCT_EXTI,
	EV_ESTOP_EXTI,
	EV_OVERHEAT_TEMP,
	EV_TEMP_NORMAL,
	EV_OVERLOAD_SET,
	EV_OVERLOAD_CLEAR,
	EV_CAN_LINK_LOST,
	EV_CAN_LINK_RESTORED,
	EV_CLEAR_FAULT,
	EV_COUNT
} elevator_ev_type_t;

typedef enum {
	CAN_LINK_MODE_LOOPBACK = 0,
	CAN_LINK_MODE_NORMAL   = 1
} can_link_mode_t;

/** @brief Distributed bus node identity for single-kit or two-kit CAN operation. */
typedef enum {
	NODE_ROLE_CABIN = 0,    /* Kit 1: Executes HSM, motor interlock, sensors & status TX */
	NODE_ROLE_HALL  = 1     /* Kit 2: Hall panel calls TX (0x100) & mirrors cabin state */
} node_role_t;

/** @brief Supervised RTOS thread slots monitored by the software+IWDG watchdog. */
typedef enum {
	WDT_MON_SAFETY = 0,
	WDT_MON_FSM,
	WDT_MON_CAN,
	WDT_MON_HMI,
	WDT_MON_COUNT
} wdt_mon_id_t;

typedef struct {
	elevator_ev_type_t type;
	uint8_t floor;          /* Tầng gọi (1..7) */
	uint32_t isr_cycles;    /* Chu kỳ CPU DWT chụp ngay trong ngắt EXTI */
	uint32_t param;
} elevator_event_t;

typedef struct {
	int32_t temp_deci_c;    /* PC2 LM35 (0.1 degC) */
	uint16_t load_kg;       /* PC1 Biến trở quy đổi 0..1000 kg */
	uint16_t light_mv;      /* PC0 Quang trở (mV) */
	uint16_t bus_mv;        /* PB0 Điện áp nguồn (mV) */
	uint16_t current_mv;    /* PB1 Dòng tải (mV) */
	bool raw_ok;
} bkit_adc_snapshot_t;

typedef struct {
	uint8_t state;          /* elevator_state_t */
	uint8_t current_floor;  /* Tầng hiện tại 1..7 */
	uint8_t target_floor;   /* Tầng đích 1..7 (0 nếu đang rảnh) */
	int8_t direction;       /* elevator_dir_t */
	uint8_t door_pos;       /* door_pos_t */
	uint8_t door_pct;       /* 0=đóng kín .. 100=mở hoàn toàn */
	bool motor_up;          /* Ngõ ra thật PE5 (Y0) */
	bool motor_down;        /* Ngõ ra thật PE6 (Y1) */
	bool fire_active;
	bool estop_active;
	bool overload_active;
	bool can_fault_active;
	bool wdt_reset_flag;
	uint8_t cabin_mask;     /* bit 0..6 ứng với tầng 1..7 */
	uint8_t hall_up_mask;   /* bit 0..5 ứng với tầng 1..6 */
	uint8_t hall_down_mask; /* bit 1..6 ứng với tầng 2..7 */
	uint32_t uptime_ms;
	uint32_t trip_count;
	uint32_t transition_count;
	uint32_t safety_abort_count;
	uint32_t last_exti_latency_us;
	uint32_t max_exti_latency_us;
	int32_t temp_deci_c;
	uint16_t load_kg;
	uint16_t light_mv;
	uint8_t can_tec;
	uint8_t can_rec;
	uint8_t can_state;
	uint8_t can_mode;
	uint8_t node_role;      /* node_role_t (CABIN or HALL) */
} elevator_telemetry_t;

typedef enum {
	BB_FAULT_BOOT_POR = 1,
	BB_FAULT_BOOT_WATCHDOG,
	BB_FAULT_DOOR_OBSTRUCT,
	BB_FAULT_ESTOP,
	BB_FAULT_FIRE_OVERHEAT,
	BB_FAULT_OVERLOAD,
	BB_FAULT_CAN_LINK_LOST,
	BB_FAULT_CAN_BUS_OFF,
	BB_FAULT_CLEARED
} blackbox_fault_code_t;

typedef struct {
	uint32_t seq;
	uint32_t uptime_ms;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t fault_code;
	uint8_t state;
	uint8_t floor;
	int16_t temp_deci_c;
	uint16_t crc16;
} __packed elevator_blackbox_rec_t;

#endif /* ELEVATOR_TYPES_H_ */
