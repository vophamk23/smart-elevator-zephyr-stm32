"""
twin_simulator.py - Bộ mô phỏng Máy trạng thái HSM 7 tầng & Cảm biến Offline trên PC.

Chức năng tổng quát của file:
  - Cho phép người dùng chạy thử toàn bộ kịch bản thang máy 7 tầng (SCAN/LOOK, kẹt cửa EXTI,
    báo cháy LM35 hysteresis, quá tải 800 kg, đứt liên kết CAN Fail-Safe 2000 ms) ngay trên PC
    ngay cả khi chưa kết nối cáp RS232 với bo mạch BKIT ARM4 thật.

Các hàm / phương thức quan trọng nhất trong file:
  - `OfflineElevatorSimulator.step()`           : Nhịp mô phỏng 50 ms cập nhật vị trí cabin, cửa, cảm biến và chuyển trạng thái HSM.
  - `OfflineElevatorSimulator.execute_cli()`    : Thông dịch các câu lệnh `elev ...` và `bkit ...` khi chạy chế độ Offline.
  - `OfflineElevatorSimulator.trigger_obstruct()`: Xử lý kẹt cửa X0 làm cửa dừng đóng và đảo chiều mở ra ngay lập tức.
  - `OfflineElevatorSimulator.set_fire()`       : Kích hoạt hoặc giải trừ chế độ báo cháy sơ tán về Tầng 1.
  - `OfflineElevatorSimulator.set_estop()`      : Kích hoạt phanh dừng khẩn cấp E-Stop chuyển sang `FAIL_SAFE`.
"""

import time
from typing import Any, Callable, Dict, Optional

from twin_config import (
    CAN_TIMEOUT_MS,
    DOOR_HOLD_MS,
    DOOR_MOVE_MS,
    LOAD_CLEAR_KG,
    LOAD_OVERLOAD_KG,
    MAX_FLOOR,
    MIN_FLOOR,
    TEMP_CLEAR_THRESH_C,
    TEMP_FIRE_THRESH_C,
    TRAVEL_FLOOR_MS,
    default_telemetry,
)
from twin_scheduler import ScanLookScheduler


class OfflineElevatorSimulator:
    """Mô phỏng hành vi thời gian thực của firmware HSM 7 tầng trên máy tính."""

    def __init__(self, log_cb: Optional[Callable[[str, str], None]] = None) -> None:
        """Khởi tạo cabin tại Tầng 1, trạng thái IDLE_CLOSED cùng bộ lập lịch LOOK."""
        self.log_cb = log_cb
        self.sched = ScanLookScheduler()

        self.start_ms = int(time.monotonic() * 1000)
        self.state = "IDLE_CLOSED"
        self.current_floor = MIN_FLOOR
        self.pos_float = float(MIN_FLOOR)   # Vị trí liên tục (1.00 .. 7.00) để vẽ chuyển động mượt
        self.target_floor = 0
        self.direction = 0                  # 1: UP, -1: DOWN, 0: IDLE
        self.scan_dir = 0                   # Nhớ hướng quét LOOK xuyên qua nhịp mở/đóng cửa
        self.door_pos = "CLOSED"            # CLOSED / OPENING / OPEN / CLOSING
        self.door_pct = 0
        self.phase_elapsed_ms = 0

        # Cờ an toàn & cảm biến mô phỏng
        self.fire_active = False
        self.estop_active = False
        self.overload_active = False
        self.can_fault_active = False
        self.can_drop_injected = False
        self.can_drop_elapsed_ms = 0

        self.temp_c = 27.0
        self.load_kg = 150
        self.trips = 0
        self.last_lat_us = 0
        self.obstruct_flash_ticks = 0

    def _emit_log(self, tag: str, msg: str) -> None:
        """Gửi thông báo sự kiện mô phỏng ra khung Console nếu có callback."""
        if self.log_cb is not None:
            self.log_cb(tag, msg)

    def _transition(self, next_state: str) -> None:
        """Chuyển trạng thái HSM kèm entry action (khóa liên động, cập nhật cửa, xóa tầng đến)."""
        if self.state == next_state:
            return
        self._emit_log("SIM", f"HSM: {self.state} -> {next_state} (F{self.current_floor})")
        self.state = next_state
        self.phase_elapsed_ms = 0

        if next_state == "IDLE_CLOSED":
            self.door_pos = "CLOSED"
            self.door_pct = 0
            self.direction = 0
        elif next_state == "DOOR_OPENING":
            self.door_pos = "OPENING"
        elif next_state == "DOOR_OPEN":
            self.door_pos = "OPEN"
            self.door_pct = 100
            self.sched.clear_floor(self.current_floor, self.scan_dir)
        elif next_state == "DOOR_CLOSING":
            self.door_pos = "CLOSING"
        elif next_state == "MOVING_UP":
            self.direction = 1
            self.scan_dir = 1
            self.door_pos = "CLOSED"
            self.door_pct = 0
        elif next_state == "MOVING_DOWN":
            self.direction = -1
            self.scan_dir = -1
            self.door_pos = "CLOSED"
            self.door_pct = 0
        elif next_state == "FIRE_EVAC":
            self.sched.reset()
            self.target_floor = MIN_FLOOR
            self.direction = 0
            self.scan_dir = -1
            if self.current_floor > MIN_FLOOR and self.door_pos != "CLOSED":
                self.phase_elapsed_ms = int(((100 - self.door_pct) * DOOR_MOVE_MS) / 100)
        elif next_state == "FAIL_SAFE":
            self.direction = 0
            self.scan_dir = 0

    def execute_cli(self, cmd: str) -> None:
        """
        Thông dịch chuỗi lệnh dạng CLI (`elev call ...`, `elev door ...`, `elev fault ...`)
        để cùng 1 bộ nút bấm UI điều khiển được cả STM32 thật lẫn Simulator Offline.
        """
        parts = cmd.strip().split()
        if len(parts) < 2 or parts[0] != "elev":
            return

        sub = parts[1]
        if sub == "call" and len(parts) >= 3:
            floor = int(parts[2])
            ctype = parts[3] if len(parts) >= 4 else "cabin"
            if not self.fire_active and not self.estop_active:
                ok = self.sched.add_request(floor, ctype)
                if ok:
                    if self.state in ("MOVING_UP", "MOVING_DOWN"):
                        nxt, nxt_d = self.sched.pick_next_target(self.current_floor, self.direction)
                        if nxt and nxt_d == self.direction:
                            self.target_floor = nxt
                    self._emit_log("SIM", f"Đã nhận cuộc gọi F{floor} ({ctype})")

        elif sub == "door" and len(parts) >= 3:
            action = parts[2]
            if action == "open":
                if self.state == "IDLE_CLOSED":
                    self._transition("DOOR_OPENING")
                elif self.state == "DOOR_CLOSING":
                    resume_pct = self.door_pct
                    self._transition("DOOR_OPENING")
                    self.phase_elapsed_ms = int((resume_pct * DOOR_MOVE_MS) / 100)
                    self.door_pct = resume_pct
                elif self.state == "DOOR_OPEN":
                    self.phase_elapsed_ms = 0
            elif action == "close" and self.state == "DOOR_OPEN":
                if not self.overload_active and not self.fire_active:
                    self._transition("DOOR_CLOSING")
            elif action == "obstruct":
                self.last_lat_us = 18  # Độ trễ ngắt EXTI danh nghĩa trong mô phỏng
                self.obstruct_flash_ticks = 70
                if self.state == "DOOR_CLOSING":
                    resume_pct = self.door_pct
                    self._transition("DOOR_OPENING")
                    self.phase_elapsed_ms = int((resume_pct * DOOR_MOVE_MS) / 100)
                    self.door_pct = resume_pct
                    self._emit_log("WARN", "Kẹt cửa X0! Đảo chiều mở cửa khẩn cấp.")
                elif self.state in ("DOOR_OPEN", "DOOR_OPENING"):
                    if self.state == "DOOR_OPEN":
                        self.phase_elapsed_ms = 0
                    self._emit_log("WARN", "Kẹt cửa X0 khi đang mở -> Gia hạn thời gian giữ cửa.")
                elif self.state == "IDLE_CLOSED" and not self.fire_active and not self.estop_active:
                    demo_pct = 35
                    self._transition("DOOR_OPENING")
                    self.phase_elapsed_ms = int((demo_pct * DOOR_MOVE_MS) / 100)
                    self.door_pct = demo_pct
                    self._emit_log("WARN", "Mô phỏng kẹt cửa X0 -> Mở ngược từ 35% lên 100%.")

        elif sub == "fault" and len(parts) >= 3:
            kind = parts[2]
            if kind == "fire":
                self.temp_c = 62.5
                self.fire_active = True
                self._transition("FIRE_EVAC")
                self._emit_log("ERR", "BÁO CHÁY (>55°C): Hủy hàng đợi, sơ tán thẳng về Tầng 1!")
            elif kind == "overload":
                self.load_kg = 920
                self.overload_active = True
                if self.state == "IDLE_CLOSED":
                    self._transition("DOOR_OPENING")
                elif self.state == "DOOR_CLOSING":
                    resume_pct = self.door_pct
                    self._transition("DOOR_OPENING")
                    self.phase_elapsed_ms = int((resume_pct * DOOR_MOVE_MS) / 100)
                    self.door_pct = resume_pct
                self._emit_log("WARN", "QUÁ TẢI (>=800 kg): Mở cửa & khóa cứng mô-tơ an toàn!")
            elif kind == "can_drop":
                self.can_drop_injected = True
                self.can_drop_elapsed_ms = 0
                self._emit_log("WARN", f"Ngắt dây CAN: sẽ kích hoạt FAIL_SAFE sau {CAN_TIMEOUT_MS} ms...")
            elif kind == "estop":
                self.estop_active = True
                self.last_lat_us = 12
                self._transition("FAIL_SAFE")
                self._emit_log("ERR", "E-STOP (X1) KÍCH HOẠT: Cắt động cơ tức thời!")
            elif kind == "clear":
                self.fire_active = False
                self.estop_active = False
                self.overload_active = False
                self.can_fault_active = False
                self.can_drop_injected = False
                self.can_drop_elapsed_ms = 0
                self.temp_c = 27.0
                self.load_kg = 150
                if self.door_pct > 0 and self.door_pos != "CLOSED":
                    cur_pct = self.door_pct
                    self._transition("DOOR_CLOSING")
                    self.phase_elapsed_ms = int(((100 - cur_pct) * DOOR_MOVE_MS) / 100)
                    self.door_pct = cur_pct
                else:
                    self._transition("IDLE_CLOSED")
                self._emit_log("OK", "Đã xóa toàn bộ chốt lỗi -> khôi phục an toàn.")

        elif sub == "sensor" and len(parts) >= 2:
            if parts[2] == "auto":
                self.update_sensors(27.0, 150)
            elif len(parts) >= 4:
                if parts[2] == "load":
                    kg = self.load_kg if parts[3] == "auto" else int(parts[3])
                    self.update_sensors(self.temp_c, kg)
                elif parts[2] == "temp":
                    tc = self.temp_c if parts[3] == "auto" else (float(parts[3]) / 10.0)
                    self.update_sensors(tc, self.load_kg)

    def update_sensors(self, temp_c: float, load_kg: int) -> None:
        """Cập nhật giá trị cảm biến từ thanh trượt UI kèm kiểm tra hysteresis cháy & quá tải."""
        self.temp_c = temp_c
        self.load_kg = load_kg

        # Hysteresis báo cháy (>= 55.0 C bật, <= 48.0 C tắt) khớp logic t1_safety trong main.c
        if not self.fire_active and temp_c >= TEMP_FIRE_THRESH_C:
            self.fire_active = True
            self._transition("FIRE_EVAC")
            self._emit_log("ERR", f"LM35={temp_c:.1f}°C >= 55°C -> Chuyển FIRE_EVAC!")
        elif self.fire_active and temp_c <= TEMP_CLEAR_THRESH_C:
            self.fire_active = False
            if not self.estop_active and not self.can_fault_active:
                self._transition("IDLE_CLOSED")
                self._emit_log("OK", f"LM35={temp_c:.1f}°C <= 48°C -> Thoát cảnh báo cháy.")

        # Quá tải cơ khí có ngưỡng trễ Hysteresis (bật >= 800 kg, tắt <= 750 kg)
        if not self.overload_active and load_kg >= LOAD_OVERLOAD_KG:
            self.overload_active = True
            if self.state == "IDLE_CLOSED":
                self._transition("DOOR_OPENING")
            elif self.state == "DOOR_CLOSING":
                resume_pct = self.door_pct
                self._transition("DOOR_OPENING")
                self.phase_elapsed_ms = int((resume_pct * DOOR_MOVE_MS) / 100)
                self.door_pct = resume_pct
            self._emit_log("WARN", f"Tải={load_kg} kg >= 800 kg -> Mở cửa & khóa mô-tơ!")
        elif self.overload_active and load_kg <= LOAD_CLEAR_KG:
            self.overload_active = False
            if self.state == "DOOR_OPEN":
                self.phase_elapsed_ms = 0
            self._emit_log("OK", f"Tải={load_kg} kg <= 750 kg -> Cho phép đóng cửa.")

    def _step_fire_evac(self, dt_ms: int) -> None:
        """Xử lý 2 pha trong trạng thái sơ tán hỏa hoạn: đóng cửa rồi hạ một mạch về Tầng 1."""
        if self.door_pos != "CLOSED":
            if self.current_floor > MIN_FLOOR:
                pct = int((self.phase_elapsed_ms * 100) / DOOR_MOVE_MS)
                self.door_pct = 0 if pct >= 100 else (100 - pct)
                if self.phase_elapsed_ms >= DOOR_MOVE_MS:
                    self.door_pos = "CLOSED"
                    self.door_pct = 0
                    self.phase_elapsed_ms = 0
            else:
                self.door_pos = "OPEN"
                self.door_pct = 100
            return

        if self.current_floor > MIN_FLOOR:
            self.direction = -1
            # Nội suy vị trí mượt từ tầng hiện tại xuống tầng dưới
            frac = min(1.0, self.phase_elapsed_ms / TRAVEL_FLOOR_MS)
            self.pos_float = float(self.current_floor) - frac
            if self.phase_elapsed_ms >= TRAVEL_FLOOR_MS:
                self.phase_elapsed_ms = 0
                self.current_floor -= 1
                self.pos_float = float(self.current_floor)
                self.trips += 1
        else:
            self.direction = 0
            self.pos_float = float(MIN_FLOOR)
            self.door_pos = "OPEN"
            self.door_pct = 100

    def step(self, dt_ms: int) -> Dict[str, Any]:
        """Thực thi 1 bước thời gian mô phỏng (`dt_ms`) và trả về gói snapshot `$TWIN`."""
        self.phase_elapsed_ms += dt_ms
        if self.obstruct_flash_ticks > 0:
            self.obstruct_flash_ticks -= 1

        # Mô phỏng đếm ngược CAN watchdog 2000 ms khi bấm "Đứt CAN Link"
        if self.can_drop_injected and not self.can_fault_active:
            self.can_drop_elapsed_ms += dt_ms
            if self.can_drop_elapsed_ms >= CAN_TIMEOUT_MS:
                self.can_fault_active = True
                if not self.fire_active:
                    self._transition("FAIL_SAFE")
                    self._emit_log("ERR", "CAN Watchdog hết hạn 2000 ms -> Đã vào FAIL_SAFE!")

        if self.state == "IDLE_CLOSED":
            self.pos_float = float(self.current_floor)
            if self.overload_active:
                self._transition("DOOR_OPENING")
            else:
                tgt, nxt_dir = self.sched.pick_next_target(self.current_floor, self.scan_dir)
                if tgt == 0:
                    self.scan_dir = 0
                    self.target_floor = 0
                elif tgt == self.current_floor:
                    self.target_floor = tgt
                    self._transition("DOOR_OPENING")
                else:
                    self.scan_dir = nxt_dir
                    self.target_floor = tgt
                    self._transition("MOVING_UP" if nxt_dir > 0 else "MOVING_DOWN")

        elif self.state == "DOOR_OPENING":
            self.door_pct = min(100, int((self.phase_elapsed_ms * 100) / DOOR_MOVE_MS))
            if self.phase_elapsed_ms >= DOOR_MOVE_MS:
                self._transition("DOOR_OPEN")

        elif self.state == "DOOR_OPEN":
            if self.fire_active and self.current_floor == MIN_FLOOR:
                pass
            elif not self.overload_active and self.phase_elapsed_ms >= DOOR_HOLD_MS:
                self._transition("DOOR_CLOSING")

        elif self.state == "DOOR_CLOSING":
            if self.overload_active:
                resume_pct = self.door_pct
                self._transition("DOOR_OPENING")
                self.phase_elapsed_ms = int((resume_pct * DOOR_MOVE_MS) / 100)
                self.door_pct = resume_pct
            else:
                pct = int((self.phase_elapsed_ms * 100) / DOOR_MOVE_MS)
                self.door_pct = 0 if pct >= 100 else (100 - pct)
                if self.phase_elapsed_ms >= DOOR_MOVE_MS:
                    if self.fire_active:
                        self._transition("FIRE_EVAC")
                    elif self.can_fault_active or self.estop_active:
                        self._transition("FAIL_SAFE")
                    else:
                        self._transition("IDLE_CLOSED")

        elif self.state in ("MOVING_UP", "MOVING_DOWN"):
            step_dir = 1 if self.state == "MOVING_UP" else -1
            frac = min(1.0, self.phase_elapsed_ms / TRAVEL_FLOOR_MS)
            self.pos_float = float(self.current_floor) + step_dir * frac

            if self.phase_elapsed_ms >= TRAVEL_FLOOR_MS:
                self.phase_elapsed_ms = 0
                if step_dir > 0 and self.current_floor < MAX_FLOOR:
                    self.current_floor += 1
                elif step_dir < 0 and self.current_floor > MIN_FLOOR:
                    self.current_floor -= 1
                self.pos_float = float(self.current_floor)
                self.trips += 1

                if (
                    self.sched.should_stop(self.current_floor, self.direction)
                    or self.current_floor == self.target_floor
                ):
                    self._transition("DOOR_OPENING")

        elif self.state == "FIRE_EVAC":
            self._step_fire_evac(dt_ms)

        elif self.state == "FAIL_SAFE":
            if self.can_fault_active and not self.estop_active and self.door_pct < 100:
                self.door_pos = "OPENING"
                self.door_pct = min(100, int((self.phase_elapsed_ms * 100) / DOOR_MOVE_MS))
                if self.door_pct >= 100:
                    self.door_pos = "OPEN"

        return self.snapshot()

    def snapshot(self) -> Dict[str, Any]:
        """Xuất cấu trúc dữ liệu đồng nhất 100% với bản tin `$TWIN` của STM32."""
        snap = default_telemetry()
        motor_up = int(self.state == "MOVING_UP")
        motor_dn = int(
            self.state == "MOVING_DOWN"
            or (self.state == "FIRE_EVAC" and self.current_floor > MIN_FLOOR and self.door_pos == "CLOSED")
        )
        # Bất biến an toàn phần mềm: không cho motor đóng điện nếu cửa chưa đóng hoặc đang E-Stop
        if self.door_pos != "CLOSED" or self.estop_active:
            motor_up = 0
            motor_dn = 0

        snap.update(
            {
                "ms": int(time.monotonic() * 1000) - self.start_ms,
                "st": self.state,
                "fl": self.current_floor,
                "pos_f": self.pos_float,
                "tgt": self.target_floor,
                "dir": self.direction,
                "door": self.door_pct,
                "y0": motor_up,
                "y1": motor_dn,
                "fire": int(self.fire_active),
                "estop": int(self.estop_active),
                "ovl": int(self.overload_active),
                "canf": int(self.can_fault_active),
                "temp": int(round(self.temp_c * 10)),
                "load": self.load_kg,
                "tec": 255 if self.can_drop_injected else 0,
                "rec": 0,
                "lat_us": self.last_lat_us,
                "cm": self.sched.cabin_mask,
                "um": self.sched.hall_up_mask,
                "dm": self.sched.hall_down_mask,
                "trips": self.trips,
                "obstruct_flash": int(self.obstruct_flash_ticks > 0),
            }
        )
        return snap
