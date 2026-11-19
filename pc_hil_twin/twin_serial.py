"""
twin_serial.py - Quản lý kết nối UART RS232, giải mã $TWIN JSON & Xuất dữ liệu CSV.

Chức năng tổng quát của file:
  1. Luồng nền `_reader_loop()` đọc Serial không làm đơ (freeze) giao diện Tkinter.
  2. Tách dòng telemetry `$TWIN,{...}` đưa vào `telemetry_queue`, còn log chữ từ Zephyr Shell
     đưa vào `shell_queue` giúp theo dõi trực tiếp phản hồi CLI ngay trên PC Twin.

Các hàm / phương thức quan trọng nhất trong file:
  - `SerialBridge.connect()`     : Mở cổng COM RS232 ở tốc độ 115200 bps và khởi chạy luồng đọc nền.
  - `SerialBridge.send_command()`: Gửi lệnh điều khiển Zephyr Shell (`elev ...` / `bkit ...`) xuống bo mạch BKIT.
  - `SerialBridge._reader_loop()`: Vòng lặp đọc UART liên tục, bóc tách JSON `$TWIN` và lưu lịch sử bản ghi.
  - `SerialBridge.export_csv()`  : Xuất toàn bộ lịch sử dữ liệu telemetry thu được ra file `.csv` phục vụ báo cáo.
"""

import csv
import json
import queue
import threading
import time
from typing import Any, Callable, Dict, List, Optional

try:
    import serial
    import serial.tools.list_ports

    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

from twin_config import HIL_SMOKE_STEPS


class SerialBridge:
    """Cầu nối truyền thông nối tiếp RS232 giữa PC và bo mạch STM32 Zephyr."""

    def __init__(self) -> None:
        """Khởi tạo các hàng đợi an toàn đa luồng và bộ đệm lưu log HIL CSV."""
        self.telemetry_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.shell_queue: "queue.Queue[str]" = queue.Queue()
        self.hil_log_rows: List[Dict[str, Any]] = []

        self._ser = None
        self._running = False
        self._smoke_running = False
        self.port_name: str = ""

    @property
    def is_connected(self) -> bool:
        """Trả về True nếu cổng COM đang mở và luồng đọc vẫn chạy."""
        return self._running and (self._ser is not None)

    @staticmethod
    def list_ports() -> List[str]:
        """Liệt kê danh sách các cổng COM/Serial tìm thấy trên máy tính."""
        if not HAS_SERIAL:
            return ["COM3", "COM4"]
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports or ["COM3"]

    def connect(self, port: str, baud: int = 115200) -> None:
        """Mở kết nối RS232 và khởi chạy luồng nền đọc dữ liệu từ STM32."""
        if not HAS_SERIAL:
            raise RuntimeError("Thiếu thư viện pyserial. Hãy cài bằng lệnh: pip install pyserial")
        self.disconnect()
        self._ser = serial.Serial(port, baud, timeout=0.2)
        self.port_name = port
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()
        self.send_cmd("elev twin on")

    def disconnect(self) -> None:
        """Dừng luồng đọc nền và đóng cổng Serial an toàn."""
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.port_name = ""

    def send_cmd(self, cmd: str) -> bool:
        """Gửi một chuỗi lệnh CLI xuống Zephyr Shell qua UART (kèm ký tự kết thúc dòng CRLF)."""
        if not self.is_connected:
            return False
        try:
            self._ser.write((cmd.strip() + "\r\n").encode("utf-8"))
            return True
        except Exception as exc:
            self.shell_queue.put(f"[Serial TX Error] {exc}")
            return False

    def _reader_loop(self) -> None:
        """Luồng nền đọc từng dòng UART: tách gói `$TWIN,` JSON riêng với log Zephyr Shell."""
        while self._running and self._ser is not None:
            try:
                raw_bytes = self._ser.readline()
                if not raw_bytes:
                    continue
                line = raw_bytes.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("$TWIN,"):
                    payload = json.loads(line[6:])
                    self.telemetry_queue.put(payload)
                    self.hil_log_rows.append(payload)
                else:
                    # Loại bỏ mã màu ANSI (ESC[...) để in sạch vào ô Console Tkinter
                    clean = line.replace("\x1b[1;32m", "").replace("\x1b[m", "")
                    clean = clean.replace("\x1b[1;31m", "").replace("\x1b[1;33m", "")
                    self.shell_queue.put(clean)
            except Exception:
                pass

    def record_sample(self, row: Dict[str, Any]) -> None:
        """Lưu bổ sung mẫu telemetry (kể cả khi chạy Offline Simulator) để xuất CSV."""
        self.hil_log_rows.append(dict(row))
        # Giới hạn bộ đệm RAM tối đa 5000 mẫu gần nhất (~16 phút ở 200 ms/mẫu)
        if len(self.hil_log_rows) > 5000:
            self.hil_log_rows.pop(0)

    def run_smoke_suite(
        self,
        dispatch_cb: Callable[[str], None],
        log_cb: Callable[[str, str], None],
    ) -> bool:
        """Chạy chuỗi kiểm thử HIL tự động 7 tầng trên luồng nền (dùng cho cả Live & Offline)."""
        if self._smoke_running:
            log_cb("WARN", "Chuỗi Auto HIL Smoke Suite đang chạy, vui lòng đợi hoàn tất!")
            return False

        def worker() -> None:
            self._smoke_running = True
            log_cb("OK", "=== BẮT ĐẦU CHUỖI AUTO DEMO & HIL SMOKE SUITE (7 TẦNG) ===")
            try:
                for cmd, delay_s, note in HIL_SMOKE_STEPS:
                    log_cb("TX", f"{cmd}  # {note}")
                    dispatch_cb(cmd)
                    time.sleep(delay_s)
                log_cb("OK", "=== HOÀN TẤT CHUỖI AUTO HIL SMOKE SUITE ===")
            finally:
                self._smoke_running = False

        threading.Thread(target=worker, daemon=True).start()
        return True

    def export_csv(self, directory: str = ".") -> Optional[str]:
        """Xuất toàn bộ bản ghi telemetry thu thập được ra file CSV phục vụ báo cáo Chương 4."""
        if not self.hil_log_rows:
            return None
        fname = f"{directory}/hil_capture_{int(time.time())}.csv"
        keys = list(self.hil_log_rows[0].keys())
        with open(fname, "w", newline="", encoding="utf-8") as fp:
            writer = csv.DictWriter(fp, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.hil_log_rows)
        return fname
