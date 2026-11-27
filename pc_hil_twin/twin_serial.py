"""
twin_serial.py - Quản lý kết nối UART RS232, giải mã $TWIN JSON & Xuất dữ liệu CSV.

Đặc biệt tích hợp lớp dự phòng `_Win32RawSerial` (Win32 API trực tiếp qua `ctypes`) giúp:
  - Vượt qua lỗi kinh điển `PermissionError(13, 'A device attached to the system is not functioning.', None, 31)`
    của driver Windows 11 `CH340 v3.8/v3.9` trên cáp USB-to-RS232 HL-340 mà KHÔNG cần hạ cấp driver.
  - Tự động dò tìm cổng USB-SERIAL `CH340` đang cắm thực tế (ví dụ tự chuyển `COM3` -> `COM5` khi đổi lỗ USB).
"""

import csv
import ctypes
from ctypes import wintypes
import json
import os
import queue
import re
import threading
import time
from typing import Any, Callable, Dict, List, Optional

ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

try:
    import serial
    import serial.tools.list_ports

    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

from twin_config import HIL_SMOKE_STEPS


class _DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wintypes.DWORD),
        ("BaudRate", wintypes.DWORD),
        ("flags", wintypes.DWORD),
        ("wReserved", wintypes.WORD),
        ("XonLim", wintypes.WORD),
        ("XoffLim", wintypes.WORD),
        ("ByteSize", wintypes.BYTE),
        ("Parity", wintypes.BYTE),
        ("StopBits", wintypes.BYTE),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", wintypes.WORD),
    ]


class _COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wintypes.DWORD),
        ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
        ("ReadTotalTimeoutConstant", wintypes.DWORD),
        ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
        ("WriteTotalTimeoutConstant", wintypes.DWORD),
    ]


class _COMSTAT(ctypes.Structure):
    _fields_ = [
        ("flags", wintypes.DWORD),
        ("cbInQue", wintypes.DWORD),
        ("cbOutQue", wintypes.DWORD),
    ]


class _Win32RawSerial:
    """
    Trình điều khiển Win32 Serial trực tiếp cho cáp CH340 trên Windows 11:
    Driver CH340 v3.8/v3.9 vẫn ghi đúng thanh ghi tốc độ Baud (115200 8N1) xuống chip nhưng trả mã
    cảnh báo WinError 31 ở cuối hàm SetCommState khiến pyserial tự đóng cổng. Lớp này giữ nguyên handle
    hợp lệ và dùng ClearCommError (không chặn) để đọc/ghi song công mượt mà 100%.
    """

    GENERIC_READ_WRITE = 0xC0000000
    OPEN_EXISTING = 3
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    def __init__(self, port: str, baud: int = 115200) -> None:
        if os.name != "nt":
            raise RuntimeError("Win32RawSerial chỉ hỗ trợ trên hệ điều hành Windows")

        self._k32 = ctypes.windll.kernel32
        self._io_lock = threading.Lock()
        self._rx_buf = bytearray()

        dev_path = port if port.startswith("\\\\.\\") else f"\\\\.\\{port}"
        h = self._k32.CreateFileW(
            dev_path,
            self.GENERIC_READ_WRITE,
            0,
            None,
            self.OPEN_EXISTING,
            0,
            None,
        )
        if h == -1 or h == self.INVALID_HANDLE_VALUE:
            err = ctypes.GetLastError()
            raise OSError(f"Không mở được {port} (WinError {err}). Kiểm tra cáp USB-RS232 hoặc tắt PuTTY!")

        self._handle = h

        dcb = _DCB()
        dcb.DCBlength = ctypes.sizeof(_DCB)
        self._k32.GetCommState(self._handle, ctypes.byref(dcb))
        dcb.BaudRate = baud
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        dcb.flags = 0x0001  # fBinary = 1, tắt flow control cứng để tránh treo đường RS232 3 dây
        # Bỏ qua mã trả về WinError 31 giả của CH340 v3.8/v3.9 (thanh ghi 115200 đã được nạp thành công)
        self._k32.SetCommState(self._handle, ctypes.byref(dcb))

        timeouts = _COMMTIMEOUTS(10, 0, 40, 0, 250)
        self._k32.SetCommTimeouts(self._handle, ctypes.byref(timeouts))
        self._k32.PurgeComm(self._handle, 0x000C)

    def reset_input_buffer(self) -> None:
        with self._io_lock:
            self._rx_buf.clear()
            if self._handle is not None:
                self._k32.PurgeComm(self._handle, 0x0008)

    def write(self, data: bytes) -> int:
        if not data or self._handle is None:
            return 0
        written = wintypes.DWORD(0)
        with self._io_lock:
            if self._handle is None:
                return 0
            ok = self._k32.WriteFile(self._handle, data, len(data), ctypes.byref(written), None)
            if not ok:
                raise OSError(f"Lỗi ghi UART (WinError {ctypes.GetLastError()})")
        return int(written.value)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        # Nếu trong đệm đã có sẵn ký tự xuống dòng -> trả về ngay lập tức
        nl_pos = self._rx_buf.find(b"\n")
        if nl_pos >= 0:
            line = bytes(self._rx_buf[: nl_pos + 1])
            del self._rx_buf[: nl_pos + 1]
            return line

        deadline = time.monotonic() + 0.18
        while time.monotonic() < deadline:
            if self._handle is None:
                return b""

            errs = wintypes.DWORD(0)
            stat = _COMSTAT()
            with self._io_lock:
                if self._handle is None:
                    return b""
                ok = self._k32.ClearCommError(self._handle, ctypes.byref(errs), ctypes.byref(stat))
                if not ok:
                    raise OSError("Cáp USB-RS232 đã bị rút hoặc mất kết nối")
                avail = int(stat.cbInQue)
                if avail > 0:
                    to_read = min(avail, 1024)
                    chunk = ctypes.create_string_buffer(to_read)
                    got = wintypes.DWORD(0)
                    if self._k32.ReadFile(self._handle, chunk, to_read, ctypes.byref(got), None) and got.value > 0:
                        self._rx_buf.extend(chunk.raw[: got.value])

            nl_pos = self._rx_buf.find(b"\n")
            if nl_pos >= 0:
                line = bytes(self._rx_buf[: nl_pos + 1])
                del self._rx_buf[: nl_pos + 1]
                return line

            time.sleep(0.012)

        return b""

    def close(self) -> None:
        with self._io_lock:
            if self._handle is not None:
                try:
                    self._k32.CloseHandle(self._handle)
                except Exception:
                    pass
                self._handle = None


class SerialBridge:
    """Cầu nối truyền thông nối tiếp RS232 giữa PC và bo mạch STM32 Zephyr."""

    def __init__(self) -> None:
        """Khởi tạo các hàng đợi an toàn đa luồng và bộ đệm lưu log HIL CSV."""
        self.telemetry_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.shell_queue: "queue.Queue[str]" = queue.Queue()
        self.hil_log_rows: List[Dict[str, Any]] = []

        self._ser: Any = None
        self._tx_lock = threading.Lock()
        self._running = False
        self._smoke_running = False
        self.port_name: str = ""
        self.used_win32_fallback: bool = False

    @property
    def is_connected(self) -> bool:
        """Trả về True nếu cổng COM đang mở và luồng đọc vẫn chạy."""
        return self._running and (self._ser is not None)

    @staticmethod
    def list_ports() -> List[str]:
        """Liệt kê danh sách các cổng COM/Serial tìm thấy trên máy tính (ưu tiên CH340/USB-Serial lên đầu)."""
        if not HAS_SERIAL:
            return ["COM5", "COM3", "COM4"]
        ports_info = list(serial.tools.list_ports.comports())
        # Ưu tiên xếp các cổng CH340 / CP210x / FTDI / USB-SERIAL lên đầu danh sách
        ports_info.sort(
            key=lambda p: (
                0 if any(k in (p.description or "").upper() for k in ("CH340", "USB-SERIAL", "CP210", "FTDI")) else 1,
                p.device,
            )
        )
        ports = [p.device for p in ports_info]
        return ports or ["COM5"]

    def connect(self, port: str, baud: int = 115200) -> None:
        """Mở kết nối RS232 (tự dò cổng CH340 nếu đổi lỗ USB & tự vượt lỗi WinError 31 trên Windows 11)."""
        self.disconnect()

        # Tự động chuyển sang cổng USB-Serial thực tế nếu cổng cũ trên combobox (vd COM3) không còn tồn tại
        target_port = port.strip()
        if HAS_SERIAL:
            live_ports = self.list_ports()
            raw_devs = [p.device for p in serial.tools.list_ports.comports()]
            if raw_devs and target_port not in raw_devs:
                target_port = live_ports[0]

        self.used_win32_fallback = False
        try:
            if not HAS_SERIAL:
                raise RuntimeError("No pyserial")
            self._ser = serial.Serial(target_port, baud, timeout=0.2, write_timeout=0.5)
        except Exception as pyser_exc:
            # Dự phòng tự động sang Win32RawSerial khi gặp lỗi driver CH340 v3.8/v3.9 (WinError 31)
            if os.name == "nt":
                try:
                    self._ser = _Win32RawSerial(target_port, baud)
                    self.used_win32_fallback = True
                except Exception:
                    raise pyser_exc
            else:
                raise pyser_exc

        self.port_name = target_port
        self._running = True
        time.sleep(0.06)
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        threading.Thread(target=self._reader_loop, daemon=True).start()
        self.send_cmd("elev twin on")

    def disconnect(self) -> None:
        """Dừng luồng đọc nền và đóng cổng Serial an toàn."""
        self._running = False
        with self._tx_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
        self.port_name = ""

    def send_cmd(self, cmd: str) -> bool:
        """
        Gửi lệnh CLI xuống Zephyr Shell qua UART an toàn đa luồng:
        Tiền tố `\\x15` (Ctrl+U) xóa sạch mọi ký tự rác còn sót trong dòng lệnh của Zephyr Shell,
        sau đó gửi câu lệnh kết thúc bằng `\\r` để thực thi chính xác 100%.
        """
        if not self.is_connected:
            return False
        clean_cmd = cmd.strip()
        if not clean_cmd:
            return False
        with self._tx_lock:
            try:
                if self._ser is None:
                    return False
                self._ser.write(("\x15" + clean_cmd + "\r").encode("utf-8"))
                self._ser.flush()
                return True
            except Exception as exc:
                self.shell_queue.put(f"[Serial TX Error] {exc}")
                return False

    def _reader_loop(self) -> None:
        """Luồng nền đọc từng dòng UART: tách gói `$TWIN,` JSON kể cả khi dính cùng dòng với prompt Shell."""
        while self._running and self._ser is not None:
            try:
                raw_bytes = self._ser.readline()
                if not raw_bytes:
                    continue
                line = raw_bytes.decode("utf-8", errors="ignore")
                clean = ANSI_ESCAPE_RE.sub("", line).strip()
                if not clean:
                    continue

                twin_idx = clean.find("$TWIN,")
                if twin_idx >= 0:
                    prefix = clean[:twin_idx].strip()
                    if prefix and prefix != "bkit-elev:~$":
                        self.shell_queue.put(prefix)
                    json_part = clean[twin_idx + 6 :]
                    brace_end = json_part.rfind("}")
                    if brace_end >= 0:
                        payload = json.loads(json_part[: brace_end + 1])
                        self.telemetry_queue.put(payload)
                        self.record_sample(payload)
                elif clean != "bkit-elev:~$":
                    self.shell_queue.put(clean)
            except OSError:
                # Cáp bị rút vật lý -> thoát vòng lặp êm ái
                self._running = False
                self.shell_queue.put("[WARN] Mất kết nối cáp USB-RS232.")
                break
            except Exception:
                pass

    def record_sample(self, row: Dict[str, Any]) -> None:
        """Lưu bổ sung mẫu telemetry (kể cả khi chạy Offline Simulator) để xuất CSV."""
        self.hil_log_rows.append(dict(row))
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
