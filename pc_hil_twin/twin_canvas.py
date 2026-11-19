"""
twin_canvas.py - Thành phần đồ họa vẽ giếng thang 7 tầng trực quan (tk.Canvas).

Chức năng tổng quát của file:
  - Hiển thị đủ 7 tầng (F7 trên đỉnh -> F1 dưới đáy) với đèn báo gọi từng tầng (CAB / ▲ / ▼).
  - Vẽ ròng rọc đỉnh giếng, cáp treo chuyển động đồng bộ và khối đối trọng đi ngược chiều cabin.
  - Nội suy mượt vị trí cabin theo biến `pos_f` (1.0..7.0) và đóng/mở 2 cánh cửa theo `door` (0..100%).
  - Đổi màu cabin tức thời khi xảy ra Hỏa hoạn (Đỏ), Lỗi CAN/E-Stop (Hổ phách) hay Quá tải (Cam).

Các phương thức quan trọng nhất trong file:
  - `ElevatorShaftCanvas.redraw()`                 : Vẽ lại toàn bộ giếng thang 7 tầng, cáp treo, đối trọng và cửa cabin mỗi 50 ms.
  - `ElevatorShaftCanvas._update_seg7_and_badges()`: Cập nhật đồng hồ LED 7 đoạn ảo ('UP_3', 'dn_5', 'FE_4', 'Err2') và đèn Y0/Y1/X0.
"""

import tkinter as tk
from typing import Any, Dict

from twin_config import COLORS, FONTS, MAX_FLOOR, MIN_FLOOR, STATE_META


class ElevatorShaftCanvas(tk.Frame):
    """Widget chứa mô hình giếng thang 7 tầng và đồng hồ LED 7 đoạn ảo phía trên."""

    def __init__(self, master: tk.Misc) -> None:
        """Khởi tạo phần hiển thị LED 7 đoạn ảo cùng vùng vẽ Canvas giếng thang."""
        super().__init__(
            master,
            bg=COLORS["bg_card"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            padx=10,
            pady=10,
        )

        # Thanh tiêu đề + Đồng hồ LED 7 đoạn ảo (mô phỏng 4 LED 7 đoạn SPI1 trên kit BKIT ARM4)
        hdr = tk.Frame(self, bg=COLORS["bg_card"])
        hdr.pack(fill=tk.X, pady=(0, 8))

        tk.Label(
            hdr,
            text="GIẾNG THANG 7 TẦNG",
            fg=COLORS["text_primary"],
            bg=COLORS["bg_card"],
            font=FONTS["section"],
        ).pack(side=tk.LEFT)

        seg_box = tk.Frame(
            hdr,
            bg="#04070E",
            highlightthickness=1,
            highlightbackground="#334155",
            padx=8,
            pady=2,
        )
        seg_box.pack(side=tk.RIGHT)
        self.seg7_lbl = tk.Label(
            seg_box,
            text="-  1",
            fg="#F87171",
            bg="#04070E",
            font=FONTS["seg7"],
            width=4,
        )
        self.seg7_lbl.pack()

        # Vùng vẽ Canvas co giãn theo chiều cao cửa sổ
        self.canvas = tk.Canvas(
            self,
            width=295,
            height=540,
            bg=COLORS["bg_shaft"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Thanh chú thích trạng thái ngõ ra phần cứng phía dưới giếng thang
        io_bar = tk.Frame(self, bg=COLORS["bg_card"], pady=6)
        io_bar.pack(fill=tk.X, pady=(6, 0))
        self.y0_badge = self._make_io_pill(io_bar, "PE5 Y0 (LÊN)")
        self.y1_badge = self._make_io_pill(io_bar, "PE6 Y1 (XUỐNG)")
        self.ir_badge = self._make_io_pill(io_bar, "PA6 X0 (IR CỬA)")

        self._last_tel: Dict[str, Any] = {}
        self.canvas.bind("<Configure>", lambda _e: self.redraw(self._last_tel))

    def _make_io_pill(self, parent: tk.Misc, text: str) -> tk.Label:
        """Tạo một đèn báo trạng thái chân GPIO nhỏ gọn phía chân giếng thang."""
        lbl = tk.Label(
            parent,
            text=text,
            fg=COLORS["text_muted"],
            bg=COLORS["btn_idle"],
            font=("Consolas", 8, "bold"),
            padx=6,
            pady=3,
        )
        lbl.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        return lbl

    def _update_seg7_and_badges(self, tel: Dict[str, Any]) -> None:
        """Cập nhật ký tự trên đồng hồ LED 7 đoạn ảo (khớp hàm bsp_spi_7seg_render_status)."""
        st = tel.get("st", "IDLE_CLOSED")
        fl = int(tel.get("fl", 1))
        fire = int(tel.get("fire", 0))
        fault = int(tel.get("estop", 0)) or int(tel.get("canf", 0))

        ms_now = int(tel.get("ms", 0))
        anim_step = (ms_now // 180) % 3
        bars_up = ("_", "-", "¯")
        bars_dn = ("¯", "-", "_")

        if fire:
            code_txt, seg_col = f"FE{bars_dn[anim_step]}{fl}", "#EF4444"
        elif fault:
            code_txt, seg_col = f"Err{fl}", "#F59E0B"
        else:
            if st == "MOVING_UP":
                code_txt = f"UP{bars_up[anim_step]}{fl}"
            elif st == "MOVING_DOWN":
                code_txt = f"dn{bars_dn[anim_step]}{fl}"
            else:
                prefix = STATE_META.get(st, ("", "", "F-"))[2]
                code_txt = f"{prefix:<2} {fl}"
            seg_col = "#34D399" if st in ("DOOR_OPEN", "IDLE_CLOSED") else "#38BDF8"

        self.seg7_lbl.config(text=code_txt, fg=seg_col)

        # Đổi màu đèn chân motor Y0 / Y1 và cảm biến IR X0
        y0 = int(tel.get("y0", 0))
        y1 = int(tel.get("y1", 0))
        ir = int(tel.get("obstruct_flash", 0))

        self.y0_badge.config(
            bg=COLORS["accent_green"] if y0 else COLORS["btn_idle"],
            fg="#052E16" if y0 else COLORS["text_muted"],
        )
        self.y1_badge.config(
            bg=COLORS["accent_amber"] if y1 else COLORS["btn_idle"],
            fg="#451A03" if y1 else COLORS["text_muted"],
        )
        self.ir_badge.config(
            bg=COLORS["accent_red"] if ir else COLORS["btn_idle"],
            fg="#FFFFFF" if ir else COLORS["text_muted"],
        )

    def _floor_to_y(self, floor_val: float, top_y: float, floor_h: float) -> float:
        """
        Đổi số tầng liên tục (1.0 .. 7.0) sang tọa độ Y (pixel đáy tầng) trên Canvas.
        Tầng 7 nằm ở đỉnh (Y nhỏ), Tầng 1 nằm ở đáy (Y lớn).
        """
        idx_from_top = float(MAX_FLOOR) - floor_val
        return top_y + (idx_from_top + 1.0) * floor_h

    def redraw(self, tel: Dict[str, Any]) -> None:
        """Vẽ lại toàn bộ giếng thang 7 tầng, cáp treo, đối trọng và cabin."""
        if not tel:
            return
        self._last_tel = tel
        self._update_seg7_and_badges(tel)

        c = self.canvas
        w = max(260, c.winfo_width())
        h = max(460, c.winfo_height())
        c.delete("all")

        top_pad = 30.0
        bot_pad = 16.0
        usable_h = h - top_pad - bot_pad
        floor_h = usable_h / float(MAX_FLOOR)

        shaft_x0 = 64.0
        shaft_x1 = w - 62.0
        cabin_x0 = shaft_x0 + 18.0
        cabin_x1 = shaft_x1 - 18.0

        # 1. Vẽ thanh ray dẫn hướng dọc giếng thang + ròng rọc trên đỉnh
        c.create_rectangle(shaft_x0, top_pad - 6, shaft_x1, h - bot_pad, outline="#1E293B", width=2)
        c.create_line(cabin_x0 - 6, top_pad, cabin_x0 - 6, h - bot_pad, fill="#334155", width=2)
        c.create_line(cabin_x1 + 6, top_pad, cabin_x1 + 6, h - bot_pad, fill="#334155", width=2)

        # Bánh xe ròng rọc đỉnh giếng
        pulley_cx = (cabin_x0 + cabin_x1) / 2.0
        c.create_oval(pulley_cx - 10, 6, pulley_cx + 10, 24, fill="#334155", outline="#64748B", width=2)

        # 2. Vẽ từng tầng F7..F1 và 3 đèn báo cuộc gọi nhỏ bên phải giếng thang
        cm = int(tel.get("cm", 0))
        um = int(tel.get("um", 0))
        dm = int(tel.get("dm", 0))
        cur_fl = int(tel.get("fl", 1))

        for fl in range(MIN_FLOOR, MAX_FLOOR + 1):
            y_bot = self._floor_to_y(float(fl), top_pad, floor_h)
            y_top = y_bot - floor_h
            y_mid = (y_top + y_bot) / 2.0
            bit = 1 << (fl - 1)

            # Đường chia tầng
            c.create_line(10, y_bot, w - 10, y_bot, fill="#1E293B", width=1, dash=(3, 3))

            # Thẻ số tầng F1..F7 (làm nổi tầng cabin đang đứng)
            is_cur = (fl == cur_fl)
            c.create_rectangle(
                10, y_mid - 12, 46, y_mid + 12,
                fill="#1D4ED8" if is_cur else "#162032",
                outline="#60A5FA" if is_cur else "#263248",
            )
            c.create_text(
                28, y_mid,
                text=f"F{fl}",
                fill="#FFFFFF" if is_cur else COLORS["text_muted"],
                font=("Consolas", 10, "bold"),
            )

            # Cụm 3 đèn LED báo gọi bên phải: Vàng=Cabin, Xanh lá=Lên, Xanh dương=Xuống
            led_x = shaft_x1 + 10
            c.create_oval(
                led_x, y_mid - 5, led_x + 10, y_mid + 5,
                fill=COLORS["accent_amber"] if (cm & bit) else "#1E293B",
                outline="#475569",
            )
            if fl < MAX_FLOOR:
                c.create_polygon(
                    led_x + 21, y_mid - 5, led_x + 16, y_mid + 4, led_x + 26, y_mid + 4,
                    fill=COLORS["accent_green"] if (um & bit) else "#1E293B",
                    outline="#475569",
                )
            if fl > MIN_FLOOR:
                c.create_polygon(
                    led_x + 36, y_mid + 5, led_x + 31, y_mid - 4, led_x + 41, y_mid - 4,
                    fill=COLORS["accent_blue"] if (dm & bit) else "#1E293B",
                    outline="#475569",
                )

        # 3. Tính vị trí cabin mượt & vẽ đối trọng + dây cáp kéo
        pos_f = float(tel.get("pos_f", float(cur_fl)))
        pos_f = max(float(MIN_FLOOR), min(float(MAX_FLOOR), pos_f))
        cab_bot = self._floor_to_y(pos_f, top_pad, floor_h) - 4.0
        cab_top = cab_bot - (floor_h - 10.0)

        # Khối đối trọng chạy ngược chiều cabin (khi cabin đi lên F7, đối trọng đi xuống F1)
        cw_floor = (float(MAX_FLOOR + MIN_FLOOR)) - pos_f
        cw_bot = self._floor_to_y(cw_floor, top_pad, floor_h) - 8.0
        cw_top = cw_bot - max(18.0, floor_h * 0.48)
        cw_x0, cw_x1 = shaft_x0 + 4.0, shaft_x0 + 13.0
        c.create_line((cw_x0 + cw_x1) / 2.0, 15, (cw_x0 + cw_x1) / 2.0, cw_top, fill="#475569", width=1.5)
        c.create_rectangle(cw_x0, cw_top, cw_x1, cw_bot, fill="#475569", outline="#94A3B8")

        # Hai sợi dây cáp nối từ ròng rọc đỉnh giếng xuống nóc cabin
        c.create_line(pulley_cx - 4, 22, pulley_cx - 4, cab_top, fill="#64748B", width=1.5)
        c.create_line(pulley_cx + 4, 22, pulley_cx + 4, cab_top, fill="#64748B", width=1.5)

        # 4. Vẽ thân cabin đổi màu theo trạng thái an toàn
        if int(tel.get("fire", 0)):
            cab_fill, cab_outline = "#991B1B", "#FCA5A5"
        elif int(tel.get("estop", 0)) or int(tel.get("canf", 0)):
            cab_fill, cab_outline = "#9A3412", "#FDBA74"
        elif int(tel.get("ovl", 0)):
            cab_fill, cab_outline = "#92400E", "#FDE047"
        else:
            cab_fill, cab_outline = "#1E3A8A", "#38BDF8"

        c.create_rectangle(cabin_x0, cab_top, cabin_x1, cab_bot, fill=cab_fill, outline=cab_outline, width=2)

        # 5. Vẽ hai cánh cửa kính trượt từ tâm ra ngoài theo tỷ lệ door (0..100%)
        door_pct = max(0, min(100, int(tel.get("door", 0))))
        inner_x0, inner_x1 = cabin_x0 + 5.0, cabin_x1 - 5.0
        inner_y0, inner_y1 = cab_top + 14.0, cab_bot - 5.0
        mid_x = (inner_x0 + inner_x1) / 2.0
        half_w = (inner_x1 - inner_x0) / 2.0
        gap = (door_pct / 100.0) * (half_w - 4.0)

        # Khoang sáng phía sau cửa khi cửa hé mở
        c.create_rectangle(inner_x0, inner_y0, inner_x1, inner_y1, fill="#0F172A", outline="#1E293B")
        if gap > 2:
            c.create_rectangle(
                mid_x - gap, inner_y0 + 1, mid_x + gap, inner_y1 - 1,
                fill="#0284C7", outline="",
            )

        # Hai cánh cửa trái/phải
        c.create_rectangle(inner_x0, inner_y0, mid_x - gap, inner_y1, fill="#60A5FA", outline="#1E3A8A")
        c.create_rectangle(mid_x + gap, inner_y0, inner_x1, inner_y1, fill="#60A5FA", outline="#1E3A8A")

        # Thanh cảm biến tia hồng ngoại IR X0 ở đáy cửa (chớp đỏ khi kẹt cửa)
        if int(tel.get("obstruct_flash", 0)):
            c.create_line(cabin_x0 + 4, cab_bot - 2, cabin_x1 - 4, cab_bot - 2, fill="#EF4444", width=3)

        # Mũi tên hướng chạy + % mở cửa trên cabin
        d = int(tel.get("dir", 0))
        arrow = "▲" if d > 0 else ("▼" if d < 0 else "●")
        c.create_text(
            mid_x, cab_top + 7.5,
            text=f"{arrow} F{cur_fl}",
            fill="#FFFFFF",
            font=("Consolas", 8, "bold"),
        )
        c.create_text(
            mid_x, (inner_y0 + inner_y1) / 2.0,
            text=f"{door_pct}%",
            fill="#FFFFFF" if gap > 12 else "#0F172A",
            font=("Consolas", 9, "bold"),
        )
