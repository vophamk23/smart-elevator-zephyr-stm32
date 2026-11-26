"""
twin_scheduler.py - Bộ lập lịch gọi tầng SCAN/LOOK 7 tầng chạy trên PC (Python thuần).

Thiết kế đồng nhất 1-1 với thuật toán trong `src/core/elevator_scheduler.c` của STM32:
  - Dùng bitmask nhị phân (bit 0 = Tầng 1 ... bit 6 = Tầng 7) cho 3 hàng đợi độc lập:
    cabin_mask, hall_up_mask, hall_down_mask.
  - Phục vụ hết các cuốc cùng chiều hiện tại trước khi đảo chiều tại điểm có người gọi xa nhất
    (biến thể LOOK của thuật toán SCAN), giúp giảm quãng đường chạy và chống bỏ đói tầng biên.
"""

from twin_config import MAX_FLOOR, MIN_FLOOR


class ScanLookScheduler:
    """Bộ lập lịch hai chiều SCAN/LOOK dùng bitmask cho thang máy 7 tầng."""

    def __init__(self) -> None:
        """Khởi tạo 3 mặt nạ bit hàng đợi ở trạng thái rỗng."""
        self.cabin_mask: int = 0
        self.hall_up_mask: int = 0
        self.hall_down_mask: int = 0

    @staticmethod
    def _floor_bit(floor: int) -> int:
        """Quy đổi số tầng (1..7) sang bit tương ứng (ví dụ Tầng 3 -> 0b0000100 = 4)."""
        if MIN_FLOOR <= floor <= MAX_FLOOR:
            return 1 << (floor - 1)
        return 0

    def reset(self) -> None:
        """Xóa trắng toàn bộ các cuộc gọi đang chờ (thường dùng khi kích hoạt báo cháy)."""
        self.cabin_mask = 0
        self.hall_up_mask = 0
        self.hall_down_mask = 0

    def merged_mask(self) -> int:
        """Gộp cả 3 loại nút gọi để biết nhanh tầng nào đang có ít nhất một yêu cầu."""
        return self.cabin_mask | self.hall_up_mask | self.hall_down_mask

    def has_request_above(self, floor: int) -> bool:
        """Kiểm tra có còn bất kỳ yêu cầu nào ở các tầng nằm phía trên `floor` hay không."""
        # Tạo mặt nạ gồm tất cả các bit cao hơn `floor` rồi AND với tổng yêu cầu
        all_req = self.merged_mask()
        return any((all_req & self._floor_bit(f)) != 0 for f in range(floor + 1, MAX_FLOOR + 1))

    def has_request_below(self, floor: int) -> bool:
        """Kiểm tra có còn bất kỳ yêu cầu nào ở các tầng nằm phía dưới `floor` hay không."""
        all_req = self.merged_mask()
        return any((all_req & self._floor_bit(f)) != 0 for f in range(MIN_FLOOR, floor))

    def add_request(self, floor: int, call_type: str) -> bool:
        """
        Ghi nhận một nút nhấn gọi tầng mới.
        :param floor: Tầng từ 1..7.
        :param call_type: 'cabin', 'up' (Tầng 1..6) hoặc 'down' (Tầng 2..7).
        """
        bit = self._floor_bit(floor)
        if bit == 0:
            return False

        if call_type == "cabin":
            self.cabin_mask |= bit
            return True
        if call_type == "up" and floor < MAX_FLOOR:
            self.hall_up_mask |= bit
            return True
        if call_type == "down" and floor > MIN_FLOOR:
            self.hall_down_mask |= bit
            return True
        return False

    def clear_floor(self, floor: int, served_dir: int = 0) -> None:
        """Tắt đèn các nút gọi tại tầng vừa mở cửa theo đúng chiều phục vụ hiện tại."""
        clear_mask = ~self._floor_bit(floor)
        self.cabin_mask &= clear_mask
        if served_dir > 0:
            self.hall_up_mask &= clear_mask
            if not self.has_request_above(floor):
                self.hall_down_mask &= clear_mask
        elif served_dir < 0:
            self.hall_down_mask &= clear_mask
            if not self.has_request_below(floor):
                self.hall_up_mask &= clear_mask
        else:
            self.hall_up_mask &= clear_mask
            self.hall_down_mask &= clear_mask

    def _pick_upward_target(self, cur_floor: int) -> int:
        """Khi quét LÊN: ưu tiên đón người bấm cabin/lên gần nhất; hết mới đón người gọi xuống cao nhất."""
        up_or_cab = self.cabin_mask | self.hall_up_mask
        for f in range(cur_floor + 1, MAX_FLOOR + 1):
            if up_or_cab & self._floor_bit(f):
                return f
        # Đi lên hẳn tầng gọi Xuống cao nhất phía trên để sau đó quay đầu 1 mạch đi Xuống
        for f in range(MAX_FLOOR, cur_floor, -1):
            if self.hall_down_mask & self._floor_bit(f):
                return f
        return 0

    def _pick_downward_target(self, cur_floor: int) -> int:
        """Khi quét XUỐNG: ưu tiên đón người bấm cabin/xuống gần nhất; hết mới đón người gọi lên thấp nhất."""
        dn_or_cab = self.cabin_mask | self.hall_down_mask
        for f in range(cur_floor - 1, MIN_FLOOR - 1, -1):
            if dn_or_cab & self._floor_bit(f):
                return f
        for f in range(MIN_FLOOR, cur_floor):
            if self.hall_up_mask & self._floor_bit(f):
                return f
        return 0

    def pick_next_target(self, cur_floor: int, cur_dir: int):
        """
        Quyết định tầng đích kế tiếp và hướng di chuyển mới theo thuật toán LOOK.
        :return: Bộ (target_floor, next_dir) trong đó next_dir thuộc {1: UP, -1: DOWN, 0: IDLE}.
        """
        if self.merged_mask() == 0:
            return 0, 0

        # 1. Nếu ngay tầng đang đứng có người bấm -> mở cửa phục vụ ngay
        if self.merged_mask() & self._floor_bit(cur_floor):
            return cur_floor, cur_dir

        # 2. Giữ nguyên đà quét đang có để không đảo chiều đột ngột giữa hành trình
        if cur_dir >= 0 and self.has_request_above(cur_floor):
            tgt = self._pick_upward_target(cur_floor)
            if tgt:
                return tgt, 1

        if self.has_request_below(cur_floor):
            tgt = self._pick_downward_target(cur_floor)
            if tgt:
                return tgt, -1

        if self.has_request_above(cur_floor):
            tgt = self._pick_upward_target(cur_floor)
            if tgt:
                return tgt, 1

        return 0, 0

    def should_stop(self, floor: int, travel_dir: int) -> bool:
        """
        Kiểm tra khi cabin đi ngang qua `floor` thì có nên dừng lại đón/trả khách hay đi thẳng.
        """
        bit = self._floor_bit(floor)
        if bit == 0:
            return False

        # Khách bên trong cabin bấm tầng này thì luôn dừng
        if self.cabin_mask & bit:
            return True

        # Khách ngoài hành lang: chỉ dừng nếu cùng chiều đi, hoặc là điểm cuối cùng của chiều quét
        if travel_dir > 0:
            if self.hall_up_mask & bit:
                return True
            return (not self.has_request_above(floor)) and bool(self.hall_down_mask & bit)

        if travel_dir < 0:
            if self.hall_down_mask & bit:
                return True
            return (not self.has_request_below(floor)) and bool(self.hall_up_mask & bit)

        return bool((self.hall_up_mask | self.hall_down_mask) & bit)
