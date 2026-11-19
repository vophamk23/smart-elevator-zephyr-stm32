# Hướng dẫn Đọc Code & Vận hành Hệ thống Thang máy Thông minh 7 Tầng

Dự án điều khiển thang máy **7 tầng** chạy trên hệ điều hành thời gian thực **Zephyr RTOS v4.4.0** cho bo mạch **BKIT ARM4 (`STM32F407ZGT6`)**, kèm phần mềm mô phỏng & theo dõi trên máy tính **PC Digital Twin (`pc_hil_twin/`)**.

---

## Mục lục

1. [Hiểu nhanh trong 30 giây: Chương trình này tổ chức như thế nào?](#1-hiểu-nhanh-trong-30-giây-chương-trình-này-tổ-chức-như-thế-nào)
2. [Đọc từ điểm bắt đầu: Hàm `main()` làm những gì khi vừa bật điện?](#2-đọc-từ-điểm-bắt-đầu-hàm-main-làm-những-gì-khi-vừa-bật-điện)
3. [Sau khi bật xong: 7 luồng chạy song song làm việc gì?](#3-sau-khi-bật-xong-7-luồng-chạy-song-song-làm-việc-gì)
4. [Ví dụ thực tế: Lần dấu code khi thang máy hoạt động](#4-ví-dụ-thực-tế-lần-dấu-code-khi-thang-máy-hoạt-động)
5. [Từ điển tra nhanh: File nào chứa hàm gì?](#5-từ-điển-tra-nhanh-file-nào-chứa-hàm-gì)
6. [Thông tin phần cứng BKIT ARM4 & Giao thức CAN / EEPROM](#6-thông-tin-phần-cứng-bkit-arm4--giao-thức-can--eeprom)
7. [Cách Build, Nạp code & Chạy giao diện PC Digital Twin](#7-cách-build-nạp-code--chạy-giao-diện-pc-digital-twin)
8. [Các lệnh gõ thử qua cổng UART (`bkit` & `elev`)](#8-các-lệnh-gõ-thử-qua-cổng-uart-bkit--elev)

---

## 1. Hiểu nhanh trong 30 giây: Chương trình này tổ chức như thế nào?

Để không bị rối khi mở thư mục dự án, bạn chỉ cần nhớ cấu trúc chia đúng **4 ngăn** cực kỳ gọn gàng:

```text
smart-elevator-zephyr-stm32/
├── app.overlay & prj.conf        <-- Khai báo chân cắm trên kit STM32 & bật các tính năng Zephyr
├── include/                      <-- Các file .h (chỉ chứa tên hàm & kiểu dữ liệu dùng chung)
├── src/
│   ├── main.c                    <-- ĐIỂM BẮT ĐẦU: khởi tạo mạch + tạo 7 luồng chạy song song
│   ├── core/                     <-- BỘ NÃO: máy trạng thái HSM + thuật toán xếp lịch tầng LOOK
│   ├── bsp/                      <-- TAY CHÂN PHẦN CỨNG: GPIO/EXTI, ADC, Còi, LED 7 đoạn/Phím, EEPROM/RTC, Watchdog
│   ├── comms/                    <-- GIAO TIẾP RA NGOÀI: mạng CAN giữa 2 bo & gửi JSON lên PC
│   └── cli/                      <-- NHẬN LỆNH TỪ BÀN PHÍM PC: lệnh gõ qua UART (`elev ...`, `bkit ...`)
└── pc_hil_twin/                  <-- Phần mềm Python vẽ thang máy 7 tầng trên máy tính
```

**Mẹo đọc code nhanh nhất:**
- Muốn biết **lúc mới bật điện mạch làm gì**, xem hàm `main()` ở cuối file `src/main.c`.
- Muốn biết **thang máy xử lý lên/xuống/mở cửa/cháy/kẹt cửa ra sao**, xem `src/core/elevator_hsm.c` và `src/core/elevator_scheduler.c`.
- Muốn biết **đọc cảm biến hay xuất tín hiệu ra chân STM32 nào**, tìm đúng tên phần cứng trong `src/bsp/`.

---

## 2. Đọc từ điểm bắt đầu: Hàm `main()` làm những gì khi vừa bật điện?

Mở file **`src/main.c`** (kéo xuống dòng **457**, hàm `int main(void)`).  
Khi vừa cấp nguồn, vi điều khiển chạy tuần tự **10 bước** từ trên xuống dưới:

| Bước | Hàm được gọi trong `main()` | Nhảy tới file nào để xem code thật? | Ý nghĩa dễ hiểu |
| :---: | :--- | :--- | :--- |
| 1 | `bsp_watchdog_init()` | `src/bsp/bsp_watchdog.c` | Kiểm tra xem lần tắt trước có phải do bị treo máy không, rồi bật bộ đếm chống treo `IWDG` (`2.5 giây`). |
| 2 | `bsp_gpio_exti_init(on_hw_exti_event)` | `src/bsp/bsp_gpio_exti.c` | Bật chân LED `PE4`, chân motor `PE5`/`PE6` (giữ tắt an toàn), chân ngõ vào `X0..X3` và gắn ngắt khẩn cho `X0` (kẹt cửa), `X1` (E-Stop). |
| 3 | `bsp_pwm_buzzer_init()` | `src/bsp/bsp_pwm_buzzer.c` | Chuẩn bị còi báo động ở chân `PF8`. |
| 4 | `bsp_adc_init()` | `src/bsp/bsp_adc_sensors.c` | Bật 5 kênh đo tương tự `ADC1` (cảm biến nhiệt `LM35`, biến trở tải trọng `VR1`, quang trở, điện áp, dòng điện). |
| 5 | `bsp_spi_hmi_init()` | `src/bsp/bsp_spi_hmi.c` | Bật bus `SPI1` (cho LED 7 đoạn + bàn phím) và `SPI2` (cho chip Flash). |
| 6 | `bsp_i2c_storage_init()` | `src/bsp/bsp_i2c_storage.c` | Bật bus `I2C1` để nói chuyện với đồng hồ `DS3231` và bộ nhớ hộp đen `AT24C512 EEPROM`. |
| 7 | `elevator_hsm_init()` | `src/core/elevator_hsm.c` | Đặt thang máy đứng yên tại **Tầng 1, cửa đóng**, bật đồng hồ nhịp `100 ms` cho máy trạng thái. |
| 8 | `can_bus_init(...)` | `src/comms/can_bus.c` | Bật mạng `CAN1` (`500 kbps`), mặc định ở chế độ tự kiểm tra `LOOPBACK` với vai trò `CABIN`. |
| 9 | `bsp_wdt_was_reset_cause()` + `bsp_blackbox_append(...)` | `src/bsp/bsp_watchdog.c` + `src/bsp/bsp_i2c_storage.c` | Ghi 1 dòng nhật ký *"Mới bật nguồn"* (hoặc *"Vừa bị reset do treo"*) kèm giờ `DS3231` vào EEPROM. |
| 10 | `k_timer_start(...)` + `bsp_pwm_buzzer_play_tone(...)` | `src/main.c` + `src/bsp/bsp_pwm_buzzer.c` | Bật 3 đồng hồ hẹn giờ định kỳ, kêu "bíp" báo khởi động xong rồi giao CPU cho **7 luồng RTOS** chạy. |

---

## 3. Sau khi bật xong: 7 luồng chạy song song làm việc gì?

Ngay phía trên `main()` trong **`src/main.c`** (dòng **436–442**), chương trình tạo sẵn **7 luồng tĩnh** bằng `K_THREAD_DEFINE`. Số ưu tiên càng nhỏ thì quyền được chạy càng cao (khi có việc gấp sẽ chen ngang luồng ưu tiên thấp ngay lập tức):

| Tên luồng | Độ ưu tiên | Hàm gốc trong `src/main.c` | Công việc hằng ngày | Các file được luồng này gọi tới |
| :--- | :---: | :--- | :--- | :--- |
| **`t1_safety`** | **2** *(Cao nhất)* | `safety_thread_entry()` (`L105`) | Thức dậy mỗi `100 ms` (hoặc khi có ngắt `EXTI`) để đo nhiệt độ `LM35`, đo tải trọng `VR1` và đọc `X0..X3`. Thấy cháy (`≥ 55°C`) hoặc quá tải (`≥ 800 kg`) là báo ngay cho bộ não. | `bsp_watchdog.c`, `bsp_adc_sensors.c`, `bsp_gpio_exti.c`, `can_bus.c`, `elevator_hsm.c` |
| **`t2_fsm`** | **3** | `fsm_thread_entry()` (`L189`) | **"Bộ não trung tâm"**: ngủ chờ thư gửi tới hòm thư `g_hsm_event_msgq`. Có lệnh gọi tầng hay báo lỗi là xử lý ngay và điều khiển đóng/mở motor. | `bsp_watchdog.c`, `elevator_hsm.c` → (`elevator_scheduler.c`, `bsp_gpio_exti.c`, `bsp_pwm_buzzer.c`, `bsp_i2c_storage.c`) |
| **`t3_can_rx`** | **4** | `can_rx_thread_entry()` (`L212`) | Nghe và giải mã gói tin mạng `CAN` gửi tới (lệnh gọi tầng từ bo khác, tin báo cháy, trả lời gói hỏi `RTR`). | `can_bus.c` → (`elevator_hsm.c`) |
| **`t3_can_tx`** | **4** | `can_tx_thread_entry()` (`L231`) | Cứ mỗi `250 ms` gửi trạng thái thang máy & nhịp tim `Heartbeat` lên mạng `CAN`; nếu bus lỗi `BUS-OFF` thì tự khôi phục. | `bsp_watchdog.c`, `elevator_hsm.c`, `can_bus.c` |
| **`t3_can_wdt`** | **4** | `can_wdt_thread_entry()` (`L252`) | Canh chừng xem đối tác bên kia mạng `CAN` có còn gửi tin không; nếu mất liên lạc quá `2 giây` thì báo lỗi `FAIL_SAFE`. | `can_bus.c` → (`elevator_hsm.c`) |
| **`t4_hmi`** | **5** | `hmi_thread_entry()` (`L283`) | Chạy mỗi `10 ms`: quét sáng 4 LED 7 đoạn (`UP`/`dn`/số tầng), tự tắt còi PWM khi hết giờ kêu, nháy LED `PE4` và quét 16 phím bấm mỗi `50 ms`. | `bsp_watchdog.c`, `elevator_hsm.c`, `can_bus.c`, `bsp_pwm_buzzer.c`, `bsp_spi_hmi.c`, `bsp_gpio_exti.c` |
| **`t5_wdt`** | **8** *(Thấp nhất)* | `telemetry_wdt_thread_entry()` (`L413`) | Chạy mỗi `100 ms`: kiểm tra 4 luồng trên có luồng nào bị đơ không để nạp `IWDG`; đồng thời cứ `200 ms` gửi chuỗi `$TWIN` JSON lên máy tính nếu bật Digital Twin. | `bsp_watchdog.c`, `twin_telemetry.c` |

> **Để khỏi xung đột phần cứng khi nhiều luồng chạy cùng lúc**, code dùng khóa `k_mutex` (tự động nhường quyền nếu luồng ưu tiên cao đang chờ):
> - `g_spi1_bus_mutex` (`bsp_spi_hmi.c`): tránh xuất LED 7 đoạn và đọc bàn phím đè nhau trên dây `SPI1`.
> - `g_i2c1_bus_mutex` (`bsp_i2c_storage.c`): tránh đọc giờ `DS3231` và ghi EEPROM đụng nhau trên dây `I2C1`.
> - `g_hsm_state_mutex` (`elevator_hsm.c`), `g_buzzer_mutex` (`bsp_pwm_buzzer.c`), `g_can_state_mutex` (`can_bus.c`).

---

## 4. Ví dụ thực tế: Lần dấu code khi thang máy hoạt động

Đây là cách nhanh nhất để hiểu toàn bộ code nối với nhau ra sao qua **3 tình huống thực tế**:

### Tình huống 1: Bạn bấm phím gọi Tầng 5 trên bàn phím của bo mạch
1. **Nhận phím bấm (`t4_hmi` trong `main.c`):** Cứ `50 ms`, luồng `t4_hmi` gọi `bsp_spi_keypad_scan()` (ở `bsp_spi_hmi.c`) để đọc phím. Thấy bạn nhấn phím tầng 5, nó tạo gói tin `EV_CALL_CABIN (floor = 5)` và bỏ vào hòm thư chung bằng `elevator_hsm_post_event()`.
2. **Ghi nhận cuốc gọi (`t2_fsm` trong `elevator_hsm.c`):** Luồng `t2_fsm` tỉnh dậy, nhận gói tin trong `elevator_hsm_step()`, rồi gọi `elev_sched_add_request()` (ở `elevator_scheduler.c`) để bật bit số 4 của `cabin_mask`.
3. **Chọn tầng đi tới (`elevator_hsm.c` ↔ `elevator_scheduler.c`):** Ở nhịp `100 ms` tiếp theo, hàm `handle_periodic_tick()` thấy thang đang đứng yên (`IDLE_CLOSED`) nên hỏi `elev_sched_pick_next_target()`. Thuật toán `LOOK` trả về *"Đi LÊN tới Tầng 5"*.
4. **Kéo thang lên & dừng đón khách dọc đường:**
   - `hsm_transition()` chuyển sang `HSM_STATE_MOVING_UP` rồi gọi `drive_interlocked_outputs()` → `bsp_gpio_set_motor_outputs(true, false)` (ở `bsp_gpio_exti.c`) để bật chân rơ-le `Y0 (PE5)` chạy lên. *(Hàm này khóa an toàn 2 lớp: nếu cửa chưa đóng `0%` hoặc `Y0` và `Y1` cùng đòi bật thì lập tức cắt sạch về `0`)*.
   - Cứ mỗi `1500 ms` qua 1 tầng, nó hỏi `elev_sched_should_stop()` xem có ai dọc đường gọi cùng chiều không; tới Tầng 5 thì mở cửa (`700 ms`), kêu "bíp" (`bsp_pwm_buzzer_play_tone`), gọi `elev_sched_clear_floor()` để xóa bit Tầng 5, chờ khách `1800 ms` rồi đóng cửa lại.

### Tình huống 2: Cửa đang đóng thì có vật cản chạm dây `X0 (PA6)` hoặc nhấn `E-Stop X1 (PA7)`
1. **Ngắt phần cứng nổ tức thì (`bsp_gpio_exti.c`):** Dòng điện kích làm hàm ngắt `exti_x0_handler()` (hoặc `exti_x1_handler()`) chạy ngay lập tức → gọi `dispatch_debounced_exti()` để chụp bộ đếm xung nhịp CPU (`k_cycle_get_32()`) và chống rung `40 ms`.
2. **Đẩy thẳng vào hòm thư (`main.c`):** Callback `on_hw_exti_event()` ném ngay sự kiện `EV_DOOR_OBSTRUCT_EXTI` (hoặc `EV_ESTOP_EXTI`) vào `elevator_hsm_post_event()` và đánh thức luồng `t1_safety`.
3. **Phản ứng an toàn (`elevator_hsm.c`):** `elevator_hsm_step()` đón sự kiện:
   - Gọi `record_exti_latency()` tính xem từ lúc ngắt phần cứng đến lúc phần mềm xử lý mất bao nhiêu **micro-giây** (`us`).
   - Nếu là kẹt cửa khi đang đóng: đảo chiều mở cửa lại đúng từ phần trăm hiện tại và phát còi báo (`2800 Hz`). Nếu là `E-Stop`: lập tức cắt động cơ sang `HSM_STATE_FAIL_SAFE`.
   - Cuối cùng gọi `bsp_blackbox_append()` (ở `bsp_i2c_storage.c`) lấy giờ từ `DS3231` và lưu sự cố vào `AT24C512 EEPROM`.

### Tình huống 3: Cảm biến nhiệt `LM35` bị hơ nóng `≥ 55°C` (Báo cháy)
1. Luồng ưu tiên cao `t1_safety` (ở `main.c`) chạy mỗi `100 ms`, gọi `bsp_adc_read_all()` (ở `bsp_adc_sensors.c`).
2. Thấy `LM35 ≥ 55.0 °C` (`550`), nó đẩy `EV_OVERHEAT_TEMP` vào `elevator_hsm_post_event()` và bắn gói CAN khẩn cấp `0x080` qua `can_bus_send_emergency()`.
3. `elevator_hsm_step()` chuyển sang `HSM_STATE_FIRE_EVAC`: gọi `elev_sched_reset()` xóa hết các cuốc gọi thường, hú còi báo cháy `3000 Hz`, ghi EEPROM, đóng cửa (nếu đang mở ở tầng trên) và ép chạy thẳng về **Tầng 1** mở cửa thoát hiểm. Nhờ lọc trễ **Hysteresis**, nhiệt độ phải nguội hẳn xuống `≤ 48.0 °C` (`480`) thì thang mới tự giải trừ báo cháy.

---

## 5. Từ điển tra nhanh: File nào chứa hàm gì?

### Ngăn Core (`src/core/` — Logic điều khiển & Xếp lịch)
- **`src/core/elevator_hsm.c`** — Máy trạng thái 9 trạng thái & Khóa an toàn tầng Core:
  - `elevator_hsm_init()`: Khởi tạo về Tầng 1, `IDLE_CLOSED`, bật timer `100 ms`.
  - `elevator_hsm_post_event()`: Cửa ngõ nhận mọi sự kiện (từ ngắt, cảm biến, phím, CAN, Shell) bỏ vào `g_hsm_event_msgq`.
  - `elevator_hsm_step()`: Vòng lặp chính của luồng `t2_fsm` lấy sự kiện ra xử lý.
  - `handle_periodic_tick()` *(static)*: Cập nhật đóng/mở cửa `700 ms`, giữ cửa `1800 ms`, chạy tầng `1500 ms`.
  - `step_fire_evac_state()` *(static)*: Quy trình sơ tán cháy chạy thẳng về Tầng 1.
  - `hsm_transition()` *(static)*: Chuyển trạng thái, đếm ma trận độ phủ `[9][9]`, kêu còi và bật/tắt motor.
  - `drive_interlocked_outputs()` *(static)*: Cấm tuyệt đối chạy motor nếu cửa chưa đóng kín hoặc đang `E-Stop`.
  - `record_exti_latency()` *(static)*: Tính độ trễ ngắt phần cứng ra micro-giây.
  - `elevator_hsm_get_telemetry()`, `elevator_hsm_get_queue_stats()`, `elevator_hsm_get_visited_state_mask()`, `elevator_hsm_get_transition_counts()`, `elevator_state_str()`, `elevator_fault_str()`: Các hàm lấy số liệu cho HMI, CAN, Shell và `$TWIN`.
- **`src/core/elevator_scheduler.c`** — Thuật toán `SCAN/LOOK` 7 tầng bằng 3 bitmask:
  - `elev_sched_add_request()`: Bật bit ghi nhận nút gọi (Cabin / Sảnh Lên / Sảnh Xuống).
  - `elev_sched_pick_next_target()`: Chọn tầng đích & hướng đi tiếp theo khi thang đang đỗ.
  - `elev_sched_should_stop()`: Kiểm tra có cần dừng gom khách dọc đường khi đang chạy không.
  - `elev_sched_clear_floor()` / `elev_sched_reset()`: Xóa bit tầng vừa phục vụ xong / xóa trắng toàn bộ khi báo cháy.
  - *(Hàm phụ `static` bên trong: `floor_bit`, `merged_requests`, `has_request_above`, `has_request_below`, `pick_upward_target`, `pick_downward_target`).*

### Ngăn BSP (`src/bsp/` — Giao tiếp phần cứng trên kit BKIT ARM4)
- **`src/bsp/bsp_gpio_exti.c`** — GPIO cách ly quang & Ngắt ngoài `PA6`/`PA7`:
  - `bsp_gpio_exti_init()`: Cài đặt toàn bộ I/O và 2 chân ngắt.
  - `exti_x0_handler()`, `exti_x1_handler()`, `dispatch_debounced_exti()` *(static)*: Xử lý sườn ngắt, chụp chu kỳ CPU và khử rung `40 ms`.
  - `bsp_gpio_set_motor_outputs()`: **Khóa liên động lớp phần cứng**, chặn `Y0 (PE5)` và `Y1 (PE6)` bật cùng lúc.
  - `bsp_gpio_set_led_debug()` / `bsp_gpio_read_inputs()`: Nháy LED `PE4` và đọc mức `X0..X3`.
- **`src/bsp/bsp_adc_sensors.c`** — Đo 5 kênh tương tự `ADC1 12-bit`:
  - `bsp_adc_init()`: Cấu hình 5 kênh từ `app.overlay`.
  - `bsp_adc_read_all()`: Đọc raw → đổi sang mV → quy đổi ra nhiệt độ `LM35` (`0.1°C`), tải trọng `VR1` (`0..1000 kg`), ánh sáng, điện áp, dòng.
- **`src/bsp/bsp_pwm_buzzer.c`** — Còi xung `PF8 (TIM13_CH1)` không làm đứng luồng:
  - `bsp_pwm_buzzer_init()`, `bsp_pwm_buzzer_beep()`, `bsp_pwm_buzzer_play_tone()` (tra 5 âm chuẩn trong bảng `k_tone_table[]`), `bsp_pwm_buzzer_mute()`, `bsp_pwm_buzzer_tick()` (hết giờ tự ngắt).
- **`src/bsp/bsp_spi_hmi.c`** — LED 7 đoạn & Phím trên `SPI1`, Flash ngoài trên `SPI2`:
  - `bsp_spi_hmi_init()`, `bsp_spi_7seg_write_raw()`, `bsp_spi_7seg_render_status()` (vẽ `F-`, `UP`, `dn`, `FE`, `Err` + vạch cuộn + số tầng), `bsp_spi_keypad_scan()` (chốt `PD3` đọc 16 phím), `bsp_spi2_probe_jedec()` (đọc ID Flash `EF 40 17`).
- **`src/bsp/bsp_i2c_storage.c`** — Đồng hồ `DS3231` & Hộp đen `AT24C512` trên `I2C1`:
  - `bsp_i2c_storage_init()`, `bsp_i2c_scan_bus()`, `bsp_rtc_get_datetime()`, `bsp_rtc_set_datetime()` (giải mã/mã hóa BCD), `bsp_blackbox_append()` (ghi vòng 32 bản ghi kèm `CRC16` + header `CRC32`), `bsp_blackbox_read()`, `bsp_blackbox_clear()`.
- **`src/bsp/bsp_watchdog.c`** — Chống treo phần cứng `IWDG` & Thông tin chip:
  - `bsp_watchdog_init()`, `bsp_wdt_checkin()` (các luồng điểm danh), `bsp_wdt_service()` (chỉ nạp `wdt_feed` nếu không luồng nào treo quá `1500 ms`), `bsp_wdt_was_reset_cause()`, `bsp_wdt_get_hwinfo()`, `bsp_wdt_slot_name()`.

### Ngăn Comms & CLI (`src/comms/`, `src/cli/`) & PC (`pc_hil_twin/`)
- **`src/comms/can_bus.c`** — Mạng `CAN1 500 kbps` hỗ trợ 1 Kit (`LOOPBACK`) hoặc 2 Kit (`NORMAL`):
  - `can_bus_init()`, `can_bus_set_mode()`, `can_bus_set_role()`: Khởi tạo đúng thứ tự driver CAN, gắn 3 bộ lọc phần cứng (`install_hw_filters`).
  - `can_bus_rx_step()` & `dispatch_rx_frame()`: Nhận khung, tự trả lời gói hỏi từ xa `RTR`, đẩy lệnh gọi tầng/khẩn cấp vào HSM, đo RTT Ping/Pong.
  - `can_bus_tx_and_recover_step()` & `can_bus_send_status()`: Gửi trạng thái `0x200` + Heartbeat `0x300` (Big-Endian) mỗi `250 ms` và phục hồi `BUS-OFF`.
  - `can_bus_partner_watchdog_step()`: Giám sát mất đối tác quá `2000 ms` để kích `FAIL_SAFE`.
  - `can_bus_send_hall_call()`, `can_bus_send_emergency()`, `can_bus_send_ping()`, `can_bus_request_cabin_rtr()`, `can_bus_set_hil_drop()`, `can_bus_get_diag()`, `can_bus_get_ext_stats()`, `can_bus_get_remote_cabin_view()`.
- **`src/comms/twin_telemetry.c`** — Đóng gói JSON `$TWIN`:
  - `twin_telemetry_set_enabled()`, `twin_telemetry_is_enabled()`, `twin_telemetry_emit_now()`.
- **`src/cli/cli_elev.c` & `src/cli/cli_bkit_test.c`** — Xử lý các lệnh gõ ở cửa sổ terminal:
  - Nhóm `elev`: `cmd_elev_status`, `cmd_elev_call`, `cmd_elev_door`, `cmd_elev_fault`, `cmd_elev_mode`, `cmd_elev_role`, `cmd_elev_twin`, `cmd_elev_log`, `cmd_elev_coverage`.
  - Nhóm `bkit`: `cmd_bkit_test_gpio`, `cmd_bkit_test_pwm`, `cmd_bkit_test_adc`, `cmd_bkit_test_i2c`, `cmd_bkit_test_rtc_set`, `cmd_bkit_test_spi1`, `cmd_bkit_test_spi2`, `cmd_bkit_test_can`, `cmd_bkit_test_wdt_hang`, `cmd_bkit_stats`.
- **`pc_hil_twin/`** — Chương trình Python chạy trên máy tính:
  - Chạy từ `elevator_twin.py` (`main()`) → mở giao diện `twin_ui.py` + vẽ giếng thang `twin_canvas.py` + nối cổng COM `twin_serial.py` (hoặc chạy thử Offline không cần mạch qua `twin_simulator.py` & `twin_scheduler.py`).

---

## 6. Thông tin phần cứng BKIT ARM4 & Giao thức CAN / EEPROM

### Gạt công tắc DIP Switch trước khi cắm điện

| Công tắc | Gạt về | Tác dụng |
| :---: | :---: | :--- |
| **SW11** | **ON** | Kết nối mạch nạp & debug ST-LINK với STM32F407 |
| **SW13** | **ON** | Kết nối chân `PB8/PB9` ra chip CAN `TJA1050` |
| **SW12** | **OFF** | Ngắt mạch RS485 để tránh nhiễu/đụng bus |
| **SW2** | **OFF** | Ngắt cổng USB-C khỏi ESP8266 khi chạy STM32 |

### Bảng chân linh kiện chính (`app.overlay`)

| Khối chức năng | Ngoại vi | Chân STM32F407 | Ghi chú |
| :--- | :--- | :--- | :--- |
| Giao tiếp PC / Shell | `USART1` | `PA9` (TX), `PA10` (RX) | Cổng RS232 (`115200 8N1`) |
| LED3 & Motor Lên/Xuống | `GPIO Out` | `PE4` (LED3), `PE5` (Y0 UP), `PE6` (Y1 DOWN) | Có khóa liên động chống bật `Y0` và `Y1` cùng lúc |
| Ngõ vào cách ly & Ngắt | `GPIO / EXTI` | `PA6` (X0 kẹt cửa), `PA7` (X1 E-Stop), `PC4` (X2), `PC5` (X3 quá tải) | Kéo lên nội, `X0`/`X1` chạy ngắt phần cứng |
| Còi báo | `TIM13_CH1` | `PF8` | Phát PWM 50% duty |
| Cảm biến tương tự | `ADC1` 12-bit | `PC2` (`LM35`), `PC1` (`VR1`), `PC0` (Quang trở), `PB0` (Áp), `PB1` (Dòng) | Quy đổi `10 mV/°C` (`LM35`) và `0..1000 kg` (`VR1`) |
| 4 LED 7 đoạn + 16 Phím | `SPI1` | `PB3/PB4/PB5`, `PG6` (`LD_LATCH`), `PD3` (`BTN_LOAD`) | `2x74HC595` (LED 7 đoạn) dùng chung bus với `2x74HC165` (Phím) |
| Flash ngoài | `SPI2` | `PB13/PB14/PB15`, `PB12` (`CS`) | Chip `W25Q64FV` (`EF 40 17`) |
| Đồng hồ & Hộp đen | `I2C1` | `PB6` (SCL), `PB7` (SDA) | `DS3231` (`0x68`) + `AT24C512` (`0x50`) |
| Mạng CAN 500 kbps | `bxCAN1` | `PB8` (RX), `PB9` (TX) | Filter HW: `0x080` (Khẩn cấp), `0x100..0x10F` (Gọi tầng), `0x200..0x3FF` (Status/HB/Ping) |

- **Bộ nhớ Hộp đen AT24C512:** Header nằm ở trang `0x0000` (`magic = "ELEV"`, kiểm tra `CRC32-IEEE`); nối tiếp từ `0x0080` lưu tối đa **32 bản ghi** theo kiểu vòng tròn (mỗi slot `32 B` chứa struct `elevator_blackbox_rec_t` `22 B` kèm `CRC16-CCITT` và thời gian từ `DS3231`).

---

## 7. Cách Build, Nạp code & Chạy giao diện PC Digital Twin

### A. Biên dịch & Nạp xuống mạch STM32 (Windows PowerShell)

```powershell
# Chạy từ thư mục gốc của repo (nơi chứa CMakeLists.txt & app.overlay)
$env:ZEPHYR_BASE = "C:\Users\ASUS\zephyrproject\zephyr"

# Biên dịch toàn bộ firmware
C:\Users\ASUS\zephyrproject\.venv\Scripts\west.exe build -p always -b stm32f4_disco .

# Nạp code qua ST-LINK (nhớ gạt SW11 = ON)
C:\Users\ASUS\zephyrproject\.venv\Scripts\west.exe flash
```

> Lần sau sửa code muốn build nhanh hơn, có thể bỏ `-p always` (`west build -b stm32f4_disco .`).

### B. Mở giao diện mô phỏng & theo dõi trên máy tính (`pc_hil_twin/`)

```powershell
# Chỉ cần cài pyserial nếu cắm cáp RS232 nối với mạch thật (chế độ Offline không cần)
pip install pyserial

# Chạy giao diện mô phỏng 7 tầng
python pc_hil_twin\elevator_twin.py
```

- **Chưa cắm mạch:** Bấm **▶ Khởi động Offline Sim** để thử chạy mô phỏng ngay trên máy tính.
- **Đã cắm mạch qua RS232 (`115200 8N1`):** Chọn đúng cổng `COMx` → bấm **Kết nối RS232** → bật `elev twin on` để giếng thang trên PC tự đồng bộ theo mạch thật mỗi `200 ms`.

---

## 8. Các lệnh gõ thử qua cổng UART (`bkit` & `elev`)

Mở terminal nối cổng COM ở tốc độ **`115200 8N1`** sẽ thấy dấu nhắc `bkit-elev:~$ `:

### Lệnh kiểm tra từng bộ phận phần cứng (`bkit`)
- `bkit test gpio`: Nháy đèn `PE4`, thử motor `Y0/Y1`, in trạng thái `X0..X3`.
- `bkit test pwm [freq]`: Kêu còi `PF8` thử trong `250 ms` (mặc định `2400 Hz`).
- `bkit test adc`: In giá trị nhiệt độ `LM35`, tải trọng `VR1`, ánh sáng, áp, dòng.
- `bkit test i2c`: Quét xem có thấy `0x50` (EEPROM) và `0x68` (RTC) không và đọc giờ hiện tại.
- `bkit test rtc_set <Năm> <Tháng> <Ngày> <Giờ> <Phút> <Giây>`: Chỉnh lại giờ cho chip `DS3231`.
- `bkit test spi1 [hex]`: Thử xuất LED 7 đoạn và đọc thử 1 lần phím đang nhấn (`0..15`).
- `bkit test spi2`: Đọc mã ID chip Flash `W25Q64FV` (ra `EF 40 17` là đạt).
- `bkit test can [rtr]`: Bắn thử gói CAN `PING` (`0x380`) hoặc hỏi từ xa `RTR` (`0x200`) và xem thời gian phản hồi `RTT us`.
- `bkit test wdt_hang`: Cố tình ngừng nạp Watchdog để mạch tự reset sau `~2.5 giây` và lưu sự kiện `BOOT_WDT`.
- `bkit stats`: Xem lý do reset, mã chip UID 96-bit và lượng RAM stack còn dư của cả 7 luồng.

### Lệnh điều khiển thang máy & thử lỗi (`elev`)
- `elev status`: In toàn bộ trạng thái thang, tầng hiện tại/đích, hàng đợi gọi tầng, ngõ vào `X0..X3`, `ADC`, độ trễ ngắt và thống kê `CAN`.
- `elev call <1..7> [cabin|up|down]`: Bấm gọi tầng (ví dụ: `elev call 5 cabin`, `elev call 3 up`).
- `elev door <open|close|obstruct>`: Mở/đóng cửa hoặc giả lập kẹt cửa `X0`.
- `elev fault <fire|overload|can_drop|estop|clear>`: Bơm thử sự cố (cháy, quá tải, đứt dây CAN `2 giây`, `E-Stop`) hoặc `clear` để xóa chốt lỗi.
- `elev mode <loopback|normal>`: Chọn tự test trên 1 bo (`loopback`) hay nối dây 2 bo (`normal`).
- `elev role <cabin|hall>`: Chọn bo làm tủ chính (`cabin`) hay bảng gọi ngoài sảnh (`hall`).
- `elev twin <on|off>`: Bật/tắt gửi dữ liệu `$TWIN` cho giao diện Python.
- `elev log [read|clear]`: Đọc nhật ký hộp đen trong EEPROM (tối đa 16 bản ghi mới nhất) hoặc xóa trắng.
- `elev coverage`: Xem thang máy đã chạy qua đủ `9/9` trạng thái chưa.

---

### Mức chiếm dụng bộ nhớ sau khi biên dịch (`stm32f4_disco / stm32f407xx`)

```text
Memory region         Used Size  Region Size  %age Used
           FLASH:      104112 B         1 MB      9.93%
             RAM:       28608 B       128 KB     21.83%
```
