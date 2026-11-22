# Vận hành Hệ thống Thang máy Thông minh 7 Tầng trên Zephyr RTOS & STM32F407

Dự án điều khiển thang máy **7 tầng** an toàn trọng yếu (Safety-Critical & Hard Real-Time) chạy trên hệ điều hành thời gian thực **Zephyr RTOS v4.4.0** cho bo mạch **BKIT ARM4 (`STM32F407ZGT6`)**, tích hợp giao diện HMI kép (Cụm 4 LED 7 đoạn ngoài sảnh + Màn hình màu cảm ứng **TFT-LCD 240x320 qua bus song song FSMC 16-bit**) kèm phần mềm mô phỏng & giám sát thời gian thực **PC Digital Twin (`pc_hil_twin/`)**.

---

## Mục lục

1. [Hiểu nhanh trong 30 giây: Cấu trúc thư mục & Module](#1-hiểu-nhanh-trong-30-giây-cấu-trúc-thư-mục--module)
2. [Đọc từ điểm bắt đầu: Hàm `main()` làm những gì khi vừa bật điện?](#2-đọc-từ-điểm-bắt-đầu-hàm-main-làm-những-gì-khi-vừa-bật-điện)
3. [Sau khi bật xong: 7 luồng RTOS chạy song song làm việc gì?](#3-sau-khi-bật-xong-7-luồng-rtos-chạy-song-song-làm-việc-gì)
4. [Nguyên lý điều phối LOOK/SCAN & Bốn quy tắc an toàn bất biến](#4-nguyên-lý-điều-phối-lookscan--bốn-quy-tắc-an-toàn-bất-biến)
5. [Giao diện HMI Kép: Cụm 4 LED 7 đoạn & Màn hình màu TFT-LCD 240x320 (FSMC)](#5-giao-diện-hmi-kép-cụm-4-led-7-đoạn--màn-hình-màu-tft-lcd-240x320-fsmc)
6. [Bảng tra cứu 16 phím bấm Ma trận phím 4x4 (`SPI1`)](#6-bảng-tra-cứu-16-phím-bấm-ma-trận-phím-4x4-spi1)
7. [Kịch bản Demo thực nghiệm chi tiết từng bước trên Bo mạch](#7-kịch-bản-demo-thực-nghiệm-chi-tiết-từng-bước-trên-bo-mạch)
8. [Từ điển tra nhanh: File nào chứa hàm gì?](#8-từ-điển-tra-nhanh-file-nào-chứa-hàm-gì)
9. [Thông tin phần cứng BKIT ARM4, Công tắc gạt & Giao thức CAN / EEPROM](#9-thông-tin-phần-cứng-bkit-arm4-công-tắc-gạt--giao-thức-can--eeprom)
10. [Cách Build, Nạp code & Chạy giao diện PC Digital Twin](#10-cách-build-nạp-code--chạy-giao-diện-pc-digital-twin)
11. [Các lệnh gõ thử qua cổng UART Console (`bkit` & `elev`)](#11-các-lệnh-gõ-thử-qua-cổng-uart-console-bkit--elev)

---

## 1. Hiểu nhanh trong 30 giây: Cấu trúc thư mục & Module

Dự án được cấu trúc thành các ngăn độc lập theo tiêu chuẩn phân tầng hệ thống nhúng:

```text
smart-elevator-zephyr-stm32/
├── app.overlay & prj.conf        <-- Cấu hình chân vi điều khiển BKIT ARM4 & tham số Zephyr RTOS
├── CMakeLists.txt                <-- Định nghĩa target nguồn biên dịch bằng CMake/Ninja
├── include/                      <-- Header files: kiểu dữ liệu (elevator_types.h), API HSM, BSP, CAN, LCD
│   ├── bsp_tft_lcd.h             <-- Trình điều khiển màn hình màu TFT-LCD 240x320 qua FSMC 16-bit
│   ├── bsp_spi_hmi.h             <-- Trình điều khiển cụm LED 7 đoạn (74HC595) & bàn phím (74HC165)
│   ├── elevator_hsm.h            <-- Máy trạng thái phân cấp an toàn 9 trạng thái
│   └── elevator_scheduler.h      <-- Bộ lập lịch điều phối gom khách LOOK/SCAN 7 tầng
├── src/
│   ├── main.c                    <-- ĐIỂM BẮT ĐẦU: Khởi tạo phần cứng + định nghĩa 7 luồng RTOS tĩnh
│   ├── core/                     <-- BỘ NÃO: elevator_hsm.c (HSM Core) + elevator_scheduler.c (LOOK)
│   ├── bsp/                      <-- DRIVER PHẦN CỨNG: GPIO/EXTI, ADC1, PWM, SPI1/2, I2C1, IWDG, TFT-LCD
│   │   ├── bsp_tft_lcd.c         <-- Driver đồ họa ILI9341 FSMC 16-bit, cuộn số, dashboard 4 khối
│   │   ├── bsp_spi_hmi.c         <-- Quét LED 7 đoạn (vạch cuộn), đọc 16 phím bàn phím ma trận
│   │   ├── bsp_adc_sensors.c     <-- Đọc LM35 (nhiệt độ), VR1 (tải trọng), LDR (rèm quang), áp/dòng
│   │   ├── bsp_gpio_exti.c       <-- Khóa liên động cứng Y0/Y1, ngắt khẩn cấp EXTI X0 (kẹt cửa)/X1 (E-Stop)
│   │   ├── bsp_pwm_buzzer.c      <-- Điều khiển còi PWM PF8 không chặn (non-blocking)
│   │   ├── bsp_i2c_storage.c     <-- Hộp đen AT24C512 EEPROM (32 slot CRC32) + RTC DS3231
│   │   └── bsp_watchdog.c        <-- Giám sát điểm danh 4 luồng RTOS nạp phần cứng IWDG 2.5s
│   ├── comms/                    <-- TRUYỀN THÔNG: can_bus.c (Mạng CAN 500kbps) + twin_telemetry.c ($TWIN JSON)
│   └── cli/                      <-- DÒNG LỆNH SHELL: cli_elev.c (Lệnh thang máy) + cli_bkit_test.c (Tự test phần cứng)
└── pc_hil_twin/                  <-- Phần mềm mô phỏng HIL Digital Twin bằng Python Tkinter trên máy tính
```

---

## 2. Đọc từ điểm bắt đầu: Hàm `main()` làm những gì khi vừa bật điện?

Mở file **`src/main.c`** (hàm `int main(void)`). Khi vừa cấp nguồn hoặc ấn nút Reset, chip `STM32F407ZGT6` chạy tuần tự **11 bước khởi tạo**:

| Bước | Hàm được gọi trong `main()` | File nguồn tương ứng | Vai trò trong hệ thống |
| :---: | :--- | :--- | :--- |
| 1 | `bsp_watchdog_init()` | `src/bsp/bsp_watchdog.c` | Bật bộ đếm chống treo cứng `IWDG` (`2.5 giây`), kiểm tra mã cờ reset trước đó. |
| 2 | `bsp_gpio_exti_init(...)` | `src/bsp/bsp_gpio_exti.c` | Cấu hình ngõ ra mô-tơ `Y0/Y1`, đèn `PE4`, và đăng ký ngắt cứng `EXTI` cho `X0` (kẹt cửa) và `X1` (E-Stop). |
| 3 | `bsp_pwm_buzzer_init()` | `src/bsp/bsp_pwm_buzzer.c` | Khởi tạo xung PWM trên chân `PF8 (TIM13_CH1)` điều khiển còi báo. |
| 4 | `bsp_adc_init()` | `src/bsp/bsp_adc_sensors.c` | Cấu hình 5 kênh đo tương tự `ADC1 12-bit` (`LM35`, `VR1`, quang trở `LDR`, áp `PB0`, dòng `PB1`). |
| 5 | `bsp_spi_hmi_init()` | `src/bsp/bsp_spi_hmi.c` | Cấu hình bus `SPI1` dùng chung cho cụm LED 7 đoạn (`PG6`) và bàn phím 4x4 (`PD3`). |
| 6 | `bsp_tft_lcd_init()` | `src/bsp/bsp_tft_lcd.c` | **Cấu hình bus song song FSMC 16-bit** (`Bank 1 NE1`), bật đèn nền `PA8` và gửi chuỗi lệnh khởi động cho chip `ILI9341`. |
| 7 | `bsp_i2c_storage_init()` | `src/bsp/bsp_i2c_storage.c` | Khởi tạo bus `I2C1` kết nối đồng hồ `DS3231` và chip nhớ hộp đen `AT24C512 EEPROM`. |
| 8 | `elevator_hsm_init()` | `src/core/elevator_hsm.c` | Đưa thang máy về trạng thái `IDLE_CLOSED` tại Tầng 1, bật timer nhịp `100 ms`. |
| 9 | `can_bus_init(...)` | `src/comms/can_bus.c` | Khởi tạo mạng `bxCAN1 500 kbps` với bộ lọc phần cứng (chế độ `LOOPBACK` hoặc `NORMAL`). |
| 10 | `bsp_blackbox_append(...)` | `src/bsp/bsp_i2c_storage.c` | Ghi 1 bản ghi bật nguồn (`BB_FAULT_BOOT_POR` hoặc `BB_FAULT_BOOT_WATCHDOG`) kèm giờ RTC vào EEPROM. |
| 11 | `k_timer_start(...)` + `bsp_pwm_buzzer_play_tone(...)` | `src/main.c` | Kích hoạt 3 `k_timer` định kỳ, phát âm boot tone và nhường CPU cho **7 luồng RTOS**. |

---

## 3. Sau khi bật xong: 7 luồng RTOS chạy song song làm việc gì?

Chương trình tạo sẵn **7 luồng tĩnh** bằng macro `K_THREAD_DEFINE` trong `src/main.c`. Mức ưu tiên được phân bổ theo nguyên lý **Deadline Monotonic** (số càng nhỏ ưu tiên càng cao):

| Luồng | Ưu tiên | Chu kỳ / Đánh thức | Nhiệm vụ chính | Các file được gọi |
| :--- | :---: | :---: | :--- | :--- |
| **`t1_safety`** | **2** *(Cao nhất)* | `100 ms` hoặc `EXTI` | Canh gác nhiệt độ `LM35`, tải trọng `VR1`, rèm quang và ngõ vào `X0..X3`. Phát hiện cháy (`≥ 55°C`) hoặc quá tải (`≥ 800 kg`) đẩy sự kiện khẩn vào HSM. | `bsp_adc_sensors.c`, `bsp_gpio_exti.c`, `can_bus.c`, `elevator_hsm.c` |
| **`t2_fsm`** | **3** | Hướng sự kiện (`MsgQ`) | **Bộ não điều khiển trung tâm**: Chạy máy trạng thái HSM 9 trạng thái, tính toán chuyển tầng, khóa liên động motor và cửa. | `elevator_hsm.c`, `elevator_scheduler.c`, `bsp_gpio_exti.c`, `bsp_i2c_storage.c` |
| **`t3_can_rx`** | **4** | Sự kiện khung đến | Nhận và giải mã khung mạng CAN từ bộ lọc phần cứng (gọi tầng `0x100`, báo cháy, trả lời `RTR`). | `can_bus.c`, `elevator_hsm.c` |
| **`t3_can_tx`** | **4** | Chu kỳ `250 ms` | Phát định kỳ khung trạng thái `0x200` + Heartbeat `0x300` lên bus CAN; tự động phục hồi lỗi `BUS-OFF`. | `can_bus.c`, `elevator_hsm.c` |
| **`t3_can_wdt`** | **4** | Chu kỳ `500 ms` | Giám sát mất kết nối đối tác CAN quá `2000 ms` để kích hoạt Fail-Safe. | `can_bus.c`, `elevator_hsm.c` |
| **`t4_hmi`** | **5** | `10 ms` / `100 ms` | **Giao tiếp HMI Kép**: Quét 4 LED 7 đoạn (100 Hz), render Dashboard TFT-LCD 240x320 (10 Hz), quét bàn phím ma trận (50 ms), nháy LED `PE4`. | `bsp_spi_hmi.c`, `bsp_tft_lcd.c`, `bsp_pwm_buzzer.c`, `bsp_gpio_exti.c` |
| **`t5_wdt`** | **8** *(Thấp nhất)* | `100 ms` / `200 ms` | Kiểm tra điểm danh 4 luồng trên để nạp `IWDG`; xuất chuỗi JSON `$TWIN` lên cổng nối tiếp máy tính. | `bsp_watchdog.c`, `twin_telemetry.c` |

> **Cơ chế chống xung đột tài nguyên:**
> - `g_spi1_bus_mutex`: Khóa đồng bộ bảo vệ bus SPI1 giữa quét LED 7 đoạn (74HC595) và đọc bàn phím (74HC165).
> - `g_i2c1_bus_mutex`: Khóa bảo vệ bus I2C1 giữa đọc RTC DS3231 và ghi nhật ký Hộp đen EEPROM AT24C512.
> - Cả hai Mutex đều hỗ trợ **Priority Inheritance (Kế thừa độ ưu tiên)** chống đảo ngược ưu tiên (Priority Inversion).

---

## 4. Nguyên lý điều phối LOOK/SCAN & Bốn quy tắc an toàn bất biến

### 4.1. Thuật toán điều phối LOOK / SCAN hai chiều (`src/core/elevator_scheduler.c`)
Thuật toán quản lý bằng 3 mặt nạ bit 7-bit (`cabin_mask`, `hall_up_mask`, `hall_down_mask`):
* **Gom khách cùng chiều:** Khi thang đang chạy LÊN, nó sẽ dừng đón tất cả hành khách có nhu cầu đi LÊN trên đường đi.
* **Bỏ qua tạm thời khách ngược chiều:** Nếu ở tầng trung gian có người bấm gọi XUỐNG, thang đang lên sẽ **lướt qua không dừng** để bảo đảm thời gian di chuyển của khách trong cabin.
* **Đảo chiều đón vét:** Khi thang lên đến tầng cao nhất của lượt quét và phía trên hết sạch yêu cầu, nó mới tự động đảo chiều chạy XUỐNG và ghé đón người khách gọi xuống lúc nãy.

### 4.2. Bốn quy tắc an toàn bất biến (Safety Invariants)
1. **Khóa liên động phần cứng (Hardware Interlock):** Ngõ ra `Y0 (PE5 - Lên)` và `Y1 (PE6 - Xuống)` không bao giờ được phép bật cùng lúc ở cả tầng driver lẫn tầng logic. Nếu có lỗi, hàm `bsp_gpio_set_motor_outputs` dập cả 2 về 0 ngay lập tức.
2. **Cửa chưa đóng kín (door_pct > 0) thì mô-tơ cấm chạy.**
3. **Lọc trễ Hysteresis chống chập chờn:**
   * Quá tải: Bật khi tải $\ge 800\text{ kg}$, chỉ tắt khi tải giảm hẳn xuống $\le 750\text{ kg}$.
   * Báo cháy: Bật khi nhiệt độ $\ge 55^\circ\text{C}$, chỉ giải trừ khi nhiệt độ hạ xuống $\le 48^\circ\text{C}$.
4. **Giám sát nhịp sống Watchdog đa luồng:** Nếu có bất kỳ luồng RTOS nào bị treo quá `1500 ms`, bộ đếm `IWDG` phần cứng hết hạn sau `2.5 giây` sẽ tự kích hoạt Hardware Reset chip STM32.

---

## 5. Giao diện HMI Kép: Cụm 4 LED 7 đoạn & Màn hình màu TFT-LCD 240x320 (FSMC)

### 5.1. Bảng trạng thái Cụm 4 LED 7 đoạn (`SPI1`)
| Trạng thái | Hiển thị | Mô tả hiệu ứng |
| :--- | :---: | :--- |
| **Đang đỗ tại Tầng N** | `F - N` | Ký tự 'F' ở LED 1, dấu gạch ngang ở LED 2, số tầng N ở LED 4. |
| **Đang chạy LÊN Tầng N** | `U P N` | Chữ 'UP' ở 2 LED đầu. Ô LED thứ 3 có **vạch ngang cuộn từ đáy lên đỉnh** (`_` $\rightarrow$ `-` $\rightarrow$ `¯`). |
| **Đang chạy XUỐNG Tầng N** | `d n N` | Chữ 'dn' ở 2 LED đầu. Ô LED thứ 3 có **vạch ngang cuộn từ đỉnh xuống đáy** (`¯` $\rightarrow$ `-` $\rightarrow$ `_`). |
| **Cửa đang mở** | `O P N` | Chữ 'OP' (Open) kèm số tầng N. |
| **Sự cố Quá tải** | `F U L L` | Nhấp nháy 4 chữ 'FULL' dồn dập + còi kêu liên tục. |
| **Sự cố Báo cháy** | `F E 1` | Nhấp nháy chữ 'FE' (Fire Evacuation) + còi hú hồi dài. |
| **Dừng khẩn / Lỗi** | `E r r` | Khóa cứng chữ 'Err' hoặc 'STOP'. |

### 5.2. Màn hình màu TFT-LCD 240x320 (Giao tiếp FSMC 16-bit)
* **Khối 1 - Top Bar (0 - 28 px):** Tiêu đề `BKIT ELEVATOR`, Đồng hồ RTC DS3231 (`HH:MM:SS`), Trạng thái mạng `[CAN:OK]`.
* **Khối 2 - Hero Section (30 - 155 px):** Mũi tên chỉ hướng `▲ UP` / `▼ DOWN` / `— IDLE`; Số tầng lớn $24 \times 32\text{ px}$ có **hiệu ứng cuộn trượt dọc** (Vertical Rolling Animation 40-50 FPS); Hàng 7 nút gọi tầng căn giữa hoàn hảo ($204\text{ px}$, lề 2 bên $18\text{ px}$, nút nào được gọi sáng rực màu vàng).
* **Khối 3 - Cửa & Rèm quang (160 - 225 px):** Thanh tiến trình đóng/mở cửa $0\% - 100\%$, đèn báo cảm biến rèm quang `Light Curtain: CLEAR / BEAM BLOCKED`.
* **Khối 4 - Cảm biến & Hộp đen (230 - 315 px):** Thanh đo tải trọng (VR1), Nhiệt độ giếng thang (LM35), dòng nhật ký Hộp đen EEPROM mới nhất.
* **Màn hình Khẩn cấp (Emergency Overlays):** Tự động bật Popup **ĐỎ RỰC** khi Quá tải (> 800 kg) hoặc Báo cháy ($\ge 55^\circ\text{C}$), Popup **VÀNG CAM** khi Dừng khẩn cấp.

---

## 6. Bảng tra cứu 16 phím bấm Ma trận phím 4x4 (`SPI1`)

| Mã Phím | Tọa độ | Chức năng điều khiển | Hành vi hệ thống |
| :---: | :---: | :--- | :--- |
| **Phím 0..6** | Hàng 0, Cột 0 $\rightarrow$ Hàng 1, Cột 2 | **Gọi Tầng 1..7 (Cabin)** | Còi chíp ngắn, ô số trên LCD sáng rực màu vàng, thang chạy đến đón. |
| **Phím 7** | Hàng 1, Cột 3 | **Mở cửa nhanh (Door Open)** | Cửa đang đóng dở lập tức mở ngược ra 100%. |
| **Phím 8** | Hàng 2, Cột 0 | **Đóng cửa nhanh (Door Close)** | Rút ngắn thời gian chờ, cửa lập tức khép lại 0%. |
| **Phím 9** | Hàng 2, Cột 1 | **Test Kẹt cửa (Opto X0)** | Bơm sự cố vật cản: cửa mở lại, LCD báo Vàng Cam. |
| **Phím 10** | Hàng 2, Cột 2 | **Test Báo cháy (62°C)** | Bơm sự cố quá nhiệt: Còi hú, LCD đỏ rực, thang lao thẳng về Tầng 1 mở toang cửa sơ tán. |
| **Phím 11** | Hàng 2, Cột 3 | **Dừng Khẩn Cấp (E-Stop)** | Cắt nguồn mô-tơ tức thời (< 12 µs), LED hiện Err, LCD báo khóa hệ thống. |
| **Phím 12..15** | Toàn bộ Hàng 3 | **Xóa lỗi / Reset hệ thống** | Giải trừ trạng thái sự cố, đưa thang máy về hoạt động bình thường. |

---

## 7. Kịch bản Demo thực nghiệm chi tiết từng bước trên Bo mạch

### Kịch bản 1: Cấp nguồn & Tự kiểm tra hệ thống (Power-On Self-Test)
1. Bấm nút đen **RESET (RST)** trên bo mạch BKIT ARM4.
2. Còi Buzzer phát 1 hồi chuông "Bính... Boong" chào mừng.
3. LED Heartbeat `PE4` nháy đều nhịp $800\text{ ms}$. Cụm LED 7 đoạn hiện `F - 1`.
4. Màn hình TFT-LCD hiển thị Dashboard ban đầu: Đồng hồ RTC đang nhảy giây, số tầng lớn `[ 1 ]`, ô số 1 có viền xanh lá, rèm quang `CLEAR (SAFE)`.

### Kịch bản 2: Gọi tầng đơn lẻ & Hiệu ứng chuyển tầng mượt
1. Bấm **Phím 3** trên bàn phím (Gọi lên **Tầng 4**).
2. Ô số **[4]** trên LCD sáng vàng, dòng target hiện `Target Floor: 4`. Đèn LED `Y0` trên bo mạch bật sáng (mô-tơ chạy LÊN).
3. LED 7 đoạn chuyển `UP 1` $\rightarrow$ `UP 2` $\rightarrow$ `UP 3` $\rightarrow$ `UP 4` kèm vạch cuộn động. Màn hình LCD cuộn số trượt dọc từ 1 lên 4.
4. Đến Tầng 4: Đèn LED `Y0` tắt, còi reo reo báo đến nơi, LED 7 đoạn hiện `OP 4`, thanh cửa mở toang $100\%$, ô [4] tắt màu vàng. Sau $1.8\text{ s}$, cửa tự đóng lại.

### Kịch bản 3: Gom khách thông minh khi bấm nhiều tầng cùng lúc (Thuật toán LOOK/SCAN)
1. Thang đang ở Tầng 1. Bạn bấm liên tiếp: **Phím 2** (Tầng 3), **Phím 5** (Tầng 6). Trong lúc thang đang chạy từ F1 lên F3, bạn bấm tiếp **Phím 3** (Tầng 4) và **Phím 0** (Tầng 1 - nằm phía sau lưng).
2. LCD sáng vàng đồng loạt các ô: **`[1]`**, **`[3]`**, **`[4]`**, **`[6]`**.
3. **Diễn biến:**
   * Thang ghé dừng ở **Tầng 3**: Mở cửa đón khách, ô [3] tắt đèn vàng.
   * Thang chạy tiếp lên và ghé dừng ở **Tầng 4**: Mở cửa đón khách, ô [4] tắt đèn vàng.
   * Thang **KHÔNG BAO GIỜ quay đầu về Tầng 1 giữa chừng**! Nó tiếp tục chạy lên và ghé dừng ở **Tầng 6** (trả khách tầng đỉnh, ô [6] tắt vàng).
   * Khi đến Tầng 6, phía trên hết yêu cầu $\rightarrow$ Thang tự động **đảo chiều sang ▼ DOWN**, đèn LED `Y1` bật sáng, kéo một mạch chạy thẳng xuống **Tầng 1** đón khách!

### Kịch bản 4: Thử nghiệm Nút Đóng / Mở cửa nhanh (Phím 7 & Phím 8)
* Khi cửa đang đóng dở ở Tầng 4: Bấm **Phím 7** $\rightarrow$ Cửa lập tức đảo chiều mở toang ra $100\%$.
* Khi cửa đang mở đón khách: Bấm **Phím 8** $\rightarrow$ Cắt ngắn thời gian chờ $1.8\text{ s}$, cửa khép lại ngay tức thì.

### Kịch bản 5: Thử nghiệm Cảm biến Rèm quang Chống kẹp cửa (Light Curtain)
* Bấm gọi tầng bất kỳ. Trong lúc cửa đang khép lại (`DOOR: MOVING`):
  * Lấy ngón tay **che kín cảm biến ánh sáng LDR (chân PC0)** trên mặt bo mạch (hoặc bấm **Phím 9**).
* **Kết quả:** Cửa đang khép lập tức **dừng lại và mở ngược ra $100\%$**; màn hình LCD hiện thanh cảnh báo màu VÀNG CAM `DOOR: OBSTACLE! RE-OPENING`.

### Kịch bản 6: Thử nghiệm Cảm biến Quá tải Cabin (Biến trở VR1)
* Dùng tay vặn núm xoay biến trở **VR1 (chân PC1)** theo chiều kim đồng hồ lên mức cao ($> 80\%$, tức $> 800\text{ kg}$).
* **Kết quả:** Còi kêu bíp bíp liên tục, LED 7 đoạn nhấp nháy chữ **`FULL`**, LCD bật Popup **ĐỎ RỰC** `!! OVERLOAD DETECTED !!`, khóa cứng mô-tơ không cho chạy. Vặn VR1 về $< 750\text{ kg}$ hệ thống khôi phục bình thường.

### Kịch bản 7: Thử nghiệm Báo cháy Khẩn cấp & Sơ tán về Tầng 1 (LM35)
* Bấm **Phím 10** trên bàn phím (hoặc hơ nóng cảm biến nhiệt LM35 chân PC2 vượt $55^\circ\text{C}$).
* **Kết quả:** Còi hú liên tục hồi dài, LED Debug PE4 nháy dồn dập $200\text{ ms}$, LED 7 đoạn hiện `FE 1`, LCD đổi sang giao diện **ĐỎ RỰC TOÀN MÀN HÌNH**, thang hủy toàn bộ lệnh khác tự động chạy thẳng xuống **Tầng 1** mở toang cửa sơ tán. Bấm **Phím 12** để xóa lỗi.

### Kịch bản 8: Phanh gấp Dừng khẩn cấp (Emergency Stop)
* Khi thang đang chạy, bấm **Phím 11** (hoặc kích hoạt opto X1 PA7).
* **Kết quả:** Mô-tơ bị cắt nguồn ngay lập tức (dưới $12\ \mu\text{s}$), LED hiện `Err`, LCD hiện Popup dừng khẩn cấp. Bấm **Phím 12** để khôi phục.

---

## 8. Từ điển tra nhanh: File nào chứa hàm gì?

* **`src/bsp/bsp_tft_lcd.c`** & **`include/bsp_tft_lcd.h`**:
  * `bsp_tft_lcd_init()`: Khởi tạo GPIO, thanh ghi FSMC 16-bit và chip ILI9341.
  * `bsp_tft_lcd_fill_rect()`, `bsp_tft_lcd_draw_string()`, `bsp_tft_lcd_draw_big_digit()`, `bsp_tft_lcd_draw_progress_bar()`: Các hàm vẽ 2D cơ bản.
  * `bsp_tft_lcd_render_dashboard()`: Render giao diện HMI 4 khối và các popup sự cố.
  * `bsp_tft_lcd_animate_floor_rolling()`: Hiệu ứng hoạt họa cuộn trượt số tầng dọc.
* **`src/core/elevator_scheduler.c`**:
  * `elev_sched_add_request()` / `elev_sched_clear_floor()` / `elev_sched_reset()`: Quản lý hàng đợi bitmask.
  * `elev_sched_pick_next_target()` / `elev_sched_should_stop()`: Thuật toán LOOK hai chiều gom khách.
* **`src/core/elevator_hsm.c`**:
  * `elevator_hsm_step()`: Máy trạng thái hữu hạn 9 trạng thái.
  * `drive_interlocked_outputs()`: Khóa an toàn 2 lớp mô-tơ và cửa.
* **`src/bsp/bsp_spi_hmi.c`**:
  * `bsp_spi_7seg_render_status()`: Xuất mã 7 đoạn kèm vạch cuộn động lên 4 LED.
  * `bsp_spi_keypad_scan()`: Quét 16 phím bàn phím ma trận qua SPI1.
* **`src/bsp/bsp_i2c_storage.c`**:
  * `bsp_blackbox_append()` / `bsp_blackbox_read()`: Ghi/đọc 32 bản ghi hộp đen có CRC32 trên EEPROM AT24C512.
  * `bsp_rtc_get_datetime()` / `bsp_rtc_set_datetime()`: Đọc/cài đặt ngày giờ RTC DS3231.
* **`src/bsp/bsp_watchdog.c`**:
  * `bsp_wdt_checkin()` / `bsp_wdt_service()`: Quản lý điểm danh 4 luồng nạp IWDG.

---

## 9. Thông tin phần cứng BKIT ARM4, Công tắc gạt & Giao thức CAN / EEPROM

### Vị trí công tắc gạt DIP Switch trước khi cắm điện:
* **SW11 = ON**: Kết nối mạch nạp & debug ST-LINK với chip STM32F407.
* **SW13 = ON**: Kết nối chân `PB8/PB9` vào chip thu phát CAN `TJA1050`.
* **SW12 = OFF**: Ngắt cổng RS485 tránh xung đột bus.
* **SW2 = OFF**: Ngắt USB-C khỏi ESP8266 khi chạy STM32.

### Bảng chân phần cứng chi tiết (`app.overlay`):
| Khối chức năng | Ngoại vi | Chân STM32F407 | Chức năng phần cứng |
| :--- | :--- | :--- | :--- |
| **Giao tiếp PC Console** | `USART1` | `PA9` (TX), `PA10` (RX) | Cổng RS232 (`115200 8N1`) |
| **LED Debug & Motor** | `GPIO Out` | `PE4` (LED Debug), `PE5` (Y0 Lên), `PE6` (Y1 Xuống) | Khóa liên động cứng chống bật đồng thời |
| **Ngõ vào cách ly quang** | `GPIO / EXTI` | `PA6` (X0 Kẹt cửa), `PA7` (X1 E-Stop), `PC4` (X2), `PC5` (X3) | Kéo lên nội, `X0`/`X1` chạy ngắt ngoài EXTI |
| **Còi báo âm thanh** | `TIM13_CH1` | `PF8` | Phát xung PWM 50% duty |
| **Cảm biến tương tự** | `ADC1 12-bit` | `PC2` (LM35), `PC1` (VR1), `PC0` (Quang trở LDR), `PB0` (Áp), `PB1` (Dòng) | Đo nhiệt độ, tải trọng, rèm quang an toàn |
| **4 LED 7 đoạn & Bàn phím** | `SPI1` | `PB3` (SCK), `PB4` (MISO), `PB5` (MOSI), `PG6` (Latch), `PD3` (Load) | 2x 74HC595 (LED) + 2x 74HC165 (Phím) |
| **Màn hình màu TFT-LCD** | `FSMC 16-bit` | `PD0/1/4/5/7/8..10/14/15`, `PE3` (RS), `PE7..15`, `PC13` (Reset), `PA8` (Đèn nền) | Driver ILI9341 bộ nhớ Bank 1 NE1 |
| **Hộp đen & Đồng hồ RTC** | `I2C1` | `PB6` (SCL), `PB7` (SDA) | `DS3231` (`0x68`) + `AT24C512` (`0x50`) |
| **Mạng CAN 500 kbps** | `bxCAN1` | `PB8` (RX), `PB9` (TX) | Bộ lọc phần cứng: `0x080`, `0x100..0x10F`, `0x200..0x3FF` |

---

## 10. Cách Build, Nạp code & Chạy giao diện PC Digital Twin

### A. Biên dịch & Nạp code xuống bo mạch (Windows PowerShell)

```powershell
# Kích hoạt môi trường ảo Zephyr
& C:\Users\ASUS\zephyrproject\.venv\Scripts\Activate.ps1

# Chuyển vào thư mục dự án
cd C:\Users\ASUS\Documents\Report\BTL_HeThongNhung\smart_elevator_zephyr

# Biên dịch toàn bộ firmware bằng Ninja
ninja -C build

# Hoặc dùng west build
west build -b stm32f4_disco .

# Nạp firmware xuống bo mạch qua ST-LINK (nhớ gạt SW11 = ON)
west flash
```

### B. Mở giao diện mô phỏng Digital Twin trên máy tính (`pc_hil_twin/`)

```powershell
# Chạy giao diện đồ họa Python Digital Twin
python pc_hil_twin\elevator_twin.py
```
* **Chưa cắm mạch:** Nhấn **▶ Khởi động Offline Sim** để thử nghiệm ngay trên máy tính.
* **Đã cắm cáp RS232 vào mạch:** Chọn cổng `COMx` tương ứng ở tốc độ `115200` $\rightarrow$ Nhấn **Kết nối RS232** $\rightarrow$ Thang máy ảo trên máy tính sẽ đồng bộ trực tiếp theo bo mạch thật thời gian thực.

---

## 11. Các lệnh gõ thử qua cổng UART Console (`bkit` & `elev`)

Mở terminal kết nối cổng COM ở tốc độ **`115200 8N1`** sẽ thấy dấu nhắc lệnh:

### Lệnh kiểm tra từng bộ phận phần cứng (`bkit test ...`)
* `bkit test lcd`: **Chạy bài kiểm tra Màn hình màu TFT-LCD 240x320** (vẽ 4 thanh màu RGB565 và kiểm tra phần cứng FSMC).
* `bkit test gpio`: Thử bật/tắt motor Y0/Y1, nháy đèn PE4 và đọc 4 ngõ vào quang X0..X3.
* `bkit test pwm [freq]`: Phát thử còi PF8 trong $250\text{ ms}$ (mặc định $2400\text{ Hz}$).
* `bkit test adc`: In giá trị đo tức thời của 5 kênh ADC1 (nhiệt độ LM35, tải trọng VR1, quang trở, áp, dòng).
* `bkit test i2c`: Quét bus I2C1 tìm chip EEPROM (`0x50`) và RTC (`0x68`), đọc ngày giờ hiện tại.
* `bkit test rtc_set <Năm> <Tháng> <Ngày> <Giờ> <Phút> <Giây>`: Cài đặt lại ngày giờ thực cho chip DS3231.
* `bkit test spi1 [hex]`: Thử xuất LED 7 đoạn và đọc 16 phím bàn phím ma trận.
* `bkit test spi2`: Đọc mã JEDEC ID chip Flash ngoài W25Q64FV (`EF 40 17`).
* `bkit test can [rtr]`: Bắn gói CAN PING (`0x380`) hoặc hỏi từ xa RTR (`0x200`) đo độ trễ RTT.
* `bkit test wdt_hang`: Cố tình dừng nạp Watchdog để kiểm chứng MCU tự reset sau đúng $2.5\text{ giây}$.
* `bkit stats`: In nguyên nhân reset (HWINFO), UID chip và mức chiếm dụng Stack của toàn bộ 7 luồng.

### Lệnh điều khiển thang máy & tiêm lỗi thử nghiệm (`elev ...`)
* `elev status`: In toàn bộ trạng thái FSM, tầng hiện tại, tải trọng, nhiệt độ, các cờ lỗi.
* `elev call <1..7> [cabin|up|down]`: Bấm gọi tầng (Ví dụ: `elev call 5 cabin`, `elev call 3 up`, `elev call 4 down`).
* `elev door <open|close|obstruct>`: Mở/đóng cửa hoặc giả lập kẹt cửa rèm quang X0.
* `elev fault <fire|overload|can_drop|estop|clear>`: Bơm thử sự cố (cháy, quá tải, đứt mạng CAN, E-Stop) hoặc xóa chốt lỗi.
* `elev log [read|clear]`: Đọc 32 bản ghi Hộp đen trong EEPROM AT24C512 hoặc xóa trắng.
* `elev coverage`: In ma trận đếm số lần chuyển trạng thái $[9][9]$ để kiểm tra độ phủ kiểm thử V-Model.

---

### Mức chiếm dụng bộ nhớ sau khi biên dịch (`stm32f4_disco / stm32f407xx`)

```text
Memory region         Used Size  Region Size  %age Used
           FLASH:      112272 B         1 MB     10.71%
             RAM:       28608 B       128 KB     21.83%
```
*(Firmware tối ưu hóa cực kỳ gọn gàng, Flash chỉ chiếm 10.7% và RAM chiếm 21.8% dung lượng chip STM32F407ZGT6).*
