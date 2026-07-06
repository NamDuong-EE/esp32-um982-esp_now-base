# ESP32 GNSS Base ESP-NOW

Firmware Base dùng ESP32 kết nối với UM980/UM982 qua UART, đọc dữ liệu RTCM3 do module GNSS Base xuất ra, chia gói và gửi sang Rover bằng ESP-NOW Long Range. Repo này sẽ được chuyển từ kiến trúc cũ dùng LoRa/NTRIP/MQTT sang kiến trúc Base ESP-NOW chuyên dụng.

```text
UM980/982 Base ── UART RTCM ──> ESP32 Base ── ESP-NOW Long Range ──> ESP32 Rover ── UART ──> UM980/982 Rover
```

## Mục tiêu chuyển đổi

- [ ] Bỏ luồng truyền RTCM qua LoRa.
- [ ] Bỏ phụ thuộc Heltec LoRa nếu phần cứng thực tế dùng ESP32U/ESP32 dev board không có LoRa.
- [ ] Không dùng Wi-Fi router/AP trong chế độ thực địa.
- [ ] Không dùng MQTT/NTRIP trong luồng chính ngoài thực địa.
- [ ] Đọc RTCM3 nhị phân trực tiếp từ UM980/UM982 Base qua UART.
- [ ] Kiểm tra frame RTCM3 bằng preamble, length và CRC24Q trước khi gửi.
- [ ] Chia một RTCM frame thành nhiều packet ESP-NOW v1 tối đa 250 byte.
- [ ] Gửi unicast ESP-NOW Long Range tới MAC cố định của Rover.
- [ ] Cùng dùng channel cố định với Rover.
- [ ] Có health log qua Serial USB để debug tại hiện trường.

## Kiến trúc đích

### Vai trò của Base

Base không nhận correction từ NTRIP caster nữa. Base là trạm phát correction cục bộ:

1. UM980/UM982 được cấu hình ở chế độ Base.
2. UM980/UM982 xuất RTCM ra UART.
3. ESP32 đọc byte stream RTCM từ UART.
4. ESP32 tách từng RTCM3 frame hoàn chỉnh.
5. ESP32 kiểm tra CRC24Q.
6. ESP32 chia frame thành fragment theo protocol chung với Rover.
7. ESP32 gửi fragment bằng ESP-NOW LR tới Rover.

### ESP-NOW field mode

- ESP32 Base chạy `WIFI_STA`.
- Không gọi `WiFi.begin()` trong chế độ thực địa.
- Wi-Fi radio vẫn phải bật vì ESP-NOW chạy trên Wi-Fi driver của ESP32.
- Base và Rover phải cùng `ESPNOW_WIFI_CHANNEL`.
- Mặc định dùng ESP-NOW LR 250 Kbps để ưu tiên tầm xa.
- Base gửi unicast tới `ESPNOW_ROVER_MAC`.
- Giai đoạn đầu dùng MAC cấu hình tĩnh, chưa triển khai broadcast discovery/pairing động.
- Có thể bật mã hóa PMK/LMK sau khi Base/Rover đã chạy ổn định.

### Vì sao không dùng LoRa nữa

- RTCM là luồng nhị phân thời gian thực, thường có burst nhiều frame liên tiếp.
- LoRa băng thông thấp, thời gian truyền dài, dễ làm correction bị trễ.
- ESP-NOW LR trên ESP32 cho throughput cao hơn và phù hợp hơn cho link Base ↔ Rover khoảng cách gần/trung bình ngoài thực địa.
- Dùng ESP32U giúp tận dụng anten ngoài cho Wi-Fi/ESP-NOW.

## Cấu hình cần có

Các cấu hình dự kiến đặt trong `include/Prog_Config.h`:

```cpp
inline constexpr int RX_GNSS = 16; // UM980/982 TX -> ESP32 RX
inline constexpr int TX_GNSS = 17; // UM980/982 RX -> ESP32 TX
inline constexpr uint32_t GNSS_BAUD = 115200;

inline constexpr uint8_t ESPNOW_WIFI_CHANNEL = 6;
inline constexpr bool ESPNOW_USE_LR_250KBPS = true;

// MAC STA của Rover, lấy từ log Serial của firmware Rover.
inline constexpr uint8_t ESPNOW_ROVER_MAC[6] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

inline constexpr bool ESPNOW_ENCRYPTION_ENABLED = false;
inline constexpr uint8_t ESPNOW_PMK[16] = {0};
inline constexpr uint8_t ESPNOW_LMK[16] = {0};
```

Cần xác nhận lại chân UART thực tế của PCB Base. Repo cũ đang dùng cấu hình Heltec V4 với `RX_GNSS = 41`, `TX_GNSS = 42`; nếu chuyển sang ESP32U/ESP32 dev board thì nhiều khả năng sẽ dùng GPIO16/GPIO17 giống Rover.

## Giao thức RTCM qua ESP-NOW

Protocol phải giống với firmware Rover.

### Giới hạn

- ESP-NOW v1 tối đa 250 byte mỗi packet.
- Header protocol dài 16 byte.
- Payload mỗi fragment tối đa 234 byte.
- Một RTCM3 frame tối đa 1029 byte.
- Một RTCM frame tối đa cần 5 fragment.
- Số nguyên nhiều byte trên wire dùng little-endian.

### Header

```cpp
#pragma pack(push, 1)
struct RtcmEspNowHeader {
    uint16_t magic;            // 0x5452; wire little-endian là 0x52, 0x54 ("RT")
    uint8_t  version;          // 1
    uint8_t  packetType;       // 1 = RTCM_DATA
    uint16_t streamId;         // Tạo mới khi Base khởi động
    uint32_t frameSequence;    // Tăng 1 sau mỗi RTCM frame
    uint16_t frameLength;      // Tổng độ dài RTCM frame: 6..1029 byte
    uint8_t  fragmentIndex;    // 0..fragmentCount-1
    uint8_t  fragmentCount;    // 1..5
    uint16_t payloadLength;    // 1..234 byte
};
#pragma pack(pop)
```

Độ dài packet gửi thực tế:

```text
sizeof(RtcmEspNowHeader) + payloadLength
```

Không gửi đủ 250 byte nếu fragment cuối không dùng hết payload.

### Quy tắc phía Base

1. Tìm preamble RTCM3 `0xD3`.
2. Đọc length 10 bit trong RTCM header.
3. Đọc đủ `3 + payloadLength + 3` byte.
4. Kiểm tra CRC24Q.
5. Tính `fragmentCount = ceil(frameLength / 234)`.
6. Gửi fragment theo thứ tự tăng dần.
7. Chỉ gửi fragment kế tiếp sau khi send callback của fragment trước trả về.
8. Nếu send callback lỗi, retry ngắn; nếu vẫn lỗi thì bỏ frame hiện tại.
9. Tăng `frameSequence` sau mỗi frame, kể cả frame bị bỏ.

## Những điểm cần sửa trong code hiện tại

### 1. Thay `RTCM_Receiver`

File hiện tại:

```text
src/functions/RTCM_Receiver.cpp
```

đang dùng `String` và `Serial1.readString()`. Cách này không phù hợp với RTCM nhị phân. Ngoài ra file này còn ghi debug bằng `Serial1.println()`, tức ghi ngược vào UART của UM980/982, cần bỏ hoàn toàn.

Thay bằng module mới:

```text
include/functions/Rtcm_Frame_Reader.h
src/functions/Rtcm_Frame_Reader.cpp
```

Trách nhiệm:

- đọc từng byte từ `Serial1`;
- tìm preamble `0xD3`;
- đọc length;
- gom đủ frame;
- kiểm tra CRC24Q;
- trả về buffer `uint8_t frame[1029]` và `frameLength`.

### 2. Thay LoRa sender bằng ESP-NOW sender

Loại bỏ luồng:

```text
src/hardware/Lora_handler.cpp
include/hardware/Lora_handler.h
taskLora()
```

Thêm module:

```text
include/hardware/BaseEspnow_sender.h
src/hardware/BaseEspnow_sender.cpp
```

Trách nhiệm:

- cấu hình `WiFi.mode(WIFI_STA)`;
- tắt sleep;
- đặt protocol `WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR`;
- đặt channel cố định;
- `esp_now_init()`;
- add peer là `ESPNOW_ROVER_MAC`;
- đặt LR PHY rate 250 Kbps;
- chia RTCM frame thành fragment và gửi bằng `esp_now_send()`;
- ghi counter: frame gửi thành công, fragment lỗi, send timeout.

### 3. Rút gọn `main.cpp`

Luồng chính mới:

```text
setup()
  Serial.begin()
  Serial1.begin(GNSS_BAUD, SERIAL_8N1, RX_GNSS, TX_GNSS)
  setupEspNowBase()
  tạo task đọc/gửi RTCM
  tạo task health log

taskRtcm()
  nếu đọc được RTCM frame hợp lệ
    baseEspNowSendRtcmFrame(frame, length)

loop()
  delay ngắn hoặc xử lý retry ESP-NOW nếu cần
```

Không cần `setupMQTT()`, `connectMQTT()`, `setupNTRIP()` hoặc task LoRa trong field mode.

### 4. Dọn `platformio.ini`

Repo hiện có nhiều environment cho Wi-Fi/4G/Heltec/LoRa. Kiến trúc mới nên còn một environment chính:

```ini
[platformio]
default_envs = esp32u_base_espnow

[env:esp32u_base_espnow]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
build_flags =
    -std=gnu++17
    -DCONNECT_USING_WIFI=1
    -DCONNECT_USING_4G=0
    -DFIRMWARE_ROLE_BASE=1
```

Sau khi chuyển xong có thể bỏ:

- thư viện Heltec LoRa;
- board variant Heltec nếu không dùng phần cứng Heltec nữa;
- mock LoRa test cũ;
- TinyGSM nếu không dùng 4G.

## Cấu hình UM980/UM982 Base

Repo Base cần quyết định cách cấu hình UM980/UM982:

### Phương án A: cấu hình thủ công

UM980/UM982 được cấu hình bằng công cụ ngoài để xuất RTCM ra UART. ESP32 chỉ đọc RTCM và gửi ESP-NOW.

Ưu điểm:

- firmware ESP32 đơn giản;
- ít rủi ro gửi sai lệnh cấu hình GNSS.

Nhược điểm:

- triển khai hàng loạt khó hơn;
- khi mất cấu hình GNSS phải cấu hình lại thủ công.

### Phương án B: ESP32 gửi lệnh cấu hình UM980/982 lúc boot

ESP32 gửi lệnh cấu hình Base mode, RTCM output, baud, saveconfig.

Ưu điểm:

- tự động hóa tốt;
- phù hợp sản phẩm hoàn chỉnh.

Nhược điểm:

- cần xác nhận đầy đủ command set UM980/UM982;
- cần cơ chế tránh ghi cấu hình sai khi chưa có tọa độ cố định/survey-in.

Khuyến nghị: giai đoạn đầu dùng phương án A để kiểm thử ESP-NOW link trước. Sau khi link ổn định mới thêm module cấu hình UM980/982.

## Checklist triển khai

1. [ ] Chốt phần cứng Base: ESP32U hay Heltec V4.
2. [ ] Chốt GPIO UART nối UM980/UM982.
3. [ ] Chốt `ESPNOW_WIFI_CHANNEL`.
4. [x] Lấy MAC STA của Rover và điền `ESPNOW_ROVER_MAC`.
5. [x] Thêm `RtcmEspNowProtocol` dùng chung với Rover.
6. [x] Thêm `Rtcm_Frame_Reader` đọc RTCM3 nhị phân từ UART.
7. [x] Thêm `BaseEspnow_sender`.
8. [x] Sửa `main.cpp` bỏ LoRa/NTRIP/MQTT khỏi field mode.
9. [x] Dọn `platformio.ini`.
10. [x] Build firmware `esp32u_base_espnow`.
11. [ ] Test Base đọc được RTCM từ UM980/982.
12. [ ] Test Base gửi ESP-NOW tới Rover cùng channel.
13. [ ] Test Rover nhận RTCM và UM980/982 Rover đạt RTK Float/Fixed.

## Log mong đợi sau khi hoàn thiện

```text
[BASE][GNSS] UART1 baud=115200 RX=16 TX=17
[WIFI] Khong ket noi router/AP; chi dung STA radio cho ESP-NOW
[WIFI] Local STA MAC: XX:XX:XX:XX:XX:XX
[WIFI] ESP-NOW fixed channel: 6
[BASE][ESP-NOW] Ready, channel=6, LR=250 Kbps, streamId=N
[BASE][SETUP] Khoi dong hoan tat
[BASE][HEALTH] rtcm_valid=..., frames_sent=..., fragments_sent=..., send_fail=...
```

## Lỗi thường gặp cần tránh

| Lỗi | Cách tránh |
|---|---|
| Dùng `String` cho RTCM | Dùng buffer `uint8_t`, frame length rõ ràng |
| Ghi debug vào `Serial1` | Chỉ log ra `Serial` USB |
| Base/Rover khác channel | Cấu hình cùng `ESPNOW_WIFI_CHANNEL` |
| Gửi broadcast RTCM | Dùng unicast tới MAC Rover |
| Gửi frame RTCM sai CRC | Kiểm tra CRC24Q trước khi chia fragment |
| Fragment cuối gửi dư byte | Gửi đúng `sizeof(header) + payloadLength` |
| Router Wi-Fi làm đổi channel | Field mode không kết nối router/AP |

## Kết luận

Repo này sẽ trở thành firmware Base ESP-NOW. Nhiệm vụ chính là thay LoRa sender bằng ESP-NOW sender và thay cách đọc RTCM kiểu `String` bằng parser RTCM3 nhị phân đúng chuẩn. Sau khi Base gửi đúng protocol, Rover hiện tại có thể nhận, reassembly và ghi RTCM vào UM980/982 để đạt RTK.

## Tiến độ công việc

### 2026-07-06

- Đã thêm `include/RtcmEspNowProtocol.h` với header 16 byte, payload fragment tối đa 234 byte và frame RTCM tối đa 1029 byte.
- Đã thêm `include/functions/Rtcm_Frame_Reader.h` và `src/functions/Rtcm_Frame_Reader.cpp` để đọc RTCM3 dạng nhị phân từ UART, kiểm tra preamble, length và CRC24Q.
- Đã thêm `include/hardware/BaseEspnow_sender.h` và `src/hardware/BaseEspnow_sender.cpp` để khởi tạo ESP-NOW STA field mode, add peer Rover, chia RTCM frame thành fragment và gửi unicast có retry/counter.
- Đã rút gọn `src/main.cpp`: bỏ luồng LoRa/NTRIP/MQTT khỏi field mode, chỉ còn UART GNSS, ESP-NOW Base, task đọc/gửi RTCM và health log qua Serial USB.
- Đã dọn `platformio.ini` còn env chính `esp32u_base_espnow`, build đúng các module firmware Base ESP-NOW mới và `lib_ignore` các mock library cũ để không shadow `WiFi.h` của Arduino ESP32.
- Đã cập nhật `include/Prog_Config.h` theo cấu hình tạm ESP32U/ESP32 dev board: UART `RX_GNSS=16`, `TX_GNSS=17`, channel ESP-NOW `6`, Rover MAC `58:2A:BD:71:E4:F0`.
- Đã build thành công lại sau khi điền MAC Rover bằng `C:\Python314\python.exe -m platformio run -e esp32u_base_espnow`. Kết quả PlatformIO: RAM dùng 44,600 bytes trên 327,680 bytes (13.6%), Flash dùng 736,077 bytes trên 1,310,720 bytes (56.2%).
