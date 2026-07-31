# ESP32 GNSS Base ESP-NOW

Repo này tập trung xây dựng Firmware cho Base dùng mạch ESP32 kết nối với UM980/UM982 qua UART, đọc dữ liệu RTCM3 do module GNSS Base xuất ra, chia gói và gửi sang Rover bằng ESP-NOW Long Range.

```text
UM980/982 Base ── UART RTCM ──> ESP32 Base ── ESP-NOW Long Range ──> ESP32 Rover ── UART ──> UM980/982 Rover
```


## muc tiêu

### Vai trò của Base

Trong phiên bản thử nghiệm hiện tại Base không nhận correction từ NTRIP caster. Base đã có transport Wi-Fi/4G để đưa ECEF của từng Rover lên MQTT; RTCM vẫn đi trực tiếp tới Rover bằng ESP-NOW LR và không phụ thuộc Internet/MQTT:

1. UM980/UM982 được cấu hình ở chế độ Base.
2. UM980/UM982 xuất RTCM ra UART.
3. ESP32 đọc byte stream RTCM từ UART.
4. ESP32 tách từng RTCM3 frame hoàn chỉnh.
5. ESP32 kiểm tra CRC24Q.
6. ESP32 chia frame thành fragment theo protocol chung với Rover.
7. ESP32 gửi fragment bằng ESP-NOW LR tới Rover.

### Telemetry ECEF Rover -> Base -> MQTT

- Wire protocol hiện là version `2`; phải nạp đồng bộ Base, Relay và toàn bộ Rover.
- Packet `ROVER_ECEF_STATUS` type `6` dài 40 byte: sequence, GNSS time trong ngày, `correctionStreamId`, `fixQuality` và X/Y/Z ECEF.
- X/Y/Z dùng `int64_t`, scale `10000`: một đơn vị wire bằng `0,0001 m`. Khi log, gửi lệnh UM980 và publish MQTT, firmware in đúng bốn chữ số sau dấu phẩy.
- Rover vẫn parse GGA nội bộ để lấy UTC, fix quality và đổi WGS84 geodetic sang ECEF bằng ellipsoidal height `altitude_MSL + geoid_separation`; LLH không còn truyền qua ESP-NOW và không publish lên MQTT.
- Rover con gửi type `6` tới Relay. Relay thêm MAC Rover con và forward bằng `RELAYED_ROVER_ECEF_STATUS` type `7`, dài 48 byte.
- Queue ECEF của Relay chỉ giữ snapshot mới nhất. Khi TX manager bận, snapshot cũ có thể bị ghi đè; không tạo backlog telemetry.
- Base chỉ nhận source radio đã pair, validate magic/version/type/length/time/ECEF và giữ tối đa 30 nguồn trong RAM. NVS chỉ lưu danh sách MAC đã pair.
- MQTT mất kết nối không tạo backlog. Sau khi reconnect, mỗi MAC chỉ publish snapshot ECEF mới nhất.

```text
[BASE][ROVER_ECEF] mac=58:2A:BD:71:E4:F0 seq=12 gnss_ms=43200100 ecef_m=(-1623456.1234,5734567.2345,2145678.3456) fix_quality=4 via=direct
[BASE][MQTT][ECEF] Published topic=aitogy/680947F84890/base/rovers/582ABD71E4F0/ecef seq=12 corrected=0 bytes=...
```

### ESP-NOW field mode

- ESP32 Base chạy `WIFI_STA`.
- Bản test hiện tại có thể gọi `WiFi.begin()` để kết nối router và MQTT; router bắt buộc ở cùng channel với ESP-NOW.
- Wi-Fi radio vẫn phải bật vì ESP-NOW chạy trên Wi-Fi driver của ESP32.
- Base và Rover phải cùng `ESPNOW_WIFI_CHANNEL`.
- Khi dùng Wi-Fi Internet, router, Base và Rover đều phải ở channel `6`; firmware chỉ thử kết nối SSID trên channel này để tránh quét mạng làm đổi channel ESP-NOW.
- Khi dùng 4G, modem có radio riêng nên ESP-NOW tiếp tục giữ channel `6` độc lập.
- Mặc định dùng ESP-NOW LR 250 Kbps để ưu tiên tầm xa.
- Base gửi unicast tới các MAC Rover đã pair và lưu trong NVS/Preferences.
- Không còn MAC Rover hard-code trong firmware. Nếu NVS chưa có Rover đã pair, Base vẫn khởi động ESP-NOW để chờ pairing nhưng chưa gửi RTCM runtime cho peer nào.
- Có thể bật mã hóa PMK/LMK sau khi Base/Rover đã chạy ổn định.

### Kiến trúc broadcast discovery -> unicast

Mục tiêu của tính năng pairing động là Base và Rover tự tìm MAC của nhau mà không bị lẫn với ESP32/ESP-NOW khác ở cùng khu vực. Discovery chỉ dùng trong giai đoạn ghép cặp, không dùng để gửi RTCM thường xuyên.

Chính sách đã chốt: **pair theo nút vật lý**.

#### Normal mode

- Base đọc danh sách MAC Rover đã lưu trong NVS/Preferences và chỉ gửi RTCM unicast tới các MAC đó. Bản hiện tại hỗ trợ tối đa 5 Rover đã pair.
- Rover đọc MAC Base đã lưu trong NVS/Preferences và chỉ chấp nhận packet từ MAC đó.
- Không gửi broadcast discovery khi đang chạy bình thường.
- Packet pairing dùng `network_id` và `auth_tag`. Packet runtime RTCM/ACK hiện vẫn giữ protocol v1: data header 16 byte, ACK 12 byte, chưa thêm `network_id` vào data/ACK để không phá pipeline đã test.
- RTCM gốc không bị sửa; mọi header ESP-NOW wrapper đều bị Rover bỏ trước khi ghi RTCM xuống UART cho UM980/982.

#### Pairing mode

Pairing mode chỉ mở trong một cửa sổ ngắn, ví dụ 60 giây, khi người dùng bấm/giữ nút vật lý trên cả Base và Rover cần ghép.

1. Người dùng bấm nút pairing trên Base để Base vào pairing mode.
2. Người dùng bấm nút pairing trên đúng Rover muốn ghép. Các Rover khác không ở pairing mode sẽ không trả lời discovery.
3. Base thêm broadcast peer `FF:FF:FF:FF:FF:FF` và gửi gói `PAIR_DISCOVERY` định kỳ, ví dụ 500 ms/lần.
4. Rover chỉ xử lý discovery nếu đang ở pairing mode và gói hợp lệ.
5. Rover trả lời unicast `PAIR_RESPONSE` về MAC của Base lấy từ callback ESP-NOW.
6. Base nhận response hợp lệ đầu tiên, thêm Rover làm peer unicast, gửi `PAIR_CONFIRM`.
7. Base lưu MAC Rover vào NVS/Preferences và thoát pairing mode.
8. Rover chỉ lưu MAC Base sau khi nhận `PAIR_CONFIRM` hợp lệ, rồi thoát pairing mode.
9. Từ thời điểm này Base và Rover dùng unicast cho RTCM/ACK; broadcast không còn dùng trong normal mode.

#### Packet pairing đề xuất

Các packet pairing không mang RTCM. Chúng là packet điều khiển riêng của protocol ESP-NOW:

```text
PAIR_DISCOVERY:
    magic
    version
    packetType = PAIR_DISCOVERY
    role       = BASE
    network_id
    base_device_id
    base_nonce
    pairing_window_ms
    auth_tag

PAIR_RESPONSE:
    magic
    version
    packetType = PAIR_RESPONSE
    role       = ROVER
    network_id
    rover_device_id
    rover_nonce
    base_nonce_echo
    auth_tag

PAIR_CONFIRM:
    magic
    version
    packetType = PAIR_CONFIRM
    role       = BASE
    network_id
    base_nonce
    rover_nonce
    auth_tag
```

`network_id` là ID dùng chung cho một hệ thống quan trắc. `auth_tag` nên được tạo từ `pairing_key` dùng chung giữa firmware Base/Rover, ví dụ HMAC hoặc một hàm xác thực nhẹ hơn nếu muốn giữ code nhỏ. Không nên chỉ dựa vào MAC vì thiết bị lạ vẫn có thể nghe broadcast.

#### Pair nhiều rover với 1 base

8/7/2026: Base đã hỗ trợ lưu tối đa 5 Rover. Quy trình vận hành vẫn pair lần lượt từng cặp Base-Rover bằng nút vật lý; không đưa nhiều Rover vào pairing mode cùng lúc đẻ tránh chọn nhầm.

Ví dụ bật 1 Base và 5 Rover:

- Nếu chỉ bấm pairing trên Base và 1 Rover mong muốn, chỉ Rover đó trả lời; Base pair với Rover đó.
- Nếu cả 5 Rover đều bị đưa vào pairing mode cùng lúc, cả 5 có thể trả lời hợp lệ. Chính sách Base là nhận response hợp lệ đầu tiên, gửi confirm cho Rover đó và bỏ qua các Rover còn lại. Cách vận hành khuyến nghị là chỉ bấm pairing trên một Rover tại một thời điểm.
- Rover đã pair sẽ không ghi đè MAC Base đang lưu nếu không bấm nút pairing/re-pair.
- Base đã pair sẽ không ghi đè MAC Rover đang lưu nếu không bấm nút pairing/re-pair.

#### Điều kiện để không lẫn thiết bị khác

- Cùng channel mới thấy nhau, nhưng cùng channel chưa đủ để pair.
- Chỉ thiết bị đang ở pairing mode mới trả lời discovery.
- Packet phải đúng `magic`, `version`, `packetType`, `role`.
- Packet phải đúng `network_id`.
- Packet phải có `auth_tag` hợp lệ từ `pairing_key`.
- Rover chỉ lưu Base sau `PAIR_CONFIRM`, không lưu ngay khi thấy discovery.
- Sau pairing, runtime chỉ nhận packet từ MAC đã lưu. `network_id` hiện dùng ở packet pairing; runtime data/ACK có thể nâng lên protocol v2 để thêm `network_id` sau khi pairing ổn định.

## Đánh giá năng lực tải của kiến trúc hiện tại

Kết luận sau tối ưu ngày 2026-07-06: kiến trúc hiện tại đủ khả năng gửi gói tin RTCM tốc độ khá cao  ~3 gói tín/s, đã tách UART reader khỏi ESP-NOW sender, có buffer chống burst, correction-age policy và ACK ứng dụng từ Rover. UART vẫn giữ 115200 và ESP-NOW vẫn giữ băng thông giao thức LR ở 250 Kbps.

### Ngân sách băng thông

| Thành phần | Giới hạn danh nghĩa | Nhận xét |
|---|---:|---|
| UART2 `115200 8N1` | khoảng 11.520 byte/s | Đủ cho bộ RTCM 1 Hz thông thường, có thể thiếu nếu bật nhiều MSM ở 5–10 Hz |
| ESP-NOW LR 250 Kbps | 31.250 byte/s trước overhead | Đủ cho tải 1 Hz, nhưng stop-and-wait và retry làm throughput thực thấp hơn |
| ESP-NOW v1 packet | 250 byte | Header 16 byte, payload RTCM tối đa 234 byte |
| RTCM frame | tối đa 1029 byte | Cần tối đa 5 fragment |

Với dữ liệu đã quan sát trước đó chỉ khoảng 35 byte/s và frame 27 byte, tải hiện tại rất thấp so với cả UART và ESP-NOW. Khi bật đồng thời `1074/1084/1094/1124` thì UM980/UM982 có thể phát nhiều frame liên tiếp trong cùng epoch; lúc này khả năng chịu burst quan trọng hơn tốc độ trung bình.

### Các tối ưu đã triển khai

1. RX buffer `Serial1` tăng từ mặc định 256 byte lên 4096 byte trước khi gọi `begin()`.
2. `RTCM Reader` priority 4 chỉ đọc UART, parse và đưa frame vào queue; `RTCM Sender` priority 3 độc lập gửi ESP-NOW.
3. Queue chứa 8 RTCM frame. Khi đầy, firmware bỏ frame cũ nhất để giữ correction mới; frame chờ quá 1000 ms bị tính `stale_drop` và không gửi.
4. Mỗi frame mang timestamp nội bộ `receivedAtMs`; timestamp không đưa lên wire, không ghi Flash và phép trừ `uint32_t` an toàn khi `millis()` wrap.
5. Send callback dùng FreeRTOS semaphore. Mỗi lần gửi nguyên frame có deadline 1000 ms, ACK timeout 300 ms và tối đa một lần retry nguyên frame.
6. Rover gửi ACK 12 byte theo `streamId + frameSequence` chỉ sau khi ghép đủ fragment, CRC24Q đúng và `Serial1.write()` chấp nhận đủ frame. Nếu ACK mất, Rover ACK lại sequence trùng mà không ghi RTCM lần hai.
7. Base thống kê cố định `1005`, `1006`, `1074`, `1084`, `1094`, `1124`, `1230` và `other`, gồm count và age; không dùng `String`, map hay cấp phát động.
8. Health log có queue depth/high-water, queue/stale drop, queue age, ACK, retry, deadline, send duration và free heap.

### Phần chủ động chưa thay đổi

- Giữ `GNSS_BAUD = 115200` và ESP-NOW LR 250 Kbps. Chỉ cân nhắc 230400/460800 hoặc LR 500 Kbps sau khi log thực tế cho thấy tải 1 Hz chạm trần.
- `uart_raw_bytes` đếm byte đã đọc nhưng driver chưa cung cấp counter overflow trực tiếp; cần đối chiếu CRC error, message ID age và queue metrics trong thử nghiệm phần cứng.
- ACK xác nhận frame đã được Rover chấp nhận vào UART TX buffer, không xác nhận chip UM980/UM982 đã sử dụng correction; trạng thái RTK vẫn phải kiểm tra trên GNSS Rover.


## Cấu hình cần có

Các cấu hình dự kiến đặt trong `include/Prog_Config.h`:

```cpp
inline constexpr char GNSS_UART_PORT_NAME[] = "UM980 UART2 TX2/RX2";
inline constexpr int RX_GNSS = 16; // UM980/982 TX2 -> ESP32 RX GPIO16
inline constexpr int TX_GNSS = 17; // UM980/982 RX2 <- ESP32 TX GPIO17
inline constexpr uint32_t GNSS_BAUD = 115200;
inline constexpr size_t GNSS_RX_BUFFER_SIZE = 4096;
inline constexpr size_t RTCM_FRAME_QUEUE_LENGTH = 8;
inline constexpr uint32_t RTCM_MAX_QUEUE_AGE_MS = 1000;

inline constexpr uint8_t ESPNOW_WIFI_CHANNEL = 6;
inline constexpr bool ESPNOW_USE_LR_250KBPS = true;

inline constexpr bool ESPNOW_ENCRYPTION_ENABLED = false;
inline constexpr uint8_t ESPNOW_PMK[16] = {0};
inline constexpr uint8_t ESPNOW_LMK[16] = {0};

inline constexpr bool DEBUG_GNSS_UART_RAW_DUMP = false;
inline constexpr bool DEBUG_RTCM_HEX_DUMP = false;
inline constexpr bool DEBUG_RTCM_FRAME_LOG = false;
inline constexpr uint8_t DEBUG_RTCM_HEX_BYTES_PER_LINE = 16;

inline constexpr uint32_t ESPNOW_FRAME_SEND_DEADLINE_MS = 1000;
inline constexpr uint32_t ESPNOW_FRAME_ACK_TIMEOUT_MS = 300;
inline constexpr uint8_t ESPNOW_FRAME_RETRY_COUNT = 1;

inline constexpr bool ESPNOW_PAIRING_ENABLED = true;
inline constexpr int PAIRING_BUTTON_PIN = 0;
inline constexpr bool PAIRING_BUTTON_ACTIVE_LOW = true;
inline constexpr uint32_t PAIRING_BUTTON_HOLD_MS = 1500;
inline constexpr uint32_t PAIRING_WINDOW_MS = 60000;
inline constexpr uint32_t PAIR_DISCOVERY_INTERVAL_MS = 500;
inline constexpr uint8_t ESPNOW_MAX_PAIRED_ROVERS = 5;
inline constexpr uint32_t ESPNOW_NETWORK_ID = 0xA1700001UL;
inline constexpr uint8_t ESPNOW_PAIRING_KEY[16] = {
    0x41, 0x49, 0x54, 0x4F, 0x47, 0x59, 0x5F, 0x50,
    0x41, 0x49, 0x52, 0x5F, 0x56, 0x30, 0x30, 0x31,
};
inline constexpr char ESPNOW_NVS_NAMESPACE[] = "espnow";
inline constexpr char ESPNOW_NVS_ROVER_COUNT_KEY[] = "rover_count";
inline constexpr char ESPNOW_NVS_ROVER_MAC_PREFIX[] = "rover";
```

Base hiện chọn dùng cổng UART2 của UM980/UM982:

```text
UM980/UM982 TX2  -> ESP32 GPIO16 / RX_GNSS
UM980/UM982 RX2  <- ESP32 GPIO17 / TX_GNSS
UM980/UM982 GND  -- ESP32 GND
```

Repo cũ từng dùng cấu hình Heltec V4 với `RX_GNSS = 41`, `TX_GNSS = 42`; nếu đổi phần cứng hoặc đổi sang UART khác của UM980/UM982 thì cần sửa lại `RX_GNSS`, `TX_GNSS` và cấu hình output RTCM trên module GNSS cho đúng cổng.

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

### Bố cục mỗi packet/fragment trên wire

Mỗi fragment là một packet ESP-NOW độc lập, gồm header 16 byte rồi đến đúng số byte payload của fragment đó:

```text
Byte 0..1    magic             0x5452 (trên wire little-endian: 52 54)
Byte 2       version           1
Byte 3       packetType        1 = RTCM_DATA
Byte 4..5    streamId          giống nhau trong một phiên chạy của Base
Byte 6..9    frameSequence     giống nhau cho mọi fragment thuộc cùng RTCM frame
Byte 10..11  frameLength       tổng số byte của RTCM frame gốc
Byte 12      fragmentIndex     chỉ số fragment, bắt đầu từ 0
Byte 13      fragmentCount     tổng số fragment của RTCM frame
Byte 14..15  payloadLength     số byte RTCM nằm trong fragment hiện tại
Byte 16..    payload           một đoạn liên tục của RTCM frame gốc
```

Các trường `streamId`, `frameSequence`, `frameLength` và `fragmentCount` không đổi giữa các fragment của cùng một frame. Chỉ `fragmentIndex`, `payloadLength` và phần `payload` thay đổi.

### Thuật toán chia RTCM frame hiện tại

Với RTCM frame dài `L` byte:

```text
fragmentCount = ceil(L / 234)

Với mỗi fragment i từ 0 đến fragmentCount - 1:
    offset        = i * 234
    payloadLength = min(234, L - offset)
    payload       = frame[offset .. offset + payloadLength - 1]
    packetLength  = 16 + payloadLength
```

Phạm vi dữ liệu mà từng fragment chứa:

| Fragment hiển thị | `fragmentIndex` | Byte lấy từ RTCM frame | Payload tối đa | Packet ESP-NOW tối đa |
|---|---:|---:|---:|---:|
| Fragment 1 | 0 | `0..233` | 234 byte | 250 byte |
| Fragment 2 | 1 | `234..467` | 234 byte | 250 byte |
| Fragment 3 | 2 | `468..701` | 234 byte | 250 byte |
| Fragment 4 | 3 | `702..935` | 234 byte | 250 byte |
| Fragment 5 | 4 | `936..L-1` | tối đa 93 byte vì `L <= 1029` | tối đa 109 byte |

Số fragment được chọn theo độ dài frame:

| Độ dài RTCM frame | Số fragment |
|---:|---:|
| `6..234` byte | 1 |
| `235..468` byte | 2 |
| `469..702` byte | 3 |
| `703..936` byte | 4 |
| `937..1029` byte | 5 |

Ví dụ frame RTCM hiện đang quan sát dài 27 byte chỉ tạo một fragment:

```text
fragmentIndex = 0
fragmentCount = 1
frameLength   = 27
payloadLength = 27
packetLength  = 16 + 27 = 43 byte
payload       = toàn bộ 27 byte RTCM, từ D3 đến hết CRC24Q
```

Ví dụ frame dài 500 byte được tách như sau:

```text
Fragment 1: index=0/3, RTCM byte 0..233,   payloadLength=234, packetLength=250
Fragment 2: index=1/3, RTCM byte 234..467, payloadLength=234, packetLength=250
Fragment 3: index=2/3, RTCM byte 468..499, payloadLength=32,  packetLength=48
```

Trong code, `fragmentIndex` bắt đầu từ 0. Log lỗi lại hiển thị `fragmentIndex + 1`, vì vậy dòng `fragment 1/3` trong log tương ứng với `fragmentIndex=0` trên wire.

### ACK, timeout và retry hiện tại

Base dùng hai lớp xác nhận:

1. **MAC callback theo fragment:** Base gửi stop-and-wait, chỉ gửi fragment kế tiếp sau callback fragment hiện tại. Callback failure được retry tối đa theo `ESPNOW_SEND_RETRY_COUNT`; callback timeout dừng attempt hiện tại để tránh chờ vô hạn.
2. **ACK ứng dụng theo frame:** Sau khi gửi các fragment, Base chờ ACK Rover tối đa 300 ms. Rover chỉ tạo ACK sau reassembly, CRC24Q và UART write thành công.

ACK có bố cục cố định 12 byte:

```cpp
struct RtcmEspNowAck {
    uint16_t magic;          // 0x5452
    uint8_t  version;        // 1
    uint8_t  packetType;     // 2 = FRAME_ACK
    uint16_t streamId;
    uint32_t frameSequence;
    uint8_t  status;         // 1 = WRITTEN
    uint8_t  reserved;
};
```

Mỗi attempt gửi nguyên frame có deadline 1000 ms. Nếu thiếu ACK, Base retry nguyên frame một lần với cùng `streamId + frameSequence`. Rover nhận sequence đã hoàn thành sẽ ACK lại từ fragment đầu nhưng không ghi lặp RTCM vào UART. Chỉ ACK hợp lệ mới tăng `frames_acked`; hết retry mà không có ACK mới tăng `frames_dropped` và chuyển sequence.

Để một Rover mất điện không chặn các Rover khác, Base theo dõi lỗi riêng từng peer. Sau 2 frame liên tiếp không có application ACK, chỉ peer đó vào cooldown 3000 ms; frame mới vẫn được gửi cho các peer còn khỏe. Hết cooldown, Base tự dùng frame kế tiếp làm probe: ACK thành công thì xóa lỗi và log `state=recovered`, thất bại thì vào cooldown lại. Thứ tự peer bắt đầu được xoay round-robin theo từng frame, vì vậy peer đang probe không luôn đứng trước cùng một Rover khỏe. Cooldown chỉ nằm trong RAM, không xóa pairing/NVS.

### Quy tắc phía Base

1. Tìm preamble RTCM3 `0xD3`.
2. Đọc length 10 bit trong RTCM header.
3. Đọc đủ `3 + payloadLength + 3` byte.
4. Kiểm tra CRC24Q.
5. Tính `fragmentCount = ceil(frameLength / 234)`.
6. Gửi fragment theo thứ tự tăng dần.
7. Chỉ gửi fragment kế tiếp sau khi send callback của fragment trước trả về.
8. Chờ ACK ứng dụng từ Rover sau khi gửi đủ frame; nếu thiếu ACK thì retry nguyên frame một lần.
9. Cấp một `frameSequence` cho mỗi frame trước khi fan-out; mọi peer nhận cùng sequence. Peer đang cooldown được bỏ qua và sẽ chấp nhận sequence mới nhất khi probe lại.

## Kết nối Internet và MQTT

Firmware hỗ trợ hai transport được chọn lúc build, không bật đồng thời:

| Environment | Internet | Trạng thái sử dụng |
|---|---|---|
| `esp32u_base_espnow` | Wi-Fi | Mặc định để phát triển và test MQTT |
| `esp32u_base_4g_mqtt` | SIM7600/4G | Dành cho mạch 4G sau khi hoàn thiện phần cứng |

Network/MQTT chạy trong task riêng priority thấp trên core 0. `RTCM Reader` và `RTCM Sender` vẫn chạy độc lập với priority cao hơn; mất Wi-Fi, 4G hoặc MQTT không dừng ESP-NOW, pairing hay pipeline RTCM. Reconnect có giới hạn chu kỳ, MQTT không được dùng để vận chuyển RTCM trong bản này.

### Cấu hình Wi-Fi/MQTT để test

Sửa file local `include/Network_Secrets.h` (file này đã được `.gitignore`, không commit mật khẩu):

```cpp
#define BASE_WIFI_SSID "TEN_WIFI"
#define BASE_WIFI_PASSWORD "MAT_KHAU_WIFI"

#define BASE_MQTT_HOST "192.168.1.10"
#define BASE_MQTT_PORT 1883
#define BASE_MQTT_USER ""
#define BASE_MQTT_PASSWORD ""
```

Mẫu cấu hình được lưu tại `include/Network_Secrets.example.h`. Router phải được đặt cố định channel `6`, trùng `ESPNOW_WIFI_CHANNEL` trên Base và Rover. Nếu router ở channel khác, firmware không đổi channel ESP-NOW để chạy theo router mà sẽ tiếp tục retry Wi-Fi trên channel 6.

Build/nạp bản Wi-Fi test bằng environment mặc định:

```powershell
pio run -e esp32u_base_espnow
pio run -e esp32u_base_espnow -t upload
```

MQTT test dùng các topic:

| Topic | Hướng | Nội dung |
|---|---|---|
| `aitogy/<BASE_MAC>/base/status` | Base publish retained | `online`; LWT ghi `offline` |
| `aitogy/<BASE_MAC>/base/rovers/<ROVER_MAC>/ecef` | Base publish khi nhận ECEF mới | JSON gồm ECEF raw, delta, ECEF corrected, GNSS time, fix quality và metadata nguồn |
| `aitogy/<BASE_MAC>/base/command` | Base subscribe | Nhận `switch_to_base_fixed_ecef` hoặc `switch_to_rover` |
| `aitogy/<BASE_MAC>/base/command-result` | Base publish | Kết quả Rover đã ghi chuỗi lệnh xuống UM980 qua COM2 hoặc lỗi/timeout |

Ví dụ topic `aitogy/680947F84890/base/rovers/582ABD71E4F0/ecef`:

```json
{"rover_mac":"58:2A:BD:71:E4:F0","sequence":12,"gnss_time_ms":43200100,"fix_quality":4,"ecef_raw_m":{"x":-1623456.1234,"y":5734567.2345,"z":2145678.3456},"correction_valid":true,"correction_ecef_m":{"dx":0.0123,"dy":-0.0045,"dz":0.0067},"ecef_corrected_m":{"x":-1623456.1111,"y":5734567.2300,"z":2145678.3523},"via_relay":false,"relay_mac":"","rtcm_source":"temp_base","temp_base_mac":"68:09:47:9E:8C:08","rtcm_source_epoch":7,"correction_stream_id":32100,"source_age_ms":20,"correction_age_ms":15}
```

Server có thể subscribe wildcard `aitogy/+/base/rovers/+/ecef` để nhận ECEF từ mọi Base, hoặc `aitogy/<BASE_MAC>/base/rovers/+/ecef` cho một Base. Payload không có latitude, longitude, MSL height hoặc ellipsoidal height. Heartbeat định kỳ đã bị loại bỏ; Base chỉ phát status/LWT và ECEF mới.

### Kiến trúc RTCM hub với Temporary Base

Kiến trúc chốt ngày 2026-07-22 và đã triển khai trong firmware Base/Rover:

```text
UM980 Temporary Base -> ESP32 Temporary Base -> ESP-NOW unicast -> Base gốc
Base gốc -> ESP-NOW multi-unicast hiện tại -> các Rover/Relay còn lại
```

- Temporary Base chỉ gửi RTCM về MAC Base gốc, không nhận route map và không tự gửi trực tiếp tới các Rover còn lại.
- Các Rover/Relay còn lại vẫn pair, nhận RTCM và trả ACK cho MAC Base gốc như hiện tại; không cần đổi `active_source_mac`.
- Base gốc là RTCM hub. Khi nguồn local active, Base forward RTCM từ UM980 local. Khi temp source active, Base ngừng phát RTCM local, chuyển UM980 local sang Rover và bơm RTCM temp vào COM2 để đo sai số hệ quy chiếu.
- Trong `TEMP_ACTIVE`, Base gốc chỉ forward RTCM nhận từ temp. Nếu temp mất, Base cấu hình UM980 local trở lại Fixed ECEF từ reference RTCM1006 đã lưu rồi mới trở lại nguồn local.
- Base không gửi frame forwarded ngược lại peer đang là Temporary Base. ACK hop `Temporary Base -> Base gốc` độc lập với ACK hop `Base gốc -> Rover/Relay`.
- Trong `TEMP_PREPARING`, Base ACK frame upstream sau khi ghép đủ và CRC đúng để Temporary Base tiếp tục gửi các message dùng đánh giá readiness. Trong `TEMP_ACTIVE`, Base chỉ ACK sau khi frame đã vào queue forwarding; queue đầy thì không ACK để nguồn retry, không giữ correction cũ để phát muộn.
- Khi mất RTCM temp quá timeout, Base bỏ queue temp, đổi lại nguồn local, tạo `streamId` mới và chỉ forward frame local mới nhất. Các Rover vẫn thấy cùng MAC Base gốc nên không cần handover phân tán.

Trạng thái nguồn trên Base:

```text
LOCAL_ACTIVE -> TEMP_PREPARING -> TEMP_RESETTING -> TEMP_ACTIVE -> LOCAL_FALLBACK
```

`TEMP_PREPARING` không làm gián đoạn Rover: Base vẫn forward RTCM local trong guard 3000 ms và đợi hai chu kỳ đủ `1006`, `1074`, `1084`, `1094`, `1124`. Deadline readiness là 15000 ms tính từ lúc bắt đầu cấu hình Temporary Base; `lastTempFrameAtMs == 0` không còn làm Base hủy ngay sau guard 3 giây. Sau đó `TEMP_RESETTING` dừng phát RTCM cũ và gửi command ID `4`. Rover thực thi `CONFIG RTK DISABLE`, chặn ghi mọi RTCM đang chờ nhưng vẫn parse GGA và gửi ECEF status. Relay fan-out lệnh có auth tới các Rover con. Mỗi nhánh phải ACK command và từng MAC Rover phải gửi một ECEF/GGA mới có `fix_quality` khác 4/5 sau ACK. Cohort gồm toàn bộ Rover trực tiếp đã pair và các Rover con từng có status; status cũ hoặc thiết bị tạm offline vẫn được giữ trong gate thay vì bị bỏ qua. Chỉ khi toàn bộ cohort đạt điều kiện, Base queue command ID `5` (`CONFIG RTK USER_DEFAULTS`) để resume RTK, tạo stream mới và vào `TEMP_ACTIVE`; `rtcmEnabled` của từng nhánh chỉ bật sau ACK resume. Nếu timeout 45 giây hoặc mất RTCM temp, Base cũng resume RTK trước khi rollback về local và yêu cầu temp trở lại Rover.

ACK uplink type `11` có hai thời điểm khác nhau theo state. Trong `TEMP_PREPARING` và `TEMP_RESETTING`, Base ACK ngay sau khi validate frame để Temporary Base tiếp tục phát đủ chu kỳ readiness. Trong `TEMP_ACTIVE`, Base lưu MAC nguồn, upstream stream ID và upstream sequence cùng frame trong queue, chờ sender hoàn tất lượt multi-unicast xuống toàn bộ Rover đủ điều kiện rồi mới ACK Temporary Base. ACK được gửi sau khi lượt forward kết thúc kể cả khi một peer thất bại; kết quả downstream vẫn được ghi bằng counter riêng và failure isolation/cooldown tiếp tục xử lý peer lỗi. Duplicate cùng upstream sequence trong lúc chờ không được enqueue hoặc forward lần hai; nếu ACK cuối bị mất, duplicate sau completion được ACK lại ngay.

Không dùng riêng `CONFIG RTK RESET` làm cổng handover vì application ACK chỉ xác nhận ESP32 đã ghi UART, không xác nhận UM980/982 đã rời RTK Float. Ngoài ra receiver có thể giữ correction cũ theo RTK timeout lâu hơn timeout handover. Cặp DISABLE/RESUME tạo một khoảng non-RTK quan sát được và ngăn correction cũ lọt qua lúc đổi stream.

Temporary Base giữ queue 3 frame theo chính sách ưu tiên correction mới, gửi fragment tối đa 234 byte tới đúng MAC Base đã pair và retry nguyên frame một lần nếu thiếu ACK. Base có queue RX fragment 16 packet, reassembly timeout 1500 ms và queue forwarding temp 3 frame. Khi `TEMP_ACTIVE` không có frame temp hợp lệ trong 3500 ms, Base chuyển `LOCAL_FALLBACK`, xóa queue temp, đổi source epoch/stream và chỉ phát các frame local mới.

Khi nhận ECEF, Base chụp kèm nguồn RTCM active **ngay tại thời điểm lưu snapshot**, không đợi tới lúc MQTT publish.

```json
{"rover_mac":"68:09:47:9E:8C:08","sequence":42,"gnss_time_ms":43200100,"fix_quality":4,"ecef_raw_m":{"x":-1623456.1234,"y":5734567.2345,"z":2145678.3456},"correction_valid":false,"correction_ecef_m":null,"ecef_corrected_m":null,"rtcm_source":"temp_base","temp_base_mac":"58:2A:BD:71:E4:F0","rtcm_source_epoch":7,"correction_stream_id":32100,"source_age_ms":20}
```

- `rtcm_source`: `local_base` hoặc `temp_base`.
- `temp_base_mac`: MAC Temporary Base cấp correction cho snapshot; bằng `null` khi nguồn là Base local.
- `rtcm_source_epoch`: tăng mỗi lần Base đổi nguồn RTCM, giúp server tách dữ liệu trước/sau handover.
- `correction_stream_id` phải khớp stream downstream hiện tại và GNSS time của Rover phải cách mẫu Base gốc không quá 1500 ms.
- ECEF của chính temp base chỉ publish raw, không cộng delta.

#### Hiệu chỉnh độ lệch hệ quy chiếu

1. Khi còn là Base local, firmware lấy ECEF chuẩn `B_ref` trực tiếp từ RTCM1006 của UM980 và giữ trong RAM.
2. Sau khi vào `TEMP_ACTIVE`, UM980 local được cấu hình thành Rover, nhận chính RTCM temp từ ESP32 Base và xuất GGA COM2.
3. Firmware đổi GGA mới sang ECEF `B_obs`, chỉ chấp nhận `fix_quality=4`, sai lệch tuyệt đối từng trục không quá 5 m và bước nhảy không quá 0,5 m.
4. Sau ba mẫu ổn định: `delta = B_ref - B_obs`.
5. Với Rover con có cùng `correction_stream_id` và GNSS time đủ gần: `R_corrected = R_raw + delta`.
6. Delta hết hiệu lực sau 3000 ms. Khi chưa hợp lệ, MQTT vẫn publish `ecef_raw_m` nhưng để `correction_ecef_m` và `ecef_corrected_m` là `null`.

### Điều khiển UM980 chuyển giữa Rover và temporary Base - V1

Để chọn một Rover làm Temporary Base, publish JSON sau vào `aitogy/<BASE_MAC>/base/command`:

```json
{"action":"switch_to_base_fixed_ecef","target_mac":"58:2A:BD:71:E4:F0","fix_timeout_s":120,"transaction_id":123}
```

- `fix_timeout_s`: thời gian tối đa Base chờ ECEF mới của đúng Rover đạt `fix_quality=4`; mặc định 120 giây, tối đa 3600 giây. Đây là timeout chờ RTK Fixed, không phải thời gian tồn tại của Temporary Base.
- `transaction_id`: số khác 0 để đối chiếu request/result. Có thể bỏ qua để Base tự sinh.
- `target_mac`: bắt buộc với action Fixed ECEF; phải là MAC của Rover trực tiếp đã pair, định dạng `XX:XX:XX:XX:XX:XX`.
- Base phải đọc được RTCM1006 local trước khi nhận lệnh để có ECEF reference cho fallback và correction; nếu chưa có sẽ trả log `local_rtcm1006_reference_unavailable`.
- Base từ chối MAC sai định dạng, MAC broadcast/multicast/zero hoặc MAC không có trong danh sách pairing với log `invalid target_mac`/`target_not_paired`. Lệnh chưa hỗ trợ target là Rover con phía sau Relay hoặc phát đồng thời tới nhiều Rover.
- Base chỉ dùng snapshot ECEF mới không quá 3000 ms và `fix_quality=4` của đúng target. Việc đổi LLH sang ECEF đã được Rover thực hiện trước khi gửi telemetry.
- Base đóng gói nguyên X/Y/Z `int64_t` scale `10000` vào packet ESP-NOW unicast type `8` dài 44 byte, kèm `network_id`, transaction ID và `auth_tag`. Rover trả packet result type `9` về đúng Base.
- Rover chỉ chấp nhận semantic command trong allowlist, không nhận chuỗi UART tùy ý từ MQTT.

Rover khóa UART COM2 và ghi tuần tự 14 bước: `unlogall`, `mode base <X_m> <Y_m> <Z_m>`, `gpgga com2 1`, RTCM `1006`, `1033`, `1074`, `1124`, `1084`, `1094`, `1042`, `1019`, `1020`, `1045` trên COM2 mỗi 1 giây, cuối cùng `saveconfig`. X/Y/Z được in với bốn chữ số thập phân. Firmware không gửi `FRESET` và không còn đường lệnh survey-in.

Để dùng tọa độ RTK Fixed hiện tại của Rover thay cho survey-in, publish:

```json
{"action":"switch_to_base_fixed_ecef","target_mac":"58:2A:BD:71:E4:F0","fix_timeout_s":120,"transaction_id":125}
```

- `target_mac` là bắt buộc và phải là Rover trực tiếp đã pair.
- Base chờ snapshot ECEF mới không quá 3000 ms của đúng Rover có `fix_quality=4`. `fix_timeout_s` mặc định 120 giây, hợp lệ `1...3600`.
- Trong lúc đang chờ, `switch_to_rover` mới nhắm cùng MAC sẽ hủy request Fixed ECEF cũ để tránh chuyển vai trò muộn ngoài ý muốn.
- ECEF truyền trên wire và dùng trong lệnh Fixed Base được lượng tử ở `0,0001 m`; command request dài 44 byte.
- Rover ghi `mode base <X> <Y> <Z>` với bốn chữ số thập phân, sau đó bật cùng bộ GGA/RTCM COM2 và `saveconfig`.
- Cú pháp fixed ECEF được UM980/UM982 hỗ trợ theo [Unicore N4 High Precision Commands Manual](https://en.unicore.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.6.pdf).
- Trong lúc chờ quality 4, Base vẫn phát RTCM local bình thường. Chỉ sau khi command được queue, Rover trả result và RTCM temp đủ readiness thì Base mới handover.

Kết quả fixed ECEF có thêm tọa độ đã dùng:

```json
{"transaction_id":125,"target_mac":"58:2A:BD:71:E4:F0","action":"switch_to_base_fixed_ecef","status":"uart_sequence_written","completed_step":14,"total_steps":14,"detail_code":0,"ecef_m":{"x":-1623456.1230,"y":5734567.4560,"z":2267890.7890},"result_age_ms":8}
```

Để đưa temporary Base đầu tiên trở lại Rover, publish một transaction ID mới:

```json
{"action":"switch_to_rover","target_mac":"58:2A:BD:71:E4:F0","transaction_id":124}
```

Rover ghi 4 bước: `unlogall`, `mode rover survey`, `gpgga com2 1`, `saveconfig`. Khi result thành công, Rover mở lại nhận RTCM/gửi ECEF và Base bật lại RTCM tới peer đó. Cú pháp `MODE ROVER` yêu cầu UM980 Build7923+ hoặc UM982 Build7650+ theo Commands Manual N4 của Unicore.

Kết quả được publish lên `aitogy/<BASE_MAC>/base/command-result`:

```json
{"transaction_id":123,"target_mac":"58:2A:BD:71:E4:F0","action":"switch_to_base_fixed_ecef","status":"uart_sequence_written","completed_step":14,"total_steps":14,"detail_code":0,"ecef_m":{"x":-1623456.1234,"y":5734567.2345,"z":2145678.3456},"result_age_ms":8}
```

`uart_sequence_written` chỉ xác nhận ESP32 Rover đã ghi đủ 14 lệnh Fixed ECEF vào UART. Base chỉ chuyển nguồn correction sau guard và hai chu kỳ RTCM hợp lệ; trước đó các Rover còn lại vẫn dùng RTCM local. Sau kết quả này Rover ngừng nhận RTCM và chuyển COM2 sang pipeline RTCM uplink về Base gốc. Trạng thái chuyển vai trò hiện ở RAM và mất khi reset ESP32.

Kết quả `switch_to_rover` dùng cùng topic/schema nhưng có `action="switch_to_rover"`, `completed_step=4` và `total_steps=4`. Firmware chỉ bật lại RTCM sau result thành công.

Base chờ result tối đa 10 giây và retry request một lần. Rover nhớ transaction hoàn tất gần nhất để request lặp chỉ trả lại result, không ghi lại chuỗi lệnh. MQTT hiện dùng TCP 1883 và ESP-NOW encryption mặc định còn tắt, vì vậy đây là bản test trong mạng tin cậy.

Bản hiện tại dùng MQTT TCP port `1883` để thử nghiệm trong mạng tin cậy, chưa bật TLS. Khi dùng server thực tế qua Internet cần bổ sung TLS/xác thực chứng chỉ trước khi triển khai.

### Cấu hình 4G dự kiến

Environment `esp32u_base_4g_mqtt` dùng TinyGSM với modem SIM7600. Theo mapping TDM240x đã chạy với firmware của Long, UART GNSS dùng ESP32 `RX=26`, `TX=27`; UART2 của SIM7600 dùng ESP32 `RX=16`, `TX=17`, baud `115200`. Firmware mặc định kích chân điều khiển nguồn `GPIO15` mức HIGH trong 1000 ms, nhả về LOW rồi chờ 8000 ms trước khi gửi lệnh AT; chuỗi này chỉ chạy một lần trong mỗi lần ESP32 khởi động. Sau khi modem boot, firmware gọi `modem.init()` thay vì reset modem lần nữa; TinyGSM được cấu hình `TINY_GSM_YIELD_MS=10` và task mạng chạy priority 0 để các vòng chờ AT không chặn idle task và kích watchdog. Có thể đổi/tắt nguồn bằng `MODEM_POWER_CONTROL_*` trong `include/Prog_Config.h`. Phải kiểm tra lại GPIO, mức kích và mạch nguồn/PWRKEY theo PCB 4G trước khi nạp. Điền `BASE_MODEM_APN`, user/password APN và MQTT trong `Network_Secrets.h`, sau đó build:

```powershell
pio run -e esp32u_base_4g_mqtt
```

### Log kiểm tra

Kết nối thành công sẽ có các log:

```text
[BASE][NETWORK] transport=wifi configured=yes
[BASE][WIFI] Connecting SSID=... fixed_channel=6
[BASE][WIFI] Connected IP=... channel=6 RSSI=... dBm
[BASE][MQTT] Connected, rover_ecef_filter=aitogy/<BASE_MAC>/base/rovers/+/ecef
[BASE][MQTT][ECEF] Published topic=aitogy/<BASE_MAC>/base/rovers/.../ecef seq=... corrected=... bytes=...
[BASE][NETWORK_HEALTH] transport=wifi configured=1 internet=1 mqtt=1 ...
```

Nếu secrets còn trống, RTCM/ESP-NOW vẫn chạy bình thường và log báo `configured=no`.

## Cách nạp firmware vào ESP32 Base

Firmware Base ESP-NOW nên được nạp bằng PlatformIO vì repo dùng `platformio.ini` và env chính `esp32u_base_espnow`.

### 1. Chuẩn bị

- Dùng cáp USB có truyền dữ liệu, không dùng cáp chỉ sạc.
- Cài driver USB-UART đúng với board ESP32 nếu máy chưa nhận cổng COM:
  - CP210x cho nhiều board ESP32 DevKit/ESP32U;
  - CH340/CH9102 nếu board dùng chip USB-UART loại này.
- Cắm ESP32 Base vào máy tính.
- Mở Device Manager trên Windows và xem cổng COM mới xuất hiện, ví dụ `COM5`.
- Kiểm tra cấu hình trong `include/Prog_Config.h` trước khi nạp:
  - `RX_GNSS = 16`;
  - `TX_GNSS = 17`;
  - dây nối `UM980 TX2 -> ESP32 GPIO16`, `UM980 RX2 -> ESP32 GPIO17`, `GND -> GND`;
  - `ESPNOW_WIFI_CHANNEL = 6`;
  - không cần điền MAC Rover trong code; MAC Rover sẽ được lưu vào NVS sau khi pairing.

### 2. Build kiểm tra trước khi nạp

Mở PowerShell tại thư mục repo:

```powershell
cd C:\Users\admin\Documents\GitHub\esp32-um982-lora-base
```

Nếu `pio` đã có trong PATH:

```powershell
pio run -e esp32u_base_espnow
```

Nếu máy đang dùng PlatformIO qua Python 3.14 như lần build hiện tại:

```powershell
C:\Python314\python.exe -m platformio run -e esp32u_base_espnow
```

Build thành công sẽ có dòng:

```text
[SUCCESS] Took ... seconds
```

### 3. Nạp firmware

Thay `COM5` bằng cổng COM thực tế của ESP32 Base.

Nếu `pio` đã có trong PATH:

```powershell
pio run -e esp32u_base_espnow -t upload --upload-port COM5
```

Nếu dùng Python 3.14:

```powershell
C:\Python314\python.exe -m platformio run -e esp32u_base_espnow -t upload --upload-port COM5
```

Khi upload chạy, nếu log đứng ở đoạn `Connecting...`, giữ nút `BOOT` trên ESP32, bấm nhả `EN/RESET`, sau đó thả `BOOT` khi bắt đầu ghi flash. Một số board tự reset được thì không cần thao tác này.

Upload thành công sẽ có dòng gần giống:

```text
Hash of data verified.
Hard resetting via RTS pin...
```

### 4. Mở Serial Monitor sau khi nạp

Baud monitor là `115200`.

Nếu `pio` đã có trong PATH:

```powershell
pio device monitor -p COM5 -b 115200
```

Nếu dùng Python 3.14:

```powershell
C:\Python314\python.exe -m platformio device monitor -p COM5 -b 115200
```

Sau khi monitor mở, bấm `EN/RESET` trên ESP32 Base. Log mong đợi:

```text
[BASE][GNSS] ESP32 Serial1 reading UM980 UART2 TX2/RX2, baud=115200 RX=16 TX=17
[BASE][DEBUG] uart_raw_dump=on rtcm_hex_dump=on
[WIFI] STA radio ready for ESP-NOW; Internet transport starts separately
[WIFI] Local STA MAC: XX:XX:XX:XX:XX:XX
[WIFI] ESP-NOW fixed channel: 6
[BASE][ESP-NOW] Ready, channel=6, LR=250 Kbps, streamId=N
[BASE][NETWORK] transport=wifi configured=...
[BASE][SETUP] Khoi dong hoan tat
```

Khi UM980/UM982 Base bắt đầu xuất RTCM hợp lệ qua UART, log sẽ có:

```text
[BASE][GNSS] RTCM frame valid, length=...
[BASE][GNSS][RTCM_HEX] valid length=...
[BASE][GNSS][RTCM_HEX] 0000: D3 ...
[BASE][HEALTH] period_ms=30000 uart_Bps=... rtcm_fps=... acked_fps=... delivery=...% queue=... queue_hwm=... queue_drop=... stale_drop=... ack_rx=...
[BASE][RTCM_TYPES] 1005=...(age=...) 1006=...(age=...) 1074=...(age=...) 1084=...(age=...) 1094=...(age=...) 1124=...(age=...) 1230=...(age=...) other=...(age=...)
```

Đổi `DEBUG_GNSS_UART_RAW_DUMP = true` khi cần in mọi byte thô ESP32 đọc được từ `Serial1`. Chế độ này mặc định tắt vì lượng log HEX lớn có thể làm chậm luồng đọc/gửi RTCM; dùng `uart_raw_bytes` và `uart_Bps` trong health log để kiểm tra UART khi raw dump đang tắt.

Nếu health log vẫn không có trường `uart_available` và `uart_raw_bytes`, ESP32 đang chạy firmware cũ và cần nạp lại đúng binary mới.

`DEBUG_RTCM_HEX_DUMP = true` sẽ in toàn bộ frame RTCM nhận từ UM980/UM982 ra Serial USB ở dạng HEX. Khi chạy ổn định có thể đổi về `false` trong `include/Prog_Config.h` để giảm log và tránh nghẽn Serial.

### 5. Lỗi nạp thường gặp

| Hiện tượng | Cách xử lý |
|---|---|
| Không thấy cổng COM | Đổi cáp USB, cài driver CP210x/CH340/CH9102, rút cắm lại board |
| Upload đứng ở `Connecting...` | Giữ `BOOT`, bấm nhả `EN/RESET`, thả `BOOT` khi bắt đầu ghi |
| `Access is denied` khi upload | Đóng Serial Monitor/Arduino IDE/terminal khác đang giữ COM |
| Log Serial rác | Chọn đúng baud `115200` |
| Không thấy log sau khi nạp | Mở monitor rồi bấm `EN/RESET` |
| Base không gửi được tới Rover | Kiểm tra Rover đang chạy cùng `ESPNOW_WIFI_CHANNEL` và MAC STA đúng |
| `[BASE][ESP-NOW][ERROR] esp_wifi_set_channel failed: 12289` | Wi-Fi driver chưa init hoặc đã bị tắt trước khi set channel. Trong code phải dùng `WiFi.mode(WIFI_STA)` rồi `WiFi.disconnect(false, true)`, không dùng `WiFi.disconnect(true, true)` vì tham số `true` đầu sẽ tắt Wi-Fi radio |

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
- cần cơ chế tránh ghi cấu hình sai khi chưa có tọa độ RTK Fixed hợp lệ.

Khuyến nghị: giai đoạn đầu dùng phương án A để kiểm thử ESP-NOW link trước. Sau khi link ổn định mới thêm module cấu hình UM980/982.

## Checklist triển khai

1. [x] phần cứng Base: ESP32U 
2. [x] Chốt GPIO UART nối UM980/UM982: dùng UM980 UART2 `TX2/RX2`, nối `TX2 -> GPIO16`, `RX2 -> GPIO17`, chung GND.
3. [x] Chốt `ESPNOW_WIFI_CHANNEL`.
4. [x] Bỏ cấu hình MAC Rover tĩnh; Base lấy MAC Rover từ pairing/NVS.
5. [x] Thêm `RtcmEspNowProtocol` dùng chung với Rover.
6. [x] Thêm `Rtcm_Frame_Reader` đọc RTCM3 nhị phân từ UART.
7. [x] Thêm `BaseEspnow_sender`.
8. [x] Sửa `main.cpp` bỏ LoRa/NTRIP/MQTT khỏi field mode.
9. [x] Dọn `platformio.ini`.
10. [x] Build firmware `esp32u_base_espnow`.
11. [x] Test Base đọc được RTCM từ UM980/982.
12. [x] Test Base gửi ESP-NOW tới Rover cùng channel.
13. [x] Test Rover nhận RTCM và UM980/982 Rover đạt RTK Float/Fixed.
14. [x] Chốt kiến trúc pairing động: dùng nút vật lý trên Base/Rover, broadcast discovery chỉ trong pairing window, sau confirm chuyển sang unicast.
15. [x] Thiết kế/triển khai `network_id`, `pairing_key/auth_tag` và packet `PAIR_DISCOVERY`/`PAIR_RESPONSE`/`PAIR_CONFIRM`.
16. [x] Lưu MAC đã pair vào NVS/Preferences và thêm cơ chế re-pair bằng nút vật lý.
17. [x] Base hỗ trợ danh sách tối đa 5 Rover đã pair và gửi RTCM multi-unicast lần lượt tới từng Rover.
18. [x] Nâng runtime protocol lên v2 cho telemetry ECEF; pairing/command vẫn kiểm tra `network_id` và auth tag.
19. [x] Thêm transport Wi-Fi/4G chọn theo environment và task reconnect riêng không chặn RTCM/ESP-NOW.
20. [x] Thêm MQTT test: status/LWT, downlink log và health counter.
21. [ ] Điền credentials và kiểm tra Wi-Fi/MQTT với broker thật trong khi Rover vẫn nhận RTCM LR.
22. [ ] Hoàn thiện mạch 4G, xác nhận GPIO/nguồn/PWRKEY/APN và kiểm tra environment `esp32u_base_4g_mqtt` trên SIM7600 thật.
23. [x] Nhận ECEF từ từng Rover, giữ snapshot mới nhất và publish MQTT theo MAC; không tạo backlog khi offline.
24. [x] Nhận MQTT command có `target_mac`, đợi ECEF RTK Fixed và cấu hình UM980 thành Temporary Base Fixed ECEF qua COM2.
25. [x] Temporary Base uplink RTCM một hop về Base gốc; Base gốc là hub và không phát nguồn local khi temp active.
26. [x] Gắn `temp_base_mac`, source epoch, stream ID và GNSS time vào snapshot ECEF/MQTT.
27. [x] Chuyển UM980 Base gốc sang Rover khi temp active, tính delta ECEF và publish ECEF corrected.
28. [x] Reset nghiệm RTK toàn cohort trước handover; Relay fan-out command tới Rover con và Base đợi fresh non-RTK status.

## Log mong đợi sau khi hoàn thiện

```text
[BASE][GNSS] ESP32 Serial1 reading UM980 UART2 TX2/RX2, baud=115200 RX=16 TX=17
[WIFI] STA radio ready for ESP-NOW; Internet transport starts separately
[WIFI] Local STA MAC: XX:XX:XX:XX:XX:XX
[WIFI] ESP-NOW fixed channel: 6
[BASE][ESP-NOW] Ready, channel=6, LR=250 Kbps, streamId=N
[BASE][NETWORK] transport=wifi configured=yes
[BASE][MQTT] Connected, rover_ecef_filter=aitogy/<BASE_MAC>/base/rovers/+/ecef
[BASE][MQTT][ECEF] Published topic=aitogy/<BASE_MAC>/base/rovers/.../ecef seq=... corrected=... bytes=...
[BASE][RTCM_SOURCE] state=TEMP_PREPARING temp=... mode=fixed_ecef
[BASE][HANDOVER_RESET] state=START rovers=... branches=... timeout_ms=45000
[BASE][HANDOVER_RESET] branch=... command_ack=1
[BASE][HANDOVER_RESET] state=COMPLETE
[BASE][RTCM_SOURCE] state=TEMP_ACTIVE temp=... epoch=2 streamId=...
[BASE][LOCAL_GNSS] role=ROVER
[BASE][CORRECTION] gnss_ms=... fix=4 valid=1 samples=3 observed=(...) delta=(...)
[BASE][RTCM_SOURCE] state=LOCAL_FALLBACK epoch=3 streamId=... reason=temp_rtcm_timeout
[BASE][SETUP] Khoi dong hoan tat
[BASE][HEALTH] rtcm_valid=..., frames_acked=..., fragments_sent=..., ack_timeout=..., queue_drop=..., stale_drop=...
[BASE][NETWORK_HEALTH] transport=wifi configured=1 internet=1 mqtt=1 signal_dbm=...
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
| Router Wi-Fi làm đổi channel | Cố định router, Base và Rover cùng channel 6; firmware Wi-Fi chỉ thử kết nối trên channel này |

## Kết luận

Repo này sẽ trở thành firmware Base ESP-NOW. Nhiệm vụ chính là thay LoRa sender bằng ESP-NOW sender và thay cách đọc RTCM kiểu `String` bằng parser RTCM3 nhị phân đúng chuẩn. Sau khi Base gửi đúng protocol, Rover hiện tại có thể nhận, reassembly và ghi RTCM vào UM980/982 để đạt RTK.

## Tiến độ công việc

### 2026-07-06

- Đã thêm `include/RtcmEspNowProtocol.h` với header 16 byte, payload fragment tối đa 234 byte và frame RTCM tối đa 1029 byte.
- Đã thêm `include/functions/Rtcm_Frame_Reader.h` và `src/functions/Rtcm_Frame_Reader.cpp` để đọc RTCM3 dạng nhị phân từ UART, kiểm tra preamble, length và CRC24Q.
- Đã thêm `include/hardware/BaseEspnow_sender.h` và `src/hardware/BaseEspnow_sender.cpp` để khởi tạo ESP-NOW STA field mode, add peer Rover, chia RTCM frame thành fragment và gửi unicast có retry/counter.
- Đã rút gọn `src/main.cpp`: bỏ luồng LoRa/NTRIP/MQTT khỏi field mode, chỉ còn UART GNSS, ESP-NOW Base, task đọc/gửi RTCM và health log qua Serial USB.
- Đã dọn `platformio.ini` còn env chính `esp32u_base_espnow`, build đúng các module firmware Base ESP-NOW mới và `lib_ignore` các mock library cũ để không shadow `WiFi.h` của Arduino ESP32.
- Đã cập nhật `include/Prog_Config.h` theo cấu hình ESP32U/ESP32 dev board: dùng UM980 UART2 `TX2/RX2`, `RX_GNSS=16`, `TX_GNSS=17`, channel ESP-NOW `6`, Rover MAC `58:2A:BD:71:E4:F0`. Riêng environment 4G tích hợp cũ override GNSS sang GPIO26/27 vì SIM7600 chiếm GPIO16/17.
- Đã build thành công lại sau khi điền MAC Rover bằng `C:\Python314\python.exe -m platformio run -e esp32u_base_espnow`. Kết quả PlatformIO: RAM dùng 44,600 bytes trên 327,680 bytes (13.6%), Flash dùng 736,077 bytes trên 1,310,720 bytes (56.2%).
- Đã thêm mục "Cách nạp firmware vào ESP32 Base" với hướng dẫn build, upload, mở Serial Monitor và xử lý lỗi nạp thường gặp trên Windows.
- Đã sửa lỗi ESP-NOW init `esp_wifi_set_channel failed: 12289` bằng cách đổi `WiFi.disconnect(true, true)` thành `WiFi.disconnect(false, true)` để không tắt Wi-Fi radio trước khi set channel. Build lại `esp32u_base_espnow` thành công.
- Đã thêm cấu hình `DEBUG_RTCM_HEX_DUMP` và hàm dump HEX để in toàn bộ frame RTCM nhận từ UM980/UM982 ra Serial USB khi debug.
- Đã thêm cấu hình `DEBUG_GNSS_UART_RAW_DUMP` để in mọi byte thô đọc được từ UART GNSS trước parser RTCM, giúp phân biệt lỗi không có tín hiệu UART với lỗi chưa ghép được frame RTCM hợp lệ.
- Đã bổ sung `uart_available` và `uart_raw_bytes` vào health log để xác nhận ESP32 có nhận byte UART từ UM980/UM982 hay chưa, kể cả khi chưa có frame RTCM hợp lệ.

## Gateway 4G rời giao tiếp UART

Branch `codex/4g-uart-gateway` có hai firmware riêng:

- `esp32u_base_uart_4g_client`: nạp vào board Base Wi-Fi/GNSS. Board này vẫn
  đọc UM980 trên GPIO16/17, chạy ESP-NOW và chuyển các bản tin MQTT sang UART
  GPIO18/19.
- `esp32u_4g_uart_gateway`: nạp vào board có ESP32 + SIM7600, không cần UM980
  và không chạy ESP-NOW. Board này nhận frame UART trên GPIO26/27, kiểm tra
  CRC32, đưa vào queue RAM rồi publish qua 4G.

Đấu dây UART chéo và nối chung mass:

```text
Base Wi-Fi GPIO18 TX  ->  4G Gateway GPIO26 RX
Base Wi-Fi GPIO19 RX  <-  4G Gateway GPIO27 TX
Base Wi-Fi GND        ---  4G Gateway GND
```

SIM7600 trên board gateway tiếp tục dùng ESP32 GPIO16 RX và GPIO17 TX. Hai
đường UART đều chạy ở 115200 baud. Frame gồm magic, version, loại gói, cờ
retain, sequence, độ dài topic/payload và CRC32. Gateway trả ACK khi gói được
đưa vào queue và khi publish thành công.

Gateway học MAC Base từ topic UART rồi subscribe QoS 1 topic `aitogy/<BASE_MAC>/base/command`. Command được chuyển
xuống Base bằng frame `UART_MQTT_FRAME_MESSAGE`; gateway chờ ACK và retry tối
đa 3 lần, cách nhau 1 giây. Base nhớ 8 sequence gần nhất để retry không làm
thực thi lệnh trùng, sau đó gọi MQTT callback hiện có để parse JSON và phát
`GNSS_COMMAND_REQUEST` tới Rover qua ESP-NOW. Kết quả từ Rover đi ngược qua
UART và được gateway ưu tiên publish vào
`aitogy/<BASE_MAC>/base/command-result`.

Telemetry ECEF vẫn là best effort. Khi có bản tin mới cùng topic, gateway thay
payload cũ trong queue bằng payload mới để không tích lũy tọa độ cũ. Queue
`command-result` được tách riêng khỏi queue telemetry.

Build hai firmware:

```powershell
pio run -e esp32u_base_uart_4g_client
pio run -e esp32u_4g_uart_gateway
```
- Đã cập nhật nhãn cấu hình/log/README theo phần cứng dự kiến chưa hàn: UM980/UM982 dùng UART2 `TX2/RX2` nối sang ESP32 GPIO16/GPIO17.
- Đã bổ sung đo tốc độ thực theo mỗi chu kỳ health 30 giây: `uart_Bps` (byte UART/giây), `rtcm_fps` (frame RTCM hợp lệ/giây), `send_fps` (frame gửi thành công/giây) và `delivery` (tỷ lệ frame gửi thành công trong chu kỳ).
- Đã tắt `DEBUG_GNSS_UART_RAW_DUMP` sau khi xác nhận UART2 nhận đúng dữ liệu và đã tắt tiếp `DEBUG_RTCM_HEX_DUMP` khi chuyển sang đo throughput, tránh log HEX làm nghẽn Serial. Bộ đếm `uart_raw_bytes` vẫn hoạt động khi raw dump tắt.
- Đã build thành công firmware sau khi thêm thống kê tốc độ. PlatformIO báo RAM 44,600/327,680 bytes (13.6%), Flash 736,965/1,310,720 bytes (56.2%).
- Đã cập nhật tài liệu thuật toán chia RTCM frame thành từng fragment, byte-range/payload/packet length của từng fragment, ví dụ frame 27 byte và 500 byte, cùng logic stop-and-wait, callback, timeout 250 ms và tối đa 3 attempt hiện tại. Đã ghi rõ send callback chưa phải ACK ứng dụng từ Rover.
- Đã review năng lực tải toàn pipeline Base. Kết luận băng thông trung bình đủ cho RTCM 1 Hz hiện tại nhưng kiến trúc chưa chịu burst/retry an toàn vì reader và sender chạy nối tiếp, RX buffer UART mặc định 256 byte, chưa có queue, correction-age policy, ACK ứng dụng và telemetry Rover. Đã ghi thứ tự tối ưu đề xuất vào README.
- Đã build xác nhận cấu hình hiện tại với cả hai HEX dump đều tắt: PlatformIO thành công, RAM 44.600/327.680 byte (13,6%), Flash 736.709/1.310.720 byte (56,2%).
- Đã triển khai tối ưu pipeline theo mục 1-6: RX buffer 4096 byte, reader/sender task độc lập, queue 8 frame, timestamp và stale-drop 1000 ms, callback semaphore, frame deadline/retry, ACK ứng dụng Base/Rover và thống kê RTCM message ID. Giữ nguyên UART 115200 và ESP-NOW LR 250 Kbps theo yêu cầu.
- Đã build thành công firmware Base tối ưu: RAM tĩnh 44.784/327.680 byte (13,7%), Flash 739.941/1.310.720 byte (56,5%). Heap runtime của queue/stack được giám sát bằng trường `free_heap` trong health log.
- Đã set cứng công suất phát WiFi/ESP-NOW của Base bằng `WiFi.setTxPower(WIFI_POWER_19_5dBm)` sau khi bật STA radio. Firmware đọc lại `esp_wifi_get_max_tx_power()` và in log `[BASE][WIFI] TX power fixed raw=... dBm=...` để xác nhận runtime.
- Đã build xác nhận sau khi set TX power 19.5 dBm: `esp32u_base_espnow` SUCCESS, RAM 44.784/327.680 byte (13,7%), Flash 740.497/1.310.720 byte (56,5%).

### 2026-07-08

- Đã chốt kiến trúc broadcast discovery -> unicast cho pairing động: chỉ vào pairing mode khi bấm nút vật lý trên Base và Rover, Base broadcast `PAIR_DISCOVERY` trong cửa sổ ngắn, Rover đang pairing trả lời unicast, Base gửi `PAIR_CONFIRM`, hai bên lưu MAC vào NVS/Preferences rồi quay về unicast runtime. README đã ghi rõ cơ chế chống lẫn thiết bị khác bằng `network_id`, `pairing_key/auth_tag`, kiểm tra role/packet type và chỉ cho phép re-pair khi bấm nút.
- Đã triển khai Base-side pairing động: Base giữ nút pairing để broadcast `PAIR_DISCOVERY`, nhận `PAIR_RESPONSE`, validate `network_id`/`auth_tag`/nonce, gửi `PAIR_CONFIRM`, lưu MAC Rover vào NVS namespace `espnow` và chuyển runtime sang unicast.
- Đã mở rộng Base sang danh sách tối đa 5 Rover đã pair. Không còn `ESPNOW_ROVER_MAC` hard-code hay fallback MAC tĩnh; nếu NVS chưa có Rover, Base chờ pairing và chưa gửi RTCM runtime. Khi có nhiều Rover, Base gửi cùng RTCM frame multi-unicast lần lượt tới từng Rover và chờ ACK theo MAC từng Rover.
- Đã bổ sung health log Base: `rovers`, `stored_rovers`, `pairing`, `pair_resp`, `pair_confirm`, `pair_auth_fail`.
- Đã build xác nhận sau pairing Base-side: `esp32u_base_espnow` SUCCESS, RAM 44.984/327.680 byte (13,7%), Flash 749.521/1.310.720 byte (57,2%).
- Đã xóa MAC Rover hard-code khỏi Base ngày 2026-07-08: không còn `ESPNOW_ROVER_MAC`, không còn fallback MAC tĩnh trong `BaseEspnow_sender`. Base chỉ lấy danh sách Rover từ NVS/Preferences do pairing ghi vào; nếu NVS rỗng thì Base chờ pairing và chưa gửi RTCM runtime. Build `esp32u_base_espnow` SUCCESS, RAM 44.984/327.680 byte (13,7%), Flash 749.409/1.310.720 byte (57,2%).

### 2026-07-16

- Đã thêm `NetworkMqttManager` hỗ trợ Wi-Fi hoặc SIM7600/4G theo environment PlatformIO, chạy trong task priority thấp riêng để reconnect Internet/MQTT không chặn RTCM reader/sender hay vòng pairing ESP-NOW.
- Environment mặc định `esp32u_base_espnow` dùng Wi-Fi để test; Wi-Fi chỉ kết nối router trên `ESPNOW_WIFI_CHANNEL=6`. Environment `esp32u_base_4g_mqtt` đã được chuẩn bị cho TinyGSM/SIM7600, chờ xác nhận phần cứng GPIO/nguồn/PWRKEY.
- Đã thêm MQTT test với retained status/LWT, subscribe command chỉ để log và thống kê `[BASE][NETWORK_HEALTH]`. RTCM vẫn chỉ truyền bằng ESP-NOW LR.
- Đã thêm `include/Network_Secrets.example.h` và file local `include/Network_Secrets.h` bị Git bỏ qua để không đưa Wi-Fi/MQTT/APN credentials vào repository.
- Đã build thành công environment Wi-Fi `esp32u_base_espnow` sau khi thêm MQTT LLH: RAM 46.736/327.680 byte (14,3%), Flash 777.833/1.310.720 byte (59,3%).
- Đã build thành công environment 4G `esp32u_base_4g_mqtt` với TinyGSM/SIM7600 sau khi thêm MQTT LLH: RAM 45.664/327.680 byte (13,9%), Flash 767.525/1.310.720 byte (58,6%). Chưa kiểm thử kết nối thực vì mạch 4G chưa hoàn thiện.
- Đã bỏ hoàn toàn heartbeat MQTT 30 giây. Base publish mỗi snapshot LLH mới của Rover lên `aitogy/<BASE_MAC>/base/rovers/<ROVER_MAC>/llh`; payload có MAC, sequence, latitude, longitude, height MSL và source age. Mỗi snapshot chỉ publish một lần; lỗi publish retry tối đa 1 lần/giây và khi reconnect chỉ gửi trạng thái mới nhất, không phát lại backlog.

### 2026-07-21

- Đã triển khai remote command V1 để Base parse JSON MQTT, chọn Rover trực tiếp đã pair và gửi packet type `8` bằng ESP-NOW unicast với `network_id`, transaction ID, COM2 và auth tag. Cơ chế thời gian ban đầu đã được thay thế hoàn toàn bằng Fixed ECEF ngày 2026-07-23.
- Đã thêm packet result type `9`, timeout 10 giây, retry request một lần, queue command/result và publish kết quả lên `aitogy/<BASE_MAC>/base/command-result`.
- Khi Rover trả `uart_sequence_written`, Base giữ peer cho control nhưng tạm ngừng gửi RTCM tới peer đó; health log thêm `rtcm_rovers` và các counter `cmd_*`. Trạng thái này chỉ ở RAM trong V1.
- Đã thêm dependency ArduinoJson để parse command theo schema và allowlist; không chuyển tiếp raw UART command từ MQTT.
- Build xác nhận sau khi thêm hai chiều: `esp32u_base_espnow` SUCCESS, RAM 48.632/327.680 byte (14,8%), Flash 788.793/1.310.720 byte (60,2%); `esp32u_base_4g_mqtt` SUCCESS, RAM 47.560 byte (14,5%), Flash 778.469 byte (59,4%). Chưa test MQTT/UM980 trên phần cứng trong lượt này.
- Đã thêm action `switch_to_rover`: Base gửi command ID `2`, chờ application result rồi bật lại RTCM cho peer; `command_builder.py` có thêm `build_geotek_lte_unicore_rover_config()` với cùng chuỗi 4 lệnh COM2.

### 2026-07-22

- Đã thêm chọn Rover đích cho lệnh GNSS qua trường MQTT `target_mac`. Base parse MAC chặt chẽ, chỉ queue lệnh cho peer trực tiếp đã pair và trả lý do `target_not_paired` trong log khi MAC không thuộc NVS/runtime peer list. Action Fixed ECEF hiện bắt buộc có `target_mac`; chỉ action trở về Rover còn có thể chọn peer đầu tiên khi bỏ trống. Build xác nhận: `esp32u_base_espnow` SUCCESS, RAM 49.120 byte (15,0%), Flash 794.873 byte (60,6%); `esp32u_base_4g_mqtt` SUCCESS, RAM 48.048 byte (14,7%), Flash 784.525 byte (59,9%).
- Da them `fixQuality` GGA vao packet LLH truc tiep/qua Relay, snapshot va log Base; MQTT chi them `fix_quality` dang so.
- Build xac nhan: `esp32u_base_espnow` SUCCESS, RAM 48.752/327.680 byte (14,9%), Flash 789.205/1.310.720 byte (60,2%); `esp32u_base_4g_mqtt` SUCCESS, RAM 47.680 byte (14,6%), Flash 778.893 byte (59,4%).
- Đã chốt thiết kế RTCM hub: Temporary Base chỉ uplink RTCM về Base gốc; Base gốc ngừng forward RTCM local khi temp active nhưng vẫn đọc local làm hot standby, sau đó multi-unicast correction temp tới các Rover/Relay còn lại.
- Đã triển khai schema LLH/MQTT gồm `rtcm_source`, `temp_base_mac` và `rtcm_source_epoch`; nguồn được chụp lúc Base nhận LLH để không gắn sai khi handover xảy ra trước lúc publish.
- Đã triển khai packet uplink type `10` và ACK type `11`, reassembly/CRC/duplicate ACK, ingress queue và forwarding queue độc lập cho hop Temporary Base -> Base gốc.
- Đã triển khai state machine `LOCAL_ACTIVE -> TEMP_PREPARING -> TEMP_ACTIVE -> LOCAL_FALLBACK`: Base chỉ activate sau fixed guard 3000 ms và 2 chu kỳ đủ `1006/1074/1084/1094/1124`; mất RTCM temp 3500 ms thì tự fallback và tạo `streamId` downstream mới.
- Trong `TEMP_ACTIVE`, RTCM local vẫn được đọc/validate/thống kê nhưng tăng `local_suppressed` và không vào queue phát. Sender chỉ lấy correction temp, bỏ queue cũ theo `source_epoch` và không gửi ngược lại peer Temporary Base đang bị disable RTCM downstream.
- Snapshot LLH đã chụp `rtcm_source`, `temp_base_mac`, `rtcm_source_epoch` lúc nhận; MQTT publish đúng metadata đã chụp và dùng `null` cho `temp_base_mac` khi nguồn local.
- Health Base đã thêm counter uplink/source: `temp_frag_rx`, `temp_frame_ok`, `temp_invalid`, `temp_queue_drop`, `temp_ack`, `temp_ack_fail`, `source`, `source_epoch`, `source_switch`, `source_fallback`.
- Từ log thực tế có một peer mất ACK làm `stale_drop` tăng rất nhanh, đã thêm failure isolation từng Rover: ngưỡng 2 frame lỗi, cooldown/probe 3000 ms và round-robin điểm bắt đầu fan-out. Health thêm `peer_cooldown`, `peer_skip`, `peer_recovery`; log chuyển trạng thái dùng `[BASE][ESP-NOW][PEER]`.
- Build xác nhận sau sửa peer isolation: `esp32u_base_espnow` SUCCESS, RAM 49.120/327.680 byte (15,0%), Flash 794.177/1.310.720 byte (60,6%); `esp32u_base_4g_mqtt` SUCCESS, RAM 48.048 byte (14,7%), Flash 783.813 byte (59,8%). Chưa kiểm thử lại tình huống ngắt/cấp nguồn Rover trên phần cứng trong lượt này.

### 2026-07-23

- Đã loại bỏ hoàn toàn action Base theo thời gian, trường duration trong wire packet và lệnh `MODE BASE TIME`. Temporary Base hiện chỉ được cấu hình bằng ECEF lấy từ LLH RTK Fixed của chính Rover đích.
- Base yêu cầu `target_mac`, đợi snapshot trực tiếp có `fix_quality=4` và tuổi không quá 3000 ms, dùng ellipsoidal height để đổi WGS84 LLH sang ECEF rồi gửi command request type `8` dài 44 byte.
- Allowlist runtime chỉ còn `switch_to_base_fixed_ecef` và `switch_to_rover`; payload/packet cũ bị từ chối. Cần nạp đồng bộ Base, Relay và toàn bộ Rover do wire protocol đã đổi.
- Build xác nhận ECEF-only: `esp32u_base_espnow` SUCCESS, RAM 49.324/327.680 byte (15,1%), Flash 803.929/1.310.720 byte (61,3%); `esp32u_base_4g_mqtt` SUCCESS, RAM 48.260 byte (14,7%), Flash 793.481 byte (60,5%).

### 2026-07-24

- [x] Nâng wire protocol lên version `2`; telemetry type `6/7` chỉ truyền ECEF X/Y/Z `int64_t` scale `10000`, GNSS time, stream ID và fix quality. LLH không còn đi qua ESP-NOW hoặc MQTT.
- [x] Temporary Base Fixed ECEF dùng trực tiếp snapshot ECEF RTK Fixed của target, giữ chính xác bốn chữ số sau dấu phẩy; command request type `8` dài 44 byte.
- [x] Thêm `TEMP_RESETTING`: ngừng RTCM cũ, gửi command ID `4` để `CONFIG RTK DISABLE`, đợi ACK theo nhánh và mẫu non-RTK mới của từng Rover. Relay fan-out tới tối đa 5 child.
- [x] Thêm command ID `5` để `CONFIG RTK USER_DEFAULTS`; Base chỉ bật RTCM cho nhánh sau ACK resume. Khi handover thành công dùng stream temp, khi timeout resume các nhánh trước khi rollback local.
- [x] Sửa timeout `TEMP_PREPARING`: chờ đúng 15000 ms từ lúc bắt đầu cấu hình temp; không còn fallback sau guard 3000 ms chỉ vì chưa nhận frame đầu tiên.
- [x] Khi temp active, UM980 Base gốc chuyển sang Rover, nhận RTCM temp và tính `delta = base_reference_ecef - base_observed_ecef`; MQTT publish ECEF raw/corrected, chỉ áp dụng khi stream/time/fix đều hợp lệ.
- [x] MQTT chuyển sang `aitogy/<BASE_MAC>/base/rovers/<ROVER_MAC>/ecef`, không còn trường LLH. Buffer MQTT tăng lên 1024 byte.
- [x] Sửa panic Core 0 trong `_dtoa_r`: chuyển mảng 30 snapshot và buffer JSON ECEF khỏi stack task MQTT, format trực tiếp `int64_t` scale `10000` thay cho `%f`, tăng stack MQTT từ 6144 lên 10240 byte và thêm `stack_hwm_bytes` vào network health.
- [x] Chuyển mảng snapshot của task log ECEF khỏi stack để tránh cùng lỗi khi nhiều Rover online.
- [x] Thêm chốt loại trừ `tempBaseMac` trực tiếp trong vòng fan-out RTCM; Base không gửi correction downstream ngược lại Temporary Base ngay cả khi sender đang giữ snapshot peer cũ trong lúc chuyển vai trò.
- [x] Sửa panic `spinlock_acquire` khi bắt đầu handover: task `Temp RTCM RX` chỉ đặt cờ readiness, còn `beginResetCohort()` chạy trong loop chính; không còn gọi chuỗi reset/ESP-NOW nặng từ stack task nhận temp.
- [x] Chuyển buffer 30 snapshot reset cohort từ stack sang RAM tĩnh và tăng stack `Temp RTCM RX` từ 6144 lên 8192 byte. ELF xác nhận frame task nhận temp là 2432 byte, còn frame `beginResetCohort()` giảm từ 2656 xuống 336 byte.
- [x] Reset cohort giữ toàn bộ Rover trực tiếp đã pair và mọi Rover con từng có status, kể cả status cũ/offline. Rover không phản hồi sẽ làm gate đợi hoặc timeout 45 giây, không còn bị bỏ qua khiến log sai `rovers=0 branches=0`.
- [x] Build Base Wi-Fi sau sửa panic SUCCESS: RAM 60.092/327.680 byte (18,3%), Flash 821.397/1.310.720 byte (62,7%).
- [x] Build Base 4G sau sửa panic SUCCESS: RAM 59.020/327.680 byte (18,0%), Flash 810.861/1.310.720 byte (61,9%).
- [x] Build Rover thường SUCCESS: RAM 47.788/327.680 byte (14,6%), Flash 806.025/1.310.720 byte (61,5%).
- [x] Build Relay SUCCESS: RAM 48.532/327.680 byte (14,8%), Flash 828.989/1.310.720 byte (63,2%); native protocol test 8/8 PASSED.
- [ ] Chưa kiểm thử end-to-end trên UM980 thật: cần nạp đồng bộ cả ba firmware và xác nhận log `TEMP_RESETTING`, `TEMP_ACTIVE`, correction delta và fallback.

### 2026-07-25

- [x] Đổi flow-control Temporary Base trong `TEMP_ACTIVE`: không ACK ngay khi enqueue; chỉ gửi `TEMP_RTCM_ACK` sau khi `RTCM Sender` hoàn tất lượt forward frame xuống các Rover.
- [x] Thêm metadata `sourceMac`, `upstreamStreamId`, `upstreamSequence` vào queue temp và state chống duplicate. Retry của Temporary Base trong lúc chờ không tạo thêm bản sao downstream; duplicate sau completion dùng để gửi lại ACK nếu ACK trước bị mất.
- [x] Giữ ACK ngay trong `TEMP_PREPARING/TEMP_RESETTING` để readiness và reset gate không bị khóa. ACK sau forward vẫn được gửi khi downstream thất bại để stop-and-wait không mắc kẹt vô hạn.
- [x] Health thêm `temp_ack_deferred`, `temp_ack_wait_dup`, `temp_forward_done`, `temp_forward_fail`.
- [x] Chuyển buffer làm việc của `RTCM Sender` sang RAM tĩnh. ELF xác nhận stack frame sender chỉ 96 byte; stack frame `Temp RTCM RX` 2448 byte trên stack task 8192 byte.
- [x] Build Base Wi-Fi SUCCESS: RAM 62.236/327.680 byte (19,0%), Flash 822.129/1.310.720 byte (62,7%).
- [x] Build Base 4G SUCCESS: RAM 61.164/327.680 byte (18,7%), Flash 811.561/1.310.720 byte (61,9%).
- [ ] Chưa kiểm thử RF end-to-end: cần so sánh `send_fail`, `frames_dropped`, `temp_ack_wait_dup` và `temp_forward_fail` trước/sau thay đổi trên phần cứng.
