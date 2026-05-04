# ESP32 GNSS Gateway (UM980 / UM982)

Firmware dành cho vi điều khiển ESP32 (ví dụ board TDM2402) đóng vai trò làm Gateway kết nối với module GNSS định vị động học thời gian thực (RTK) như Unicore UM980 hoặc UM982. 

## Tính năng chính
- **Giao tiếp GNSS**: Đọc dữ liệu NMEA (như `$GNGGA`, `$GPGGA`) từ module UM980/UM982 qua Serial.
- **Phân tích dữ liệu**: Trích xuất các thông số tọa độ (Latitude, Longitude), số lượng vệ tinh, trạng thái RTK từ chuỗi NMEA và đóng gói thành định dạng JSON.
- **MQTT Publisher**: Gửi dữ liệu vị trí JSON lên MQTT Broker để giám sát tọa độ theo thời gian thực.
- **Giám sát trạng thái (Health Check)**: Định kỳ báo cáo các thông số như RAM trống, Uptime, cường độ tín hiệu WiFi, trạng thái MQTT/NTRIP và chất lượng dữ liệu GNSS.
- **NTRIP Client over IP**: Kết nối đến NTRIP Caster qua mạng IP, nhận dữ liệu cải chính định vị (RTCM) và đẩy ngược lại cho module GNSS để đạt được độ chính xác RTK (mức cm). Hỗ trợ truyền lại chuỗi GGA cho Caster để xác thực.

## Yêu cầu môi trường và thiết bị
- Môi trường phát triển: PlatformIO IDE hoặc bất kỳ IDE nào hỗ trợ PlatformIO.
- Bảng mạch được hỗ trợ: esp32dev (TDM240x series), Heltec WiFi LoRa 32 V4 (có thể chỉnh trong `platformio.ini`).
- Module GNSS: Unicore UM980 hoặc UM982 (kết nối qua UART).
- Các thư viện được sử dụng:
    - `Arduino` cho lập trình cơ bản trên ESP32.
    - `PubSubClient` cho MQTT.
    - `WiFi` cho kết nối mạng WiFi.
    - `TinyGSM` cho kết nối mạng 4G (nếu sử dụng modem 4G).
    - `LoRaWANHeltec ESP32 Dev-Boards` cung cấp các thư viện LoRa nếu sử dụng Heltec V4, bao gồm thư viện `LoRaWan_APP`.
    - `ArduinoJson` để xử lý JSON.

## Tổ chức mã nguồn:
### Tổ chức nhánh:
- `main`: Nhánh chính chứa mã nguồn ổn định, đã được kiểm tra kỹ lưỡng. Chỉ được nhận merge từ nhánh `develop` dưới dạng squash commit để giữ lịch sử sạch sẽ. Không được phép merge trực tiếp vào `main` từ các nhánh tính năng hoặc test.
- `develop`: Nhánh phát triển, cũng là nhánh mặc định trên GitHub, nơi các tính năng mới được thêm vào và thử nghiệm. Sau khi hoàn thiện và kiểm tra, các thay đổi sẽ được merge vào nhánh `main`. Squash commit từ `main` phải được merge ngược lại vào `develop` ngay sau khi merge để đồng bộ lịch sử.
- `feature/<tên-tính-năng>`: Các nhánh tính năng riêng biệt để phát triển các tính năng mới hoặc sửa lỗi cụ thể. Sau khi hoàn thành, sẽ được merge vào `develop`.
- `feature-test/<tên-tính-năng>`: Các nhánh riêng để viết và chạy unit test cho các tính năng tương ứng với các nhánh `feature/<tên-tính-năng>`. Sau khi hoàn thành, sẽ được merge vào `feature/<tên-tính-năng>`. Khi nhánh `feature/<tên-tính-năng>` bị xoá, nhánh `feature-test/<tên-tính-năng>` sẽ được xoá.
- `documentation`: Các nhánh riêng để viết tài liệu hướng dẫn sử dụng, cấu hình, v.v. Sau khi hoàn thành phần tài liệu, sẽ được merge vào `develop`. Nhánh này chỉ được phép merge vào `develop` và cũng chỉ được nhận merge từ `develop` để đồng bộ.

### Tổ chức thư mục:
```
├── .pio/           # Thư mục do PlatformIO tạo ra chứa file biên dịch, 
|               thư viện đã cài, v.v. Không chỉnh sửa trực tiếp.
|               thư mục này không xuất hiện trong repo vì đã được thêm vào .gitignore
| 
├── boards/
|   └── heltec_wifi_lora_32_v4.json  # Cấu hình PlatformIO để biên dịch
|                                     mã nguồn cho Heltec V4
|                                     (đây là file tuỳ chỉnh do PlatformIO chưa
|                                     duyệt cấu hình Heltec V4 chính thức)
| 
├──lib/                             # Thư viện riêng của dự án (hiện chưa cần thiết).
|                                  Sẽ được biên dịch thành thư viện liên kết tĩnh
| 
├──include/                         # Header files chứa cấu hình, định nghĩa hằng số,
|   |                                 khai báo biến, đối tượng và prototype của hàm.
|   |                                 Ngoại trừ main không có header và một số header
|   |                                 được khai báo trong đoạn markdown này,
|   |                                 các file header còn lại được đặt tên và tổ chức hoàn toàn
|   |                                 giống với tên và vị trí file .cpp tương ứng trong src/
|   |
|   ├── Top_Lvl_Config.h            # Cấu hình cấp cao trước khi biên dịch
|   └── Prog_Config.h               # Cấu hình hoạt động của chương trình
| 
├──src/                             # Mã nguồn chính của chương trình.
|    ├── main.cpp                       # Điểm vào chính của chương trình, chứa hàm setup() và loop()
|    ├── helper.cpp                     # Các hàm phụ trợ để xử lý dữ liệu, kết nối, v.v.
|    ├── functions/                      # Thư mục con chứa các hàm được tổ chức theo chức năng (MQTT, NTRIP, NMEA parsing)
|    |  ├── MQTT_Manager.cpp                     # Hàm xử lý kết nối và gửi dữ liệu qua MQTT
|    |  ├── NTRIP_Handler_IP.cpp                 # Hàm xử lý kết nối và nhận dữ liệu từ NTRIP Caster qua IP
|    |  └── NMEA_Parser.cpp                      # Hàm xử lý phân tích chuỗi NMEA
|    └──hardware/                       # Thư mục con chứa các hàm liên quan đến phần cứng (wifi, 4g, lora)
|       ├── WiFi_handler.cpp                     # Hàm xử lý kết nối WiFi
|       ├── Sim_handler.cpp                      # Hàm xử lý kết nối 4G
|       └── LoRa_handler.cpp                     # Hàm xử lý kết nối LoRa
|
├──test/                            # Thư mục dành cho việc viết unit test
|                                  và sử dụng PlatformIO Test Runner.
| 
├──variants/                        # Cấu hình biến thể phần cứng (ví dụ: Heltec V4, TDM2402)
|   |                                được định nghĩa dưới dạng macro trong file header (.h)
|   |
|   └── heltec_V4
|           |
|           └── pins_arduino.h     # Định nghĩa chân GPIO cho board Heltec V4
|     
└──platformio.ini                    # File cấu hình chính của PlatformIO, xác định board,
```

## Cấu hình chương trình và hệ thống:
### Cấu hình cấp cao trước khi biên dịch, được lưu trong `include/Top_Lvl_Config.h`:
Các cấu hình sau có thể được sửa trong file `Top_Lvl_Config.h` hoặc đưa vào dưới dạng tham số biên dịch trong `platformio.ini` (`-D<MACRO>[=<VALUE>]`).
- `WIFI_LORA_32_V4`: Định nghĩa loại cấu hình phần cứng (ví dụ Heltec V4).
- `LORAWAN_DEBUG_LEVEL`: Mức độ debug cho thư viện LoRaWAN từ 0 đến 2 (0 = tắt debug, 1 = cơ bản, 2 = chi tiết).
- `CONNECT_USING_WIFI`: Kết nối mạng bằng WiFi (0 = tắt, 1 = bật).
- `CONNECT_USING_4G`: Kết nối mạng bằng 4G (0 = tắt, 1 = bật). Nếu cấu hình này và `CONNECT_USING_WIFI` đều được bật, báo lỗi. Nếu cùng tắt, chọn WiFi.
- `NMEA_COMMUNICATION_PROTOCOL`: Chồng giao thức truyền dữ liệu cải chính NTRIP (0 = qua TCP/IP stack, 1 = qua LoRa).

### Cấu hình hoạt động:
Các cấu hình sau được khai báo dưới dạng hằng số inline trong `include/Prog_Config.h` và có thể được sửa trực tiếp trong file này:

- Thiết lập chân UART kết nối với module GNSS (`RX_GNSS`, `TX_GNSS`)
- Thiết lập chân kết nối với module 4G (nếu sử dụng) (`TX_TO_MODEM_RX`, `RX_TO_MODEM_TX`, `MODEM_DC_PIN`, `MODEM_DTR_PIN`)
- Thông tin mạng WiFi (nếu sử dụng) (`SSID`, `Password`)
- Thông tin mạng 4G (`APN`, `User`, `Pass`) nếu sử dụng modem 4G
- Cấu hình kết nối MQTT (`Server`, `Port`, `User`, `Pass`, `Topics`)
- Cấu hình tài khoản NTRIP (`NTRIP_MODE`, `NTRIP_CASTER_IP`, `NTRIP_CASTER_PORT`, `NTRIP_MOUNTPOINT`, `NTRIP_AUTH`(base64))
- Cấu hình kiểm tra sức khoẻ định kỳ (`HEALTH_INTERVAL`)