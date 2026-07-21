import logging

UBLOX_SAVE_CONFIG = b'\xb5\x62\x06\x09\x0d\x00\x00\x00\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x03\x1d\xab'

DEFAULT_GNSS_OPTIONS = {
    "constellations": {
        "gps": True,
        "glo": True,
        "gal": True,
        "bds": True,
        "qzss": False,
        "sbas": False,
    },
    "output_mode": "diagnostics_rtcm",
    "ports": ["USB"],
}

UBLOX_NMEA_KEYS = {
    "GGA": {"UART1": 0x209100BB, "UART2": 0x209100BC, "USB": 0x209100BD},
    "GSA": {"UART1": 0x209100C0, "UART2": 0x209100C1, "USB": 0x209100C2},
    "GSV": {"UART1": 0x209100C5, "UART2": 0x209100C6, "USB": 0x209100C7},
    "GST": {"UART1": 0x209100D4, "UART2": 0x209100D5, "USB": 0x209100D6},
    "DTM": {"UART1": 0x209100A7, "UART2": 0x209100A8, "USB": 0x209100A9},
    "RMC": {"UART1": 0x209100AC, "UART2": 0x209100AD, "USB": 0x209100AE},
    "GNS": {"UART1": 0x209100B6, "UART2": 0x209100B7, "USB": 0x209100B8},
    "VTG": {"UART1": 0x209100B1, "UART2": 0x209100B2, "USB": 0x209100B3},
    "GLL": {"UART1": 0x209100CA, "UART2": 0x209100CB, "USB": 0x209100CC},
    "GRS": {"UART1": 0x209100CF, "UART2": 0x209100D0, "USB": 0x209100D1},
    "ZDA": {"UART1": 0x209100D9, "UART2": 0x209100DA, "USB": 0x209100DB},
    "GBS": {"UART1": 0x209100DE, "UART2": 0x209100DF, "USB": 0x209100E0},
    "VLW": {"UART1": 0x209100E8, "UART2": 0x209100E9, "USB": 0x209100EA},
    "PUBX00": {"UART1": 0x209100ED, "UART2": 0x209100EE, "USB": 0x209100EF},
    "PUBX03": {"UART1": 0x209100F2, "UART2": 0x209100F3, "USB": 0x209100F4},
    "PUBX04": {"UART1": 0x209100F7, "UART2": 0x209100F8, "USB": 0x209100F9},
}

UBLOX_NAV_KEYS = {
    "NAV_PVT": {"UART1": 0x20910007, "UART2": 0x20910008, "USB": 0x20910009},
    "NAV_HPPOSLLH": {"UART1": 0x20910034, "UART2": 0x20910035, "USB": 0x20910036},
}

UBLOX_RTCM_KEYS = {
    "1005": {"UART1": 0x209102BE, "UART2": 0x209102BF, "USB": 0x209102C0},
    "1074": {"UART1": 0x2091035F, "UART2": 0x20910360, "USB": 0x20910361},  # GPS MSM4
    "1084": {"UART1": 0x20910364, "UART2": 0x20910365, "USB": 0x20910366},  # GLONASS MSM4
    "1094": {"UART1": 0x20910368, "UART2": 0x20910369, "USB": 0x2091036A},  # Galileo MSM4
    "1124": {"UART1": 0x2091036E, "UART2": 0x2091036F, "USB": 0x20910370},  # BeiDou MSM4
    "1230": {"UART1": 0x20910304, "UART2": 0x20910305, "USB": 0x20910306},  # GLONASS bias
}

UNICORE_NMEA_BY_CONSTELLATION = {
    "gps": ("gpgga", "gpgsa", "gpgsv", "gpgst"),
    "glo": ("glgga", "glgsa", "glgsv"),
    "gal": ("gagga", "gagsa", "gagsv"),
    "bds": ("bdgga", "bdgsa", "bdgsv"),
}

UNICORE_RTCM_BY_CONSTELLATION = {
    "gps": ("1074", "1019"),
    "glo": ("1084", "1020"),
    "gal": ("1094", "1045"),
    "bds": ("1124", "1042"),
    "qzss": ("1114", "1044"),
}


def _ubx_packet(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    header = b'\xb5\x62' + bytes([msg_class, msg_id]) + len(payload).to_bytes(2, 'little')
    ck_a, ck_b = 0, 0
    for byte in header[2:] + payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return header + payload + bytes([ck_a, ck_b])


def _ubx_cfg_valset_u1(entries: dict[int, int], layers: int = 0x07) -> bytes:
    payload = bytearray([0x00, layers & 0x07, 0x00, 0x00])
    for key, value in entries.items():
        payload.extend(int(key).to_bytes(4, 'little'))
        payload.append(1 if value else 0)
    return _ubx_packet(0x06, 0x8A, bytes(payload))


def normalize_gnss_options(options: dict | None) -> dict:
    raw = options if isinstance(options, dict) else {}
    constellations = dict(DEFAULT_GNSS_OPTIONS["constellations"])
    for key, value in (raw.get("constellations") or {}).items():
        normalized_key = str(key).strip().lower()
        if normalized_key in constellations:
            constellations[normalized_key] = bool(value)
    if not any(constellations.values()):
        constellations["gps"] = True

    output_mode = str(raw.get("output_mode") or DEFAULT_GNSS_OPTIONS["output_mode"]).strip().lower()
    if output_mode not in {"diagnostics_rtcm", "rtcm_only"}:
        output_mode = DEFAULT_GNSS_OPTIONS["output_mode"]

    raw_ports = raw.get("ports") or DEFAULT_GNSS_OPTIONS["ports"]
    if isinstance(raw_ports, str):
        raw_ports = [raw_ports]
    ports = []
    for port in raw_ports:
        normalized_port = str(port).strip().upper()
        if normalized_port in {"UART1", "UART2", "USB"} and normalized_port not in ports:
            ports.append(normalized_port)
    if not ports:
        ports = list(DEFAULT_GNSS_OPTIONS["ports"])

    return {
        "constellations": constellations,
        "output_mode": output_mode,
        "ports": ports,
    }


def build_ublox_output_config_command(gnss_options: dict | None = None) -> bytes:
    opts = normalize_gnss_options(gnss_options)
    constellations = opts["constellations"]
    output_mode = opts["output_mode"]
    ports = opts["ports"]
    diagnostics_enabled = output_mode != "rtcm_only"
    entries: dict[int, int] = {}

    for message, keys_by_port in UBLOX_NMEA_KEYS.items():
        enabled = diagnostics_enabled and message in {"GGA", "GSA", "GSV", "GST"}
        for port in ports:
            entries[keys_by_port[port]] = 1 if enabled else 0

    for keys_by_port in UBLOX_NAV_KEYS.values():
        for port in ports:
            entries[keys_by_port[port]] = 1 if diagnostics_enabled else 0

    rtcm_enabled = {
        "1005": True,
        "1074": constellations["gps"],
        "1084": constellations["glo"],
        "1094": constellations["gal"],
        "1124": constellations["bds"],
        "1230": constellations["glo"],
    }
    for message, keys_by_port in UBLOX_RTCM_KEYS.items():
        for port in ports:
            entries[keys_by_port[port]] = 1 if rtcm_enabled.get(message, False) else 0

    return _ubx_cfg_valset_u1(entries)


def _append_unicore_output_commands(commands: list[bytes], port: str, gnss_options: dict | None = None) -> None:
    opts = normalize_gnss_options(gnss_options)
    constellations = opts["constellations"]
    diagnostics_enabled = opts["output_mode"] != "rtcm_only"

    if diagnostics_enabled:
        for constellation, messages in UNICORE_NMEA_BY_CONSTELLATION.items():
            if not constellations.get(constellation):
                continue
            for message in messages:
                commands.append(f'{message} {port} 1\r\n'.encode('ascii'))
                commands.append(b'$DELAY_200$')

    rtcm_msgs = ["1006", "1033"]
    for constellation, messages in UNICORE_RTCM_BY_CONSTELLATION.items():
        if constellations.get(constellation):
            rtcm_msgs.extend(messages)
    for message in dict.fromkeys(rtcm_msgs):
        commands.append(f'rtcm{message} {port} 1\r\n'.encode('ascii'))
        commands.append(b'$DELAY_200$')


def build_base_survey_in_command(sensor_type: str, duration: int, accuracy: float, gnss_options: dict | None = None) -> list[bytes]:
    commands = []
    
    if sensor_type == 'Ublox':
        # Tạo message với cấu trúc chính xác
        message = bytearray(b'\xb5\x62\x06\x71\x28\x00' + b'\x00' * 42)
        
        # Byte 8: Mode = 1 (Survey-In)
        message[8] = 1
        
        # Chuyển đổi duration và accuracy (ép kiểu float đề phòng dữ liệu là string)
        svinMinDur_bytes = int(duration).to_bytes(4, byteorder='little')
        svinAccLimit_bytes = int(float(accuracy) * 10000).to_bytes(4, byteorder='little')
        
        # Ghi vào vị trí đúng
        for i in range(4):
            message[30 + i] = svinMinDur_bytes[i]
            message[34 + i] = svinAccLimit_bytes[i]
        
        # Tính checksum
        CK_A, CK_B = 0, 0
        for i in range(2, 46):
            CK_A = (CK_A + message[i]) & 0xff
            CK_B = (CK_B + CK_A) & 0xff
        
        message[46] = CK_A
        message[47] = CK_B
        
        commands.append(bytes(message))
        commands.append(build_ublox_output_config_command(gnss_options))
        # Lệnh Save Config
        commands.append(UBLOX_SAVE_CONFIG)
        
    elif sensor_type == 'Unicorecomm':
        # BƯỚC 1: Clear all logs (TẮT TẤT CẢ OUTPUT)
        commands.append(b'unlogall\r\n')
        commands.append(b'$DELAY_1000$')
        
        # BƯỚC 2: Set base mode (Survey-In)
        cmd_str = f'mode base time {duration}\r\n'
        logging.info(f"🔍 Survey-In command: {repr(cmd_str)}")
        commands.append(cmd_str.encode('ascii'))
        commands.append(b'$DELAY_2000$')
        
        logging.info("Configuring Unicore output messages on COM3")
        _append_unicore_output_commands(commands, "com3", gnss_options)
        
        # BƯỚC 5: Save config
        commands.append(b'saveconfig\r\n')
        
    return commands


def build_base_fixed_lla_command(sensor_type: str, lat: float, lon: float, alt: float, accuracy: float, gnss_options: dict | None = None) -> list[bytes]:
    commands = []
    
    if sensor_type == 'Ublox':
        message = bytearray(b'\xb5\x62\x06\x71\x28\x00' + b'\x00' * 42)
        
        # Byte 8: Mode = 2 (Fixed)
        message[8] = 2
        # Byte 9: LLA mode = 1
        message[9] = 1
        
        # Ép kiểu float cho tọa độ (đề phòng trường hợp đầu vào là string để giữ precision)
        f_lat = float(lat)
        f_lon = float(lon)
        f_alt = float(alt)
        f_acc = float(accuracy)

        multiplier = 10000000  # LLA multiplier
        
        # === XỬ LÝ LATITUDE ===
        XOrLat_int = int(f_lat * multiplier)
        XOrLat_hp_value = int((f_lat * multiplier - XOrLat_int) * 100)
        XOrLat_hp_bytes = XOrLat_hp_value.to_bytes(1, byteorder='little', signed=True)
        XOrLat_bytes = XOrLat_int.to_bytes(4, byteorder='little', signed=True)
        
        # === XỬ LÝ LONGITUDE ===
        YOrLon_int = int(f_lon * multiplier)
        YOrLon_hp_value = int((f_lon * multiplier - YOrLon_int) * 100)
        YOrLon_hp_bytes = YOrLon_hp_value.to_bytes(1, byteorder='little', signed=True)
        YOrLon_bytes = YOrLon_int.to_bytes(4, byteorder='little', signed=True)
        
        # === XỬ LÝ ALTITUDE ===
        ZOrAlt_int = int(f_alt * 100)
        ZOrAlt_hp_value = int((f_alt * 100 - ZOrAlt_int) * 100)
        ZOrAlt_hp_bytes = ZOrAlt_hp_value.to_bytes(1, byteorder='little', signed=True)
        ZOrAlt_bytes = ZOrAlt_int.to_bytes(4, byteorder='little', signed=True)
        
        # Fixed Position Accuracy
        fixedPosAcc_bytes = int(f_acc * 10000).to_bytes(4, byteorder='little')
        
        # Ghi vào message
        for i in range(4):
            message[10 + i] = XOrLat_bytes[i]
            message[14 + i] = YOrLon_bytes[i]
            message[18 + i] = ZOrAlt_bytes[i]
            message[26 + i] = fixedPosAcc_bytes[i]
        
        # Ghi HP bytes
        message[22] = XOrLat_hp_bytes[0]
        message[23] = YOrLon_hp_bytes[0]
        message[24] = ZOrAlt_hp_bytes[0]
        
        # Tính checksum
        CK_A, CK_B = 0, 0
        for i in range(2, 46):
            CK_A = (CK_A + message[i]) & 0xff
            CK_B = (CK_B + CK_A) & 0xff
        
        message[46] = CK_A
        message[47] = CK_B
        
        commands.append(bytes(message))
        commands.append(build_ublox_output_config_command(gnss_options))
        # Lệnh Save Config
        commands.append(UBLOX_SAVE_CONFIG)
        
    elif sensor_type == 'Unicorecomm':
        # BƯỚC 1: Clear all logs (TẮT TẤT CẢ OUTPUT)
        commands.append(b'unlogall\r\n')
        commands.append(b'$DELAY_1000$')
        
        # BƯỚC 2: Set fixed position - GIỮ NGUYÊN FORMAT NGƯỜI DÙNG NHẬP
        lat_str = str(lat)
        lon_str = str(lon)
        alt_str = str(alt)
        
        cmd_str = f'mode base {lat_str} {lon_str} {alt_str}\r\n'
        logging.info(f"🔍 Fixed position command: {repr(cmd_str)}")
        commands.append(cmd_str.encode('ascii'))
        commands.append(b'$DELAY_2000$')
        logging.info("Configuring Unicore output messages on COM3")
        _append_unicore_output_commands(commands, "com3", gnss_options)
        
        commands.append(b'saveconfig\r\n')
        
    return commands


def build_geotek_lte_unicore_config(setup_method: str, duration: int = 60, lat: float = 0, lon: float = 0, alt: float = 0) -> list[bytes]:
    """
    Hàm chuyên biệt dành riêng cho chip Unicorecomm LTE.
    Sử dụng mặc định cổng COM2 và chuỗi lệnh tối ưu theo yêu cầu người dùng.
    Trình tự: FRESET -> unlogall -> mode base -> NMEA -> RTCM -> saveconfig
    """
    commands = []
    port = "com2"
    
    # 1. Reset vật lý chip và xóa hết log cũ
    commands.append(b'FRESET\r\n')
    commands.append(b'unlogall\r\n')
    
    # 2. Cấu hình chế độ Base (Survey-In hoặc Fixed)
    if setup_method.upper() == 'SURVEY_IN':
        commands.append(f'mode base time {duration}\r\n'.encode('ascii'))
    else:
        # Giữ nguyên độ chính xác của tọa độ
        commands.append(f'mode base {lat} {lon} {alt}\r\n'.encode('ascii'))
    
    # 3. Bật các bản tin NMEA cần thiết trên COM2 (12 bản tin)
    # Unicore LTE: chi bat GGA/GST. GSA/GSV lam stream MQTT/RTCM khong on dinh.
    nmea_msgs = [
        'gpgga',
    ]
    for msg in nmea_msgs:
        commands.append(f'{msg} {port} 1\r\n'.encode('ascii'))
        
    # 4. Bật các bản tin RTCM cần thiết trên COM2 (10 bản tin)
    rtcm_msgs = [
        '1006', '1033', '1074', '1124', '1084', '1094', '1042', '1019', '1020', '1045'
    ]
    for msg in rtcm_msgs:
        commands.append(f'rtcm{msg} {port} 1\r\n'.encode('ascii'))
    
    # 5. Lưu cấu hình
    commands.append(b'saveconfig\r\n')
    
    return commands


def build_geotek_lte_unicore_rover_config() -> list[bytes]:
    """Return the COM2 command sequence that restores a temporary Base to Rover."""
    return [
        b'unlogall\r\n',
        b'mode rover survey\r\n',
        b'gpgga com2 1\r\n',
        b'saveconfig\r\n',
    ]


# === HÀM DEBUG ===
def debug_command(command_bytes: bytes) -> str:
    """In ra chuỗi byte dưới dạng hex để debug"""
    return ' '.join(f'{b:02x}' for b in command_bytes)
