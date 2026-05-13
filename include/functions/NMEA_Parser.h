#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <Arduino.h>
#include "../DataStructs.h"
#include "Top_Lvl_Config.h"

// Hàm chuyển đổi DDMM.MMMM sang Decimal
double nmeaToDecimal(String const &nmeaPos, String const &dir);

// Hàm nhận chuỗi GGA gốc và trả về chuỗi JSON
String parseGGA_toJSON(gga_data_t const &ggaData);
boolean parseGGA_toStruct(String ggaMsg, gga_data_t &ggaData);

#endif