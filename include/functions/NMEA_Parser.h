#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <Arduino.h>
#include "../DataStructs.h"
#include "Top_Lvl_Config.h"

// Hàm chuyển đổi DDMM.MMMM sang Decimal
double nmeaToDecimal(String nmeaPos, String dir);

// Hàm nhận chuỗi GGA gốc và trả về chuỗi JSON
String parseGGA_toJSON(gga_data_struct ggaData);
boolean parseGGA_toStruct(String ggaMsg, gga_data_struct &ggaData);

String parseKSXT_toJSON(ksxt_data_struct ksxtData);
boolean parseKSXT_toStruct(String ksxtMsg, ksxt_data_struct &ksxtData);

#endif