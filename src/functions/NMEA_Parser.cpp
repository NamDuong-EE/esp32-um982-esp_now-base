#include "functions/NMEA_Parser.h"
#include <string>

double nmeaToDecimal(String const &nmeaPos, String const &dir) {
  if (nmeaPos.length() < 4) return 0.0;
  int dotIndex = nmeaPos.indexOf('.');
  if (dotIndex < 2) return 0.0; 
  int degrees = nmeaPos.substring(0, dotIndex - 2).toInt();
  double minutes = nmeaPos.substring(dotIndex - 2).toDouble();
  double decimal = degrees + (minutes / 60.0);
  if (dir == "S" || dir == "W") decimal = -decimal;
  return decimal;
}

String parseGGA_toJSON(gga_data_t const &ggaData) {
  std::string jsonPayload(128, '\0');
  snprintf(&jsonPayload[0], jsonPayload.size(),
           R"({"lat":%.7f,"lon":%.7f,"rtk_status":%s,"satellites":%s})",
           ggaData.lat, ggaData.lon, ggaData.rtk_status.c_str(), ggaData.satellites.c_str());
  return String(jsonPayload.c_str());
}

boolean parseGGA_toStruct(String ggaMsg, gga_data_t &ggaData) {
  std::array<int, 15> comma{};
  int count = 0;
  for (int i = 0; i < ggaMsg.length(); i++) {
    if (ggaMsg[i] == ',') {
      comma[count] = i;
      count++;
      if (count >= 15) break;
    }
  }

  if (count >= 14) {
    String latStr = ggaMsg.substring(comma[1] + 1, comma[2]);
    String latDir = ggaMsg.substring(comma[2] + 1, comma[3]);
    String lonStr = ggaMsg.substring(comma[3] + 1, comma[4]);
    String lonDir = ggaMsg.substring(comma[4] + 1, comma[5]);
    String rtkStr = ggaMsg.substring(comma[5] + 1, comma[6]);
    String satStr = ggaMsg.substring(comma[6] + 1, comma[7]);

    if (!latStr.isEmpty() && !lonStr.isEmpty()) {
      double latDD = nmeaToDecimal(latStr, latDir);
      double lonDD = nmeaToDecimal(lonStr, lonDir);
      
      ggaData.lat = latDD;
      ggaData.lon = lonDD;
      ggaData.rtk_status = rtkStr;
      ggaData.satellites = satStr;
               
      return true;
    }
  }
  return false;
}
