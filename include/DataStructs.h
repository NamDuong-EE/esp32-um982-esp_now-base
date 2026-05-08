#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

struct gga_data_struct {
  double lat;
  double lon;
  String rtk_status;
  String satellites;
};
using gga_data_t = struct gga_data_struct;

struct ksxt_data_struct {
  double height_m;
  double heading_deg;
  double pitch_deg;
  double roll_deg;
  double velocity_kmh;
};
using ksxt_data_t = struct ksxt_data_struct;

#endif