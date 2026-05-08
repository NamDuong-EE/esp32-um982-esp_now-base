#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

typedef struct {
  double lat;
  double lon;
  String rtk_status;
  String satellites;
} gga_data_struct;

typedef struct {
  double height_m;
  double heading_deg;
  double pitch_deg;
  double roll_deg;
  double velocity_kmh;
} ksxt_data_struct;

#endif