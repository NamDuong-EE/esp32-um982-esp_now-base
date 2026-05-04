#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

typedef struct {
  double lat;
  double lon;
  String rtk_status;
  String satellites;
} gga_data_struct;

typedef struct {
  String height_m;
  String heading_deg;
  String pitch_deg;
  String roll_deg;
  String velocity_kmh;
} ksxt_data_struct;

#endif