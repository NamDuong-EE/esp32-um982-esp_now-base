#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

struct gga_data_struct {
  double lat;
  double lon;
  String satellites;
};
using gga_data_t = struct gga_data_struct;

#endif