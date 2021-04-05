#pragma once

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

typedef struct _dataType {

	double Q_tar[2];
	double Q_cur[2];

}DataType_t;


typedef struct _controlData {

  double position_Y;
  double velocity_Y;
  double position_P;
  double velocity_P;

}ControlData_t;


typedef struct _packet_data {

	unsigned char header[4];
	unsigned char size, id;
	unsigned char mode, check;

  int pos_Y;
  int velo_Y;
  int pos_P;
  int velo_P;

}Packet_data_t;


typedef union _packet {

	Packet_data_t data;
	unsigned char buffer[sizeof(Packet_data_t)];

}Packet_t;
