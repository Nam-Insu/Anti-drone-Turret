/*
 * dataType.h
 *
 * Created: 2019-10-27 오후 7:50:24
 *  Author: Administrator
 */ 


#ifndef DATATYPE_H_
#define DATATYPE_H_

typedef struct _packet_data{
	
	unsigned char header[4];
	unsigned char size, id;
	unsigned char mode, check;

	int32_t pos_Y;
	int32_t velo_Y;
        int32_t pos_P;
	int32_t velo_P;
        
        
}Packet_data_t;


typedef union _packet{
	
	Packet_data_t data;
	unsigned char buffer[sizeof(Packet_data_t)];
	
}Packet_t;



#endif /* DATATYPE_H_ */