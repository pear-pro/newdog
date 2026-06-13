#ifndef _filter_h
#define _filter_h
#include "math.h"

typedef struct
{
	float LastP;
	float NowP;
	float Out;
	float Kg;
	float Q;
	float R;
}Kalman_filter;
void Kalman(Kalman_filter*ekf,float input);
extern Kalman_filter GyroZ_Kalman;
#endif
