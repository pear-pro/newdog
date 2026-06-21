#include "filter.h"

void SystemClock_Config(void);

float NowState=0.0f;
float LastState=0.0f;
void Kalman(Kalman_filter*ekf,float input)
{
	
	LastState=NowState;
	ekf->NowP=ekf->LastP+ekf->Q;
	ekf->Kg=ekf->NowP/(ekf->NowP+ekf->R);
	ekf->Out=ekf->Out+ekf->Kg*(input-ekf->Out);
	ekf->LastP=(1-ekf->Kg)*ekf->NowP;
	NowState=ekf->Out;
	if(fabsf(NowState-LastState)>50.0f)
	{
		ekf->Out=LastState;
	}
}

Kalman_filter GyroZ_Kalman={0.02,0,0,0,0.02,0.001};
