#include "gait.h"
#include <math.h>
#include "kinematic.h"
#define pi 3.1415926f


void Jump(Position_HandleTypeDef * hposition )
	{
		static float t = 0.0f;
		float X1, Y1, sigma;
		t=t + g_gait_config.freq;
	  sigma = 2.0f * pi * t / (0.5f * g_gait_config.T);
    if(t<=g_gait_config.T/2.0f)
		{
			
			X1 = g_gait_config.stride * ((sigma - sin(sigma)) / (2.0f * pi));
			Y1 = g_gait_config.maxHeight - g_gait_config.height * (1.0f - cos(sigma)) / 2.0f;
			
		}
		else if(t>g_gait_config.T/2.0f && t <=g_gait_config.T)
		{
			X1 = g_gait_config.stride/2.0f - g_gait_config.stride * ((sigma - sin(sigma))/ (2.0f * pi));
			Y1 = g_gait_config.maxHeight;
		}
		hposition->B_x = X1;
		hposition->B_y = Y1;
}
	    





	