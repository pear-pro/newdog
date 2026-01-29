#include "gait.h"
#include <math.h>
#include "kinematic.h"
#define pi 3.1415926f


void Jump(Position_HandleTypeDef * hposition )
	{
		static float t = 0.0f;
		float X1, Y1, sigma;
		t=t + g_gait_config.freq;
		 if (t >= g_gait_config.T) {
        t = 0.0f; // 超过完整周期后重置
    }
	  sigma = 2.0f * pi * t / (0.5f * g_gait_config.T);
    if(t<=g_gait_config.T/2.0f)
		{
			
			X1 = g_gait_config.stride * ((sigma - sin(sigma)) / (2.0f * pi));
			Y1 = g_gait_config.maxHeight - g_gait_config.height * (1.0f - cos(sigma)) / 2.0f;
			
		}
    else
		{
			X1 = g_gait_config.stride/2.0f - g_gait_config.stride * ((sigma - sin(sigma))/ (2.0f * pi));
			float landing_sigma=sigma-2.0f*pi;
			Y1 = g_gait_config.maxHeight + (g_gait_config.height / 2.0f) * (1.0f - cos(landing_sigma)) / 2.0f;
        // 落地后（后半周期后期），锁定支撑高度，避免小幅波动
        if (landing_sigma >= pi) {
            Y1 = g_gait_config.maxHeight;
        }
		}
		hposition->B_x = X1;
		hposition->B_y = Y1;
}
	    





	