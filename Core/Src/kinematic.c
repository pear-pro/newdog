#include "kinematic.h"
#include "gait.h"
#include <math.h>

#define L1 12.5f
#define L2 26.0f
#define L3 26.0f
#define L4 12.5f
#define pi 3.141592f
#define R_min 15.2f // 足离电机轴心的最近距离。手量的，后期需要验证
#define R_max 38.5f
#define Res_x -16.4f // 挡板坐标。手量的，后期需要验证
#define Res_y 4.0f
#define Y_max 9.52f // y的最大值，由最小半径方程和经过挡板坐标的切线方程联立求得。后期精确测量挡板坐标后可更新此值


void inverseKinematic(Position_HandleTypeDef *hposition){
	float x = hposition->B_x;
	float y = hposition->B_y;
	
	/*---------------------对x，y限位-----------------------*/
	
	// (0,0)不可达
	if (x == 0 && y == 0){ return;}
	
	// 限高
	if (y < Y_max){y = Y_max;}
	
	// 运动半径之外的区域不取
	if ((x*x + y*y) < R_min*R_min){
		float k = R_min/sqrtf(x*x + y*y);
		x = k*x;
		y = k*y;}
	if ((x*x + y*y) > R_max*R_max){
		float k = R_max/sqrtf(x*x + y*y);
		x = k*x;
		y = k*y;}

	// 不撞挡板
	if (x < (L1*L1+Res_y*y)/Res_x){	x = (L1*L1+Res_y*y)/Res_x;}
	if (x > -(L1*L1+Res_y*y)/Res_x){ x = -(L1*L1+Res_y*y)/Res_x;}

	/*---------------------逆解算---------------------------*/
	
	// 三角几何项计算
    float a = 2.0f * x * L1;
    float b = 2.0f * y * L1;
    float c = x*x + y*y + L1*L1 - L2*L2;
	
	float d = 2.0f * x * L4;
	float e = 2.0f * y * L4;
	float f = x*x + y*y + L4*L4 - L3*L3;
	
	// 判别式（防 NaN）
	float delta_alpha = a*a + b*b - c*c;
	float delta_beta  = d*d + e*e - f*f;

	if (delta_alpha < 0.0f || delta_beta < 0.0f)
	{
		// 目标点不可达，直接返回或标记错误
		return;
	}
	
	// α 与 β 的两个解
	float alpha1 = (2.0f * atan2f(b + sqrtf(delta_alpha), a + c )- pi/2) * 180.0f / pi;
	float alpha2 = (2.0f * atan2f(b - sqrtf(delta_alpha), a + c) - pi/2) * 180.0f / pi;
	float beta1 = (pi/2 - 2.0f * atan2f(e + sqrtf(delta_beta),d + f)) * 180.0f / pi;
	float beta2 = (pi/2 - 2.0f * atan2f(e - sqrtf(delta_beta),d + f)) * 180.0f / pi;
	
	alpha1 = fmodf(alpha1,360.0f);
	alpha2 = fmodf(alpha2,360.0f);
	beta1 = fmodf(beta1,360.0f);
	beta2 = fmodf(beta2,360.0f);
	
	if (alpha1 == beta1) {
		float temp = beta1;
		beta1 = beta2;
		beta2 = temp;
	}

	// 防止两条腿打架
	if (alpha1 >= 0.0f && beta1 < 0.0f){
		if (alpha1 >= fabsf(beta1)){
			hposition->alpha = alpha1;
			hposition->beta = beta1;
		}else{
			hposition->alpha = alpha2;
			hposition->beta = beta2;
		}
	}else if (alpha1 < 0.0f && beta1 >= 0.0f){
		if (beta1 >= fabsf(alpha1)){
			hposition->beta = beta1;
			hposition->alpha = alpha1;
		}else{
			hposition->beta = beta2;
			hposition->alpha = alpha2;
		}
	}else if (alpha1 < 0.0f && beta1 < 0.0f){
		hposition->alpha = alpha2;
		hposition->beta = beta2;
	}else{
		hposition->alpha = alpha1;
		hposition->beta = beta1;
	}
}


