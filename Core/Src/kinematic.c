#include "kinematic.h"
#include "gait.h"
#include "global_var.h"
#include <math.h>
#include <stdint.h>

#define L1 12.5f
#define L2 26.0f
#define L3 26.0f
#define L4 12.5f
#define pi 3.141592f
#define R_min 15.2f // 足离电机轴心的最近距离。
#define R_max 38.5f // 足离电机轴心的最远距离。
#define Res_x -16.4f // 挡板坐标。手量的，后期需要验证
#define Res_y 4.0f
#define Y_max 9.52f 
// y的最大值，由最小半径方程和经过挡板坐标的切线方程联立求得。意思是向下半边的扇形的顶点的y轴值

// debug 临时用,影响性能
static volatile float err_count = 0;
static volatile float B_x_debug = 0.0f;
static volatile float B_y_debug = 0.0f;


typedef struct {
    float x;
    float y;
} Vec2;

typedef struct {
    float J11;   // vx 对 alpha_dot 的系数
    float J12;   // vx 对 beta_dot 的系数
    float J21;   // vy 对 alpha_dot 的系数
    float J22;   // vy 对 beta_dot 的系数
} Jacobian2D;

typedef struct {
    float Kpx;
    float Kpy;
    float Kdx;
    float Kdy;
    float Fx_max;
    float Fy_max;
    float Tau_max;
} VMCParam;


static float limit_float(float x, float min_value, float max_value)
{
    if (x > max_value) return max_value;
    if (x < min_value) return min_value;
    return x;
}

Jacobian2D jac;


// -------------------------------------------------------------

void normalize_angle_deg(float* angle)
{ 
	*angle = fmodf(*angle + 180.0f, 360.0f);
	if (*angle < 0.0f) *angle += 360.0f;
	*angle -= 180.0f;
}

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
	float alpha1 = 2.0f * atan2f(b + sqrtf(delta_alpha), a + c ) * 180.0f / pi;
	float alpha2 = 2.0f * atan2f(b - sqrtf(delta_alpha), a + c )  * 180.0f / pi;
	float beta1 = 2.0f * atan2f(e + sqrtf(delta_beta),d + f) * 180.0f / pi;
	float beta2 = 2.0f * atan2f(e - sqrtf(delta_beta),d + f) * 180.0f / pi;
	
	alpha1 = fmodf((alpha1 - 90.0f), 360.0f);
	alpha2 = fmodf((alpha2 - 90.0f), 360.0f);
	beta1 = fmodf((90.0f - beta1), 360.0f);
	beta2 = fmodf((90.0f - beta2), 360.0f);

	if (alpha1 == -beta1) {
		float temp = beta1;
		beta1 = beta2;
		beta2 = temp;
	}

	normalize_angle_deg(&alpha1);
	normalize_angle_deg(&alpha2);
	normalize_angle_deg(&beta1);
	normalize_angle_deg(&beta2);

	// 防止两条腿交叉
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

void crawl_inverseKinematic(Position_HandleTypeDef *hposition, LegSide_t leg_side){
	float x = hposition->B_x;
	float y = hposition->B_y;
	
	/*---------------------对x，y限位-----------------------*/
	
	// (0,0)不可达
	if (x == 0 && y == 0){ return;}
	
	// 限高 26/4/18更新
//	if (y < Y_max && x>-11.93f && x<11.93f){y = Y_max;}
//	
	// 运动半径之外的区域不取
	if ((x*x + y*y) < R_min*R_min){
		float k = R_min/sqrtf(x*x + y*y);
		x = k*x;
		y = k*y;
		return;
		}
	if ((x*x + y*y) > R_max*R_max){
		float k = R_max/sqrtf(x*x + y*y);
		x = k*x;
		y = k*y;
		return;
		}

	// 不撞挡板
//	if (leg_side == FRONT){
//		if (x < (L1*L1+Res_y*y)/Res_x){	x = (L1*L1+Res_y*y)/Res_x;}
//	}else{
//		if (x > -(L1*L1+Res_y*y)/Res_x){ x = -(L1*L1+Res_y*y)/Res_x;}
//	}

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
	float alpha1 = 2.0f * atan2f(b + sqrtf(delta_alpha), a + c ) * 180.0f / pi;
	float alpha2 = 2.0f * atan2f(b - sqrtf(delta_alpha), a + c )  * 180.0f / pi;
	float beta1 = 2.0f * atan2f(e + sqrtf(delta_beta),d + f) * 180.0f / pi;
	float beta2 = 2.0f * atan2f(e - sqrtf(delta_beta),d + f) * 180.0f / pi;
	
	alpha1 = fmodf((alpha1 - 90.0f), 360.0f);
	alpha2 = fmodf((alpha2 - 90.0f), 360.0f);
	beta1 = fmodf((90.0f - beta1), 360.0f);
	beta2 = fmodf((90.0f - beta2), 360.0f);

	if (alpha1 == -beta1) {
		float temp = beta1;
		beta1 = beta2;
		beta2 = temp;
	}

	normalize_angle_deg(&alpha1);
	normalize_angle_deg(&alpha2);
	normalize_angle_deg(&beta1);
	normalize_angle_deg(&beta2);

	// 防止两条腿交叉
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

void inverseKinematic_All(){// 前面两条腿装反了
//	hposition1.B_x = -hposition1.B_x;
//	hposition4.B_x = -hposition4.B_x; 
	
	hposition2.B_x = -hposition2.B_x;
	hposition3.B_x = -hposition3.B_x; 
	
	inverseKinematic(&hposition1);
	inverseKinematic(&hposition2);
	inverseKinematic(&hposition3);
	inverseKinematic(&hposition4);	
}

void crawl_inverseKinematic_All(){
	hposition1.B_x = -hposition1.B_x;
	hposition4.B_x = -hposition4.B_x;
	crawl_inverseKinematic(&hposition1,FRONT);
	crawl_inverseKinematic(&hposition2,BACK);
	crawl_inverseKinematic(&hposition3,BACK);
	crawl_inverseKinematic(&hposition4,FRONT);
}


/*-------------- 正解算 ---------------*/
// 输入: alpha_fb, beta_fb, 输出: B_x_real, B_y_real
void FK(Position_HandleTypeDef *hposition){
	
	float alpha_rad = hposition->alpha_fb * pi / 180.0f+pi/2.0f;
	float beta_rad = pi/2.0f - hposition->beta_fb * pi / 180.0f;

	// 计算a c点的实际坐标
	float xa = L1 * cosf(alpha_rad);
	float ya = L1 * sinf(alpha_rad);
	float xc = L4 * cosf(beta_rad);
	float yc = L4 * sinf(beta_rad);

	// 计算求解theta的中间变量
	float Lac = sqrtf((xa-xc)*(xa-xc) + (ya-yc)*(ya-yc));
	float a = 2*L2*(xa-xc);
	float b = 2*L2*(ya-yc);
	float c = Lac*Lac + L2*L2 - L3*L3;

	// 计算theta1, 符号不太确定，不知道有两个还是四个解
	float theta1_1 = 2.0f*atan2f(b + sqrtf(a*a + b*b - c*c), a + c);
	float theta1_2 = 2.0f*atan2f(b - sqrtf(a*a + b*b - c*c), a + c);

	// 计算B点的实际坐标
	float B_x1 = L1*cosf(alpha_rad) + L2*cosf(theta1_1);
	float B_y1 = L1*sinf(alpha_rad) + L2*sinf(theta1_1);
	float B_x2 = L1*cosf(alpha_rad) + L2*cosf(theta1_2);
	float B_y2 = L1*sinf(alpha_rad) + L2*sinf(theta1_2);

	// 排除物理不可达点
	if (B_y1>0.0f&&B_y2<0.0f){
		hposition->B_x_real = B_x1;
		hposition->B_y_real = B_y1;
		B_x_debug = B_x1;
		B_y_debug = B_y1;
		return;
	}else if (B_y1<0.0f&&B_y2>0.0f){
		hposition->B_x_real = B_x2;
		hposition->B_y_real = B_y2;
		B_x_debug = B_x2;
		B_y_debug = B_y2;
		return;
	}else {
		err_count++; // 错误情况，两个解的y都大于0或者都小于0
		B_x_debug = -1.0f;
		B_y_debug = -1.0f;
		return;
	}
}

void FK_All(){
	FK(&hposition1);
	FK(&hposition2);
	FK(&hposition3);
	FK(&hposition4);
}

// ----------- 雅可比矩阵 -------------

void calc_Jacobian( Position_HandleTypeDef *hposition)
{
	float alpha_rad = hposition->alpha_fb * pi / 180.0f+pi/2.0f;
	float beta_rad = pi/2.0f - hposition->beta_fb * pi / 180.0f;

// 先计算theta1
	// 计算a c点的实际坐标
	float xa = L1 * cosf(alpha_rad);
	float ya = L1 * sinf(alpha_rad);
	float xc = L4 * cosf(beta_rad);
	float yc = L4 * sinf(beta_rad);

	// 计算求解theta的中间变量
	float Lac = sqrtf((xa-xc)*(xa-xc) + (ya-yc)*(ya-yc));
	float a1 = 2*L2*(xa-xc);
	float b1 = 2*L2*(ya-yc);
	float c1 = Lac*Lac + L2*L2 - L3*L3;

	// 计算theta1, 符号不太确定，不知道有两个还是四个解
	float theta1_1 = 2.0f*atan2f(b1 + sqrtf(a1*a1 + b1*b1 - c1*c1), a1 + c1);
	float theta1_2 = 2.0f*atan2f(b1 - sqrtf(a1*a1 + b1*b1 - c1*c1), a1 + c1);

	// 计算B点的实际坐标
	float B_y1_1 = L1*sinf(alpha_rad) + L2*sinf(theta1_1);
	float B_y1_2 = L1*sinf(alpha_rad) + L2*sinf(theta1_2);

	// 取正确的theta1
	float jac_theta1 = 0.0f;
	if (B_y1_1>0.0f&&B_y1_2<0.0f){
		jac_theta1 = theta1_1;
		return;
	}else if (B_y1_1<0.0f&&B_y1_2>0.0f){
		jac_theta1 = theta1_2;		
		return;
	}

// 再计算theta2

	// 计算求解theta2的中间变量
	float a2 = 2*L3*(xc-xa);
	float b2 = 2*L3*(yc-ya);
	float c2 = L2*L2 - Lac*Lac - L3*L3;

	// 计算theta2, 符号不太确定，不知道有两个还是四个解
	float theta2_1 = 2.0f*atan2f(b2 + sqrtf(a2*a2 + b2*b2 - c2*c2), a2 + c2);
	float theta2_2 = 2.0f*atan2f(b2 - sqrtf(a2*a2 + b2*b2 - c2*c2), a2 + c2);

}










