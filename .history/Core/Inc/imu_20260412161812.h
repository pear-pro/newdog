/**
 * @file    imu.h
 * @brief   ά������ HWT901B ��̬����������ͷ�ļ�
 */
#ifndef __IMU_H
#define __IMU_H

#include "stdint.h"

#include "main.h"
#include "can.h"
/* * ====================================================================
 * �����ڳ����۵��� (����ת��ϵ��)
 * ���棺���鲻Ҫ���жϣ�IMU_CAN_RXCALLback����ʹ�ø��������ͨ������Ԥ�ȼ���õĳ�����
 * �ɽ� FPU ָ��������ʮ����ʱ�������Ż��� 1 ��ʱ�����ڡ�
 * ====================================================================
 */
#define IMU_ACC_RATIO   (16.0f / 32768.0f)     // ���� 16g
#define IMU_GYRO_RATIO  (2000.0f / 32768.0f)   // ���� 2000��/s
#define IMU_ANGLE_RATIO (180.0f / 32768.0f)    // ���� 180��

extern float body_roll;
extern float body_pitch;
extern float body_yaw;
extern float prev_body_roll;

extern float stab_roll;
extern float kp_roll;
extern float kd_roll;

extern float GyroX;
extern float GyroY;
extern float GyroZ;

extern float AccX;
extern float AccY;
extern float AccZ;
/**
 * @brief ������̬�뻷�����ݽṹ��
 */
typedef struct
{
    /* ������ٶ� (��λ: g) */
    float AccX;
    float AccY;
    float AccZ;
    
    /* ������ٶ� (��λ: ��/s) */
    float GyroX;
    float GyroY;
    float GyroZ;
    
    /* ����ŷ���� (��λ: ��) */
    float Roll;
    float Pitch;
    float Yaw;
    
    /* �����Ϣ: ֡��������� */
    uint32_t FrameCount;
    /* �����Ϣ: ���һ�γɹ�ˢ�����ݵ�ʱ��� (ms) */
    uint32_t last_update_time;

} IMU_Info_t;

/* ���Ⱪ¶��ȫ�� IMU ʵ�� (������� volatile �ؼ���) */
extern volatile IMU_Info_t IMU_rx_data;

/* �жϲ㼶ר�õļ��ٽ������� */
void IMU_CAN_RXCALLback(CAN_HandleTypeDef *hcan);
#endif /* __IMU_H */
