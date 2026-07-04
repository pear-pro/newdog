
/**
 * @file    IMU.c
 * @brief   ά������ HWT901B ��̬������ CAN Э�����
 * @note    ������ coreimu ����ʵ��ʹ�õ� 0x50 ��ʶ�� + 0x55 ֡ͷ͸��Э���װ
 */
#include "IMU.h"
#include "gpio.h"
#include "main.h"
#include <math.h>

/* * ǿ������Ϊ volatile�����������������Ż����塣
 * ȷ��Ӧ�ò��㷨ÿ�ζ�ȡ���Ķ����жϸո�д�� RAM ���ʻ����ݡ�
 */

volatile IMU_Info_t IMU_rx_data = {0};

// ----------speed of x,y,z---------
float VeloY=0.0f;

// ----ˮƽƽ��pid����---
float stab_roll = 0.0f; 
float kp_roll = 0.01f; // 0.03
float kd_roll = 0.001f;

// ----角度--------------
volatile float body_roll = 0.0f;
volatile float body_pitch = 0.0f;
volatile float body_yaw = 0.0f;
float prev_body_roll = 0.0f;

// ----���ٶ�--------------
float GyroX=0.0f;
float GyroY=0.0f;
float GyroZ=0.0f;

// ----�Ǽ��ٶ�--------------
float AccX=0.0f;
float AccY=0.0f;
float AccZ=0.0f;

/**
 * @brief IMU ���ݽ�����ɺ��Ӧ�ò㴦���ص�
 * @details
 *   - �ӽ��ջ����� (IMU_rx_data) ����Ӧ�ò�ȫ�ֱ���
 *   - ���ж��������и����ȼ�ִ��
 *   - ���ڴ������˲����������ںϵ�ʵʱ�㷨
 */
static void IMU_App_Update(void)
{
    /* 将最新接收的欧拉角复制到应用层全局变量，加 isfinite 检查防止 NaN/inf 传播 */
    if (isfinite(IMU_rx_data.Roll))  body_roll  = IMU_rx_data.Roll;
    if (isfinite(IMU_rx_data.Pitch)) body_pitch = IMU_rx_data.Pitch - 90.0f;
    if (isfinite(IMU_rx_data.Yaw))   body_yaw   = IMU_rx_data.Yaw;

    if (isfinite(IMU_rx_data.GyroX)) GyroX = IMU_rx_data.GyroX;
    if (isfinite(IMU_rx_data.GyroY)) GyroY = IMU_rx_data.GyroY;
    if (isfinite(IMU_rx_data.GyroZ)) GyroZ = IMU_rx_data.GyroZ;

    if (isfinite(IMU_rx_data.AccX)) AccX = IMU_rx_data.AccX;
    if (isfinite(IMU_rx_data.AccY)) AccY = IMU_rx_data.AccY;
    if (isfinite(IMU_rx_data.AccZ)) AccZ = IMU_rx_data.AccZ;

     /* ����ʵʱ���㣨�� PID ���������ڴ����ӣ�ȷ������Ч������Ӧ��Ƶ�ж� */

    /* ���ڴ�����ʵʱ����
     * ���磺�����ȶ���ȵ� PID ����������PID���㻹û��ɣ��ȷŸ�ռλ
     * kp_roll = body_roll * 0.6f + ...
     */
    // stab_roll = body_roll * kp_roll + ...
}


/**
 * @brief  HWT901B CAN Э�鳬���ٱ��Ľ�������
 * @param  can_id: CAN ��׼֡ ID (������һ��·��У�飬�˴���Ϊռλ������δ����չ)
 * @param  rx_data:   8 �ֽڵ� CAN ������ָ��
 * @note   �ú��������ڼ���Ƶ�� CAN RX0 �ж��������У��Ͻ��ڴ�������ʱ�� printf��
 */

void IMU_CAN_RXCALLback(CAN_HandleTypeDef *hcan)
{

    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
     int16_t raw_x, raw_y, raw_z;
    
    if (hcan->Instance == CAN2)
    {
       
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {                       
                /* ά�ر�׼Э�����У�飺֡ͷ����Ϊ 0x55 */           
              if ( rx_header.IDE == CAN_ID_STD && rx_data[0] == 0x55 && rx_header.StdId == 0x50)
                {
                    /* ˢ��ȫ��ʱ�����֡������Ϊ�ϲ�����/���߿��Ź��ṩ�ж����� */
                    IMU_rx_data.last_update_time = HAL_GetTick();
                    IMU_rx_data.FrameCount++;

                   /* ������ͬ�������� */
                 switch (rx_data[1])
                  {                /* ʹ�ñ������۵��˷���������ʱ������ѹե FPU ���� */
                        case 0x51: /* ���ٶ�֡ (Ax, Ay, Az) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
                            
                         
                            // ���� 16g��ԭʼֵΪ 16 λ�з�����������Χ -32768~32767
                            IMU_rx_data.AccX = (float)raw_x * IMU_ACC_RATIO;
                            IMU_rx_data.AccY = (float)raw_y * IMU_ACC_RATIO;
                            IMU_rx_data.AccZ = (float)raw_z * IMU_ACC_RATIO;
                            break;

                        case 0x52: /* ���ٶ�֡ (Wx, Wy, Wz) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
                            
                            // ���� 2000��/s
                            IMU_rx_data.GyroX = (float)raw_x * IMU_GYRO_RATIO;
                            IMU_rx_data.GyroY = (float)raw_y * IMU_GYRO_RATIO;
                            IMU_rx_data.GyroZ = (float)raw_z * IMU_GYRO_RATIO;
                            break;

                        case 0x53: /* 欧拉角帧 (Roll, Pitch, Yaw) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);

                            IMU_rx_data.Roll  = (float)raw_x * IMU_ANGLE_RATIO;
                            IMU_rx_data.Pitch = (float)raw_y * IMU_ANGLE_RATIO;
                            IMU_rx_data.Yaw   = (float)raw_z * IMU_ANGLE_RATIO;

                            /* 角度帧更新后立即刷新应用层变量 */
                            IMU_App_Update();
                            break;

                        default:
                            /* �������� (��ų���) Ŀǰϵͳ�����ע��ֱ�ӷ���
                            ������������������Ӵ���
                            */
                            break;
                    }

                    /* ==============================================================
                     * ̽��Ϳ�����ת�������� 20 ֡��תһ������
                     * ��;�����۹۲� GPIO_PIN_14��PF14������˸Ƶ�ʣ����� IMU CAN ͨ�Ž�����
                     * ԭ����ÿ�յ� 20 ֡��Ϣ��תһ�Σ�����ͨ��ʱԼ 50Hz ��תƵ��
                     * ============================================================== */
                    if (IMU_rx_data.FrameCount >= 20)
                    {
                        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
                        IMU_rx_data.FrameCount = 0;
                    }

               }
           } 

      }
}

