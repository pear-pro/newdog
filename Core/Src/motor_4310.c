#include "motor_4310.h"
/**************达妙电机 ******** */
motor_info_t damiao[4];


float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

float float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void Set_dm_mit(CAN_HandleTypeDef* hcan,int16_t ID)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
	if (ID==0){
  can1TxMsg.StdId =0x00;
  }
  else if(ID==1)
  {
  can1TxMsg.StdId =0x01;

  }
   else if(ID==2)
  {
  can1TxMsg.StdId =0x02 ;

  }
  else if(ID==3)
  {
  can1TxMsg.StdId =0x03;

  }
    pos_tmp = float_to_uint(damiao[ID].angle, -12.5, 12.5, 16);
    vel_tmp = float_to_uint(damiao[ID].speed, -30, 30, 12);
    tor_tmp = float_to_uint(damiao[ID].tor, -10,10, 12);
    kp_tmp  = float_to_uint(damiao[ID].KP, 0.0, 500.0, 12);
    kd_tmp  = float_to_uint(damiao[ID].KD,  0.0, 5.0, 12);
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度

    can1TxData[0] = (pos_tmp >> 8);
    can1TxData[1] = pos_tmp;
    can1TxData[2] = (vel_tmp >> 4);
    can1TxData[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    can1TxData[4] = kp_tmp;
    can1TxData[5] = (kd_tmp >> 4);
    can1TxData[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    can1TxData[7] = tor_tmp;

	/* 先检查是否有空闲的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
 			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, (uint32_t*)CAN_TX_MAILBOX0);//发送报文
	}
}

void Set_dm_speed(CAN_HandleTypeDef* hcan,int16_t ID,float  speed)
{
	uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8] = {0};

    tx_msg.StdId = 0x200 + ID;  //电机ID 请根据你设定的 CAN ID 值 + 0x200
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 4;   // 速度模式只需要发送4字节浮点数

    uint8_t *p = (uint8_t *)&speed;

    tx_data[0] = p[0];
    tx_data[1] = p[1];
    tx_data[2] = p[2];
    tx_data[3] = p[3];

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
    {
        HAL_CAN_AddTxMessage(hcan, &tx_msg, tx_data, &tx_mailbox);
    }
}

void Set_dm_pos(CAN_HandleTypeDef* hcan,uint16_t ID,float pos,float vel)
{
	uint32_t tx_mailbox;
	CAN_TxHeaderTypeDef tx_msg;
	uint8_t tx_data[8] = {0};

	tx_msg.StdId = 0x100 + ID;  //电机ID 请根据你设定的 CAN ID 值 + 0x100
	tx_msg.IDE = CAN_ID_STD;
	tx_msg.RTR = CAN_RTR_DATA;
	tx_msg.DLC = 8;   // 位置模式需要发送8字节数据

	uint8_t *p_pos = (uint8_t *)&pos;
	uint8_t *p_vel = (uint8_t *)&vel;

	tx_data[0] = p_pos[0];
	tx_data[1] = p_pos[1];
	tx_data[2] = p_pos[2];
	tx_data[3] = p_pos[3];
	tx_data[4] = p_vel[0];
	tx_data[5] = p_vel[1];
	tx_data[6] = p_vel[2];
	tx_data[7] = p_vel[3];

	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
		HAL_CAN_AddTxMessage(hcan, &tx_msg, tx_data, &tx_mailbox);
	}
}

void Set_dm_enable(CAN_HandleTypeDef* hcan,uint8_t ID)
{
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};

  can1TxMsg.StdId = 0x00+ID;  //模式偏移ID：MIT模式偏移0x00，位置速度模式偏移0x100，速度模式偏移0x200，纯位置模式偏移0x300
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度

    can1TxData[0] = 0xFF;
    can1TxData[1] = 0xFF;
    can1TxData[2] = 0xFF;
    can1TxData[3] = 0xFF;
    can1TxData[4] = 0xFF;
    can1TxData[5] = 0xFF;
    can1TxData[6] = 0xFF;
    can1TxData[7] = 0xFC;


	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, (uint32_t*)CAN_TX_MAILBOX0);
	}
}

void Set_dm_disable(CAN_HandleTypeDef* hcan,uint8_t ID)
{
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  can1TxMsg.StdId = 0x00+ID;
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度

	can1TxData[0] = 0xFF;
	can1TxData[1] = 0xFF;
	can1TxData[2] = 0xFF;
	can1TxData[3] = 0xFF;
	can1TxData[4] = 0xFF;
	can1TxData[5] = 0xFF;
	can1TxData[6] = 0xFF;
	can1TxData[7] = 0xFD;


	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, (uint32_t*)CAN_TX_MAILBOX0);
	}
}

void Set_dm_zeropoint(CAN_HandleTypeDef* hcan,uint16_t CAN_ID)
{
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  can1TxMsg.StdId = CAN_ID;
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度
  can1TxData[0]=0xff;
  can1TxData[1]=0xff;
  can1TxData[2]=0xff;
  can1TxData[3]=0xff;
  can1TxData[4]=0xff;
  can1TxData[5]=0xff;
  can1TxData[6]=0xff;
  can1TxData[7]=0xfe;

	/* 先检查是否有空闲的 TX mailbox，只有有空位才发送报文 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
			HAL_CAN_AddTxMessage(hcan, &can1TxMsg, can1TxData, (uint32_t*)CAN_TX_MAILBOX0);//发送报文
	}
}

void dm_motor_fbdata(motor_info_t *motor, uint8_t *rx_data) //master_id默认为0(不影响解算)
{
//    // 电机编号ID和状态（一般不用，看协议就有）
//    motor->para.id = (rx_data[0]) & 0x0F;
//    motor->para.state = (rx_data[0]) >> 4;

    // 读取位置原始值（高字节+低字节）
    uint16_t p_int = (rx_data[1] << 8) | rx_data[2];
    // 读取速度原始值（高字节拼接）
    uint16_t v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    // 读取转矩原始值（高字节拼接）
    uint16_t t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    // 将原始值转换为实际物理值
    motor->Rxmsg.Angle = uint_to_float(p_int, -12.5,12.5,16);
    motor->Rxmsg.Speed = uint_to_float(v_int, -30,30,12);
    motor->Rxmsg.Torque = uint_to_float(t_int, -10,10,12);
	//motor->Angle_pid.get = motor->Rxmsg.Angle;
}
