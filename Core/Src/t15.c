#include "t15.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;


uint8_t   sbus_buffer[SBUS_BUFLEN];//SBUS接收缓冲区

static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl);

static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];//DMA双缓冲

SBUS_ctrl_t sbus_ctrl;


void sbus_remote_control_init(void)//SBUS遥控器初始化
{
   RC_init(sbus_rx_buffer[0], sbus_rx_buffer[1], SBUS_RX_BUF_NUM);

   
   sbus_ctrl.last_swa_state = POS_MID;  // SWA初始化状态MID
   sbus_ctrl.last_swb_state = POS_MID;        // SWB初始化状态MID
   sbus_ctrl.last_swc_state = POS_MID;        // SWC初始化状态MID
   sbus_ctrl.last_swd_state = POS_MID;  // SWD初始化状态MID
   sbus_ctrl.last_se_state = POS_UP;  // SE初始化状态UP
   sbus_ctrl.last_sf_state = POS_UP;  // SF初始化状态UP
   sbus_ctrl.key_flag = KEY_NONE;       // 初始化标志位NONE
}

const SBUS_ctrl_t *get_sbus_remote_control_point(void)//获取SBUS遥控器指针
{
   return &sbus_ctrl;
}

//串口中断
void USART1_IRQHandlerCallBack(void)
{
   if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
   {
       __HAL_UART_CLEAR_PEFLAG(&huart1);
   }
   else if(USART1->SR & UART_FLAG_IDLE)//空闲中断
   {
       static uint16_t this_time_rx_len = 0;

       __HAL_UART_CLEAR_PEFLAG(&huart1);

       if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
       {
           /* Current memory buffer used is Memory 0 */
   
           //disable DMA
           //失效DMA
           __HAL_DMA_DISABLE(&hdma_usart1_rx);

           //get receive data length, length = set_data_length - remain_length
           //获取接收数据长度,长度 = 设定长度 - 剩余长度
           this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

           //reset set_data_lenght
           //重新设定数据长度
           hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

           //set memory buffer 1
           //设定缓冲区1
           hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
           
           //enable DMA
           //使能DMA
           __HAL_DMA_ENABLE(&hdma_usart1_rx);

           if(this_time_rx_len == RC_FRAME_LENGTH)
           {
               sbus_to_remote_control(sbus_rx_buffer[0], &sbus_ctrl);
           }
       }
       else
       {
           /* Current memory buffer used is Memory 1 */
           //disable DMA
           //失效DMA
           __HAL_DMA_DISABLE(&hdma_usart1_rx);

           //get receive data length, length = set_data_length - remain_length
           //获取接收数据长度,长度 = 设定长度 - 剩余长度
           this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

           //reset set_data_lenght
           //重新设定数据长度
           hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

           //set memory buffer 0
           //设定缓冲区0
           DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
           
           //enable DMA
           //使能DMA
           __HAL_DMA_ENABLE(&hdma_usart1_rx);

           if(this_time_rx_len == RC_FRAME_LENGTH)
           {
               //处理遥控器数据
               sbus_to_remote_control(sbus_rx_buffer[1], &sbus_ctrl);
           }
       }
   }
}

static uint8_t detect_switch_position(int16_t value)
{
   if (value <= SWITCH_SBUS_CH_VALUE_MIN + DEADZONE)
   {
       return POS_UP;  // 上
   } 
   else if (value >= SWITCH_SBUS_CH_VALUE_MAX - DEADZONE) 
   {
       return POS_DOWN;  // 下
   } 
   else 
   {
       return POS_MID;  // 中
   }
}

static void virtual_key_update(SBUS_ctrl_t *sbus_ctrl)
{
   // 更新开关状态
   uint8_t SWA_pos = detect_switch_position(sbus_ctrl -> ch[4]);
   uint8_t SWB_pos = detect_switch_position(sbus_ctrl -> ch[5]);
   uint8_t SWC_pos = detect_switch_position(sbus_ctrl -> ch[6]);
   uint8_t SWD_pos = detect_switch_position(sbus_ctrl -> ch[7]);
   uint8_t SE_pos = detect_switch_position(sbus_ctrl -> ch[8]);
   uint8_t SF_pos = detect_switch_position(sbus_ctrl -> ch[9]);

   //初始化虚拟键位状态
   sbus_ctrl->key_flag = KEY_NONE;

   //SWA处理
   if(sbus_ctrl -> last_swa_state != SWA_pos)
   {
       if(SWA_pos == POS_UP)
       {
           sbus_ctrl -> key_flag |= KEY_SWA_UP;
       }
       else if(SWA_pos == POS_DOWN)
       {
           sbus_ctrl -> key_flag |= KEY_SWA_DOWN;
       }
       else if(SWA_pos == POS_MID)
       {
           sbus_ctrl->key_flag |= KEY_SWA_MID; 
       }
   }
   else
   {
   // 状态不变时设置对应位置的标志
       if(SWA_pos == POS_UP)
       {
           sbus_ctrl->key_flag |= KEY_SWA_UP;
       }
       else if(SWA_pos == POS_MID)
       {
           sbus_ctrl->key_flag |= KEY_SWA_MID;
       }
       else if(SWA_pos == POS_DOWN)
       {
           sbus_ctrl->key_flag |= KEY_SWA_DOWN;
       }
   }

   //SWB处理
   if(sbus_ctrl -> last_swb_state != SWB_pos)
   {
       if(SWB_pos == POS_UP)
       {
           sbus_ctrl -> key_flag |= KEY_SWB_UP;
       }
       else if(SWB_pos == POS_DOWN)
       {
           sbus_ctrl -> key_flag |= KEY_SWB_DOWN;
       }
   }
   else
   {
   // 状态不变时设置对应位置的标志
       if(SWB_pos == POS_UP)
       {
           sbus_ctrl->key_flag |= KEY_SWB_UP;
       }
       else if(SWB_pos == POS_MID)
       {
           sbus_ctrl->key_flag |= KEY_SWB_MID;
       }
       else if(SWB_pos == POS_DOWN)
       {
           sbus_ctrl->key_flag |= KEY_SWB_DOWN;
       }
   }

   //SWC处理
   if(sbus_ctrl -> last_swc_state != SWC_pos)
   {
       if(SWC_pos == POS_UP)
       {
           sbus_ctrl -> key_flag |= KEY_SWC_UP;
       }
       else if(SWC_pos == POS_DOWN)
       {
           sbus_ctrl -> key_flag |= KEY_SWC_DOWN;
       }
   }
   else
   {
   // 状态不变时设置对应位置的标志
       if(SWC_pos == POS_UP)
       {
           sbus_ctrl->key_flag |= KEY_SWC_UP;
       }
       else if(SWC_pos == POS_MID)
       {
           sbus_ctrl->key_flag |= KEY_SWC_MID;
       }
       else if(SWC_pos == POS_DOWN)
       {
           sbus_ctrl->key_flag |= KEY_SWC_DOWN;
       }
   }

   //SWD处理
   if(sbus_ctrl -> last_swd_state != SWD_pos)
   {
       if(SWD_pos == POS_UP)
       {
           sbus_ctrl -> key_flag |= KEY_SWD_UP;
       }
       else if(SWD_pos == POS_DOWN)
       {
           sbus_ctrl -> key_flag |= KEY_SWD_DOWN;
       }
   }
   else
   {
   // 状态不变时设置对应位置的标志
       if(SWD_pos == POS_UP)
       {
           sbus_ctrl->key_flag |= KEY_SWD_UP;
       }
       else if(SWD_pos == POS_MID)
       {
           sbus_ctrl->key_flag |= KEY_SWD_MID;
       }
       else if(SWD_pos == POS_DOWN)
       {
           sbus_ctrl->key_flag |= KEY_SWD_DOWN;
       }
   }

   //SE处理
   if(sbus_ctrl -> last_se_state == POS_UP && SE_pos == POS_DOWN)
   {
       sbus_ctrl->key_flag |= KEY_SE_DOWN;
   }
   else if(sbus_ctrl -> last_se_state == POS_DOWN && SE_pos == POS_UP)
   {
       sbus_ctrl->key_flag |= KEY_SE_UP;
   }
   else
   {
       // 保持状态时也设置对应状态
       if(SE_pos == POS_UP)
       {
           sbus_ctrl->key_flag |= KEY_SE_UP;
       }
       else if(SE_pos == POS_DOWN)
       {
           sbus_ctrl->key_flag |= KEY_SE_DOWN;
       }
   }

   //SF处理
   if(sbus_ctrl -> last_sf_state == POS_UP && SF_pos == POS_DOWN)
   {
       sbus_ctrl->key_flag |= KEY_SF_PRESSED;
   }

   //更新状态
   sbus_ctrl->last_swa_state = SWA_pos;
   sbus_ctrl->last_swb_state = SWB_pos;
   sbus_ctrl->last_swc_state = SWC_pos;
   sbus_ctrl->last_swd_state = SWD_pos;
   sbus_ctrl->last_se_state = SE_pos;
   sbus_ctrl->last_sf_state = SF_pos;

}


static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl)
{
   if((sbus_buffer [0] == 0x0f) && (sbus_buffer[24] == 0x00))//判断头帧和尾帧
   {
       sbus_ctrl -> ch[0] = ((sbus_buffer[1] )| (sbus_buffer[2] << 8 )) & 0x07ff;//右左右
       sbus_ctrl -> ch[1] = ((sbus_buffer[2] >> 3 )| (sbus_buffer[3] << 5 )) & 0x07ff;//右上下
       sbus_ctrl -> ch[2] = ((sbus_buffer[3] >> 6 )| (sbus_buffer[4] << 2 ) | (sbus_buffer[5] << 10)) & 0x07ff;//左上下
       sbus_ctrl -> ch[3] = ((sbus_buffer[5] >> 1 )| (sbus_buffer[6] << 7 )) & 0x07ff;//左左右
       sbus_ctrl -> ch[4] = ((sbus_buffer[6] >> 4 )| (sbus_buffer[7] << 4 )) & 0x07ff;//SWA
       sbus_ctrl -> ch[5] = ((sbus_buffer[7] >> 7 )| (sbus_buffer[8] << 1 )| (sbus_buffer[9] << 9 )) & 0x07ff;//SWB
       sbus_ctrl -> ch[6] = ((sbus_buffer[9] >> 2 )| (sbus_buffer[10] << 6 )) & 0x07ff;//SWC
       sbus_ctrl-> ch[7] = ((sbus_buffer[10] >> 5 )| (sbus_buffer[11] << 3 )) & 0x07ff;//SWD
       sbus_ctrl ->ch[8] = ((sbus_buffer[12])| (sbus_buffer[13] << 8 )) & 0x07ff;//SE
       sbus_ctrl ->ch[9] = ((sbus_buffer[13] >> 3 )| (sbus_buffer[14] << 5 )) & 0x07ff;//SF

      //数据偏移
       for(int i = 0;i<10;i++)
       {
           sbus_ctrl->ch[i] = (int16_t)(sbus_ctrl->ch[i] - SBUS_CH_VALUE_OFFSET);
       }
      
       // 更新虚拟键位状态
       virtual_key_update(sbus_ctrl);

       if (KEY_SWB_DOWN & sbus_ctrl -> key_flag)
       {
          
       }
       else if (KEY_SWB_MID & sbus_ctrl -> key_flag)
       {
           
       }
       else
       {
           
       }
     
   }
}




