 #include "ht_10a_remote_control.h"
 //#include "includes.h"
 #include "main.h"
 #include "usart.h"

 float rc_x;
 float rc_y;

 extern UART_HandleTypeDef huart1;
 extern DMA_HandleTypeDef hdma_usart1_rx;

 uint8_t   sbus_buffer[SBUS_BUFLEN];//SBUS���ջ�����

 Remote_Control_struct rcData;
 static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl);

 static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];//DMA˫����

 SBUS_ctrl_t sbus_ctrl;

 void sbus_remote_control_init(void)//SBUSң������ʼ��
 {
     RC_init(sbus_rx_buffer[0], sbus_rx_buffer[1], SBUS_RX_BUF_NUM);

    
     sbus_ctrl.last_swa_state = POS_MID;  // SWA��ʼ��״̬MID
     sbus_ctrl.last_swb_state = POS_DOWN;        // SWB��ʼ��״̬DOWN
     sbus_ctrl.last_swc_state = POS_DOWN;        // SWC��ʼ��״̬DOWN
     sbus_ctrl.last_swd_state = POS_MID;  // SWD��ʼ��״̬MID
     sbus_ctrl.key_flag = KEY_NONE;       // ��ʼ����־λNONE
 }

 const SBUS_ctrl_t *get_sbus_remote_control_point(void)//��ȡSBUSң����ָ��
 {
     return &sbus_ctrl;
 }

 //�����ж�
 void USART1_IRQHandlerCallBack(void)
 {
     if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
     {
         __HAL_UART_CLEAR_PEFLAG(&huart1);
     }
     else if(USART1->SR & UART_FLAG_IDLE)//�����ж�
     {
         static uint16_t this_time_rx_len = 0;

         __HAL_UART_CLEAR_PEFLAG(&huart1);

         if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
         {
             /* Current memory buffer used is Memory 0 */
    
             //disable DMA
             //ʧЧDMA
             __HAL_DMA_DISABLE(&hdma_usart1_rx);

             //get receive data length, length = set_data_length - remain_length
             //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
             this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

             //reset set_data_lenght
             //�����趨���ݳ���
             hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

             //set memory buffer 1
             //�趨������1
             hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
             //enable DMA
             //ʹ��DMA
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
             //ʧЧDMA
             __HAL_DMA_DISABLE(&hdma_usart1_rx);

             //get receive data length, length = set_data_length - remain_length
             //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
             this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

             //reset set_data_lenght
             //�����趨���ݳ���
             hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

             //set memory buffer 0
             //�趨������0
             DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
             //enable DMA
             //ʹ��DMA
             __HAL_DMA_ENABLE(&hdma_usart1_rx);

             if(this_time_rx_len == RC_FRAME_LENGTH)
             {
                 //����ң��������
                 sbus_to_remote_control(sbus_rx_buffer[1], &sbus_ctrl);
             }
         }
     }
 }

 static void sbus_to_remote_control(volatile const uint8_t *sbus_buffer, SBUS_ctrl_t *sbus_ctrl)
 {
     if((sbus_buffer [0] == 0x0f) && (sbus_buffer[24] == 0x00))//�ж�ͷ֡��β֡
     {
         sbus_ctrl -> ch[0] = ((sbus_buffer[1] )| (sbus_buffer[2] << 8 )) & 0x07ff;//������
         sbus_ctrl -> ch[1] = ((sbus_buffer[2] >> 3 )| (sbus_buffer[3] << 5 )) & 0x07ff;//������
         sbus_ctrl -> ch[2] = ((sbus_buffer[3] >> 6 )| (sbus_buffer[4] << 2 ) | (sbus_buffer[5] << 10)) & 0x07ff;//������
         sbus_ctrl -> ch[3] = ((sbus_buffer[5] >> 1 )| (sbus_buffer[6] << 7 )) & 0x07ff;//������
         sbus_ctrl -> ch[4] = ((sbus_buffer[6] >> 4 )| (sbus_buffer[7] << 4 )) & 0x07ff;//SWA
         sbus_ctrl -> ch[5] = ((sbus_buffer[7] >> 7 )| (sbus_buffer[8] << 1 )| (sbus_buffer[9] << 9 )) & 0x07ff;//SWB
         sbus_ctrl -> ch[6] = ((sbus_buffer[9] >> 2 )| (sbus_buffer[10] << 6 )) & 0x07ff;//SWC
         sbus_ctrl-> ch[7] = ((sbus_buffer[10] >> 5 )| (sbus_buffer[11] << 3 )) & 0x07ff;//SWD

        //����ƫ��
         for(int i = 0;i<8;i++)
         {
             sbus_ctrl->ch[i] = (int16_t)(sbus_ctrl->ch[i] - SBUS_CH_VALUE_OFFSET);
         }
       
         // ���������λ״̬
         virtual_key_update(sbus_ctrl);
			
        rcData.R_x = sbus_ctrl -> ch[0]-9 / 800.0f;
        rcData.R_y = sbus_ctrl -> ch[1] / 800.0f;
        rcData.L_x = sbus_ctrl -> ch[3] / 800.0f;
        rcData.L_y = sbus_ctrl -> ch[2] / 800.0f;
        rcData.sw5 = sbus_ctrl -> ch[4];
        rcData.sw6 = sbus_ctrl -> ch[5];
        rcData.sw7 = sbus_ctrl -> ch[6];
        rcData.sw8 = sbus_ctrl -> ch[7];

     }

 }
 
 static uint8_t detect_switch_position(int16_t value)
 {
     if (value <= SWITCH_SBUS_CH_VALUE_MIN + DEADZONE) 
     {
         return POS_UP;  // ��
     } 
     else if (value >= SWITCH_SBUS_CH_VALUE_MAX - DEADZONE) 
     {
         return POS_DOWN;  // ��
     } 
     else 
     {
         return POS_MID;  // ��
     }
 }

 static void virtual_key_update(SBUS_ctrl_t *sbus_ctrl)
 {
     // ���¿���״̬
     uint8_t SWA_pos = detect_switch_position(sbus_ctrl -> ch[4]);
     uint8_t SWB_pos = detect_switch_position(sbus_ctrl -> ch[5]);
     uint8_t SWC_pos = detect_switch_position(sbus_ctrl -> ch[6]);
     uint8_t SWD_pos = detect_switch_position(sbus_ctrl -> ch[7]);

     //��ʼ�������λ״̬
     sbus_ctrl->key_flag = KEY_NONE;

     //SWA����
     if(sbus_ctrl -> last_swa_state == POS_MID && SWA_pos == POS_UP)
     {
         sbus_ctrl->key_flag |= KEY_SWA_UP;
     }
     else if(sbus_ctrl -> last_swa_state == POS_MID && SWA_pos == POS_DOWN)
     {
         sbus_ctrl->key_flag |= KEY_SWA_DOWN;
     }
     else if(SWA_pos == POS_MID)
     {
         sbus_ctrl->key_flag |= KEY_SWA_MID;
     }

     //SWB����
     if(sbus_ctrl -> last_swb_state == POS_DOWN && SWB_pos == POS_UP)
     {
         sbus_ctrl->key_flag |= KEY_SWB_UP;
     }
     else if(sbus_ctrl -> last_swb_state == POS_UP && SWB_pos == POS_DOWN)
     {
         sbus_ctrl->key_flag |= KEY_SWB_DOWN;
     }

     //SWC����
     if(sbus_ctrl -> last_swc_state == POS_DOWN && SWC_pos == POS_UP)
     {
         sbus_ctrl->key_flag |= KEY_SWC_UP;
     }
     else if(sbus_ctrl -> last_swc_state == POS_UP && SWC_pos == POS_DOWN)
     {
         sbus_ctrl->key_flag |= KEY_SWC_DOWN;
     }

     //SWD����
     if(sbus_ctrl -> last_swd_state == POS_MID && SWD_pos == POS_UP)
     {
         sbus_ctrl->key_flag |= KEY_SWD_UP;
     }
     else if(sbus_ctrl -> last_swd_state == POS_MID && SWD_pos == POS_DOWN)
     {
         sbus_ctrl->key_flag |= KEY_SWD_DOWN;
     }
     else if(SWD_pos == POS_MID)
     {
         sbus_ctrl->key_flag |= KEY_SWD_MID;
     }

     //����״̬
     sbus_ctrl->last_swa_state = SWA_pos;
     sbus_ctrl->last_swb_state = SWB_pos;
     sbus_ctrl->last_swc_state = SWC_pos;
     sbus_ctrl->last_swd_state = SWD_pos;
 }


