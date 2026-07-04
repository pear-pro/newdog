
/**
 * @file    IMU.c
 * @brief   Î¬ÌØÖÇÄÜ HWT901B ×ËÌ¬´«¸ĞÆ÷ CAN Ğ­Òé½âÎö
 * @note    ÒÑÊÊÅä coreimu ¹¤³ÌÊµ¼ÊÊ¹ÓÃµÄ 0x50 ±êÊ¶·û + 0x55 Ö¡Í·Í¸´«Ğ­Òé·â×°
 */
#include "IMU.h"
<<<<<<< HEAD
#include "gpio.h" // ÒıÈëÌ½ÕëÒı½Å¶¨Òå
#include "main.h" // ÒıÈë»ñÈ¡ÏµÍ³Ê±¼äµÄ HAL_GetTick() 
=======
#include "gpio.h" // GPIOæ¢é’ˆå¼•è„šå®šä¹‰
#include "main.h" // ç”¨äºè·å–ç³»ç»Ÿæ—¶é’Ÿ HAL_GetTick()
#include <math.h>
>>>>>>> 89005bde61cd4fcf161cb4570fff459858e577cf

/* * Ç¿ÖÆÉùÃ÷Îª volatile£¬·ÀÓù±àÒëÆ÷¼¤½øÓÅ»¯ÏİÚå¡£
 * È·±£Ó¦ÓÃ²ãËã·¨Ã¿´Î¶ÁÈ¡µ½µÄ¶¼ÊÇÖĞ¶Ï¸Õ¸ÕĞ´Èë RAM µÄÏÊ»îÊı¾İ¡£
 */

volatile IMU_Info_t IMU_rx_data = {0};

// ----------speed of x,y,z---------
float VeloY=0.0f;

// ----Ë®Æ½Æ½ºâpid²ÎÊı---
float stab_roll = 0.0f; 
float kp_roll = 0.01f; // 0.03
float kd_roll = 0.001f;

<<<<<<< HEAD
// ----½Ç¶È--------------
float body_roll = 0.0f; 
float body_pitch = 0.0f; 
float body_yaw = 0.0f;
=======
// ----ï¿½Ç¶ï¿½--------------
volatile float body_roll = 0.0f;
volatile float body_pitch = 0.0f;
volatile float body_yaw = 0.0f;
>>>>>>> 89005bde61cd4fcf161cb4570fff459858e577cf
float prev_body_roll = 0.0f;

// ----½ÇËÙ¶È--------------
float GyroX=0.0f;
float GyroY=0.0f;
float GyroZ=0.0f;

// ----½Ç¼ÓËÙ¶È--------------
float AccX=0.0f;
float AccY=0.0f;
float AccZ=0.0f;

/**
 * @brief IMU Êı¾İ½ÓÊÕÍê³ÉºóµÄÓ¦ÓÃ²ã´¦Àí»Øµ÷
 * @details
 *   - ´Ó½ÓÊÕ»º³åÇø (IMU_rx_data) ¸üĞÂÓ¦ÓÃ²ãÈ«¾Ö±äÁ¿
 *   - ÔÚÖĞ¶ÏÉÏÏÂÎÄÖĞ¸ßÓÅÏÈ¼¶Ö´ĞĞ
 *   - ¿ÉÔÚ´ËÌí¼ÓÂË²¨¡¢²¹³¥¡¢ÈÚºÏµÈÊµÊ±Ëã·¨
 */
static void IMU_App_Update(void)
{
<<<<<<< HEAD
    /* ½«×îĞÂ½ÓÊÕµÄÅ·À­½Ç¸´ÖÆµ½Ó¦ÓÃ²ãÈ«¾Ö±äÁ¿ */
    body_roll = IMU_rx_data.Roll;
    body_pitch = IMU_rx_data.Pitch-90.0f;
    body_yaw = IMU_rx_data.Yaw;
    
    GyroX = IMU_rx_data.GyroX;
    GyroY = IMU_rx_data.GyroY;
    GyroZ = IMU_rx_data.GyroZ;
=======
    /* å°†æœ€æ–°æ¥æ”¶çš„æ¬§æ‹‰è§’å¤åˆ¶åˆ°åº”ç”¨å±‚å…¨å±€å˜é‡ï¼ŒåŠ  isfinite æ£€æŸ¥é˜²æ­¢ NaN/inf ä¼ æ’­ */
    if (isfinite(IMU_rx_data.Roll))  body_roll  = IMU_rx_data.Roll;
    if (isfinite(IMU_rx_data.Pitch)) body_pitch = IMU_rx_data.Pitch - 90.0f;
    if (isfinite(IMU_rx_data.Yaw))   body_yaw   = IMU_rx_data.Yaw;
>>>>>>> 89005bde61cd4fcf161cb4570fff459858e577cf

    if (isfinite(IMU_rx_data.GyroX)) GyroX = IMU_rx_data.GyroX;
    if (isfinite(IMU_rx_data.GyroY)) GyroY = IMU_rx_data.GyroY;
    if (isfinite(IMU_rx_data.GyroZ)) GyroZ = IMU_rx_data.GyroZ;

    if (isfinite(IMU_rx_data.AccX)) AccX = IMU_rx_data.AccX;
    if (isfinite(IMU_rx_data.AccY)) AccY = IMU_rx_data.AccY;
    if (isfinite(IMU_rx_data.AccZ)) AccZ = IMU_rx_data.AccZ;

     /* ÆäËûÊµÊ±¼ÆËã£¨Èç PID µ÷Õû£©¿ÉÔÚ´ËÌí¼Ó£¬È·±£¼ÆËãĞ§ÂÊÒÔÊÊÓ¦¸ßÆµÖĞ¶Ï */

    /* ¿ÉÔÚ´ËÌí¼ÓÊµÊ±¼ÆËã
     * ÀıÈç£º»ùÓÚÎÈ¶¨Óà¶ÈµÄ PID µ÷Õû£¬ºóĞøPID¼ÆËã»¹Ã»Íê³É£¬ÏÈ·Å¸öÕ¼Î»
     * kp_roll = body_roll * 0.6f + ...
     */
    // stab_roll = body_roll * kp_roll + ...
}


/**
 * @brief  HWT901B CAN Ğ­Òé³¬¸ßËÙ±¨ÎÄ½âÎöº¯Êı
 * @param  can_id: CAN ±ê×¼Ö¡ ID (ÒÑÔÚÉÏÒ»²ãÂ·ÓÉĞ£Ñé£¬´Ë´¦×÷ÎªÕ¼Î»£¬±£ÁôÎ´À´À©Õ¹)
 * @param  rx_data:   8 ×Ö½ÚµÄ CAN Êı¾İÓòÖ¸Õë
 * @note   ¸Ãº¯ÊıÔËĞĞÔÚ¼«¸ßÆµµÄ CAN RX0 ÖĞ¶ÏÉÏÏÂÎÄÖĞ£¬ÑÏ½ûÔÚ´ËÌí¼ÓÑÓÊ±»ò printf£¡
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
                /* Î¬ÌØ±ê×¼Ğ­Òé¶ş´ÎĞ£Ñé£ºÖ¡Í·±ØĞëÎª 0x55 */           
              if ( rx_header.IDE == CAN_ID_STD && rx_data[0] == 0x55 && rx_header.StdId == 0x50)
                {
                    /* Ë¢ĞÂÈ«¾ÖÊ±¼ä´ÁÓëÖ¡¼ÆÊı£¬ÎªÉÏ²ãËÀ»ú/ÀëÏß¿´ÃÅ¹·Ìá¹©ÅĞ¶¨ÒÀ¾İ */
                    IMU_rx_data.last_update_time = HAL_GetTick();
                    IMU_rx_data.FrameCount++;

                   /* ½âÎö²»Í¬±¨ÎÄÀàĞÍ */
                 switch (rx_data[1])
                  {                /* Ê¹ÓÃ±àÒëÆÚÕÛµş³Ë·¨´úÌæÔËĞĞÊ±³ı·¨£¬Ñ¹Õ¥ FPU ĞÔÄÜ */
                        case 0x51: /* ¼ÓËÙ¶ÈÖ¡ (Ax, Ay, Az) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
                            
                         
                            // Á¿³Ì 16g£¬Ô­Ê¼ÖµÎª 16 Î»ÓĞ·ûºÅÕûÊı£¬·¶Î§ -32768~32767
                            IMU_rx_data.AccX = (float)raw_x * IMU_ACC_RATIO;
                            IMU_rx_data.AccY = (float)raw_y * IMU_ACC_RATIO;
                            IMU_rx_data.AccZ = (float)raw_z * IMU_ACC_RATIO;
                            break;

                        case 0x52: /* ½ÇËÙ¶ÈÖ¡ (Wx, Wy, Wz) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
                            
                            // Á¿³Ì 2000¡ã/s
                            IMU_rx_data.GyroX = (float)raw_x * IMU_GYRO_RATIO;
                            IMU_rx_data.GyroY = (float)raw_y * IMU_GYRO_RATIO;
                            IMU_rx_data.GyroZ = (float)raw_z * IMU_GYRO_RATIO;
                            break;

                        case 0x53: /* Å·À­½ÇÖ¡ (Roll, Pitch, Yaw) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
<<<<<<< HEAD
                            
                            // Á¿³Ì 180¡ã
=======

                            // é‡ç¨‹ 180Â°
>>>>>>> 89005bde61cd4fcf161cb4570fff459858e577cf
                            IMU_rx_data.Roll  = (float)raw_x * IMU_ANGLE_RATIO;
                            IMU_rx_data.Pitch = (float)raw_y * IMU_ANGLE_RATIO;
                            IMU_rx_data.Yaw   = (float)raw_z * IMU_ANGLE_RATIO;

                            /* è§’åº¦å¸§æ›´æ–°åç«‹å³åˆ·æ–°åº”ç”¨å±‚å˜é‡ */
                            IMU_App_Update();
                            break;

                        default:
                            /* ÆäËû±¨ÎÄ (Èç´Å³¡µÈ) Ä¿Ç°ÏµÍ³ÎŞĞè¹Ø×¢£¬Ö±½Ó·ÅĞĞ
                            ºóĞøÈôÓĞĞèÇó¿ÉÔÙÌí¼Ó´¦Àí
                            */
                            break;
                    }
<<<<<<< HEAD
                    
                    /* Êı¾İÍê³Éºó£¬Á¢¼´¸üĞÂÓ¦ÓÃ²ã±äÁ¿ */
                    /* ½« IMU_rx_data ½ÓÊÕµ½µÄÊı¾İ¸³Öµ¸øÓ¦ÓÃ²ã±äÁ¿£¬ÓÃÓÚºóĞø¿ØÖÆ¼ÆËã */
                    IMU_App_Update();
                            
=======

>>>>>>> 89005bde61cd4fcf161cb4570fff459858e577cf
                    /* ==============================================================
                     * Ì½ÕëµÍ¿ªÏú·­×ª£º¼ÆÊıµ½ 20 Ö¡·´×ªÒ»´ÎÒı½Å
                     * ÓÃÍ¾£ºÈâÑÛ¹Û²ì GPIO_PIN_14£¨PF14£©µÄÉÁË¸ÆµÂÊ£¬ÆÀ¹À IMU CAN Í¨ĞÅ½¡¿µ¶È
                     * Ô­Àí£ºÃ¿ÊÕµ½ 20 Ö¡ÏûÏ¢·­×ªÒ»´Î£¬½¡¿µÍ¨ĞÅÊ±Ô¼ 50Hz ·­×ªÆµÂÊ
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

