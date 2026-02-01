#ifndef __CRC_CCITT__
#define __CRC_CCITT__


#include <stdint.h>

static const uint16_t crc_ccitt_table[256];
static uint16_t crc_ccitt_byte(uint16_t crc, uint8_t c) ;
static uint16_t crc_ccitt(uint16_t crc, const uint8_t* buffer, uint32_t len);

// Unitree CRC计算函数
void unitree_crc_complete(uint8_t frame[17]) ;

#endif
