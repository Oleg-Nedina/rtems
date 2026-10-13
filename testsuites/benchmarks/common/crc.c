#include "crc.h"

uint32_t crc32(const unsigned char *message, uint32_t len) {
   uint32_t crc = 0xFFFFFFFF;
   for(uint32_t i = 0; i < len; i++) {
      // xor the crc with the message byte summed to the index shifted by 0,1,2,3 bytes
      crc = crc ^ (((message[i] + (i & 0xff)) % 0x100)<< (8*(i%4)));      
   }
   return crc;
}