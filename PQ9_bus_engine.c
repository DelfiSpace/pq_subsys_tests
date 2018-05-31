#include "PQ9_bus_engine.h"
#include <stdint.h>


#define CRC_POLY 0x1021

union _cnv {
    double cnvD;
    float cnvF;
    uint32_t cnv32;
    uint16_t cnv16[4];
    uint8_t cnv8[8];
};

// need to check endiannes
void cnv32_8(const uint32_t from, uint8_t *to) {

    union _cnv cnv;

    cnv.cnv32 = from;
    to[0] = cnv.cnv8[0];
    to[1] = cnv.cnv8[1];
    to[2] = cnv.cnv8[2];
    to[3] = cnv.cnv8[3];
}

void cnv16_8(const uint16_t from, uint8_t *to) {

    union _cnv cnv;

    cnv.cnv16[0] = from;
    to[0] = cnv.cnv8[0];
    to[1] = cnv.cnv8[1];

}

void cnv8_32(uint8_t *from, uint32_t *to) {

    union _cnv cnv;

    cnv.cnv8[3] = from[3];
    cnv.cnv8[2] = from[2];
    cnv.cnv8[1] = from[1];
    cnv.cnv8[0] = from[0];
    *to = cnv.cnv32;

}

void cnv8_16LE(uint8_t *from, uint16_t *to) {

    union _cnv cnv;

    cnv.cnv8[1] = from[1];
    cnv.cnv8[0] = from[0];
    *to = cnv.cnv16[0];
}

void cnv8_16(uint8_t *from, uint16_t *to) {

    union _cnv cnv;

    cnv.cnv8[1] = from[0];
    cnv.cnv8[0] = from[1];
    *to = cnv.cnv16[0];
}


void cnvF_8(const float from, uint8_t *to) {

    union _cnv cnv;

    cnv.cnvF = from;
    to[0] = cnv.cnv8[0];
    to[1] = cnv.cnv8[1];
    to[2] = cnv.cnv8[2];
    to[3] = cnv.cnv8[3];
}

void cnv8_F(uint8_t *from, float *to) {

    union _cnv cnv;

    cnv.cnv8[3] = from[3];
    cnv.cnv8[2] = from[2];
    cnv.cnv8[1] = from[1];
    cnv.cnv8[0] = from[0];
    *to = cnv.cnvF;

}

void cnvD_8(const double from, uint8_t *to) {

    union _cnv cnv;

    cnv.cnvD = from;
    to[0] = cnv.cnv8[0];
    to[1] = cnv.cnv8[1];
    to[2] = cnv.cnv8[2];
    to[3] = cnv.cnv8[3];
    to[4] = cnv.cnv8[4];
    to[5] = cnv.cnv8[5];
    to[6] = cnv.cnv8[6];
    to[7] = cnv.cnv8[7];
}

void cnv8_D(uint8_t *from, double *to) {

    union _cnv cnv;

    cnv.cnv8[7] = from[7];
    cnv.cnv8[6] = from[6];
    cnv.cnv8[5] = from[5];
    cnv.cnv8[4] = from[4];
    cnv.cnv8[3] = from[3];
    cnv.cnv8[2] = from[2];
    cnv.cnv8[1] = from[1];
    cnv.cnv8[0] = from[0];
    *to = cnv.cnvD;

}

/**** CRC calculator ****/
uint16_t crc_PQ9(uint16_t crc1, uint8_t data, uint16_t poly) {

    for (unsigned char i = 0; i < 8; i++)
    {
      if (((( crc1 & 0x8000) >> 8) ^ (data & 0x80)) != 0)
      {
        crc1 <<= 1;
        crc1 ^= poly;
      }
      else
      {
        crc1 <<= 1;
      }
      data <<= 1;
    }
    return crc1;
}

uint16_t calculate_crc_PQ9(uint8_t *buf, const uint16_t size) {

  uint16_t val = 0xFFFF;

  for(uint16_t i = 0; i < size; i++) {
    val = crc_PQ9(val, buf[i], 0x1021);
  }

  return val;
}

bool unpack_PQ9_BUS(const uint8_t *buf,
                    const uint16_t size,
                    pq9_pkt *pq_pkt) {

  pq_pkt->dest_id = buf[0];
  pq_pkt->size = buf[1];

  if(pq_pkt->size != size - 4) {
    return true;
  }

//  if(pq_pkt->dest_id != SYSTEM_APP_ID) {
//    return true;
//  }

  uint16_t crc_calc = calculate_crc_PQ9(buf, size - 2);
  uint16_t crc_pkt = 0;

  cnv8_16(crc_pkt, &buf[size-2]);

  if(crc_calc == crc_pkt) {
    return true;
  }

  memcpy(pq_pkt->msg, &buf[2], pq_pkt->size);

// #if(SYSTEM_APP_ID != PQ9_MASTER_APP_ID)
//   enable_PQ9_tx();
// #endif

  return false;
}

bool pack_PQ9_BUS(pq9_pkt *pq_pkt, uint8_t *buf, uint16_t *size) {

// #if(SYSTEM_APP_ID == PQ9_MASTER_APP_ID)
//   if(pq_pkt->dest_id == PQ9_MASTER_APP_ID) {
//     return true;
//   }
// #else
//   if(pq_pkt->dest_id != PQ9_MASTER_APP_ID) {
//     return true;
//   }
// #endif

  *size = pq_pkt->size + 5;

  buf[0] = pq_pkt->dest_id;
  buf[1] = pq_pkt->size;
  buf[2] = pq_pkt->src_id;

  memcpy(&buf[3], pq_pkt->msg, pq_pkt->size);

  uint16_t crc = calculate_crc_PQ9(buf, *size-2);
  cnv16_8(crc, &buf[*size-2]);

  return false;
}
