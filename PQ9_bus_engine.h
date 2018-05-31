#ifndef __PQ9_BUS_ENGINE_H
#define __PQ9_BUS_ENGINE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define PQ9_MASTER_APP_ID _OBC_APP_ID_
typedef uint8_t TC_TM_app_id;

typedef struct {
  TC_TM_app_id dest_id;
  TC_TM_app_id src_id;
  uint8_t size;
  uint8_t *msg;
}pq9_pkt;

bool unpack_PQ9_BUS(const uint8_t *buf,
                    const uint16_t size,
                    pq9_pkt *pq_pkt);

bool pack_PQ9_BUS(pq9_pkt *pq_pkt, uint8_t *buf, uint16_t *size);

#endif
