#pragma once
#include <stddef.h>
#include <stdint.h>
#include "ring_buffer.h"

typedef struct {
    // counters
    uint32_t pkts_in;
    uint32_t bytes_in;
    uint32_t drops_total;
    uint32_t seq_missed;

    // last seq seen (for 1-byte seq_id)
    int last_seq; // -1 = not set
} microros_bridge_stats_t;

void microros_bridge_init(rb_t *rb);
const microros_bridge_stats_t* microros_bridge_stats(void);
