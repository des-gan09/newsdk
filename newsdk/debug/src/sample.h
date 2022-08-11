#ifndef SAMPLE_H
#define SAMPLE_H

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define FULL_SAMPLE 1
#define REALTIME_SAMPLE 2
#define VALIDATION 3

#include <stdio.h>
#include <zephyr.h>
#include "peripheral/lis3mdl.h"

extern struct k_fifo fifo_stream;

struct command_t {
    uint8_t mode;
    uint32_t samples;
};

struct stream_data_t {
	void *fifo_reserved;
	struct sensor_data_t data;
	uint16_t len;
};

extern void sensor_mode(uint8_t mode);

#endif