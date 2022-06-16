#ifndef SAMPLE_H
#define SAMPLE_H

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define FULL_SAMPLE 1
#define REALTIME_SAMPLE 2
#define VALIDATION 3

#include <stdio.h>
#include "peripheral/lis3mdl.h"

struct command_t {
    uint8_t mode;
    uint32_t samples;
};

struct sensor_data_t *magnet;

#endif