#ifndef HAL_UART_H
#define HAL_UART_H

#include <drivers/uart.h>
#include <device.h>

#define MSG_SIZE 32

extern const struct device *uart;
extern void hal_uart_init();
