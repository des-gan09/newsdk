#include <drivers/uart.h>
#include <device.h>
#include "hal_uart.h"
#include <logging/log.h>

#define LOG_MODULE_NAME uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
char __aligned(4) uart_msgq_buffer[MSG_SIZE];
struct k_msgq uart_msgq;


struct k_sem sample_ok;
const struct device *uart;
/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart)) {
		return;
	}

	while (uart_irq_rx_ready(uart)) {

		uart_fifo_read(uart, &c, 1);

		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			
			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

void hal_uart_init() {
	k_msgq_init(&uart_msgq, uart_msgq_buffer, MSG_SIZE, 10);
    uart = device_get_binding("UART_1");
    if (uart == NULL) {
        LOG_ERR("Cannot find UART device");
        return;
    }
	k_sem_init(&sample_ok, 0, 1);
    // k_msgq_init(&uart_msg, MSG_SIZE, sizeof(struct data_item_type), 10);

    uart_irq_callback_user_data_set(uart, serial_cb, NULL);
	uart_irq_rx_enable(uart);
	k_sem_give(&sample_ok);
}