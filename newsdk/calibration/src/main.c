#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/uart.h>
#include <drivers/spi.h>
#include <logging/log.h>

#include <string.h>
#define LOG_MODULE_NAME calibration
LOG_MODULE_REGISTER(LOG_MODULE_NAME);
/* change this to any other UART peripheral if desired */
// #define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
static K_SEM_DEFINE(sample_ok, 0, 1);

static const struct device *uart_dev;

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

#define LIS3MDL_I2C_ADDR 0x1EU
#define LIS3MDL_SPI_READ		(1 << 7)
#define LIS3MDL_WHOAMI_REG 		0x0FU
#define LIS3MDL_CTRL_REG1 		0x20U
#define LIS3MDL_CTRL_REG2 		0x21U
#define LIS3MDL_CTRL_REG3 		0x22U
#define LIS3MDL_CTRL_REG4 		0x23U
#define LIS3MDL_CTRL_REG5 		0x24U
#define LIS3MDL_STATUS_REG 		0x27U
#define LIS3MDL_OUT_X_L 		0x28U
#define LIS3MDL_OUT_X_H 		0x29U
#define LIS3MDL_OUT_Y_L 		0x2AU
#define LIS3MDL_OUT_Y_H 		0x2BU
#define LIS3MDL_OUT_Z_L 		0x2CU
#define LIS3MDL_OUT_Z_H 		0x2DU

#define LIS3MDL_ID 0x3DU

#define NUM_SENSOR 7

const struct device *cs1;
const struct device *cs3;
const struct device *spi;

struct spi_config spi_ctg1 = {0};
struct spi_config spi_ctg2 = {0};
struct spi_config spi_ctg3 = {0};
struct spi_config spi_ctg4 = {0};
struct spi_config spi_ctg5 = {0};
struct spi_config spi_ctg6 = {0};
struct spi_config spi_ctg7 = {0};

struct spi_cs_control spi_cs1;
struct spi_cs_control spi_cs2;
struct spi_cs_control spi_cs3;
struct spi_cs_control spi_cs4;
struct spi_cs_control spi_cs5;
struct spi_cs_control spi_cs6;
struct spi_cs_control spi_cs7;

struct sensor_data_t {
	uint8_t sensor_id;
	uint16_t x_value;
	uint16_t y_value;
	uint16_t z_value;
	uint32_t timestamp;
};



void lis3mdl_spi_read(struct spi_config spi_ctg, uint8_t reg_addr, uint8_t *value, uint8_t len) {

	uint8_t buffer_tx[2] = { reg_addr | LIS3MDL_SPI_READ, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	int err = spi_transceive(spi, &spi_ctg, &tx, &rx);
	if (err){
		LOG_WRN("Fail SPI read");
	}
}

void lis3mdl_spi_write(struct spi_config spi_ctg, uint8_t reg_addr, uint8_t *value, uint8_t len) {
	uint8_t buffer_tx[1] = { reg_addr & ~LIS3MDL_SPI_READ };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

	int err = spi_transceive(spi, &spi_ctg, &tx, NULL);
	if (err){
		LOG_WRN("Fail SPI write");
	}
}

bool magnet_present(struct spi_config spi_ctg) {
	uint8_t whoamI;

	lis3mdl_spi_read(spi_ctg, LIS3MDL_WHOAMI_REG, (uint8_t * ) &whoamI, sizeof(whoamI));
	return whoamI == LIS3MDL_ID;
}

void lis3mdl_data_rate(struct spi_config spi_ctg) {

	uint8_t ctrl_reg1 = 0b00000010; // FAST MODE
	uint8_t ctrl_reg4 = 0b00000000;
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG1, (uint8_t * ) &ctrl_reg1, sizeof(ctrl_reg1));
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG4, (uint8_t * ) &ctrl_reg4, sizeof(ctrl_reg4));
}

void lis3mdl_full_scale_set(struct spi_config spi_ctg) {

	uint8_t ctrl_reg2 = 0b01100000; // 16 gauss
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG2, (uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
}

void lis3mdl_opearting_mode(struct spi_config spi_ctg) {

	uint8_t ctrl_reg3 = 0b00000000; // Continuous conversion mode
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG3, (uint8_t * ) &ctrl_reg3, sizeof(ctrl_reg3));
}

void lis3mdl_block_data_update_set(struct spi_config spi_ctg) {

	uint8_t ctrl_reg5 = 0x40; // Could this be the culprit???
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG5, (uint8_t * ) &ctrl_reg5, sizeof(ctrl_reg5));
}

void lis3mdl_init(struct spi_config spi_ctg) {

	lis3mdl_block_data_update_set(spi_ctg); // No idea what this does
	lis3mdl_data_rate(spi_ctg);
	lis3mdl_full_scale_set(spi_ctg);
	lis3mdl_opearting_mode(spi_ctg);
}

uint8_t lis3mdl_status(struct spi_config spi_ctg) {
	uint8_t ret;
	lis3mdl_spi_read(spi_ctg, LIS3MDL_STATUS_REG, (uint8_t * ) &ret, sizeof(ret));
	// i2c_reg_read_byte(i2c_device, LIS3MDL_I2C_ADDR, LIS3MDL_STATUS_REG, &ret);
	return ((ret >> 4) & 0x01);
}

float convert(int16_t lsb) {
	return (((float) lsb) / 1711.0f);
}

void lis3mdl_get_x(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];

	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_X_L, (uint8_t * ) &data[0], sizeof(data[0]));
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_X_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->x_value = (data[1] << 8 | data[0]);

}

void lis3mdl_get_y(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_Y_L, (uint8_t * ) &data[0], sizeof(data[0]));
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_Y_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->y_value = (data[1] << 8 | data[0]);
}

void lis3mdl_get_z(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_Z_L, (uint8_t * ) &data[0], sizeof(data[0]));
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_Z_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->z_value = (data[1] << 8 | data[0]);
}

void lis3mdl_get_xyz(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t status;
	uint8_t data[6];
	// if (sensor_data->sensor_id == 0) {
	// 	do{
	// 		status = lis3mdl_status(spi_ctg);
	// 	}while(!status);
	// }

	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_X_L | (1 << 6), (uint8_t * ) &data, sizeof(data));
	sensor_data->x_value = (data[1] << 8 | data[0]);
	sensor_data->y_value = (data[3] << 8 | data[2]);
	sensor_data->z_value = (data[5] << 8 | data[4]);
}

void lis3mdl_poweroff(struct spi_config spi_ctg) {
	// uint8_t ctrl_reg2 = 0x08; // reboot sensor
	uint8_t ctrl_reg = 0b00000011; // power down
	// lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG2, (uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG3, (uint8_t * ) &ctrl_reg, sizeof(ctrl_reg));
}

void lis3mdl_powerlow(struct spi_config spi_ctg) {
	uint8_t ctrl_reg = 0x20;
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG3, (uint8_t * ) &ctrl_reg, sizeof(ctrl_reg));
}

void lis3mdl_poweron(struct spi_config spi_ctg) {
	uint8_t ctrl_reg = 0x20;
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG3, (uint8_t * ) &ctrl_reg, sizeof(ctrl_reg));
}

void sensor_mode(uint8_t mode) {

	struct spi_config spi_ctg;

	for (int i=0; i < NUM_SENSOR; i++) {
		switch (i)
		{
		case 0:
			spi_ctg = spi_ctg1;
			break;
		case 1:
			spi_ctg = spi_ctg2;
			break;
		case 2:
			spi_ctg = spi_ctg3;
			break;
		case 3:
			spi_ctg = spi_ctg4;
			break;
		case 4:
			spi_ctg = spi_ctg5;
			break;
		case 5:
			spi_ctg = spi_ctg6;
			break;
		case 6:
			spi_ctg = spi_ctg7;
			break;
		default:
			break;
		}
		switch (mode)
		{
		case 0:
			lis3mdl_poweroff(spi_ctg);
			k_msleep(20);
			break;
		case 1:
			lis3mdl_init(spi_ctg);
			k_msleep(20);
			break;
		case 2:
			lis3mdl_powerlow(spi_ctg);
			k_msleep(20);
		default:
			break;
		}
	}
}


void hal_spi_init(void) {
	spi = device_get_binding("SPI_2");

	if (spi == NULL) {
		LOG_ERR("Cannot create Spi instance. No spi device found.");
		return;
	}

	spi_ctg1.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg1.frequency = 8000000;
	spi_ctg1.slave = 0;
	spi_cs1.gpio_dev = device_get_binding("GPIO_1");
	spi_cs1.gpio_pin = 4;
	spi_cs1.delay = 0;
	spi_cs1.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg1.cs = &spi_cs1;

	spi_ctg2.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg2.frequency = 8000000;
	spi_ctg2.slave = 0;
	spi_cs2.gpio_dev = device_get_binding("GPIO_1");
	spi_cs2.gpio_pin = 5;
	spi_cs2.delay = 0;
	spi_cs2.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg2.cs = &spi_cs2;

	spi_ctg3.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg3.frequency = 8000000;
	spi_ctg3.slave = 0;
	spi_cs3.gpio_dev = device_get_binding("GPIO_1");
	spi_cs3.gpio_pin = 6;
	spi_cs3.delay = 0;
	spi_cs3.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg3.cs = &spi_cs3;

	spi_ctg4.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg4.frequency = 8000000;
	spi_ctg4.slave = 0;
	spi_cs4.gpio_dev = device_get_binding("GPIO_1");
	spi_cs4.gpio_pin = 7;
	spi_cs4.delay = 0;
	spi_cs4.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg4.cs = &spi_cs4;

	spi_ctg5.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg5.frequency = 8000000;
	spi_ctg5.slave = 0;
	spi_cs5.gpio_dev = device_get_binding("GPIO_1");
	spi_cs5.gpio_pin = 8;
	spi_cs5.delay = 0;
	spi_cs5.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg5.cs = &spi_cs5;

	spi_ctg6.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg6.frequency = 8000000;
	spi_ctg6.slave = 0;
	spi_cs6.gpio_dev = device_get_binding("GPIO_1");
	spi_cs6.gpio_pin = 9;
	spi_cs6.delay = 0;
	spi_cs6.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg6.cs = &spi_cs6;

	spi_ctg7.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg7.frequency = 8000000;
	spi_ctg7.slave = 0;
	spi_cs7.gpio_dev = device_get_binding("GPIO_1");
	spi_cs7.gpio_pin = 10;
	spi_cs7.delay = 0;
	spi_cs7.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg7.cs = &spi_cs7;

	sensor_mode(1);
	sensor_mode(0);
	
}

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);

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


/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}


void transfer(struct sensor_data_t *data, uint16_t count) {

	static char buf[50];
    uint32_t send_count = 0;
    int err;
    uint32_t send_count_uplimit = count;
	uint8_t err_tick;

	err_tick = 0; // Check connection is still there
	uint32_t start = k_uptime_get_32();
    while(send_count < send_count_uplimit){
		memset(buf,0,sizeof(buf));
		sprintf(buf, "%d %u %u %u %u\r\n",data[send_count].sensor_id, data[send_count].x_value, data[send_count].y_value, data[send_count].z_value,  data[send_count].timestamp);
		// if (err_tick > 2000) {

		// 	// force disconnect from gateway if data cant send after a few tries, more functionalities can be added here
		// 	// IE: fs for unsent data 
		// 	bt_conn_disconnect(current_conn ,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		// 	break;  
		// }
        // err = bt_nus_send(NULL, buf, sizeof(buf));
        // if (err) {
        //     LOG_WRN("Failed to send data over BLE connection");
		// 	err_tick++;
        // }
        // else {
        //     send_count++;
        // }
		print_uart(buf);
		send_count++;
    }
	uint32_t end = k_uptime_get_32() - start;
	LOG_INF("Write time:%u", end);
	// Send acknoledge when data is done transferring
	memset(buf,0,sizeof(buf));
	sprintf(buf, "0\n");
	print_uart(buf);
	if (err) {
            LOG_WRN("Failed to send data over BLE connection");
    }
	// bt_conn_disconnect(current_conn ,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

void send_data(uint16_t count) {
	struct sensor_data_t *magnet;
	magnet = k_malloc(sizeof(struct sensor_data_t) * count);

	uint8_t count_temp = 0;
	struct spi_config spi_ctg;

	sensor_mode(1); // turn on all sensors
	
	for (int i=0; i < count; i++) {
			if (count_temp == 7) {
				count_temp = 0;
			}

			switch (count_temp)
			{
			case 0:
				spi_ctg = spi_ctg1;
				break;
			case 1:
				spi_ctg = spi_ctg2;
				break;
			case 2:
				spi_ctg = spi_ctg3;
				break;
			case 3:
				spi_ctg = spi_ctg4;
				break;
			case 4:
				spi_ctg = spi_ctg5;
				break;
			case 5:
				spi_ctg = spi_ctg6;
				break;
			case 6:
				spi_ctg = spi_ctg7;
				break;
			default:
				break;

			}
		// lis3mdl_poweron(spi_ctg);
		magnet[i].sensor_id = count_temp;
		while(1) {
			uint32_t temp_time = k_cyc_to_us_floor32(k_cycle_get_32());
			if (i < 7) {
				lis3mdl_get_xyz(spi_ctg, &(magnet[i]));
				magnet[i].timestamp = temp_time;
				break;
			}

			else if (temp_time - magnet[i-6].timestamp > 999) {
				lis3mdl_get_xyz(spi_ctg, &(magnet[i]));
				magnet[i].timestamp = temp_time; // get timestamp in microseconds 
				break;
			}
		}
		// lis3mdl_powerlow(spi_ctg);
		count_temp++;
	}

	sensor_mode(0); // turn off all sensors
	transfer(magnet, count);
	k_free(magnet);
	
}

static void process_command(char *buf) {
	char string[20]; // string test
	uint16_t count;
	LOG_INF("Command received: %s", string);
	char *token;
	char *rest = buf;
	char **command = (char**)k_malloc(3*sizeof(char*));
	for (int j=0; j < 3; j++)
		command[j] = (char*) k_malloc(sizeof(char)*10);

	int i = 0;
	while((token = strtok_r(rest, " ", &rest))) {
		strcpy(command[i], token);
		i++;
		if (i > 2) {
			break;
		}
	}

	if (strcmp("sensor", command[0]) == 0) { // why is this statement not working?????
			if (strcmp(NULL, command[1]) == 0) {

			}
		}

		if (strcmp(NULL, command[1]) == 0) {
			if (strcmp("sample", command[0]) == 0) {
				// memset(buf, 0, sizeof(buf));
				// sprintf(buf, "Need sample size(1 - 10000)\n");
				// int err = bt_nus_send(NULL, buf, sizeof(buf));
				// if (err) {
				// 	LOG_WRN("Failed to send data over BLE connection");
				// }
			}
		}

		// This part is shitty...
		else if (strcmp("sample", command[0]) == 0) {
			if (strcmp(NULL, command[1])!=0) {
				count = atoi(command[1]);
				if (count > 10002){

				} else
					send_data(count);
			}
		}

		for (i=0; i< 3; i++) {        
   			free(command[i]); 
		}
		k_free(command);
}





void main(void)
{
	
	uart_dev = device_get_binding("UART_1");
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not found!");
		return;
	}

	hal_spi_init();
	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	// print_uart("Hello! I'm your echo bot.\r\n");
	// print_uart("Tell me something and press enter:\r\n");
	k_sem_give(&sample_ok);
	// /* indefinitely wait for input from the user */
	// while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
	// 	print_uart("Echo: ");
	// 	print_uart(tx_buf);
	// 	print_uart("\r\n");
	// }
}

void sample_thread() {
	k_sem_take(&sample_ok, K_FOREVER);
	char tx_buf[MSG_SIZE];
	uint16_t count;
	for (;;) {
		k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER);
		// print_uart("Received:");
		// print_uart(tx_buf);
		// print_uart("\r\n");
		// char **command = (char**)k_malloc(3*sizeof(char*));
		// for (int j=0; j < 3; j++)
		// 	command[j] = (char*) k_malloc(sizeof(char)*10);
		process_command(tx_buf);
		// if (strcmp("sensor", command[0]) == 0) { // why is this statement not working?????
		// 	if (strcmp(NULL, command[1]) == 0) {

		// 	}
		// }

		// if (strcmp(NULL, command[1]) == 0) {
		// 	if (strcmp("sample", command[0]) == 0) {
		// 		// memset(buf, 0, sizeof(buf));
		// 		// sprintf(buf, "Need sample size(1 - 10000)\n");
		// 		// int err = bt_nus_send(NULL, buf, sizeof(buf));
		// 		// if (err) {
		// 		// 	LOG_WRN("Failed to send data over BLE connection");
		// 		// }
		// 	}
		// }

		// // This part is shitty...
		// else if (strcmp("sample", command[0]) == 0) {
		// 	if (strcmp(NULL, command[1])!=0) {
		// 		count = atoi(command[1]);
		// 		if (count > 10002){

		// 		} else
		// 			send_data(count);
		// 	}
		// }
		// k_free(command);
	}
}

K_THREAD_DEFINE(sample_thread_id, STACKSIZE, sample_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

