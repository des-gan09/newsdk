#include <string.h>
#include <zephyr/types.h>
#include <sys/byteorder.h>
#include "sample.h"
#include "peripheral/lis3mdl.h"
#include "hal/ble.h"
#include "hal/hal_spi.h"
#include <logging/log.h>

#define LOG_MODULE_NAME sampling
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define THROUGH_PACKET_SIZE 50
struct k_fifo fifo_stream;
struct k_sem stream_sem;
void sensor_mode(uint8_t mode) {

	for (int i=0; i < NUM_SENSOR; i++) {
		switch (mode)
		{
		case 0:
			lis3mdl_poweroff(spi_group[i].spi_ctg);
			k_msleep(20);
			break;
		case 1:
			lis3mdl_init(spi_group[i].spi_ctg);
			k_msleep(20);
			break;
		// case 2:
		// 	lis3mdl_powerlow(spi_ctg);
		// 	k_msleep(20);
		default:
			break;
		}
	}
}

void streaming_data() {
	struct stream_data_t *stream;
	struct sensor_data_t magnet;
	magnet.sensor_id = 0;
	uint32_t temp_time = k_cyc_to_us_floor32(k_cycle_get_32());
	if (k_cyc_to_us_floor32(k_cycle_get_32()) - temp_time > 999) {
		lis3mdl_get_xyz(spi_group[0].spi_ctg, &magnet);
		magnet.timestamp = temp_time; // get timestamp in microseconds 
	}
	stream = k_malloc(sizeof(*stream));
	stream->data = magnet;
	stream->len = sizeof(magnet);

	k_fifo_put(&fifo_stream, stream);
}

void ble_transfer(struct sensor_data_test *data, uint16_t count) {

	static char buf[100];
    uint32_t send_count = 0;
    int err;
    uint32_t send_count_uplimit = count;
	uint8_t err_tick;

	sent_cnt = 0; // clear sent counting
	err_tick = 0; // Check connection is still there
	uint32_t start = k_uptime_get_32();
    while(send_count < send_count_uplimit){
		memset(buf,0,sizeof(buf));
		sprintf(buf, "%d %u %u %u %u %u %u\n",data[send_count].sensor_id, 
				data[send_count].x_value, 
				data[send_count].y_value, 
				data[send_count].z_value,  
				data[send_count].timestamp_x,
				data[send_count].timestamp_y,
				data[send_count].timestamp_z);
		if (err_tick > 200) {

			// force disconnect from gateway if data cant send after a few tries, more functionalities can be added here
			// IE: fs for unsent data 
			bt_conn_disconnect(current_conn ,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			break;  
		}
        err = bt_nus_send(NULL, buf, sizeof(buf));
        if (err) {
            LOG_WRN("Failed to send data over BLE connection");
			err_tick++;
        }
        else {
            send_count++;
        }

        if(send_count - get_sent_cnt() > (CONFIG_BT_L2CAP_TX_BUF_COUNT)){
             LOG_WRN("Buffer getting tight, wait sometime here");
             k_sleep(K_MSEC(3));
        }
    }
	uint32_t end = k_uptime_get_32() - start;
	LOG_INF("BLE write time:%u", end);
	// Send acknoledge when data is done transferring
	memset(buf,0,sizeof(buf));
	sprintf(buf, "0\n");
	err = bt_nus_send(NULL, buf, sizeof(buf));
	if (err) {
            LOG_WRN("Failed to send data over BLE connection");
    }
	// bt_conn_disconnect(current_conn ,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

void check_sensor() {

	for (int i=0; i < NUM_SENSOR; i++) {
		
		if (lis3mdl_present(spi_group[i].spi_ctg)) {
			// LOG_INF("SENSOR %d present", i);
			spi_group[i].state = 1; // sensor is present
			// k_sleep(K_MSEC(1));
		}
	}
}

void sample_data(uint16_t count) {
	
	struct sensor_data_test *magnet;
	magnet = k_malloc(sizeof(struct sensor_data_test) * count);
	struct sensor_data_t junk;
	uint8_t id, count_temp, offsetlimit, sampling = 0;
	uint8_t tries = 3;
	int sampled = 0;
	uint32_t timestamp;
	uint8_t order[NUM_SENSOR], new_order[NUM_SENSOR];
	sensor_mode(1); // turn on all sensors
	check_sensor();
	uint8_t reg;
	hal_spi_read(spi_group[0].spi_ctg, LIS3MDL_CTRL_REG4 | LIS3MDL_SPI_READ, (uint8_t * ) &reg, sizeof(reg)); // check z settings
	LOG_INF("Register value:%x", reg);

	for (int i=0; i<NUM_SENSOR; i++) { // get default order (ie:1,2,3,4,5,6,7)
		order[i] = spi_group[i].sensor_id;
	}

	for (int i=0; i < NUM_SENSOR; i++) {
		// LOG_INF("Sensor %d PIN: %d", i, spi_group[i].spi_ctg.cs->gpio_pin);
		lis3mdl_get_xyz(spi_group[i].spi_ctg, &junk);
		if (junk.x_value == junk.y_value && junk.x_value == junk.z_value) {
			spi_group[i].state = 2;
		}
	}
	

	while (sampled < count) {

		uint8_t ret = lis3mdl_status(spi_group[order[count_temp]].spi_ctg);
		ret = ret & 0x0f;
		timestamp = k_cyc_to_us_floor32(k_cycle_get_32());
		switch (ret)
		{
		case 0x01: // X ready
			/* code */
			magnet[sampled].timestamp_x = timestamp;
			lis3mdl_get_x(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			magnet[sampled].sensor_id = spi_group[order[count_temp]].sensor_id;
			magnet[sampled].timestamp_y = 0;
			magnet[sampled].timestamp_z = 0;
			magnet[sampled].y_value = 0;
			magnet[sampled].z_value = 0;
			// sampling |= (1 << 0);
			sampled++;
			break;
		case 0x02: // Y ready
			magnet[sampled].timestamp_y = timestamp;
			lis3mdl_get_y(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			magnet[sampled].sensor_id = spi_group[order[count_temp]].sensor_id;
			magnet[sampled].timestamp_x = 0;
			magnet[sampled].timestamp_z = 0;
			magnet[sampled].x_value = 0;
			magnet[sampled].z_value = 0;
			// sampling |= (1 << 1);
			sampled++;
			break;
		case 0x03: // X & Y ready
			magnet[sampled].timestamp_x = timestamp;
			magnet[sampled].timestamp_y = timestamp;
			lis3mdl_get_x(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			lis3mdl_get_y(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			magnet[sampled].sensor_id = spi_group[order[count_temp]].sensor_id;
			magnet[sampled].timestamp_z = 0;
			magnet[sampled].z_value = 0;
			// sampling |= (1 << 0);
			// sampling |= (1 << 1);
			sampled++;
			break;
		case 0x04: // Z ready
			magnet[sampled].timestamp_z = timestamp;
			lis3mdl_get_z(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			magnet[sampled].sensor_id = spi_group[order[count_temp]].sensor_id;
			magnet[sampled].timestamp_y = 0;
			magnet[sampled].timestamp_x = 0;
			magnet[sampled].y_value = 0;
			magnet[sampled].x_value = 0;
			// sampling |= (1 << 2);
			sampled++;
			break;
		case 0x05: // X & Z ready
			magnet[sampled].timestamp_x = timestamp;
			magnet[sampled].timestamp_z = timestamp;
			lis3mdl_get_x(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			lis3mdl_get_z(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			magnet[sampled].sensor_id = spi_group[order[count_temp]].sensor_id;
			magnet[sampled].timestamp_y = 0;
			magnet[sampled].y_value = 0;
			// sampling |= (1 << 0);
			// sampling |= (1 << 1);
			sampled++;
			break;
		case 0x06: // Y & Z ready
			magnet[sampled].timestamp_z = timestamp;
			magnet[sampled].timestamp_y = timestamp;
			lis3mdl_get_z(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			lis3mdl_get_y(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
			magnet[sampled].sensor_id = spi_group[order[count_temp]].sensor_id;
			magnet[sampled].timestamp_x = 0;
			magnet[sampled].x_value = 0;
			// sampling |= (1 << 0);
			// sampling |= (1 << 1);
			sampled++;
			break;
		case 0x07: // X,Y,Z ready
				magnet[sampled].timestamp_x = timestamp;
				magnet[sampled].timestamp_y = timestamp;
				magnet[sampled].timestamp_z = timestamp;
				lis3mdl_get_xyz(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
				magnet[sampled].sensor_id = spi_group[order[count_temp]].sensor_id;
				sampled++;
			break; 
		case 0x0f: // X,Y,Z ready
				magnet[sampled].timestamp_x = timestamp;
				magnet[sampled].timestamp_y = timestamp;
				magnet[sampled].timestamp_z = timestamp;
				lis3mdl_get_xyz(spi_group[order[count_temp]].spi_ctg, &(magnet[sampled]));
				magnet[sampled].sensor_id = spi_group[order[count_temp]].sensor_id;
				sampled++;
			break; 
		default:
			break;
		}
	}

	sensor_mode(0); // turn off all sensors
	ble_transfer(magnet, count);
	k_free(magnet);
}

static uint8_t process_command(struct uart_data_t *buf, struct command_t *command) {
	char string[51]; // string test
	memcpy(string, buf->data, sizeof(buf->data));
	string[sizeof(buf->data)] = '\0';
	LOG_INF("Command received: %s", log_strdup(string));
	char *token;
	char *rest = string;
	char **array = (char**)k_malloc(3*sizeof(char*));
	for (int j=0; j < 3; j++)
		array[j] = (char*) k_malloc(sizeof(char)*10);

	int i = 0;
	while((token = strtok_r(rest, " ", &rest))) {
		strcpy(array[i], token);
		i++;
		if (i > 2) {
			break;
		}
	}

    if (strcmp("sample", array[0]) == 0) {
        command->mode = FULL_SAMPLE;
        command->samples = atoi(array[1]);
		return 0;
    } else if (strcmp("real", array[0]) == 0) {
        command->mode = REALTIME_SAMPLE;
        command->samples = 0;
		return 0;
    } else if (strcmp("check", array[0]) == 0) {
        command->mode = VALIDATION;
        command->samples = 0;
		return 0;
    }
	return 1;
}

void sampling_thread() {
	k_fifo_init(&fifo_stream);
	k_sem_init(&stream_sem, 0 , 1);
    k_sem_take(&ble_init_ok, K_FOREVER);
    for(;;) {
        struct command_t command;
        struct uart_data_t *buf = k_fifo_get(&fifo_transfer,
							K_FOREVER);
        k_sem_take(&throughput_sem, K_FOREVER);
		if (!process_command(buf, &command)) {
			switch (command.mode)
			{
			case FULL_SAMPLE:
				LOG_INF("Taking %d samples.", command.samples);
				sample_data(command.samples);
				break;
			case REALTIME_SAMPLE:
				LOG_INF("Sampling in realtime.");
				k_sem_give(&stream_sem);
				streaming_data();
				break;
			case VALIDATION:
				lis3mdl_validation();
				break;
			default:
				break;
			}
		}
		k_free(buf);
    }
}

K_THREAD_DEFINE(sampling_thread_id, STACKSIZE, sampling_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);