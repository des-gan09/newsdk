#include <string.h>
#include "sample.h"
#include "peripheral/lis3mdl.h"
#include "hal/ble.h"
#include "hal/hal_spi.h"
#include <logging/log.h>


#define LOG_MODULE_NAME sampling
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define THROUGH_PACKET_SIZE 50

void sensor_mode(uint8_t mode) {

	for (int i=0; i < spi_count; i++) {
		switch (mode)
		{
		case 0:
			lis3mdl_poweroff(spi_ctgx[i]);
			k_msleep(20);
			break;
		case 1:
			lis3mdl_init(spi_ctgx[i]);
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

void ble_transfer(struct sensor_data_t *data, uint16_t count) {

	static char buf[THROUGH_PACKET_SIZE];
    uint32_t send_count = 0;
    int err;
    uint32_t send_count_uplimit = count;
	uint8_t err_tick;

	sent_cnt = 0; // clear sent counting
	err_tick = 0; // Check connection is still there
	uint32_t start = k_uptime_get_32();
    while(send_count < send_count_uplimit){
		memset(buf,0,sizeof(buf));
		sprintf(buf, "%d %ld %ld %ld %u\n",data[send_count].sensor_id, data[send_count].x_value, data[send_count].y_value, data[send_count].z_value,  data[send_count].timestamp);
		// if (err_tick > 2000) {

		// 	// force disconnect from gateway if data cant send after a few tries, more functionalities can be added here
		// 	// IE: fs for unsent data 
		// 	bt_conn_disconnect(current_conn ,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		// 	break;  
		// }
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
             k_sleep(K_MSEC(10));
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


void sample_data(uint16_t count) {
	struct sensor_data_t *magnet;
	magnet = k_malloc(sizeof(struct sensor_data_t) * count);

	uint8_t count_temp = 0;
	struct spi_config spi_ctg;

	sensor_mode(1); // turn on all sensors
	
	for (int i=0; i < count; i++) {
        if (count_temp == spi_count) {
            count_temp = 0;
        }

			
		// lis3mdl_poweron(spi_ctg);
		magnet[i].sensor_id = count_temp;
		while(1) {
			uint32_t temp_time = k_cyc_to_us_floor32(k_cycle_get_32());
			if (i < 7) {
				lis3mdl_get_xyz(spi_ctgx[count_temp], &(magnet[i]));
				magnet[i].timestamp = temp_time;
				break;
			}

			else if (temp_time - magnet[i-6].timestamp > 999) {
				lis3mdl_get_xyz(spi_ctgx[count_temp], &(magnet[i]));
				magnet[i].timestamp = temp_time; // get timestamp in microseconds 
				break;
			}
		}
		// lis3mdl_powerlow(spi_ctg);
		count_temp++;
	}

	sensor_mode(0); // turn off all sensors
	ble_transfer(magnet, count);
	k_free(magnet);
}

static void process_command(struct uart_data_t *buf, struct command_t *command) {
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
    } else if (strcmp("real", array[0]) == 0) {
        command->mode = REALTIME_SAMPLE;
        command->samples = 0;
    } else if (strcmp("check", array[0]) == 0) {
        command->mode = VALIDATION;
        command->samples = 0;
    }
}

void sampling_thread() {
    k_sem_take(&ble_init_ok, K_FOREVER);
    // k_sem_take(&throughput_sem, K_FOREVER);
    uint16_t count;
    for(;;) {
        struct command_t command;
        struct uart_data_t *buf = k_fifo_get(&fifo_transfer,
							K_FOREVER);
        // printk("SPIs: %d", spi_count);
        k_sem_take(&throughput_sem, K_FOREVER);
        process_command(buf, &command);
        switch (command.mode)
        {
        case FULL_SAMPLE:
            LOG_INF("Taking %d samples.", command.samples);
            sample_data(command.samples);
            break;
        case REALTIME_SAMPLE:
            LOG_INF("Sampling in realtime.");
            break;
        case VALIDATION:
            lis3mdl_validation();
            break;
        default:
            break;
        }
        // // sensor_mode(0); // turn off all sensors
        // k_free(command);
        // k_free(buf); 
    }
}

K_THREAD_DEFINE(sampling_thread_id, STACKSIZE, sampling_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);