#include <drivers/watchdog.h>
#include <sys/reboot.h>
#include <device.h>
#include "wdt.h"
#include <logging/log.h>

struct wdt_timeout_cfg wdt_config;
int wdt_channel_id;
const struct device *wdt;

#define LOG_MODULE_NAME watchdog
LOG_MODULE_REGISTER(LOG_MODULE_NAME);


void wdt_callback(struct device *wdt_dev, int channel_id) {
    static bool handled_event;
    if (handled_event) {
        
    }

    wdt_feed(wdt_dev, channel_id);
    handled_event = true;
}

void install_watchdog() {
    /* Reset SoC when watchdog timer expires. */
	wdt = device_get_binding(DT_LABEL(DT_INST(0, nordic_nrf_watchdog)));
	int err;
	wdt_config.flags = WDT_FLAG_RESET_SOC;

	/* Expire watchdog after 1000 milliseconds. */
	wdt_config.window.min = 0U;
	wdt_config.window.max = CONFIG_WATCHDOG_DURATION;
	// wdt_config.window.max = 900000U;
	/* Set up watchdog callback. Jump into it when watchdog expired. */
	wdt_config.callback = wdt_callback;

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	// if (wdt_channel_id == -ENOTSUP) {
	// 	/* IWDG driver for STM32 doesn't support callback */
	// 	wdt_config.callback = NULL;
	// 	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	// }
	if (wdt_channel_id < 0) {
		LOG_ERR("Watchdog install error\n");
		return;
	}

	err = wdt_setup(wdt, 0);
}