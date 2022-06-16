#ifndef WDT_H
#define WDT_H

#include <device.h>
#include <drivers/watchdog.h>

extern const struct device *wdt;

void wdt_callback(struct device *wdt_dev, int channel_id);
void install_watchdog();

#endif