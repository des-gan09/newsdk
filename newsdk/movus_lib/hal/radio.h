#ifndef RADIO_H
#define RADIO_H

void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl);
void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl);

extern void tx_pwr_init();

#endif