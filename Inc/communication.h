#ifndef COMMU_H
#define COMMU_H

#include "stm32f3xx_hal.h"
#include "usbd_cdc_if.h"
#include <stdlib.h> 
#include <string.h>

extern uint8_t receive_buf[64];

extern uint32_t rot_val;

void clean_buf(void);

void read_data(uint8_t* Buf, uint32_t *Len);

void send_data(uint8_t* Buf, uint16_t Len);

#endif
