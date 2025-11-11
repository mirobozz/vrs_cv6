/*
 * hts221.h
 *
 *  Created on: Nov 1, 2025
 *      Author: mirom
 */

#include <stdint.h>
#include "main.h"

#ifndef INC_HTS221_H_
#define INC_HTS221_H_

//defines
enum { HTS221_ADDR_7B = 0x5F };

#define HTS221_WHO_AM_I_REG 0x0F
#define HTS221_WHO_AM_I_VAL 0xBC
#define HTS221_AV_CONF      0x10
#define HTS221_CTRL_REG1    0x20
#define HTS221_STATUS_REG   0x27

//func prototypes
uint8_t hts221_init(void);

float   hts221_read_temperature(void);

float   hts221_read_humidity(void);

#endif /* INC_HTS221_H_ */
