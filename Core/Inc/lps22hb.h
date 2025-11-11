/*
 * lps22hb.h
 *
 *  Created on: Nov 1, 2025
 *      Author: mirom
 */

#ifndef LPS22HB_H
#define LPS22HB_H
#include <stdint.h>

enum { LPS22HB_ADDR_7B = 0x5D }; /* 0x5D if SA0=1 */

#define LPS22HB_WHO_AM_I_REG 0x0F
#define LPS22HB_WHO_AM_I_VAL 0xB1
#define LPS22HB_CTRL_REG1    0x10
#define LPS22HB_PRESS_OUT_XL 0x28

uint8_t lps22hb_init(void);
float   lps22hb_read_pressure_hpa(void);
float   lps22hb_altitude_from_pressure(float p_hpa, float p0_hpa);

#endif /* INC_LPS22HB_H_ */
