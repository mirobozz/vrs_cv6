/*
 * lps22hb.c
 *
 *  Created on: Nov 2, 2025
 *      Author: mirom
 */


#include "lps22hb.h"
#include "i2c.h"
#include <math.h>

#define AI 0x80 /* auto-increment */

uint8_t lps22hb_init(void)
{
    uint8_t who = i2c_read_reg(LPS22HB_ADDR_7B, LPS22HB_WHO_AM_I_REG);
    if (who != LPS22HB_WHO_AM_I_VAL) return 1;

    /* ODR=1Hz, BDU=1 */
    i2c_write_reg(LPS22HB_ADDR_7B, LPS22HB_CTRL_REG1, 0x12);
    return 0;
}

float lps22hb_read_pressure_hpa(void)
{
    uint8_t raw[3];
    i2c_read_buf(LPS22HB_ADDR_7B, LPS22HB_PRESS_OUT_XL | AI, raw, 3);

    int32_t p = (int32_t)((uint32_t)raw[2] << 16 |
                          (uint32_t)raw[1] << 8  |
                          (uint32_t)raw[0]);
    if (p & 0x00800000) p |= 0xFF000000; /* sign-extend */

    return (float)p / 4096.0f;
}

float lps22hb_altitude_from_pressure(float p_hpa, float p0_hpa)
{
    if (p_hpa <= 0.0f || !isfinite(p_hpa)) return NAN;
    return 44330.0f * (1.0f - powf(p_hpa / p0_hpa, 0.1903f));
}

