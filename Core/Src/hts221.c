/*
 * hts221.c
 *
 *  Created on: Nov 1, 2025
 *      Author: mirom
 */


#include "hts221.h"
#include "i2c.h"

#define AI 0x80 /* auto-increment */

static float   T0_degC, T1_degC;
static int16_t T0_OUT, T1_OUT;
static float   H0_rH, H1_rH;
static int16_t H0_T0_OUT, H1_T0_OUT;

uint8_t hts221_init(void)
{
    if (i2c_read_reg(HTS221_ADDR_7B, HTS221_WHO_AM_I_REG) != HTS221_WHO_AM_I_VAL)
        return 1;

    /* averages */
    i2c_write_reg(HTS221_ADDR_7B, HTS221_AV_CONF, 0x1B); /* AVGT=16, AVGH=32 */
    /* power on + BDU + 1Hz */
    i2c_write_reg(HTS221_ADDR_7B, HTS221_CTRL_REG1, 0x85);

    /* humidity calib */
    uint8_t H0_rH_x2 = i2c_read_reg(HTS221_ADDR_7B, 0x30);
    uint8_t H1_rH_x2 = i2c_read_reg(HTS221_ADDR_7B, 0x31);
    H0_rH = H0_rH_x2 / 2.0f;
    H1_rH = H1_rH_x2 / 2.0f;

    uint8_t b[2];
    i2c_read_buf(HTS221_ADDR_7B, 0x36 | AI, b, 2);
    H0_T0_OUT = (int16_t)(b[1] << 8 | b[0]);
    i2c_read_buf(HTS221_ADDR_7B, 0x3A | AI, b, 2);
    H1_T0_OUT = (int16_t)(b[1] << 8 | b[0]);

    /* temperature calib */
    uint8_t T0_degC_x8 = i2c_read_reg(HTS221_ADDR_7B, 0x32);
    uint8_t T1_degC_x8 = i2c_read_reg(HTS221_ADDR_7B, 0x33);
    uint8_t msb        = i2c_read_reg(HTS221_ADDR_7B, 0x35);
    T0_degC = ((msb & 0x03) << 8 | T0_degC_x8) / 8.0f;
    T1_degC = ((msb & 0x0C) << 6 | T1_degC_x8) / 8.0f;

    i2c_read_buf(HTS221_ADDR_7B, 0x3C | AI, b, 2);
    T0_OUT = (int16_t)(b[1] << 8 | b[0]);
    i2c_read_buf(HTS221_ADDR_7B, 0x3E | AI, b, 2);
    T1_OUT = (int16_t)(b[1] << 8 | b[0]);

    return 0;
}

float hts221_read_humidity(void)
{
    uint8_t buf[2];
    i2c_read_buf(HTS221_ADDR_7B, 0x28 | AI, buf, 2);
    int16_t H_T0_OUT = (int16_t)(buf[1] << 8 | buf[0]);

    float rh = H0_rH + (H1_rH - H0_rH) *
              (float)(H_T0_OUT - H0_T0_OUT) /
              (float)(H1_T0_OUT - H0_T0_OUT);

    if (rh < 0.f)   rh = 0.f;
    if (rh > 100.f) rh = 100.f;
    return rh;
}

float hts221_read_temperature(void)
{
    uint8_t buf[2];
    i2c_read_buf(HTS221_ADDR_7B, 0x2A | AI, buf, 2);
    int16_t T_OUT = (int16_t)(buf[1] << 8 | buf[0]);

    return T0_degC + (T1_degC - T0_degC) *
           (float)(T_OUT - T0_OUT) /
           (float)(T1_OUT - T0_OUT);
}
