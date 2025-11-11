/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "stm32f3xx_ll_utils.h"   /* LL_GetTick */

/* USER CODE BEGIN 0 */
#define I2C_TIMEOUT_MS  10

static inline uint32_t tick_ms(void){ return LL_GetTick(); }

/* wait for LL flag or timeout; 0 ok, -1 NACK, -2 BERR, -3 TO */
static int wait_flag_or_timeout(uint32_t (*flag_fn)(const I2C_TypeDef*),
                                I2C_TypeDef *i2c, uint32_t ms)
{
    uint32_t t0 = tick_ms();
    while(!flag_fn(i2c)){
        if (LL_I2C_IsActiveFlag_NACK(i2c)) { LL_I2C_ClearFlag_NACK(i2c); return -1; }
        if (LL_I2C_IsActiveFlag_BERR(i2c)) { LL_I2C_ClearFlag_BERR(i2c); return -2; }
        if ((tick_ms()-t0) > ms) return -3;
    }
    return 0;
}
/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00201D2B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */
  LL_I2C_Enable(I2C1);
  /* USER CODE END I2C1_Init 2 */

}

/* USER CODE BEGIN 1 */

void i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val)
{
    LL_I2C_HandleTransfer(I2C1, (addr7 << 1), LL_I2C_ADDRSLAVE_7BIT,
                          2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    if (wait_flag_or_timeout(LL_I2C_IsActiveFlag_TXIS, I2C1, I2C_TIMEOUT_MS) < 0) return;
    LL_I2C_TransmitData8(I2C1, reg);

    if (wait_flag_or_timeout(LL_I2C_IsActiveFlag_TXIS, I2C1, I2C_TIMEOUT_MS) < 0) return;
    LL_I2C_TransmitData8(I2C1, val);

    (void)wait_flag_or_timeout(LL_I2C_IsActiveFlag_STOP, I2C1, I2C_TIMEOUT_MS);
    LL_I2C_ClearFlag_STOP(I2C1);
}

uint8_t i2c_read_reg(uint8_t addr7, uint8_t reg)
{
    uint8_t d = 0;

    /* send reg */
    LL_I2C_HandleTransfer(I2C1, (addr7 << 1), LL_I2C_ADDRSLAVE_7BIT,
                          1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    if (wait_flag_or_timeout(LL_I2C_IsActiveFlag_TXIS, I2C1, I2C_TIMEOUT_MS) < 0) goto out;
    LL_I2C_TransmitData8(I2C1, reg);
    if (wait_flag_or_timeout(LL_I2C_IsActiveFlag_STOP, I2C1, I2C_TIMEOUT_MS) < 0) goto out;
    LL_I2C_ClearFlag_STOP(I2C1);

    /* read 1 byte */
    LL_I2C_HandleTransfer(I2C1, (addr7 << 1), LL_I2C_ADDRSLAVE_7BIT,
                          1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    if (wait_flag_or_timeout(LL_I2C_IsActiveFlag_RXNE, I2C1, I2C_TIMEOUT_MS) < 0) goto out;
    d = LL_I2C_ReceiveData8(I2C1);
    (void)wait_flag_or_timeout(LL_I2C_IsActiveFlag_STOP, I2C1, I2C_TIMEOUT_MS);
    LL_I2C_ClearFlag_STOP(I2C1);
out:
    return d;
}

/* read N bytes from reg (auto-inc if reg|0x80) */
void i2c_read_buf(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* send start register */
    LL_I2C_HandleTransfer(I2C1, (addr7 << 1), LL_I2C_ADDRSLAVE_7BIT,
                          1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    if (wait_flag_or_timeout(LL_I2C_IsActiveFlag_TXIS, I2C1, I2C_TIMEOUT_MS) < 0) return;
    LL_I2C_TransmitData8(I2C1, reg);
    if (wait_flag_or_timeout(LL_I2C_IsActiveFlag_STOP, I2C1, I2C_TIMEOUT_MS) < 0) return;
    LL_I2C_ClearFlag_STOP(I2C1);

    /* read bytes */
    LL_I2C_HandleTransfer(I2C1, (addr7 << 1), LL_I2C_ADDRSLAVE_7BIT,
                          len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    for (uint8_t i = 0; i < len; i++) {
        if (wait_flag_or_timeout(LL_I2C_IsActiveFlag_RXNE, I2C1, I2C_TIMEOUT_MS) < 0) return;
        buf[i] = LL_I2C_ReceiveData8(I2C1);
    }
    (void)wait_flag_or_timeout(LL_I2C_IsActiveFlag_STOP, I2C1, I2C_TIMEOUT_MS);
    LL_I2C_ClearFlag_STOP(I2C1);
}

/* USER CODE END 1 */
