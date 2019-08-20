/*****************************************************************************
 *
 * Copyright (c) 2017 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/
/**
 * @file    m_drv_i2c.c
 * @author  mCube
 * @date    25 May 2017
 * @brief   C file of I2C driver interface based on Nordic nrf52832.
 * @see     http://www.mcubemems.com
 */

/* platform function includes */
#include "m_drv_i2c.h"

void M_DRV_I2C_Init(void)
{
	//platform bus initilization
}

int M_DRV_I2C_Write(uint8_t bI2CAddr, uint8_t bRegAddr, uint8_t *pbRegDataBuf, uint8_t bLength)
{
   //platform write function
   return E_M_DRV_MCUBE_RATCODE_SUCCESS;
}

int M_DRV_I2C_Read(uint8_t bI2CAddr, uint8_t bRegAddr, uint8_t *pbDataBuf, uint8_t bLength)
{
   //platform read function
   return E_M_DRV_MCUBE_RATCODE_SUCCESS;   
}
