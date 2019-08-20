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
 * @file    m_drv_i2c.h
 * @author  mCube
 * @date    25 May 2017
 * @brief   Header file of I2C driver interface based on Nordic nrf52832.
 * @see     http://www.mcubemems.com
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#ifndef _M_DRV_I2C_H_
#define _M_DRV_I2C_H_

#include "m_drv_mc_utility.h"

/* User implemented functions */
   
/** Initialize I2C bus.
 *
 * This is platform dependent. 
 */
void M_DRV_I2C_Init(void);

/** Write data to sensor through I2C bus.
 *
 * @param[in] slaveAdd The 7bit I2C slave address for the sensor to be accessed.
 * @param[in] regAdd Sensor register offset value.
 * @param[in] *data The data to be written to the sensor.  
 * @param[in] length The number of sensor bytes to be written. 
 * @return mCubeResult_t indicating success or failure.
 */
int M_DRV_I2C_Write(uint8_t bI2CAddr, uint8_t bRegAddr, uint8_t *pbRegDataBuf, uint8_t bLength);

/** Read/poll data from sensor through I2C bus.
 *
 * General poll function for chip data information.
 * The regAdd parameter is the starting register offset where
 * the read should begin, and length is the number of bytes
 * that should be read starting from the regAdd offset.  Data
 * will be returned to the caller via the recvData pointer into
 * a buffer that is at least as large as length.
 
 * @param[in] slaveAdd The 7bit I2C slave address for the sensor to be accessed.
 * @param[in] regAdd The register offset from which to start polling data.
 * @param[in] length The number of bytes to be polled.
 * @param[out] *recvData A pointer to a storage array for receiving the
 *                       bytes read from the bus.
 * @return mCubeResult_t indicating success or failure.
 */
int M_DRV_I2C_Read(uint8_t bI2CAddr, uint8_t bRegAddr, uint8_t *pbDataBuf, uint8_t bLength);

#endif /* MCUBE_I2C_H */
