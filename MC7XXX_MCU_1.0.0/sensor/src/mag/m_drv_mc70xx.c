/*****************************************************************************
 *
 * Copyright (c) 2014 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/
#include <stdint.h>
#include "m_drv_i2c.h"
#include "m_drv_mc70xx.h"
#include "m_drv_mc_utility.h"

#include "m_drv_console.h"

/*******************************************************************************
 *** CONFIGURATION
 *******************************************************************************/
#define M_DRV_MC70XX_I2C_ADDR    (0x0C)

//#define M_DRV_MC70XX_SINGLE_SHOT_MODE
#define M_DRV_MC70XX_CONT_READ_MOD

typedef enum
{
    E_M_DRV_MC70XX_RANGE_14BIT = 0x00,
    E_M_DRV_MC70XX_RANGE_15BIT = 0x10
}   E_M_DRV_MC70XX_RANGE;

#define M_DRV_MC70XX_RANGE    E_M_DRV_MC70XX_RANGE_15BIT

#define M_DRV_MC70XX_ORIENTATION_MAP    E_M_DRV_UTIL_ORIENTATION_TOP_LEFT_DOWN

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define M_DRV_MC70XX_I2C_WRITE_ADDR    ((M_DRV_MC70XX_I2C_ADDR << 1) | 0x00)
#define M_DRV_MC70XX_I2C_READ_ADDR     ((M_DRV_MC70XX_I2C_ADDR << 1) | 0x01)

#define M_DRV_MC70XX_SENSITIVITY       (0.15f)

/*******************************************************************************
 *** MACRO
 *******************************************************************************/
#define _M_DRV_MC70XX_I2C_WRITE(bRegAddr, pbDataBuf)            M_DRV_I2C_Write(M_DRV_MC70XX_I2C_ADDR, bRegAddr, pbDataBuf, 1)
//#define _M_DRV_MC70XX_I2C_WRITE(bRegAddr, pbDataBuf)			mCube_I2C_RegWrite(M_DRV_MC70XX_I2C_ADDR, bRegAddr, pbDataBuf, 1)
#define _M_DRV_MC70XX_I2C_READ(bRegAddr, pbDataBuf, bLength)    M_DRV_I2C_Read(M_DRV_MC70XX_I2C_ADDR, bRegAddr, pbDataBuf, bLength)
//#define _M_DRV_MC70XX_I2C_READ(bRegAddr, pbDataBuf, bLength)    mCube_I2C_RegRead(M_DRV_MC70XX_I2C_ADDR, bRegAddr, bLength, pbDataBuf)

/*******************************************************************************
 *** FUNCTION
 *******************************************************************************/

/*********************************************************************
 *** _M_DRV_MC70XX_InitSR
 *********************************************************************/
static int    _M_DRV_MC70XX_InitSR(void)
{
    unsigned char    _bSR = 0x80;

    return (_M_DRV_MC70XX_I2C_WRITE(0x1D, &_bSR));
}

#ifdef M_DRV_MC70XX_SINGLE_SHOT_MODE
/*********************************************************************
 *** _M_DRV_MC70XX_SetSingleShotMode
 *********************************************************************/
static int    _M_DRV_MC70XX_SetSingleShotMode(void)
{
    unsigned char    _bSingleShotMode = (0x80 | 0x18 | 0x02);

    return (_M_DRV_MC70XX_I2C_WRITE(0x1B, &_bSingleShotMode));
}

/*********************************************************************
 *** _M_DRV_MC70XX_SetForceState
 *********************************************************************/
static int    _M_DRV_MC70XX_SetForceState(void)
{
    unsigned char    _bForceState = 0x40;

    return (_M_DRV_MC70XX_I2C_WRITE(0x1D, &_bForceState));
}
#endif

#ifdef M_DRV_MC70XX_CONT_READ_MOD
/*********************************************************************
 *** _M_DRV_MC70XX_SetContMode
 *********************************************************************/
static int    _M_DRV_MC70XX_SetContMode(void)
{
    unsigned char    _bContMode = (0x80 | 0x18);

    return (_M_DRV_MC70XX_I2C_WRITE(0x1B, &_bContMode));
}
#endif

/*********************************************************************
 *** _M_DRV_MC70XX_SetDynamicRange
 *********************************************************************/
static int    _M_DRV_MC70XX_SetDynamicRange(void)
{
    unsigned char    _bDynamicRange = (0x80 | M_DRV_MC70XX_RANGE);

    return (_M_DRV_MC70XX_I2C_WRITE(0x1E, &_bDynamicRange));
}

/*********************************************************************
 *** _M_DRV_MC70XX_StartSingleRead
 *********************************************************************/
#ifdef M_DRV_MC70XX_SINGLE_SHOT_MODE
static void    _M_DRV_MC70XX_StartSingleRead(void)
{
    _M_DRV_MC70XX_SetForceState();
    _M_DRV_MC70XX_SetDynamicRange();
    _M_DRV_MC70XX_SetSingleShotMode();
}
#endif

/*********************************************************************
 *** _M_DRV_MC70XX_StartContRead
 *********************************************************************/
#ifdef M_DRV_MC70XX_CONT_READ_MOD
static void    _M_DRV_MC70XX_StartContRead(void)
{
    _M_DRV_MC70XX_SetDynamicRange();
    _M_DRV_MC70XX_SetContMode();
}
#endif

/*********************************************************************
 *** M_DRV_MC70XX_Init
 *********************************************************************/
void    M_DRV_MC70XX_Init(void)
{
    uint64_t    _lDelayCount;

	M_PRINTF("[M_DRV_MC70XX_Init]\r\n"); //DEBUG USE

	_M_DRV_MC70XX_InitSR();

    #ifdef M_DRV_MC70XX_SINGLE_SHOT_MODE
        _M_DRV_MC70XX_StartSingleRead();
    #endif

    #ifdef M_DRV_MC70XX_CONT_READ_MOD
        _M_DRV_MC70XX_SetContMode();
    #endif
}

/*********************************************************************
 *** M_DRV_MC70XX_ReadSensorData
 *********************************************************************/
void    M_DRV_MC70XX_ReadSensorData(float faOutput[M_DRV_MC70XX_AXES_NUM])
{
    int              _nWaitDataReadyCount          = 0;
    unsigned char    _bDataReadyStatus             = 0;
    unsigned char    _bDaDataBuf[6]                = { 0 };
    short            _waRaw[M_DRV_MC70XX_AXES_NUM] = { 0 };

    const S_M_DRV_MC_UTIL_ORIENTATIONREMAP_TYPEDEF   *_ptOrienMap = &g_MDrvUtilOrientationReMap[M_DRV_MC70XX_ORIENTATION_MAP];

    #ifdef M_DRV_MC70XX_SINGLE_SHOT_MODE
        _M_DRV_MC70XX_StartSingleRead();

        while (1)
        {
            _nWaitDataReadyCount++;

            _M_DRV_MC70XX_I2C_READ(0x18, &_bDataReadyStatus, 1);

            if (_bDataReadyStatus & 0x40)
                break;
        
            if (_nWaitDataReadyCount > 64)
                break;
        }
    #endif

    _M_DRV_MC70XX_I2C_READ(0x10, _bDaDataBuf, 6);

    _waRaw[M_DRV_MC70XX_AXIS_X] = (_bDaDataBuf[0] | (_bDaDataBuf[1] << 8));
    _waRaw[M_DRV_MC70XX_AXIS_Y] = (_bDaDataBuf[2] | (_bDaDataBuf[3] << 8));
    _waRaw[M_DRV_MC70XX_AXIS_Z] = (_bDaDataBuf[4] | (_bDaDataBuf[5] << 8));

    faOutput[M_DRV_MC70XX_AXIS_X] = ((float)(_ptOrienMap->bSign[M_DRV_MC70XX_AXIS_X] * _waRaw[_ptOrienMap->bMap[M_DRV_MC70XX_AXIS_X]]));
    faOutput[M_DRV_MC70XX_AXIS_Y] = ((float)(_ptOrienMap->bSign[M_DRV_MC70XX_AXIS_Y] * _waRaw[_ptOrienMap->bMap[M_DRV_MC70XX_AXIS_Y]]));
    faOutput[M_DRV_MC70XX_AXIS_Z] = ((float)(_ptOrienMap->bSign[M_DRV_MC70XX_AXIS_Z] * _waRaw[_ptOrienMap->bMap[M_DRV_MC70XX_AXIS_Z]]));

    faOutput[M_DRV_MC70XX_AXIS_X] = (faOutput[M_DRV_MC70XX_AXIS_X] * M_DRV_MC70XX_SENSITIVITY);
    faOutput[M_DRV_MC70XX_AXIS_Y] = (faOutput[M_DRV_MC70XX_AXIS_Y] * M_DRV_MC70XX_SENSITIVITY);
    faOutput[M_DRV_MC70XX_AXIS_Z] = (faOutput[M_DRV_MC70XX_AXIS_Z] * M_DRV_MC70XX_SENSITIVITY);
}

/*********************************************************************
 *** M_DRV_MC70XX_SensorValidation
 *********************************************************************/
int    M_DRV_MC70XX_SensorValidation(unsigned char *pCode)
{
	 _M_DRV_MC70XX_I2C_READ(MAG_PRODUCT_CODE_REG, pCode, 1);	
	return M_DRV_MC70XX_RETCODE_SUCCESS;
}

#if 0    // for debug
/*********************************************************************
 *** M_DRV_MC70XX_ReadRegMap
 *********************************************************************/
void    M_DRV_MC70XX_ReadRegMap(unsigned char baRegMap[M_DRV_MC70XX_REG_MAP_SIZE])
{
    uint8_t    _bIndex = 0;

    for (_bIndex = 0; _bIndex < M_DRV_MC70XX_REG_MAP_SIZE; _bIndex++)
        _M_DRV_MC70XX_I2C_READ(_bIndex, &baRegMap[_bIndex], 1);
}
#endif

