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
#include "m_drv_mc3xxx.h"
#include "m_drv_mc_utility.h"

#include "m_drv_console.h"

/*******************************************************************************
 *** CONFIGURATION
 *******************************************************************************/
#define M_DRV_MC3XXX_I2C_ADDR    (0x4C)

typedef enum
{
    E_M_DRV_MC3XXX_RESOLUTION_LOW = 0,
    E_M_DRV_MC3XXX_RESOLUTION_8BIT_1p5G,
    E_M_DRV_MC3XXX_RESOLUTION_8BIT_2G,

    E_M_DRV_MC3XXX_RESOLUTION_HIGH,
    E_M_DRV_MC3XXX_RESOLUTION_14BIT_8G,
	E_M_DRV_MC3XXX_RESOLUTION_10BIT_2G,
}   E_M_DRV_MC3XXX_RESOLUTION;

#define M_DRV_MC3XXX_RESOLUTION    E_M_DRV_MC3XXX_RESOLUTION_14BIT_8G //E_M_DRV_MC3XXX_RESOLUTION_10BIT_2G

#define M_DRV_MC3XXX_ORIENTATION_MAP    E_M_DRV_UTIL_ORIENTATION_TOP_LEFT_DOWN

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define M_DRV_MC3XXX_I2C_WRITE_ADDR    ((M_DRV_MC3XXX_I2C_ADDR << 1) | 0x00)
#define M_DRV_MC3XXX_I2C_READ_ADDR     ((M_DRV_MC3XXX_I2C_ADDR << 1) | 0x01)

//================================
#define M_DRV_MC3XXX_PCODE_3210     0x90
#define M_DRV_MC3XXX_PCODE_3230     0x19
#define M_DRV_MC3XXX_PCODE_3250     0x88
#define M_DRV_MC3XXX_PCODE_3410     0xA8
#define M_DRV_MC3XXX_PCODE_3410N    0xB8
#define M_DRV_MC3XXX_PCODE_3430     0x29
#define M_DRV_MC3XXX_PCODE_3430N    0x39
#define M_DRV_MC3XXX_PCODE_3510     0x40
#define M_DRV_MC3XXX_PCODE_3530     0x30
#define M_DRV_MC3XXX_PCODE_3216     0x10
#define M_DRV_MC3XXX_PCODE_3236     0x60
#define M_DRV_MC3XXX_PCODE_7030     0x00

#define M_DRV_MC3XXX_PCODE_RESERVE_1    0x20
#define M_DRV_MC3XXX_PCODE_RESERVE_2    0x11
#define M_DRV_MC3XXX_PCODE_RESERVE_3    0x21
#define M_DRV_MC3XXX_PCODE_RESERVE_4    0x61
#define M_DRV_MC3XXX_PCODE_RESERVE_5    0xA0
#define M_DRV_MC3XXX_PCODE_RESERVE_6    0xE0
#define M_DRV_MC3XXX_PCODE_RESERVE_7    0x91
#define M_DRV_MC3XXX_PCODE_RESERVE_8    0xA1
#define M_DRV_MC3XXX_PCODE_RESERVE_9    0xE1

#define M_DRV_MC3XXX_PCODE_RESERVE_10    0x99

//================================
#define M_DRV_MC3XXX_GRAVITY          (9.807f)
#define M_DRV_MC3XXX_GAIN_14BIT_8G    1024
#define M_DRV_MC3XXX_GAIN_10BIT_2G 	   256
#define M_DRV_MC3XXX_GAIN_8BIT_2G 	  	64

/*******************************************************************************
 *** MACRO
 *******************************************************************************/
#define _M_DRV_MC3XXX_I2C_WRITE(bRegAddr, pbDataBuf)            M_DRV_I2C_Write(M_DRV_MC3XXX_I2C_ADDR, bRegAddr, pbDataBuf, 1)
//#define _M_DRV_MC3XXX_I2C_WRITE(bRegAddr, pbDataBuf)            M_DRV_I2C_Write(M_DRV_MC3XXX_I2C_WRITE_ADDR, bRegAddr, pbDataBuf, 1)
#define _M_DRV_MC3XXX_I2C_READ(bRegAddr, pbDataBuf, bLength)    M_DRV_I2C_Read(M_DRV_MC3XXX_I2C_ADDR, bRegAddr, pbDataBuf, bLength)
//#define _M_DRV_MC3XXX_I2C_READ(bRegAddr, pbDataBuf, bLength)    M_DRV_I2C_Read(M_DRV_MC3XXX_I2C_READ_ADDR, bRegAddr, pbDataBuf, bLength)

/*******************************************************************************
 *** FUNCTION
 *******************************************************************************/
/*********************************************************************
 *** _M_DRV_MC3XXX_Delay
 *********************************************************************/
void    _M_DRV_MC3XXX_Delay(void)
{
    uint64_t    _lDelayCount;

    for (_lDelayCount = 0; _lDelayCount < 10240; _lDelayCount++)
    {
        // dummy loop
    }
}

/*********************************************************************
 *** M_DRV_MC3XXX_SetMode
 *********************************************************************/
int    M_DRV_MC3XXX_SetMode(unsigned char bMode)
{
    unsigned char    _bMode = 0;

    _bMode = bMode;
	
	M_PRINTF("[M_DRV_MC3XXX_SetMode]\r\n"); //DEBUG USE
	
    _M_DRV_MC3XXX_I2C_WRITE(0x07, &_bMode);

    if (M_DRV_MC3XXX_MODE_WAKE == bMode)
        _M_DRV_MC3XXX_Delay();

    return (M_DRV_MC3XXX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** _M_DRV_MC3XXX_ReadChipCode
 *********************************************************************/
static void    _M_DRV_MC3XXX_ReadChipCode(unsigned char *pbHwID, unsigned char *pbPCODE)
{
    _M_DRV_MC3XXX_I2C_READ(0x18, pbHwID, 1);
    _M_DRV_MC3XXX_I2C_READ(0x3B, pbPCODE, 1);
}

/*****************************************
 *** _M_DRV_MC3XXX_ValidateSensorIC
 *****************************************/
static int    _M_DRV_MC3XXX_ValidateSensorIC(void)
{
    unsigned char    _bHwID  = 0;
    unsigned char    _bPCODE = 0;

    _M_DRV_MC3XXX_ReadChipCode(&_bHwID, &_bPCODE);

    if (   (0x01 == _bHwID)
        || (0x03 == _bHwID)
        || ((0x04 <= _bHwID) && (_bHwID <= 0x0F)))
    {
        if ((M_DRV_MC3XXX_PCODE_3210 == _bPCODE) || (M_DRV_MC3XXX_PCODE_3230 == _bPCODE) || (M_DRV_MC3XXX_PCODE_3250 == _bPCODE))
            return (M_DRV_MC3XXX_RETCODE_SUCCESS);
    }
    else if (   (0x02 == _bHwID)
             || (0x21 == _bHwID)
             || ((0x10 <= _bHwID) && (_bHwID <= 0x1F)))
    {
        if (   (M_DRV_MC3XXX_PCODE_3210 == _bPCODE) || (M_DRV_MC3XXX_PCODE_3230  == _bPCODE)
            || (M_DRV_MC3XXX_PCODE_3250 == _bPCODE)
            || (M_DRV_MC3XXX_PCODE_3410 == _bPCODE) || (M_DRV_MC3XXX_PCODE_3410N == _bPCODE)
            || (M_DRV_MC3XXX_PCODE_3430 == _bPCODE) || (M_DRV_MC3XXX_PCODE_3430N == _bPCODE))
        {
            return (M_DRV_MC3XXX_RETCODE_SUCCESS);
        }
    }
    else if ((0xC0 <= _bHwID) && (_bHwID <= 0xCF))
    {
        _bPCODE = (_bPCODE & 0x71);

        if ((M_DRV_MC3XXX_PCODE_3510 == _bPCODE) || (M_DRV_MC3XXX_PCODE_3530 == _bPCODE))
            return (M_DRV_MC3XXX_RETCODE_SUCCESS);
    }
    else if ((0x20 == _bHwID) || ((0x22 <= _bHwID) && (_bHwID <= 0x2F)))
    {
        _bPCODE = (_bPCODE & 0xF1);

        if (   (M_DRV_MC3XXX_PCODE_3210      == _bPCODE) || (M_DRV_MC3XXX_PCODE_3216      == _bPCODE) || (M_DRV_MC3XXX_PCODE_3236      == _bPCODE)
            || (M_DRV_MC3XXX_PCODE_7030      == _bPCODE)
            || (M_DRV_MC3XXX_PCODE_RESERVE_1 == _bPCODE) || (M_DRV_MC3XXX_PCODE_RESERVE_2 == _bPCODE) || (M_DRV_MC3XXX_PCODE_RESERVE_3 == _bPCODE)
            || (M_DRV_MC3XXX_PCODE_RESERVE_4 == _bPCODE) || (M_DRV_MC3XXX_PCODE_RESERVE_5 == _bPCODE) || (M_DRV_MC3XXX_PCODE_RESERVE_6 == _bPCODE)
            || (M_DRV_MC3XXX_PCODE_RESERVE_7 == _bPCODE) || (M_DRV_MC3XXX_PCODE_RESERVE_8 == _bPCODE) || (M_DRV_MC3XXX_PCODE_RESERVE_9 == _bPCODE))
        {
            return (M_DRV_MC3XXX_RETCODE_SUCCESS);
        }
    }

    return (M_DRV_MC3XXX_RETCODE_ERROR_IDENTIFICATION);
}

/*****************************************
 *** _M_DRV_MC3XXX_SetSampleRate
 *****************************************/
static void    _M_DRV_MC3XXX_SetSampleRate(void)
{
    unsigned char    _bHwID          = 0;
    unsigned char    _bPCODE         = 0;
    unsigned char    _bCfgSampleRate = 0;
	M_PRINTF("[_M_DRV_MC3XXX_SetSampleRate]\r\n"); //DEBUG USE

    _bCfgSampleRate = 0x08;

    _M_DRV_MC3XXX_I2C_WRITE(0x08, &_bCfgSampleRate);
}

/*****************************************
 *** _M_DRV_MC3XXX_ConfigRegRange
 *****************************************/
static void    _M_DRV_MC3XXX_ConfigRegRange(void)
{
    unsigned char    _bHwID               = 0;
    unsigned char    _bPCODE              = 0;
    unsigned char    _bPrjCfgResolution   = M_DRV_MC3XXX_RESOLUTION;
    unsigned char    _bCfgRangeResolution = 0x3F;

	M_PRINTF("[_M_DRV_MC3XXX_ConfigRegRange]\r\n"); //DEBUG USE
    _M_DRV_MC3XXX_ReadChipCode(&_bHwID, &_bPCODE);

    if (_bPrjCfgResolution < E_M_DRV_MC3XXX_RESOLUTION_HIGH)
        _bCfgRangeResolution = 0x32;

    if (   ((0xC0 <= _bHwID) && (_bHwID <= 0xCF))
        || ((0x20 == _bHwID) || ((0x22 <= _bHwID) && (_bHwID <= 0x2F))))
    {
        if (_bPrjCfgResolution < E_M_DRV_MC3XXX_RESOLUTION_HIGH)
            _bCfgRangeResolution = 0x02;
        else
            _bCfgRangeResolution = 0x25;
    }
	else	
		_bCfgRangeResolution = 0x33;	

	M_PRINTF("0x20 : %x\r\n", _bCfgRangeResolution); //DEBUG USE
    _M_DRV_MC3XXX_I2C_WRITE(0x20, &_bCfgRangeResolution);
}

/*********************************************************************
 *** M_DRV_MC3XXX_Init
 *********************************************************************/
int    M_DRV_MC3XXX_Init(void)
{
    M_DRV_MC3XXX_SetMode(M_DRV_MC3XXX_MODE_STANDBY);

    if (M_DRV_MC3XXX_RETCODE_SUCCESS != _M_DRV_MC3XXX_ValidateSensorIC())
        return (M_DRV_MC3XXX_RETCODE_ERROR_IDENTIFICATION);

    _M_DRV_MC3XXX_SetSampleRate();
    _M_DRV_MC3XXX_ConfigRegRange();

    unsigned char    _bData = 0;
    _M_DRV_MC3XXX_I2C_READ(0x08, &_bData, 1);
    M_PRINTF("0x08 = %d\r\n",_bData); //DEBUG USE

    M_DRV_MC3XXX_SetMode(M_DRV_MC3XXX_MODE_WAKE);

    return (M_DRV_MC3XXX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3XXX_ReadSensorData
 *********************************************************************/
int    M_DRV_MC3XXX_ReadSensorData(float faOutput[M_DRV_MC3XXX_AXES_NUM])
{
    short            _waRaw[M_DRV_MC3XXX_AXES_NUM] = { 0 };
    short Atmp[3]={0};
    unsigned char    _baData[6]                    = { 0 };
    unsigned char    _bPrjCfgResolution            = M_DRV_MC3XXX_RESOLUTION;

    const S_M_DRV_MC_UTIL_ORIENTATIONREMAP_TYPEDEF   *_ptOrienMap = &g_MDrvUtilOrientationReMap[M_DRV_MC3XXX_ORIENTATION_MAP];

    if (_bPrjCfgResolution < E_M_DRV_MC3XXX_RESOLUTION_HIGH)
    {
        _M_DRV_MC3XXX_I2C_READ(0x00, _baData, 3);

        _waRaw[M_DRV_MC3XXX_AXIS_X] = ((signed char) _baData[0]);
        _waRaw[M_DRV_MC3XXX_AXIS_Y] = ((signed char) _baData[1]);
        _waRaw[M_DRV_MC3XXX_AXIS_Z] = ((signed char) _baData[2]);
    }
    else
    {
        _M_DRV_MC3XXX_I2C_READ(0x0D, _baData, 6);

        _waRaw[M_DRV_MC3XXX_AXIS_X] = ((signed short) ((_baData[0]) | (_baData[1] << 8)));
        _waRaw[M_DRV_MC3XXX_AXIS_Y] = ((signed short) ((_baData[2]) | (_baData[3] << 8)));
        _waRaw[M_DRV_MC3XXX_AXIS_Z] = ((signed short) ((_baData[4]) | (_baData[5] << 8)));
    }

    faOutput[M_DRV_MC3XXX_AXIS_X] = ((float)(_ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_X] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_X]]));
    faOutput[M_DRV_MC3XXX_AXIS_Y] = ((float)(_ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_Y] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_Y]]));
    faOutput[M_DRV_MC3XXX_AXIS_Z] = ((float)(_ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_Z] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_Z]]));

    Atmp[M_DRV_MC3XXX_AXIS_X] = _ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_X] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_X]];
	Atmp[M_DRV_MC3XXX_AXIS_Y] = _ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_Y] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_Y]];
	Atmp[M_DRV_MC3XXX_AXIS_Z] = _ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_Z] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_Z]];

    switch(_bPrjCfgResolution)
    {
    case E_M_DRV_MC3XXX_RESOLUTION_14BIT_8G:
         faOutput[M_DRV_MC3XXX_AXIS_X] = ((faOutput[M_DRV_MC3XXX_AXIS_X] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_14BIT_8G);
         faOutput[M_DRV_MC3XXX_AXIS_Y] = ((faOutput[M_DRV_MC3XXX_AXIS_Y] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_14BIT_8G);
         faOutput[M_DRV_MC3XXX_AXIS_Z] = ((faOutput[M_DRV_MC3XXX_AXIS_Z] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_14BIT_8G);
         break;
    case E_M_DRV_MC3XXX_RESOLUTION_8BIT_2G:
         faOutput[M_DRV_MC3XXX_AXIS_X] = ((faOutput[M_DRV_MC3XXX_AXIS_X] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_8BIT_2G);
         faOutput[M_DRV_MC3XXX_AXIS_Y] = ((faOutput[M_DRV_MC3XXX_AXIS_Y] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_8BIT_2G);
         faOutput[M_DRV_MC3XXX_AXIS_Z] = ((faOutput[M_DRV_MC3XXX_AXIS_Z] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_8BIT_2G);
         break;
	case E_M_DRV_MC3XXX_RESOLUTION_10BIT_2G:
		 faOutput[M_DRV_MC3XXX_AXIS_X] = ((faOutput[M_DRV_MC3XXX_AXIS_X] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_10BIT_2G);
		 faOutput[M_DRV_MC3XXX_AXIS_Y] = ((faOutput[M_DRV_MC3XXX_AXIS_Y] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_10BIT_2G);
		 faOutput[M_DRV_MC3XXX_AXIS_Z] = ((faOutput[M_DRV_MC3XXX_AXIS_Z] * M_DRV_MC3XXX_GRAVITY) / M_DRV_MC3XXX_GAIN_10BIT_2G);
		 break;
    }

    return (M_DRV_MC3XXX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3XXX_ReadSensorData, for FixPt algorithm
 *********************************************************************/
int    M_DRV_MC3XXX_FP_ReadSensorData(short _fpRAW[M_DRV_MC3XXX_AXES_NUM])
{
    short            _waRaw[M_DRV_MC3XXX_AXES_NUM] = { 0 };
    unsigned char    _baData[6]                    = { 0 };
    unsigned char    _bPrjCfgResolution            = M_DRV_MC3XXX_RESOLUTION;

    const S_M_DRV_MC_UTIL_ORIENTATIONREMAP_TYPEDEF   *_ptOrienMap = &g_MDrvUtilOrientationReMap[M_DRV_MC3XXX_ORIENTATION_MAP];


    if (_bPrjCfgResolution < E_M_DRV_MC3XXX_RESOLUTION_HIGH)
    {
        _M_DRV_MC3XXX_I2C_READ(0x00, _baData, 3);

        _waRaw[M_DRV_MC3XXX_AXIS_X] = ((signed char) _baData[0]);
        _waRaw[M_DRV_MC3XXX_AXIS_Y] = ((signed char) _baData[1]);
        _waRaw[M_DRV_MC3XXX_AXIS_Z] = ((signed char) _baData[2]);
    }
    else
    {
        _M_DRV_MC3XXX_I2C_READ(0x0D, _baData, 6);

        _waRaw[M_DRV_MC3XXX_AXIS_X] = ((signed short) ((_baData[0]) | (_baData[1] << 8)));
        _waRaw[M_DRV_MC3XXX_AXIS_Y] = ((signed short) ((_baData[2]) | (_baData[3] << 8)));
        _waRaw[M_DRV_MC3XXX_AXIS_Z] = ((signed short) ((_baData[4]) | (_baData[5] << 8)));
    }

    _fpRAW[M_DRV_MC3XXX_AXIS_X] = (short) (_ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_X] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_X]]);
    _fpRAW[M_DRV_MC3XXX_AXIS_Y] = (short) (_ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_Y] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_Y]]);
    _fpRAW[M_DRV_MC3XXX_AXIS_Z] = (short) (_ptOrienMap->bSign[M_DRV_MC3XXX_AXIS_Z] * _waRaw[_ptOrienMap->bMap[M_DRV_MC3XXX_AXIS_Z]]);

    return (M_DRV_MC3XXX_RETCODE_SUCCESS);
}

/*********************************************************************
 *** M_DRV_MC3XXX_SensorValidation
 *********************************************************************/
int    M_DRV_MC3XXX_SensorValidation(unsigned char *pCode)
{
	 _M_DRV_MC3XXX_I2C_READ(ACC_PRODUCT_CODE_REG, pCode, 1);	
	return M_DRV_MC3XXX_RETCODE_SUCCESS;
}

#if 0    // for debug
/*********************************************************************
 *** M_DRV_MC3XXX_ReadRegMap
 *********************************************************************/
int    M_DRV_MC3XXX_ReadRegMap(unsigned char baRegMap[M_DRV_MC3XXX_REG_MAP_SIZE])
{
    uint8_t    _bIndex = 0;

    for (_bIndex = 0; _bIndex < M_DRV_MC3XXX_REG_MAP_SIZE; _bIndex++)
        _M_DRV_MC3XXX_I2C_READ(_bIndex, &baRegMap[_bIndex], 1);

    return (M_DRV_MC3XXX_RETCODE_SUCCESS);
}
#endif
