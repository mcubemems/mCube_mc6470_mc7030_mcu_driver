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

#ifndef _M_DRV_MC3XXX_H_
    #define _M_DRV_MC3XXX_H_

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define M_DRV_MC3XXX_RETCODE_SUCCESS                 (0)
#define M_DRV_MC3XXX_RETCODE_ERROR_I2C               (-1)
#define M_DRV_MC3XXX_RETCODE_ERROR_NULL_POINTER      (-2)
#define M_DRV_MC3XXX_RETCODE_ERROR_STATUS            (-3)
#define M_DRV_MC3XXX_RETCODE_ERROR_SETUP             (-4)
#define M_DRV_MC3XXX_RETCODE_ERROR_GET_DATA          (-5)
#define M_DRV_MC3XXX_RETCODE_ERROR_IDENTIFICATION    (-6)

#define M_DRV_MC3XXX_AXIS_X      0
#define M_DRV_MC3XXX_AXIS_Y      1
#define M_DRV_MC3XXX_AXIS_Z      2
#define M_DRV_MC3XXX_AXES_NUM    3

#define M_DRV_MC3XXX_MODE_SLEEP      (0x00)
#define M_DRV_MC3XXX_MODE_WAKE       (0x01)
#define M_DRV_MC3XXX_MODE_STANDBY    (0x03)

#define M_DRV_MC3XXX_REG_MAP_SIZE    64

#define ACC_PRODUCT_CODE_REG		0x3B

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/
typedef struct
{
    short    waRaw[M_DRV_MC3XXX_AXES_NUM];
    short    waAccelMillig[M_DRV_MC3XXX_AXES_NUM];
}   S_M_DRV_MC3XXX_SensorData;

/*******************************************************************************
 *** EXTERNAL FUNCTION
 *******************************************************************************/
extern int    M_DRV_MC3XXX_Init(void);
extern int    M_DRV_MC3XXX_SetMode(unsigned char bMode);
extern int    M_DRV_MC3XXX_ReadSensorData(float faOutput[M_DRV_MC3XXX_AXES_NUM]);
extern int    M_DRV_MC3XXX_FP_ReadSensorData(short _waRaw[M_DRV_MC3XXX_AXES_NUM]);
extern int    M_DRV_MC3XXX_ReadRegMap(unsigned char baRegMap[M_DRV_MC3XXX_REG_MAP_SIZE]);
extern int    M_DRV_MC3XXX_SensorValidation(unsigned char *pCode);
#endif    // END of _M_DRV_MC3XXX_H_

