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

#ifndef _M_DRV_MC70XX_H_
    #define _M_DRV_MC70XX_H_

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#if 1
#define M_DRV_MC70XX_RETCODE_SUCCESS                 (0)
#define M_DRV_MC70XX_RETCODE_ERROR_I2C               (-1)
#define M_DRV_MC70XX_RETCODE_ERROR_NULL_POINTER      (-2)
#define M_DRV_MC70XX_RETCODE_ERROR_STATUS            (-3)
#define M_DRV_MC70XX_RETCODE_ERROR_SETUP             (-4)
#define M_DRV_MC70XX_RETCODE_ERROR_GET_DATA          (-5)
#define M_DRV_MC70XX_RETCODE_ERROR_IDENTIFICATION    (-6)
#endif

#define M_DRV_MC70XX_AXIS_X      0
#define M_DRV_MC70XX_AXIS_Y      1
#define M_DRV_MC70XX_AXIS_Z      2
#define M_DRV_MC70XX_AXES_NUM    3

#define M_DRV_MC70XX_REG_MAP_SIZE    64

#define MAG_PRODUCT_CODE_REG	  0x00	//MAG_PRODUCT_CODE_REG

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/
typedef struct
{
    short    waRaw[M_DRV_MC70XX_AXES_NUM];
    int      naData_uT[M_DRV_MC70XX_AXES_NUM];
}   S_M_DRV_MC70XX_SensorData;

/*******************************************************************************
 *** EXTERNAL FUNCTION
 *******************************************************************************/
extern void    M_DRV_MC70XX_Init(void);
extern void    M_DRV_MC70XX_ReadSensorData(float faOutput[M_DRV_MC70XX_AXES_NUM]);
extern int     M_DRV_MC70XX_SensorValidation(unsigned char *pCode);
//extern void    M_DRV_MC70XX_ReadRegMap(unsigned char baRegMap[M_DRV_MC70XX_REG_MAP_SIZE]);

#endif    // END of _M_DRV_MC70XX_H_

