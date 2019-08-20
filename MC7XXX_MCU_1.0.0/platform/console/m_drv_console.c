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
 * @file    m_drv_console.c
 * @author  mCube
 * @date    19 June 2017
 * @brief   Driver interface for gyroscope mc5010.
 * @see     http://www.mcubemems.com
 */

/*******************************************************************************
 *** INCLUDE FILES
 *******************************************************************************/
/* platform function includes */

/*******************************************************************************
 *** MACRO
 *******************************************************************************/
#ifdef M_DRV_DEBUG_MSG
	#define M_PRINTF(...)    {NRF_LOG_INFO( __VA_ARGS__);NRF_LOG_FLUSH();}
	#define M_INT_TO_FLOAT(val) (uint32_t)(((val) < 0 && (val) > -1.0) ? "-" : ""),   \
								(int32_t)(val),                                       \
								(int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
													   : (int32_t)(val) - (val))*100)
#else
	#define M_PRINTF(...)
#endif
	
