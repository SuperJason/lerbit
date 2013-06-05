/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef NRF_TEMP_H__
#define NRF_TEMP_H__

#include "nrf51.h"

/**
* @defgroup nrf_temperature TEMP (temperature) abstraction
* @{
* @ingroup nrf_drivers
* @brief Temperature module init and read functions.
*
*/



/**
 * @brief Prepare the temp module for temperature measurement.
 *
 * This function initializes the TEMP module and writes to the hidden configuration register.
 * 
 * @param none
 */
static __INLINE void nrf_temp_init(void)
{
    *(uint32_t *) 0x4000C504 = 0;
}



#define MASK_SIGN           (0x00000200UL)
#define MASK_SIGN_EXTENSION (0xFFFFFC00UL)

/**
 * @brief Read temperature measurement.
 *
 * The function reads the 10 bit 2's complement value and transforms it to a 32 bit 2's complement value.
 * 
 * @param none
 */
static __INLINE int32_t nrf_temp_read(void)
{    
    // Workaround for PAN_028 rev1.1 anomaly 24 - TEMP: Negative measured values are not represented correctly
    return ((NRF_TEMP->TEMP & MASK_SIGN) != 0) ? (NRF_TEMP->TEMP | MASK_SIGN_EXTENSION) : (NRF_TEMP->TEMP);    
}

/** @} */

#endif
