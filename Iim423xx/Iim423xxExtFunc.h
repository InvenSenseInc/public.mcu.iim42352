/*
 * ________________________________________________________________________________________________________
   Copyright (C) [2022] by InvenSense, Inc.
   Permission to use, copy, modify, and/or distribute this software for any purpose
   with or without fee is hereby granted.
    
   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
   REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
   FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
   INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
   OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
   TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
   THIS SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/** @defgroup DriverIim423xxExt Iim423xx driver extern functions
 *  @brief    Extern functions for Iim423xx devices
 *  @ingroup  DriverIim423xx
 *  @{
 */

/** @file Iim423xxExtFunc.h
 * Extern functions for Iim423xx devices
 */

#ifndef _INV_IIM423XX_EXTFUNC_H_
#define _INV_IIM423XX_EXTFUNC_H_


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/** @brief Hook for low-level high res system sleep() function to be implemented by upper layer
 *  ~100us resolution is sufficient
 *  @param[in] us number of us the calling thread should sleep
 */
extern void inv_iim423xx_sleep_us(uint32_t us);


/** @brief Hook for low-level high res system get_time() function to be implemented by upper layer
 *  Timer should be on 64bit with a 1 us resolution
 *  @param[out] The current time in us
 */
extern uint64_t inv_iim423xx_get_time_us(void);


#ifdef __cplusplus
}
#endif

#endif /* _INV_IIM423XX_EXTFUNC_H_ */

/** @} */
