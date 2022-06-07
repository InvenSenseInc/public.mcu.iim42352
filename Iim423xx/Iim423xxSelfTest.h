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

/** @defgroup DriverIim423xxSelfTest Iim423xx selftest
 *  @brief Low-level function to run selftest on a Iim423xx device
 *  @ingroup  DriverIim423xx
 *  @{
 */

/** @file Iim423xxSelfTest.h
 * Low-level function to run selftest on a Iim423xx device
 */

#ifndef _INV_IIM423XX_SELFTEST_H_
#define _INV_IIM423XX_SELFTEST_H_

// #include "Invn/InvExport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* forward declaration */
struct inv_iim423xx;

/**
*  @brief      Perform hardware self-test for Accel
*  @param[in]  result containing ACCEL_SUCCESS<<1 so 2
*  @return     0 if success, error code if failure
*/
int inv_iim423xx_run_selftest(struct inv_iim423xx * s, int * result);

/**
*  @brief      Retrieve bias collected by self-test.
*  @param[out] st_bias bias scaled by 2^16, accel is gee.
*                      The buffer will be filled as below.
*                      Accel LN mode X,Y,Z
*  @return     0 if success, error code if failure
*/
int inv_iim423xx_get_st_bias(struct inv_iim423xx * s, int st_bias[6]);

/**
*  @brief      Apply bias.
*  @param[in] st_bias bias scaled by 2^16, accel is gee.
*                      The buffer must be filled as below.
*                      Accel LN mode X,Y,Z
*  @return     0 if success, error code if failure
*/
int inv_iim423xx_set_st_bias(struct inv_iim423xx * s, const int st_bias[6]);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IIM423XX_SELFTEST_H_ */

/** @} */
