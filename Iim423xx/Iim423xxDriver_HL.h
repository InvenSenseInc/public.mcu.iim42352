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

/** @defgroup DriverIim423xxDriver_HL Iim423xx driver high level functions
 *  @brief High-level function to setup an Iim423xx device
 *  @ingroup  DriverIim423xx
 *  @{
 */

/** @file Iim423xxDriver_HL.h
 * High-level function to setup an Iim423xx device
 */

#ifndef _INV_ICM4235x_DRIVER_HL_H_
#define _INV_ICM4235x_DRIVER_HL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Iim423xxDefs.h"
#include "Iim423xxTransport.h"

#include "InvError.h"

#include <stdint.h>
#include <string.h>

/** @brief Ligthen driver logic by stripping out procedures on transitions
 *  @details
 *  In the nominal case, ie. when sensors are enabled and their output has
 *  settled, IIM-423XX will not need the logic to handle each transition. They
 *  are part of each API function so this code will be linked in regardless.
 *  This might not be desirable for the most size-constrained platforms and
 *  it can be avoided by setting this define to 1.
 */
#ifndef INV_IIM423XX_LIGHTWEIGHT_DRIVER
	#define INV_IIM423XX_LIGHTWEIGHT_DRIVER 0
#endif

/** @brief Scale factor and max ODR 
 *  Dependant of chip
 */

	/* 
	 * For parts with maximum ODR of 32 KHz,
	 * PLL is running at 19.2 MHz instead of the nominal 20.48 MHz
	 * Time resolution has to be scaled by 20.48/19.2=1.06666667
	 * To prevent floating point usage, one can use 32/30
	 *
	 * The decimator also need to know the maximum ODR.
	 */
	#define PLL_SCALE_FACTOR_Q24 ((32UL<<24)/30)
	#define ICM_PART_DEFAULT_OIS_MODE IIM423XX_SENSOR_CONFIG2_OIS_MODE_32KHZ


/** @brief Max FSR values for accel
 *  Dependant of chip
 */

	#define ACCEL_CONFIG0_FS_SEL_MAX IIM423XX_ACCEL_CONFIG0_FS_SEL_16g

	#define ACCEL_OFFUSER_MAX_MG 1000 


/** @brief RTC Support flag
 *  Define whether the RTC mode is supported
 *  Dependant of chip
 */	

	#define RTC_SUPPORTED 1


/** @brief Iim423xx maximum buffer size mirrored from FIFO at polling time
 *  @warning fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE
 */
#define IIM423XX_FIFO_MIRRORING_SIZE 16 * 129 // packet size * max_count = 2064

/** @brief Default value for the WOM threshold
 *  Resolution of the threshold is ~= 4mg
 */
#define IIM423XX_DEFAULT_WOM_THS_MG 52>>2 /* = 52mg/4 */

/** @brief Iim423xx Accelerometer start-up time before having correct data
 */
#define IIM423XX_ACC_STARTUP_TIME_US 20000U


/** @brief Sensor identifier for UI control and OIS function
 */
enum inv_iim423xx_sensor {
	INV_IIM423XX_SENSOR_ACCEL,               /**< Accelerometer (UI control path) */
	INV_IIM423XX_SENSOR_FSYNC_EVENT,         /**< Used by OIS and UI control layers */
	INV_IIM423XX_SENSOR_TEMPERATURE,         /**< Chip temperature, enabled by default. However, it will be reported only if Accel also enabled. 
	                                              The Temperature's ODR (Output Data Rate) will match the ODR of Accel, or the fastest if both are enabled*/
	INV_IIM423XX_SENSOR_TAP,                 /**< Tap and Double tap */
	INV_IIM423XX_SENSOR_DMP_PEDOMETER_EVENT, /**< Pedometer: step is detected */
	INV_IIM423XX_SENSOR_DMP_PEDOMETER_COUNT, /**< Pedometer: step counter */
	INV_IIM423XX_SENSOR_DMP_TILT,            /**< Tilt */
#if defined(ICM_FAMILY_BPLUS)
	INV_IIM423XX_SENSOR_DMP_R2W,             /**< Raise to wake */
#elif defined(ICM_FAMILY_CPLUS)
	INV_IIM423XX_SENSOR_DMP_FF,              /**< Free Fall */
	INV_IIM423XX_SENSOR_DMP_LOWG,            /**< Low G */
#endif
	INV_IIM423XX_SENSOR_MAX
};

/** @brief Configure Fifo usage
 */
typedef enum {
	INV_IIM423XX_FIFO_DISABLED = 0,              /**< Fifo is disabled and data source is sensors registers */
	INV_IIM423XX_FIFO_ENABLED  = 1,              /**< Fifo is used as data source */
}INV_IIM423XX_FIFO_CONFIG_t;

/** @brief Sensor event structure definition
 */
typedef struct {
	int sensor_mask;
	uint16_t timestamp_fsync;
	int16_t accel[3]; 
	int16_t temperature;
	int8_t accel_high_res[3];
} inv_iim423xx_sensor_event_t;

/** @brief Iim423xx driver states definition
 */
struct inv_iim423xx {
	struct inv_iim423xx_transport transport;                              /**< Warning : this field MUST be the first one of 
	                                                                   struct iim423xx */

	void (*sensor_event_cb)(inv_iim423xx_sensor_event_t * event); /**< callback executed by inv_iim423xx_get_data_from_fifo function
	                                                                   for each data packet extracted from fifo or inv_iim423xx_get_data_from_registers read data from register
	                                                                   This field may be NULL if inv_iim423xx_get_data_from_fifo/inv_iim423xx_get_data_from_registers
	                                                                   is not used by application. */


	int accel_st_bias[3];
	int st_result;                                                   /**< Flag to keep track if self-test has been already run by storing acc results */

	uint8_t fifo_data[IIM423XX_FIFO_MIRRORING_SIZE];              /**<  FIFO mirroring memory area */

	uint8_t tmst_to_reg_en_cnt;                                   /**< internal counter to keep track of the timestamp to register access availability */
	
	uint8_t dmp_is_on;                                            /**< DMP started status */
	uint8_t dmp_from_sram;                                        /**< DMP executes from SRAM */


	uint64_t accel_start_time_us;                                 /**< internal state needed to discard first accel samples */
	uint8_t endianess_data;                                       /**< internal status of data endianess mode to report correctly data */
	uint8_t fifo_highres_enabled;                                 /**< FIFO packets are 20 bytes long */
	INV_IIM423XX_FIFO_CONFIG_t fifo_is_used;                      /**< Data are get from FIFO or from sensor registers. By default Fifo is used*/
	
	#if (!INV_IIM423XX_LIGHTWEIGHT_DRIVER)
		/* First FSYNC event after enable is irrelevant 
		 * When the sensors are switched off and then on again, an FSYNC tag with incorrect FSYNC value can be generated 
		 * on the next first ODR. 
		 * Solution: FSYNC tag and FSYNC data should be ignored on this first ODR.
		 */
		uint8_t fsync_to_be_ignored;
	#endif
	
	/* Accel Low Power could report with wrong ODR if internal counter for ODR changed overflowed
	 * WUOSC clock domain is informed through an 3bit counter that ODR
	 * has changed in RC/PLL mode. Every 8 event, this counter overflows
	 * and goes back to 0, same as 'no ODR changes', therefore previous ALP
	 * ODR is re-used.
	 * Solution: Keep track of ODR changes when WUOSC is disabled and perform dummy ODR changes when re-enabling WU after 8*n ODR changes.
	 */
	uint32_t wu_off_acc_odr_changes;

	uint8_t wom_smd_mask;   /**< This variable keeps track if wom or smd is enabled */

	uint8_t wom_enable;   /**< This variable keeps track if wom is enabled */
	
	/* Software mirror of BW and AVG settings in hardware, to be re-applied on each enabling of sensor */
	struct {
		uint8_t acc_lp_avg; /**< Low-power averaging setting for accelerometer */
		uint8_t reserved;   /**< reserved field */
		uint8_t acc_ln_bw;  /**< Low-noise filter bandwidth setting for accelerometer */

	} avg_bw_setting;

};

/* Interrupt enum state for INT1, INT2, and IBI */
typedef enum{
	INV_IIM423XX_DISABLE = 0,
	INV_IIM423XX_ENABLE
}inv_iim423xx_interrupt_value;

/** @brief Iim423xx set of interrupt enable flag
 */
typedef struct {
	inv_iim423xx_interrupt_value INV_IIM423XX_UI_FSYNC;
	inv_iim423xx_interrupt_value INV_IIM423XX_UI_DRDY;
	inv_iim423xx_interrupt_value INV_IIM423XX_FIFO_THS;
	inv_iim423xx_interrupt_value INV_IIM423XX_FIFO_FULL;
	inv_iim423xx_interrupt_value INV_IIM423XX_SMD;
	inv_iim423xx_interrupt_value INV_IIM423XX_WOM_X;
	inv_iim423xx_interrupt_value INV_IIM423XX_WOM_Y;
	inv_iim423xx_interrupt_value INV_IIM423XX_WOM_Z;
	inv_iim423xx_interrupt_value INV_IIM423XX_STEP_DET;
	inv_iim423xx_interrupt_value INV_IIM423XX_STEP_CNT_OVFL;
	inv_iim423xx_interrupt_value INV_IIM423XX_TILT_DET;
#if defined(ICM_FAMILY_BPLUS) 
	inv_iim423xx_interrupt_value INV_IIM423XX_SLEEP_DET;
	inv_iim423xx_interrupt_value INV_IIM423XX_WAKE_DET;
#elif defined(ICM_FAMILY_CPLUS)
	inv_iim423xx_interrupt_value INV_IIM423XX_FF_DET;
	inv_iim423xx_interrupt_value INV_IIM423XX_LOWG_DET;
#endif
	inv_iim423xx_interrupt_value INV_IIM423XX_TAP_DET;
}inv_iim423xx_interrupt_parameter_t;

#if defined(ICM_FAMILY_CPLUS)
/* Possible interface configurations mode */
typedef enum {
	INV_IIM423XX_SINGLE_INTERFACE    = 0x01,
	INV_IIM423XX_DUAL_INTERFACE      = 0x02,
	INV_IIM423XX_DUAL_INTERFACE_SPI4 = 0x06,
	INV_IIM423XX_TRIPLE_INTERFACE    = 0x08,
}inv_iim423xx_interface_mode_t;
#endif

/** @brief Set register bank index
 *  @param bank new bank to be set
 *  @return 0 on success, negative value otherwise
 */
int inv_iim423xx_set_reg_bank(struct inv_iim423xx * s, uint8_t bank);

/** @brief Configure the serial interface used to access the device and execute hardware initialization.
 *
 *  This functions first configures serial interface passed in parameter to make sure device 
 *  is accessible both in read and write. Thus no serial access should be done before 
 *  succesfully executing the present function.
 *
 *  Then if requested serial interface is a primary interface (aka UI interface or AP 
 *  interface), this function initializes the device using the following hardware settings:
 *    - accelerometer fsr = 4g
 *    - set timestamp resolution to 16us
 *    - enable FIFO mechanism with the following configuration:
 *        - FIFO record mode i.e FIFO count unit is packet 
 *        - FIFO snapshot mode i.e drop the data when the FIFO overflows
 *        - Timestamp is logged in FIFO
 *        - Little Endian fifo_count and fifo_data
 *        - generate FIFO threshold interrupt when packet count reaches FIFO watermark
 *        - set FIFO watermark to 1 packet
 *        - enable temperature and timestamp data to go to FIFO
 *
 *  In case requested serial interface is an auxliary interface (i.e. AUX1 or AUX2) this 
 *  function returns an error.
 *
 *
 *  @param[in] s driver structure. Note that first field of this structure MUST be a struct 
 *  inv_iim423xx_serif.
 *
 *  @param[in] serif pointer on serial interface structure to be used to access iim423xx.
 *
 *  @param[in] sensor_event_cb callback executed by inv_iim423xx_get_data_from_fifo function
 *  each time it extracts some valid data from fifo. Or inv_iim423xx_get_data_from_registers read data
 *  from register. Thus this parameter is optional as long
 *  as inv_iim423xx_get_data_from_fifo/inv_iim423xx_get_data_from_registers function is not used.
 *
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_init(struct inv_iim423xx * s, struct inv_iim423xx_serif * serif, void (*sensor_event_cb)(inv_iim423xx_sensor_event_t * event));

/** @brief Perform a soft reset of the device
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_device_reset(struct inv_iim423xx * s);

/** @brief return WHOAMI value
 *  @param[out] who_am_i WHOAMI for device
 *  @return     0 on success, negative value on error
 */
int inv_iim423xx_get_who_am_i(struct inv_iim423xx * s, uint8_t * who_am_i);

/** @brief Configure Accel clock source
 *  @param[in] new clock source to use
 *  @return 0 on success, negative value on error
 *  @note Transitions when enabling/disabling sensors are already handled by
 *        the driver. This function is here only to force a specific clock
 *        source and shall be used with care.
 */
int inv_iim423xx_force_clock_source(struct inv_iim423xx * s, IIM423XX_INTF_CONFIG1_ACCEL_LP_CLK_t clk_src);

/** @brief Enable/put accel in low power mode
 *  @return 0 on success, negative value on error.
 *  @details
 *  It enables accel data in the FIFO (so
 *  the packet format is 16 bytes). If called first,
 *  the configuration will be applied, otherwise it
 *  will be ignored if the FIFO is not empty (but since
 *  the new configuration is identical it is not a issue).
 *  @warning inv_iim423xx::register_cache::pwr_mngt_0_reg is modified by this function
 */
int inv_iim423xx_enable_accel_low_power_mode(struct inv_iim423xx * s);

/** @brief Enable/put accel in low noise mode
 *  @return 0 on success, negative value on error.
 *  @details
 *  It enables accel data in the FIFO (so
 *  the packet format is 16 bytes). If called first,
 *  the configuration will be applied, otherwise it
 *  will be ignored if the FIFO is not empty (but since
 *  the new configuration is identical it is not a issue).
 *  @warning inv_iim423xx::register_cache::pwr_mngt_0_reg is modified by this function
 */
int inv_iim423xx_enable_accel_low_noise_mode(struct inv_iim423xx * s);

/** @brief Disable all 3 axes of accel
 *  @return 0 on success, negative value on error.
 *  @details
 *  If  accel is turned off as a result of this
 *  function, they will also be removed from the FIFO and a
 *  FIFO reset will be performed (to guarantee no side effects
 *  until the next enable sensor call)
 *  @warning inv_iim423xx::register_cache::pwr_mngt_0_reg is modified by this function
 */
int inv_iim423xx_disable_accel(struct inv_iim423xx * s);


/** @brief Enable fsync tagging functionnality.
 *  In details it:
 *     - enables fsync
 *     - enables timestamp to registers. Once fysnc is enabled fsync counter is pushed to 
 *       fifo instead of timestamp. So timestamp is made available in registers. Note that 
 *       this increase power consumption.
 *     - enables fsync related interrupt
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_enable_fsync(struct inv_iim423xx * s);

/** @brief Disable fsync tagging functionnality.
 *  In details it:
 *     - disables fsync
 *     - disables timestamp to registers. Once fysnc is disabled  timestamp is pushed to fifo 
 *        instead of fsync counter. So in order to decrease power consumption, timestamp is no 
 *        more available in registers.
 *     - disables fsync related interrupt
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_disable_fsync(struct inv_iim423xx * s);

/** @brief  Configure timestamp resolution from FIFO.
 *  @param[in] resol The expected resolution of the timestamp. See enum IIM423XX_TMST_CONFIG_RESOL_t.
 *  @return 0 on success, negative value on error.
 *  @warning The resolution will have no effect if RTC is enabled
 */
int inv_iim423xx_configure_timestamp_resolution(struct inv_iim423xx * s, IIM423XX_TMST_CONFIG_RESOL_t resol);

/** @brief  Configure which interrupt source can trigger ibi interruptions.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage ibi interruptions.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_set_config_ibi(struct inv_iim423xx * s, inv_iim423xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Retrieve interrupts configuration.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage IBI interruptions.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_get_config_ibi(struct inv_iim423xx * s, inv_iim423xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Configure which interrupt source can trigger INT1.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage INT1.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_set_config_int1(struct inv_iim423xx * s, inv_iim423xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Retrieve interrupts configuration.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage INT1.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_get_config_int1(struct inv_iim423xx * s, inv_iim423xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Configure which interrupt source can trigger INT2.
 *  @param[in] interrupt_to_configure structure with the corresponding state to INT2.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_set_config_int2(struct inv_iim423xx * s, inv_iim423xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Retrieve interrupts configuration.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage INT2.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_get_config_int2(struct inv_iim423xx * s, inv_iim423xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief Read all registers containing data (tempereature, accelerometer). Then it calls 
 *  sensor_event_cb funtion passed in parameter of inv_iim423xx_init function for each packet 
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_get_data_from_registers(struct inv_iim423xx * s);

/** @brief Read all available packets from the FIFO. For each packet function builds a
 *  sensor event containing packet data and validity information. Then it calls 
 *  sensor_event_cb funtion passed in parameter of inv_iim423xx_init function for each 
 *  packet.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_get_data_from_fifo(struct inv_iim423xx * s);

/** @brief Converts IIM423XX_ACCEL_CONFIG0_ODR_t enums to period expressed in us
 *  @param[in] odr_bitfield An IIM423XX_ACCEL_CONFIG0_ODR_t enum
 *  @return The corresponding period expressed in us
 */
uint32_t inv_iim423xx_convert_odr_bitfield_to_us(uint32_t odr_bitfield);

/** @brief Configure accel Output Data Rate
 *  @param[in] frequency The requested frequency.
 *  @sa IIM423XX_ACCEL_CONFIG0_ODR_t
 *  @return 0 on success, negative value on error.
 *  @warning inv_iim423xx::register_cache::accel_cfg_0_reg is modified by this function
 */
int inv_iim423xx_set_accel_frequency(struct inv_iim423xx * s, const IIM423XX_ACCEL_CONFIG0_ODR_t frequency);


/** @brief Set accel full scale range
 *  @param[in] accel_fsr_g requested full scale range.
 *  @sa IIM423XX_ACCEL_CONFIG0_FS_SEL_t.
 *  @return 0 on success, negative value on error.
 *  @warning inv_iim423xx::register_cache::accel_cfg_0_reg is modified by this function
 */
int inv_iim423xx_set_accel_fsr(struct inv_iim423xx * s, IIM423XX_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g);

/** @brief Access accel full scale range
 *  @param[out] accel_fsr_g current full scale range.
 *  @sa IIM423XX_ACCEL_CONFIG0_FS_SEL_t.
 *  @return 0 on success, negative value on error.
 *  @warning inv_iim423xx::register_cache::accel_cfg_0_reg is relied upon by this function
 */
int inv_iim423xx_get_accel_fsr(struct inv_iim423xx * s, IIM423XX_ACCEL_CONFIG0_FS_SEL_t * accel_fsr_g);



/** @brief Set accel Low-Power averaging value
 *  @param[in] acc_avg requested averaging value
 *  @sa IIM423XX_ACCEL_FILT_CONFIG_FILT_AVG_t
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_set_accel_lp_avg(struct inv_iim423xx * s, IIM423XX_ACCEL_FILT_CONFIG_FILT_AVG_t acc_avg);

/** @brief Set accel Low-Noise bandwidth value
 *  @param[in] acc_bw requested averaging value
 *  @sa IIM423XX_ACCEL_FILT_CONFIG_FILT_BW_t
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_set_accel_ln_bw(struct inv_iim423xx * s, IIM423XX_ACCEL_FILT_CONFIG_FILT_BW_t acc_bw);



/** @brief reset IIM423XX fifo
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_reset_fifo(struct inv_iim423xx * s);

/** @brief Enable the 20bits-timestamp register access to read in a reliable way the strobed timestamp. To do that, the fine clock is forced enabled at some power cost.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_enable_timestamp_to_register(struct inv_iim423xx * s);

/** @brief Disable the 20bits-timestamp register access. Register read always return 0's.
 *  @return 0 on success, negative value on error.
 */
int inv_iim423xx_disable_timestamp_to_register(struct inv_iim423xx * s);

/** @brief Get the timestamp value of iim423xx from register
 *  @param[in] icm_time timestamp read from register
 *  @return 0 on success, negative value on error.
 *  @warning Prior to call this API, the read access to timestamp register must be enabled (see @ref inv_iim423xx_enable_timestamp_to_register() function)
 */
int inv_iim423xx_get_current_timestamp(struct inv_iim423xx * s, uint32_t * icm_time);

/** @brief Enable or disable CLKIN/RTC capability
 *  @param[in] enable 1 if external 32kHz is provided to ICM, 0 otherwise
 *  @return 0 on success, negative value on error.
 *  @warning In case CLKIN is disabled, it is recommended to call inv_iim423xx_configure_timestamp_resolution()
 *  just afterwards so that timestamp resolution is in line with system request
 *  @warning inv_iim423xx::register_cache::intf_cfg_1_reg is modified by this function
 */
int inv_iim423xx_enable_clkin_rtc(struct inv_iim423xx * s, uint8_t enable);

/** @brief Get CLKIN/RTC feature status
 *  @return 0 if CLKIN is disabled, 1 if enabled.
 *  @warning In case CLKIN is disabled, it is recommended to call inv_iim423xx_configure_timestamp_resolution()
 *  just afterwards so that timestamp resolution is in line with system request
 *  @warning inv_iim423xx::register_cache::intf_cfg_1_reg is relied upon by this function
 */
int inv_iim423xx_get_clkin_rtc_status(struct inv_iim423xx * s);

/** @brief Enable 20 bits raw acc data in fifo.
 *  @return 0 on success, negative return code otherwise
 */
int inv_iim423xx_enable_high_resolution_fifo(struct inv_iim423xx * s);

/** @brief Disable 20 bits raw acc data in fifo.
 *  @return 0 on success, negative return code otherwise
 */
int inv_iim423xx_disable_high_resolution_fifo(struct inv_iim423xx * s);

 /** @brief Configure Fifo to select the way data are gathered
 *  @param[in] fifo_config Fifo configuration method : 
 *  if enabled data are comming from fifo and Interrupt is configured on Fifo Watermark
 *  if disabled data are comming from sensor registers and Interrupt is configured on Data ready
 *  @sa INV_IIM423XX_FIFO_CONFIG_t
 */
int inv_iim423xx_configure_fifo(struct inv_iim423xx * s, INV_IIM423XX_FIFO_CONFIG_t fifo_config);

 /** @brief Configure Fifo watermark (also refered to as fifo threshold)
 *  @param[in] wm Watermark value
 */
int inv_iim423xx_configure_fifo_wm(struct inv_iim423xx * s, uint16_t wm);

/** @brief Get FIFO timestamp resolution
 *  @return the timestamp resolution in us as a q24 or 0 in case of error
 */
uint32_t inv_iim423xx_get_fifo_timestamp_resolution_us_q24(struct inv_iim423xx * s);

/** @brief Get register timestamp resolution
 *  @return the timestamp resolution in us as a q24 or 0 in case of error
 */
uint32_t inv_iim423xx_get_reg_timestamp_resolution_us_q24(struct inv_iim423xx * s);


#if defined(ICM_FAMILY_CPLUS)
 /** @brief Configure Fifo decimation rate
 *  @param[in] dec_rate decimation valmue from 0 to 127
 *  if 0, no sample is decimated, 1 means 1 sample over 2 are skipped.
 */
int inv_iim423xx_set_fifo_dec_rate(struct inv_iim423xx * s, uint8_t dec_rate);

/** @brief Configure interface mode
 *  @param[in] interface_mode value can be single, dual (AUX1-SPI3/AUX1-SPI4) or triple interface according to inv_iim423xx_interface_mode_t
 *  @return 0 on success, negative return code otherwise
 */

#endif

/** @brief Return driver version x.y.z-suffix as a char array
 *  @retval driver version a char array "x.y.z-suffix"
 */
const char * inv_iim423xx_get_version(void);



#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM4235x_DRIVER_HL_H_ */

/** @} */
