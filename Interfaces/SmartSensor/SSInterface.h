/***************************************************************************
* Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
****************************************************************************
*/

#ifndef _SSINTERFACE_H_
#define _SSINTERFACE_H_

#include "stdbool.h"
#include "stdint.h"
#include "app.h"

#define SS_PLATFORM_MAX3263X               "SmartSensor_MAX3263X"
#define SS_PLATFORM_MAX32660               "SiLab_BGM220SC22WGA"                   // "SmartSensor_MAX32660"
#define SS_BOOTLOADER_PLATFORM_MAX3263X    "Bootloader_MAX3263X"
#define SS_BOOTLOADER_PLATFORM_MAX32660    "Bootloader_MAX32660"


#define SS_I2C_8BIT_SLAVE_ADDR    0xAA
#define SS_DEFAULT_CMD_SLEEP_MS   2
#define SS_DUMP_REG_SLEEP_MS      100
#define SS_ENABLE_SENSOR_SLEEP_MS 20

#define SS_SENSORIDX_MAX8614X	    0x00
#define SS_SENSORIDX_MAX30205	    0x01
#define SS_SENSORIDX_MAX30001	    0x02
#define SS_SENSORIDX_MAX30101	    0x03
#define SS_SENSORIDX_ACCEL	      0x04
#define SS_NUM_CURRENT_SENSORS	  5                                     // Added by Jason

#define SS_ALGOIDX_AGC	            0x00
#define SS_ALGOIDX_AEC	            0x01
#define SS_ALGOIDX_WHRM	            0x02
#define SS_ALGOIDX_ECG	            0x03
#define SS_ALGOIDX_BPT	            0x04
#define SS_ALGOIDX_WSPO2            0x05                               // Added by Jason
#define SS_ALGOIDX_WSPO3            0x06                               // Added by Jason
#define SS_ALGOIDX_WHRM_WSPO2_SUITE 0x07
#define SS_NUM_CURRENT_ALGOS	      8

#define SS_FAM_R_STATUS		                    0x00
	#define SS_CMDIDX_STATUS	                  0x00
		#define SS_SHIFT_STATUS_ERR				        0
		#define SS_MASK_STATUS_ERR				        (0x07 << SS_SHIFT_STATUS_ERR)
		#define SS_SHIFT_STATUS_DATA_RDY		      3
		#define SS_MASK_STATUS_DATA_RDY			      (1 << SS_SHIFT_STATUS_DATA_RDY)
		#define SS_SHIFT_STATUS_FIFO_OUT_OVR	    4
		#define SS_MASK_STATUS_FIFO_OUT_OVR		    (1 << SS_SHIFT_STATUS_FIFO_OUT_OVR)
		#define SS_SHIFT_STATUS_FIFO_IN_OVR		    5
		#define SS_MASK_STATUS_FIFO_IN_OVR		    (1 << SS_SHIFT_STATUS_FIFO_IN_OVR)

		#define SS_SHIFT_STATUS_LOG_OVR			6
		#define SS_MASK_STATUS_LOG_OVR			(1 << SS_SHIFT_STATUS_LOG_OVR)

		#define SS_SHIFT_STATUS_LOG_RDY			7
		#define SS_MASK_STATUS_LOG_RDY			(1 << SS_SHIFT_STATUS_LOG_RDY)


#define SS_FAM_W_MODE	                0x01
#define SS_FAM_R_MODE	                0x02
	#define SS_CMDIDX_MODE	            0x00
		#define SS_SHIFT_MODE_SHDN		    0
		#define SS_MASK_MODE_SHDN		      (1 << SS_SHIFT_MODE_SHDN)
		#define SS_SHIFT_MODE_RESET		    1
		#define SS_MASK_MODE_RESET		    (1 << SS_SHIFT_MODE_RESET)
		#define SS_SHIFT_MODE_FIFORESET	  2
		#define SS_MASK_MODE_FIFORESET	  (1 << SS_SHIFT_MODE_FIFORESET)
		#define SS_SHIFT_MODE_BOOTLDR	    3
		#define SS_MASK_MODE_BOOTLDR	    (1 << SS_SHIFT_MODE_BOOTLDR)

#define SS_I2C_READ		0x03

#define SS_FAM_W_COMMCHAN	                        0x10
#define SS_FAM_R_COMMCHAN	                        0x11
	#define SS_CMDIDX_OUTPUTMODE	                  0x00
		#define SS_SHIFT_OUTPUTMODE_DATATYPE	        0
		#define SS_MASK_OUTPUTMODE_DATATYPE		        (0x03 << SS_SHIFT_OUTPUTMODE_DATATYPE)
			#define SS_DATATYPE_PAUSE				            0
			#define SS_DATATYPE_RAW					            1
			#define SS_DATATYPE_ALGO				            2
			#define SS_DATATYPE_BOTH				            3
		#define SS_SHIFT_OUTPUTMODE_SC_EN		          2
		#define SS_MASK_OUTPUTMODE_SC_EN		          (1 << SS_SHIFT_OUTPUTMODE_SC_EN)
	#define SS_CMDIDX_FIFOAFULL		                  0x01
	#define SS_CMDIDX_I2C_ADDR		                  0x03

#define SS_FAM_R_OUTPUTFIFO	0x12
	#define SS_CMDIDX_OUT_NUMSAMPLES	0x00
	#define SS_CMDIDX_READFIFO		    0x01

#define SS_FAM_R_INPUTFIFO	      0x13
	#define SS_CMDIDX_SAMPLESIZE	  0x00
	#define SS_CMDIDX_FIFOSIZE		  0x01
	#define SS_CMDIDX_IN_NUMSAMPLES	0x02
#define SS_FAM_W_INPUTFIFO	      0x14
	#define SS_CMDIDN_WRITEFIFO		  0x00

#define SS_FAM_W_WRITEREG		  0x40
#define SS_FAM_R_READREG		  0x41
#define SS_FAM_R_REGATTRIBS		0x42
#define SS_FAM_R_DUMPREG		  0x43

#define SS_FAM_W_SENSORMODE	0x44
#define SS_FAM_R_SENSORMODE	0x45

//TODO: Fill in known configuration parameters
#define SS_FAM_W_ALGOCONFIG	        0x50
#define SS_FAM_R_ALGOCONFIG	        0x51
	#define SS_CFGIDX_AGC_TARGET		  0x00
	#define SS_CFGIDX_AGC_CORR_COEFF	0x01
	#define SS_CFGIDX_AGC_SENSITIVITY	0x02
	#define SS_CFGIDX_AGC_SMP_AVG		  0x03

	#define SS_CFGIDX_WHRM_SR			    0x00
	#define SS_CFGIDX_WHRM_MAX_HEIGHT	0x01
	#define SS_CFGIDX_WHRM_MAX_WEIGHT	0x02
	#define SS_CFGIDX_WHRM_MAX_AGE		0x03
	#define SS_CFGIDX_WHRM_MIN_HEIGHT	0x04
	#define SS_CFGIDX_WHRM_MIN_WEIGHT	0x05
	#define SS_CFGIDX_WHRM_MIN_AGE		0x06
	#define SS_CFGIDX_WHRM_DEF_HEIGHT	0x07
	#define SS_CFGIDX_WHRM_DEF_WEIGHT	0x08
	#define SS_CFGIDX_WHRM_DEF_AGE		0x09
	#define SS_CFGIDX_WHRM_INIT_HR		0x0A

	// additional for WHRM_AEC_SCD
	#define SS_CFGIDX_WHRM_AEC_ENABLE						          0x0B
	#define SS_CFGIDX_WHRM_SCD_ENABLE						          0x0C
	#define SS_CFGIDX_WHRM_ADJ_TARGET_PD_CURRENT_PERIOD		0x0D
	#define SS_CFGIDX_WHRM_SCD_DEBOUNCE_WINDOW				    0x0E
	#define SS_CFGIDX_WHRM_MOTION_MAG_THRESHOLD				    0x0F
	#define SS_CFGIDX_WHRM_MIN_PD_CURRENT			    	      0x10
	#define SS_CFGIDX_WHRM_PD_CONFIG				    	        0x11
	#define SS_CFGIDX_WHRM_LED_CONFIG				    	        0x12

	// config for WSPO2
	#define SS_CFGIDX_WSPO2_CAL                             0x00
	#define SS_CFGIDX_WSPO2_SR                              0x01
	#define SS_CFGIDX_WSPO2_ALGO_MODE                       0x02
	#define SS_CFGIDX_WSPO2_AGC_MODE                        0x03
	#define SS_CFGIDX_WSPO2_MOTION_DET                      0x04
	#define SS_CFGIDX_WSPO2_MOTION_PERIOD                   0x05
	#define SS_CFGIDX_WSPO2_MOTION_THRESHOLD                0x06
	#define SS_CFGIDX_WSPO2_AGC_TIMEOUT                     0x07
	#define SS_CFGIDX_WSPO2_TIMEOUT                         0x08
	#define SS_CFGIDX_WSPO2_PD_CONFIG                       0x09

	#define SS_CFGIDX_BP_USE_MED		      0x00
	#define SS_CFGIDX_BP_SYS_BP_CAL		    0x01
	#define SS_CFGIDX_BP_DIA_BP_CAL		    0x02
	#define SS_CFGIDX_BP_CAL_DATA		      0x03
	#define SS_CFGIDX_BP_EST_DATE		      0x04
	#define SS_CFGIDX_BP_EST_NONREST	    0x05
    #define SS_CFGIDX_BP_SPO2_COEFS     0x06
	#define SS_CFGIDX_BP_SYS_DIA		      0x07                                      // Added by Jason
	#define SS_CFGIDX_BP_CAL_INDEX		    0x08                                      // Added by Jason
    #define SS_NUM_CURRENT_CFGIDX	      9                                         // Added by Jason

	//config for WHRM+WSPO2 ALGO SUITE
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2_CAL		                  0x00
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_PERIOD          0x01
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_MOTION_THRESHOLD       0x02
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_AFE_TIMEOUT       		    0x03
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_WSPO2_TIMEOUT       			      0x04
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_INITIAL_HR       			      0x05
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_HEIGHT       			    0x06
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_WEIGHT       			    0x07
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_AGE       		      	0x08
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_PERSON_GENDER       			    0x09
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_ALGO_MODE        	          0x0A
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_AEC_ENABLE           	      0x0B
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_SCD_ENABLE           	      0x0C
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT_PERIOD       0x0D
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MOTION_MAG_THRESHOLD         0x0E
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_PD_CURRENT               0x0F
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_INIT_PD_CURRENT              0x10
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_TARGET_PD_CURRENT            0x11
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_AUTO_PD_CURRENT_ENABLE       0x12
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_INTEGRATION_TIME         0x13
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MIN_SAMPLING_AVERAGE         0x14
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_INTEGRATION_TIME         0x15
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_MAX_SAMPLING_AVERAGE         0x16
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_WHRM_LED_PD_CONFIG           0x17
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_SPO2_LED_PD_CONFIG           0x18
    #define SS_CFGIDX_WHRM_WSPO2_SUITE_LED_FIRING_SEQ               0x19
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_INIT_INTEGRATION_TIME		      0x1A
	#define SS_CFGIDX_WHRM_WSPO2_SUITE_INIT_SAMPLING_AVERAGE          0x1B

#define SS_FAM_W_ALGOMODE	0x52
#define SS_FAM_R_ALGOMODE	0x53

#define SS_FAM_W_EXTERNSENSORMODE	0x60
#define SS_FAM_R_EXTERNSENSORMODE	0x61

#define SS_FAM_R_SELFTEST    0x70

#define SS_FAM_W_BOOTLOADER	      0x80
	#define SS_CMDIDX_SETIV			    0x00
	#define SS_CMDIDX_SETAUTH		    0x01
	#define SS_CMDIDX_SETNUMPAGES	  0x02
	#define SS_CMDIDX_ERASE			    0x03
	#define SS_CMDIDX_SENDPAGE		  0x04
	#define SS_CMDIDX_ERASE_PAGE	  0x05
#define SS_FAM_R_BOOTLOADER	      0x81
	#define SS_CMDIDX_BOOTFWVERSION	0x00
	#define SS_CMDIDX_PAGESIZE		  0x01

#define SS_FAM_W_BOOTLOADER_CFG	      0x82
#define SS_FAM_R_BOOTLOADER_CFG	      0x83
	#define SS_CMDIDX_BL_SAVE		        0x00
	#define SS_CMDIDX_BL_ENTRY		      0x01
		#define SS_BL_CFG_ENTER_BL_MODE		0x00
		#define SS_BL_CFG_EBL_PIN			    0x01
		#define SS_BL_CFG_EBL_POL			    0x02
	#define SS_CMDIDX_BL_EXIT		        0x02
		#define SS_BL_CFG_EXIT_BL_MODE		0x00
		#define SS_BL_CFG_TIMEOUT			    0x01

/* Enable logging/debugging */
#define SS_FAM_R_LOG				    0x90
	#define SS_CMDIDX_R_LOG_DATA	0x00
	#define SS_CMDIDX_R_LOG_LEN		0x01

	#define SS_CMDIDX_R_LOG_LEVEL	0x02
		#define SS_LOG_DISABLE		  0x00
		#define SS_LOG_CRITICAL		  0x01
		#define SS_LOG_ERROR		    0x02
		#define SS_LOG_INFO			    0x04
		#define SS_LOG_DEBUG		    0x08

#define SS_FAM_W_LOG_CFG			    0x91
	#define SS_CMDIDX_LOG_GET_LEVEL	0x00
	#define SS_CMDIDX_LOG_SET_LEVEL	0x01

#define SS_FAM_R_IDENTITY			   0xFF
	#define SS_CMDIDX_PLATTYPE		 0x00
	#define SS_CMDIDX_PARTID		   0x01
	#define SS_CMDIDX_REVID			   0x02
	#define SS_CMDIDX_FWVERSION		 0x03
	#define SS_CMDIDX_AVAILSENSORS 0x04
	#define SS_CMDIDX_DRIVERVER		 0x05
	#define SS_CMDIDX_AVAILALGOS	 0x06
	#define SS_CMDIDX_ALGOVER		   0x07

#define SH_FAM_R_SHADIGEST          0xB2
#define SH_FAM_W_SHANONCE           0xB1
#define SH_FAM_R_DHPARAMS           0xB3
#define SH_FAM_W_DHPUBLICKEY        0xB4
#define SH_FAM_R_DHPUBLICKEY        0xB5

typedef enum {
	SS_SUCCESS            = 0x00,
	SS_ERR_COMMAND        = 0x01,
	SS_ERR_UNAVAILABLE    = 0x02,
	SS_ERR_DATA_FORMAT    = 0x03,
	SS_ERR_INPUT_VALUE    = 0x04,
	SS_ERR_BTLDR_GENERAL  = 0x80,
	SS_ERR_BTLDR_CHECKSUM = 0x81,
	SS_ERR_TRY_AGAIN      = 0xFE,
	SS_ERR_UNKNOWN        = 0xFF,
} SS_STATUS;

typedef enum {
    SS_PLAT_MAX3263X = 0,
    SS_PLAT_MAX32660 = 1,
} SS_PLAT_TYPE;

typedef enum {
    PIN_INPUT  = 0, /* MXC_V_GPIO_OUT_MODE_HIGH_Z,*/
    PIN_OUTPUT = 1  /* MXC_V_GPIO_OUT_MODE_NORMAL_DRIVE */
} PinDirection;

//self test result masks
#define FAILURE_COMM        0x01
#define FAILURE_INTERRUPT   0x02

#define SS_SMALL_BUF_SIZE   32
#define SS_MED_BUF_SIZE     512
#define SS_LARGE_BUF_SIZE   1024//8224

#define SS_RESET_TIME	              20
#define SS_STARTUP_TO_BTLDR_TIME    50
#define SS_STARTUP_TO_MAIN_APP_TIME	2000

#define SS_MAX_SUPPORTED_SENSOR_NUM	    SS_NUM_CURRENT_SENSORS//0xFE                       Modified by Jason
#define SS_MAX_SUPPORTED_ALGO_NUM	      SS_NUM_CURRENT_ALGOS  //0xFE                       Modified by Jason
#define SS_MAX_SUPPORTED_ALGO_CFG_NUM	  SS_NUM_CURRENT_CFGIDX                            //SS_NUM_CURRENT_ALGOS//0xFE,    Modified by Jason
#define SS_MAX_SUPPORTED_MODE_NUM	      10//0xFF                                           Modified by Jason

//function pointer use to perform arithmetic operation
typedef void (*rx_data_callback)(uint8_t *);
typedef struct {
	int data_size;
	rx_data_callback rx_data_parser;
} ss_data_req;

typedef union {
	struct {
		uint16_t whrm_enabled    :1;
		uint16_t bpt_enabled     :1;
		uint16_t agc_enabled     :1;
		uint16_t max8614x_enabled:1;
		uint16_t max3010x_enabled:1;
		uint16_t kx122_enabled   :1;
		uint16_t sensorhub_accel :1;
		uint16_t whrm_wspo2_suite_enabled :1;
		uint16_t placeholder     :8;
	};
	uint16_t status_vals;
} status_algo_sensors_st;


void ss_init_hwcomm_interface(void);

//void ss_clear_interrupt_flag(void);


/**
 * @brief   SSInterface is Maxim's SmartSensor Interface class
 */

//public:
	/* PUBLIC FUNCTION DECLARATIONS */
	/**
	* @brief	Write a command to the SmartSensor and get status response
	*
	* @param[in]	cmd_bytes - Pointer to the command's family and index bytes
	* @param[in]	cmd_idx_len - The number of bytes in the command
	* @param[in]	data - Pointer to the command's data bytes
	* @param[in]	data_len - The number data bytes
	*
	* @return SS_STATUS byte
	*/
	SS_STATUS write_cmd2(uint8_t *cmd_bytes, int cmd_bytes_len, uint8_t *data, int data_len, int sleep_ms);// = SS_DEFAULT_CMD_SLEEP_MS);


	/**
	* @brief	Write a command to the SmartSensor and get status response
	*
	* @param[in]	tx_buf - Pointer to the command's family, index bytes and data bytes
	* @param[in]	tx_len - Total transaction lenght to send
	*
	* @return SS_STATUS byte
	*/
	SS_STATUS write_cmd(uint8_t *tx_buf, int tx_len, int sleep_ms);// = SS_DEFAULT_CMD_SLEEP_MS);

	/**
	* @brief	Write a command to the SmartSensor and get status response
	*
	* @param[in]	cmd_bytes - Pointer to the command's family and index bytes
	* @param[in]	cmd_idx_len - The number of bytes in the command
	* @param[in]	data - Pointer to the command's data bytes
	* @param[in]	data_len - The number data bytes
	* @param[in]	rxbuf - Buffer to fill in with read data (including status byte)
	* @param[in]	rxbuf_sz - Size of the rx buffer (to prevent overflow)
	*
	* @return SS_STATUS byte
	*/
	SS_STATUS read_cmd( uint8_t *cmd_bytes, int cmd_bytes_len,
						uint8_t *data, int data_len,
						uint8_t *rxbuf, int rxbuf_sz,
                        int sleep_ms);// = SS_DEFAULT_CMD_SLEEP_MS);

	/**
	* @brief	Get a string representing the SmartSensor firmware version
	* @details	If in bootloader mode, returns bootloader version
	*
	* @return   Pointer to firmware version string
	*/
	  const char* get_ss_fw_version(void);

	/**
	* @brief	Get a string representing the SmartSensor algo version
	* @details	If in bootloader mode, returns bootloader version
	*
	* @return   Pointer to algo version string
	*/
    const char* get_ss_algo_version(void);

	/**
	* @brief	Get a string representing the SmartSensor platform type
	*
	* @return   Pointer to platform type string
	*/
    const char* get_ss_platform_name(void);

	/**
	 * @brief	Reset the SmartSensor and jump to main application
	 *
	 * @return 	SS_STATUS code indicating success
	 */ 
	SS_STATUS reset_to_main_app(void);

	/**
	 * @brief	Reset the SmartSensor and jump to bootloader
	 *
	 * @return 	SS_STATUS code indicating success
	 */ 
	SS_STATUS reset_to_bootloader(void);

	/**
	 * @brief	Reset the SmartSensor
	 * @details	If the SmartSensor was in bootloader, it will jump back into bootloader
	 * 			If the SmartSensor was in main app, it will jump back into main app
	 *
	 * @return 	SS_STATUS code indicating success
	 */ 
	SS_STATUS sensorHub_reset(void);

	/**
	 * @brief 		run the self test commands
	 * param[in] 	idx - the id of the sensor for the self test
	 * param[in] 	result - self-test response
	 * param[in] 	sleep_ms - duration of wait for read command
	 * @return 		SS_STATUS code indicating success
	 */
	SS_STATUS self_test(int idx, uint8_t *result, int sleep_ms);// = SS_DEFAULT_CMD_SLEEP_MS);

	/**
	 * @brief	Check if SmartSensor is in bootloader mode
	 *
	 * @return	1 if in bootloader mode, 0 if in main app, -1 if comm error
	 */
	int ss_in_bootldr_mode(void);

	/**
	 * @brief	Set the I2C address.
	 *
	 * @param[in]	addr - I2C address
	 *
	 * @return SS_SUCCESS on success
	 */
	SS_STATUS set_i2c_addr(uint8_t addr);

	/**
	 * @brief	Read register from a device onboard SmartSensor
	 *
	 * @param[in] idx - Index of device to read
	 * @param[in] addr - Register address
	 * @param[out] val - Register value
	 *
	 * @return	SS_SUCCESS on success
	 */
	SS_STATUS get_reg(int idx, uint8_t addr, uint32_t *val);

	/**
	 * @brief	Set register of a device onboard SmartSensor
	 *
	 * @param[in] idx - Index of device to read
	 * @param[in] addr - Register address
	 * @param[in] val - Register value
	 * @param[in] byte_size - Size of IC register in bytes
	 *
	 * @return	SS_SUCCESS on success
	 */
	SS_STATUS set_reg(int idx, uint8_t addr, uint32_t val, int byte_size);

	/**
	 * @brief	Dump registers of a device onboard SmartSensor
	 *
	 * @param[in] idx - Index of device
	 * @param[in] reg_vals - Array of addr_val_pairs
	 * @param[in] reg_vals_sz - Number of items reg_vals can hold
	 * @param[out] num_regs - Number of registers returned by command
	 *
	 * @return	SS_SUCCESS on success
	 */
	SS_STATUS dump_reg(int idx, addr_val_pair* reg_vals, int reg_vals_sz, int* num_regs);


	/**
	 * @brief	Enable a device on the SmartSensor
	 *
	 * @param[in] idx - Index of device
	 * @param[in] mode - Mode to set the device to
	 * @param[in] data_req - Data request
	 *
	 * @return	SS_SUCCESS on success
	 */
	SS_STATUS enable_sensor(int idx, int mode, ss_data_req* data_req);

	/**
	 * @brief	Disable a device on the SmartSensor
	 *
	 * @param[in] idx - Index of device
	 *
	 * @return	SS_SUCCESS on success
	 */
	SS_STATUS disable_sensor(int idx);
	SS_STATUS disable_sensor_A(void);

	/**
	 * @brief	Enable an algorithm on the SmartSensor
	 *
	 * @param[in] idx - Index of device
	 * @param[in] mode - Mode to set the device to
	 * @param[in] data_req - Data request
	 *
	 * @return	SS_SUCCESS on success
	 */
	SS_STATUS enable_algo(int idx, int mode, ss_data_req* data_req);

	/**
	 * @brief	Disable an algorithm on the SmartSensor
	 *
	 * @param[in] idx - Index of device
	 *
	 * @return	SS_SUCCESS on success
	 */
	SS_STATUS disable_algo(int idx);

	/**
	 * @brief	Set the value of an algorithm configuration parameter
	 *
	 * @param[in] algo_idx Index of algorithm
	 * @param[in] cfg_idx Index of configuration parameter
	 * @param[in] cfg Array of configuration bytes
	 * @param[in] cfg_sz Size of cfg array
	 *
	 * @return SS_SUCCESS on success
	 */
	SS_STATUS set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz);

	/**
	 * @brief	Get the value of an algorithm configuration parameter
	 *
	 * @param[in] algo_idx Index of algorithm
	 * @param[in] cfg_idx Index of configuration parameter
	 * @param[in] cfg Array of configuration bytes to be filled in
	 * @param[in] cfg_sz Number of bytes to be read
	 *
	 * @return SS_SUCCESS on success
	 */
	SS_STATUS get_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz);

	/**
	 * @brief	Set the CommChannel Output Mode options
	 *
	 * @param[in] data_type - Set to 0 for only algorithm data
	 * 							Set to 1 for only raw sensor data
	 * 							Set to 2 for algo + raw sensor data
	 * 							Set to 3 for no data
	 * @param[in] sc_en - Set to true to receive 1 byte sample count from SmartSensor
	 *
	 * @return 	SS_SUCCESS on success
	 */
	SS_STATUS set_data_type(int data_type1, bool sc_en1);

	/**
	 * @brief	Get the CommChannel Output Mode options
	 *
	 * @param[out] data_type - 0 for only algorithm data
	 * 							1 for only raw sensor data
	 * 							2 for algo + raw sensor data
	 * 							3 for no data
	 * @param[in] sc_en - If true, SmartSensor prepends data with 1 byte sample count
	 *
	 * @return 	SS_SUCCESS on success
	 */
	SS_STATUS get_data_type(int* data_type, bool *sc_en);

	/**
	 * @brief	Set the number of samples for the SmartSensor to collect
	 * 			before issuing an interrupt
	 *
	 * @param[in]	thresh - Number of samples (1-255) to collect before interrupt
	 *
	 * @return SS_SUCCESS on success
	 */
	SS_STATUS set_fifo_thresh(int thresh);

	/**
	 * @brief	Get the number of samples the SmartSensor will collect
	 * 			before issuing an interrupt
	 *
	 * @param[out]	thresh - Number of samples (1-255) collected before interrupt
	 *
	 * @return SS_SUCCESS on success
	 */
	SS_STATUS get_fifo_thresh(int *thresh);

	/**
	 * @brief	Check that the SmartSensor is connected
	 */
	SS_STATUS ss_comm_check(void);

	//SS_STATUS ss_stay_in_bootloader(void);
	SS_STATUS ss_set_hub_mode(bio_sensor_mode_t mode);

	void ss_enable_irq(void);
	void ss_disable_irq(void);

	void ss_mfio_selftest(void);
	bool ss_reset_mfio_irq(void);

	void ss_execute_once(void);
	void ss_clear_interrupt_flag(void);


	void start_emul_mfio_event(float fPeriodinSecs);

	SS_STATUS max32664_soft_reset(void);
	SS_STATUS MAX30101_reset(void);                                                                       // Start MAX30101 reset Shutdown MAX32664A/B/C/D Added by Jason Chen, 2022.09.26
	SS_STATUS set_shut_down_enter(void);
	void set_shut_down_exit(void);

	SS_STATUS get_dhparams( uint8_t *response, int response_sz );
	SS_STATUS set_dhlocalpublic(  uint8_t *publicKey , int public_sz );
	SS_STATUS get_dhremotepublic( uint8_t *response, int response_sz );
	SS_STATUS get_authentication( uint8_t *response, int response_sz );

	void pipe_accel_FIFO(void);
	SS_STATUS enable_sensor_A(int idx, ss_data_req *data_req);
//SS_STATUS disable_sensor_A(int idx);
//extern char fw_version[128];
//extern char algo_version[128];

#endif
