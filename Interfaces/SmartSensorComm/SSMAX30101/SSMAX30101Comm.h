/***************************************************************************
 *
 *            Copyright (c) 2012 by Artafelx INC.
 *
 * This software is copyrighted by and is the sole property of
 * Artaflex INC.  All rights, title, ownership, or other interests
 * in the software remain the property of Artaflex INC.  This
 * software may only be used in accordance with the corresponding
 * license agreement.  Any unauthorized use, duplication, transmission,
 * distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior
 * written consent of Artaflex INC.
 *
 * Artaflex INC reserves the right to modify this software without notice.
 *
 * Artaflex INC.
 * 174 W Beaver Creek Rd.
 * Richmond Hill, ON, L4B 1B4,
 * Canada
 *
 * Modified by Jason Chen, 2021.12.15
 * Tel:   (905) 470-0109
 * http://www.artaflex.com
 *
 ***************************************************************************/

#ifndef _SSMAX30101COMM_H_
#define _SSMAX30101COMM_H_

#include "app.h"
#include "SSInterface.h"

#define INCLUDE_REDCNT_ANDROID_APK

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} accel_mode1_data;

typedef struct {
	uint8_t status;
	uint8_t sys_bp;
	uint8_t dia_bp;
	uint8_t prog;
	uint16_t hr;
	uint16_t spo2;
	uint16_t r_value;
#if defined(BPT_OLDER_THAN_40_2_7)
	uint8_t hr_excthresh;
#else
	uint8_t pulse_flag;
	uint16_t ibi;
	uint8_t spo2_conf;
#endif

} bpt_mode1_2_data;

typedef struct {
	uint32_t led1;
	uint32_t led2;
	uint32_t led3;
	uint32_t led4;
} max30101_mode1_data;

typedef struct {
	uint16_t hr;
	uint8_t hr_conf;
	uint16_t spo2;
	uint8_t status;
} whrm_mode1_data;

/* PRIVATE TYPE DEFINITIONS */
typedef enum _cmd_state_t {
  never_used,
  read_bpt_0,
  read_bpt_1,
  read_ppg_9,
  //bptv30,                                                            // For MAX32664D, read ppg data for V3 process by Jason
  //bptv31,                                                            // For MAX32664D, read ppg data for V3 process by Jason
  set_agc_dis,
  set_agc_en,
  set_cfg_bpt_med,
  set_cfg_bpt_sys_bp,
  set_cfg_bpt_dia_bp,
  get_cfg_bpt_cal_data,
  set_cfg_bpt_cal_data,
  set_cfg_bpt_date_time,
  set_cfg_bpt_spo2_coefs,
  self_test_ppg_os24,
  self_test_ppg_acc,
  set_cfg_bpt_nonrest,
  backup_shut_down,
  backup_wake_up,
  set_cfg_bpt_cal_index,                                 // Added by Jason
  set_cfg_bpt_sys_dia,                                   // Added by Jason
  NUM_CMDS_VALUE,
} cmd_state_t;

typedef struct __attribute__((packed)) {
	uint32_t start_byte	:8;

	uint32_t sample_cnt	:32;
	uint32_t led1	:20;
	uint32_t led2	:20;
	uint32_t led3	:20;
	uint32_t led4	:20;
	uint32_t x	    :14;	//Represent values of 0.000 through 8.191
	uint32_t y	    :14;	//Represent values of 0.000 through 8.191
	uint32_t z	    :14;	//Represent values of 0.000 through 8.191
	uint32_t hr	    :12;	//Represent values of 0.0 through 204.7
	uint32_t spo2	  :11;	//Represent values of 0.0 through 102.3 (only need up to 100.0)
	uint32_t status	:8;

	uint8_t	:0;			//Align CRC byte on byte boundary
	uint8_t crc8:8;
} ds_pkt_data_mode1;

typedef struct __attribute__((packed)) {
  uint32_t start_byte  :8;
  uint32_t status      :6;  // MYG was 4 bit originally
  uint32_t irCnt       :19;
  uint32_t redCnt      :19;
  uint32_t hr          :9;
  uint32_t prog        :9;
  uint32_t sys_bp      :9;
  uint32_t dia_bp      :9;
  uint32_t spo2        :8;
  uint32_t hr_excthresh:8;
  uint32_t r           :16;
  uint32_t ibi         :16;
  uint32_t spo2_conf   :8;
  uint32_t sample_count :32;                                                   // Added the sample for Jared time count, 20220113
  uint32_t temp         :16;                                                   // Added biosensor temperature value, 16bit, 20220111
  uint32_t batsoc       :16;                                                   // Added biosensor battery level(%) value, 16bit, 20220111
#if BPT_ACCEL_ENABLE
  uint32_t accl_x       :16;                                                   // Added by Jason Chen, 2022.05.16
  uint32_t accl_y       :16;
  uint32_t accl_z       :16;//32 bytes
#endif
  uint32_t dummy       :3;
  uint8_t              :0; //Align to next byte
  uint8_t  crc8        :8;
} ds_pkt_bpt_data;                                                            // for Max32664D, HR SPO2, BP

typedef struct __attribute__((packed)) {
    uint8_t reportCnt;
    uint8_t status;
    uint8_t prog;
    uint8_t hr;
    uint8_t spo2;
    uint8_t sys_bp;
    uint8_t dia_bp;
    uint8_t batsoc;
    uint16_t temp0;
    uint16_t batvcell;
    uint32_t cali_date;
    uint8_t statusa;
    uint8_t statusb;
    uint16_t crate;
    uint8_t crc8;
} ds_pkt_bptv3_data;                                                            // for Max32664D, HR SPO2, BP

typedef struct __attribute__((packed)){
	uint8_t current_operating_mode;  // mode 1 & 2
	// WHRM data
	uint16_t hr;         	           // mode 1 & 2
	uint8_t  hr_conf;     	         // mode 1 & 2
	uint16_t rr;         	           // mode 1 & 2
	uint8_t  rr_conf;		             // mode 1 & 2
	uint8_t  activity_class;         // mode 1 & 2
	// WSPO2 data
	uint16_t r;						           // mode 1 & 2
	uint8_t  spo2_conf;		           // mode 1 & 2
	uint16_t spo2;			             // mode 1 & 2
	uint8_t  percentComplete;		     // mode 1 & 2
	uint8_t  lowSignalQualityFlag;	 // mode 1 & 2
	uint8_t  motionFlag;			       // mode 1 & 2
	uint8_t  lowPiFlag;				       // mode 1 & 2
	uint8_t  unreliableRFlag;		     // mode 1 & 2
	uint8_t  spo2State;   			     // mode 1 & 2
	uint8_t  scd_contact_state;
	//Extended Report (mode2)
	uint32_t walk_steps;	           // mode 2
	uint32_t run_steps;		           // mode 2
	uint32_t kcal;			             // mode 2
	uint32_t totalActEnergy;		     // mode 2
	uint8_t  is_led_cur1_adj;	       // mode 2
	uint16_t adj_led_cur1;	         // mode 2
	uint8_t  is_led_cur2_adj;        // mode 2
	uint16_t adj_led_cur2;	         // mode 2
	uint8_t  is_led_cur3_adj;        // mode 2
	uint16_t adj_led_cur3;	         // mode 2
	uint8_t  is_int_time_adj;	        // mode 2
	uint8_t  t_int_code;	            // mode 2
	uint8_t  is_f_smp_adj;	          // mode 2
	uint8_t  adj_f_smp;		            // mode 2
	uint8_t  smp_ave;		              // mode 2
	uint8_t  hrm_afe_state;           // mode 2
	uint8_t  is_high_motion;	        // mode 2
	uint8_t  ibi_offset;
} whrm_wspo2_suite_modeX_data;

/* TODO: Implement:  REVIEW FIELDS!!!*/
typedef struct __attribute__((packed)) {

    uint32_t start_byte			 :8;
	uint32_t sample_cnt			   :8;
	uint32_t sampleTime;

	uint32_t grnCnt				     :20;
	uint32_t grn2Cnt			     :20;
	uint32_t irCnt	    		   :20;
	uint32_t redCnt				     :20;

	uint32_t x					       :14;	//Represent values of 0.000 through 8.191
	uint32_t y					       :14;	//Represent values of 0.000 through 8.191
	uint32_t z					       :14;	//Represent values of 0.000 through 8.191

	uint32_t curr_opmode         :4;
	uint32_t hr					         :12;	//Represent values of 0.0 through 204.7
	uint32_t hr_confidence  	   :8;	//Represent values of 0.0 through 100
	uint32_t rr					         :14;	//
	uint32_t rr_confidence  	   :8;	//Represent values of 0.0 through 100
	uint32_t activity			       :4;
	uint32_t r					         :12;
	uint32_t spo2_confidence  	 :8;	//Represent values of 0.0 through 100
	uint32_t spo2				         :11;	//Represent values of 0.0 through 102.3 (only need up to 100.0)
	uint32_t percentComplete	   :8;
	uint32_t lowSignalQualityFlag:1;
	uint32_t motionFlag			     :1;
	uint32_t lowPiFlag			     :1;
	uint32_t unreliableRflag     :1;
	uint32_t spo2State			     :4;
	uint32_t scdState			       :4;
	uint32_t ibiOffset           :8;
	uint8_t						           :0;			//Align CRC byte on byte boundary
	uint8_t crc8				         :8;

} ds_pkt_data_mode1_whrm_wspo2_suite;

typedef struct __attribute__((packed)) {

  uint32_t start_byte          :8;
  uint32_t sample_cnt          :8;
  uint32_t sampleTime          :32;

  uint32_t grnCnt              :20;
  uint32_t grn2Cnt             :20;
  uint32_t irCnt               :20;
  uint32_t redCnt              :20;

  uint32_t x                   :14;	//Represent values of 0.000 through 8.191
  uint32_t y                   :14;	//Represent values of 0.000 through 8.191
  uint32_t z                   :14;	//Represent values of 0.000 through 8.191

  uint32_t curr_opmode         :4;
  uint32_t hr                  :12;	//Represent values of 0.0 through 204.7
  uint32_t hr_confidence       :8;	//Represent values of 0.0 through 100
  uint32_t rr                  :14;	//Represent values of 0.0 through 102.3 (only need up to 100.0)
  uint32_t rr_confidence       :8;	//Represent values of 0.0 through 100
  uint32_t activity            :4;
  uint32_t r                   :12;
  uint32_t spo2_confidence     :8;	//Represent values of 0.0 through 100
  uint32_t spo2                :11;	//Represent values of 0.0 through 102.3 (only need up to 100.0)
  uint32_t percentComplete     :8;
  uint32_t lowSignalQualityFlag:1;
  uint32_t motionFlag          :1;
  uint32_t lowPiFlag           :1;
  uint32_t unreliableRflag     :1;
  uint32_t spo2State           :4;
  uint32_t scdState            :4;

  uint32_t walk_steps          :32;
  uint32_t run_steps           :32;
  uint32_t kcal                :32;
  uint32_t totalActEnergy      :32;
  uint32_t sample_count        :32;                                                   // Added the sample for Jared time count, 20220113
  uint32_t temp                :16;                                                   // Added biosensor temperature value, 16bit, 20220111
  uint32_t batsoc              :16;                                                   // Added biosensor battery level(%) value, 16bit, 20220111
  uint32_t ibiOffset           :8;
  uint8_t                      :0;			//Align CRC byte on byte boundary
  uint8_t crc8                 :8;
} ds_pkt_data_mode_maxim_sensors_app;                                           // for Max32664A/Z (10.4.1), HR SPO2, and HRV.....

void SSMAX30101Comm_init(void);

// sensor and algo status
  //status_algo_sensors_st sensor_algo_en_dis_;

//extern bool             cal_data_flag;
extern volatile uint8_t data_report_mode;

//inline bool get_cal_data_flag(void)  {
	//return cal_data_flag;
//}

inline bool is_active_measurement(void)  {
	bool is_asctive_mes = ( data_report_mode == read_bpt_0 || data_report_mode == read_bpt_1 /*|| data_report_mode == read_ppg_4*/ || data_report_mode == read_ppg_9 )? true:false;
	return is_asctive_mes;
}

/**
 * @brief	SSMAX30101Comm Command handler class for communication with MAX30101 on SmartSensor board
 * @details
 */
//class SSMAX30101Comm:	public SensorComm
typedef void(*set_sensorhub_accel_callback)(void);
typedef struct
{
	get_type_callback            get_type;
	is_visible_callback          is_visible;
	get_part_info_callback       get_part_info;
	//is_enabled_callback          is_enabled;
	get_part_name_callback       get_part_name;
	get_algo_ver_callback        get_algo_ver;
	stop_callback                stop;
	parse_command_callback       parse_command;
	data_report_execute_callback data_report_execute;
	set_sensorhub_accel_callback set_sensorhub_accel;

	SensorComm_Set_Ble_Status_callback SensorComm_Set_Ble_Status;
} SSMAX30101Comm;

extern SSMAX30101Comm ssMAX30101;

#endif /* _SSMAX30101COMM_H_ */
