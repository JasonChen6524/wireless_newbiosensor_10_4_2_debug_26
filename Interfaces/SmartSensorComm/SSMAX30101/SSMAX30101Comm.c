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
#include "app.h"
#include "SSInterface.h"
#include "SSMAX30101Comm.h"
#include "CRC8.h"

#define ENABLE_WHRM_AND_SP02x
#define ENABLE_WHRM_AND_SP02_MODE2
#define ENABLE_BPT

#define ENABLE_SS_MAX30101
//#define ENABLE_ACCEL

static bool send_battery_flag = false;
volatile uint8_t data_report_mode;
static uint8_t reportCnt = 0;
volatile bool m_sensorcomm_ble_interface_exists_;

uint8_t sample_count;
uint32_t sample_count32;
#define bptFifoThreh               1

//char cal_buf[1720];
char charbuf[1024 + 64] __attribute__((aligned(4)));                                 // [1792]

char cal_buf[1024 + 64];//824];//[512];cal_buf
addr_val_pair reg_vals[64];

struct queue_t max30101_queue;
uint8_t max30101_queue_buf[8 * sizeof(max30101_mode1_data)];

#ifdef ENABLE_WHRM_AND_SP02
struct queue_t whrm_queue;
uint8_t whrm_queue_buf[8 * sizeof(whrm_mode1_data)];
#endif

struct queue_t accel_queue;
uint8_t accel_queue_buf[8 * sizeof(accel_mode1_data)];

struct queue_t bpt_queue;
uint8_t bpt_queue_buf[8 * sizeof(bpt_mode1_2_data)];

struct queue_t whrm_wspo2_suite_queue;
uint8_t whrm_wspo2_suite_queue_buf[16 * sizeof(whrm_wspo2_suite_modeX_data)];

ss_data_req accel_mode1_data_req;                                                 // Wellness library report needs it
ss_data_req max30101_mode1_data_req;                                              // Wellness library report needs it
ss_data_req whrm_wspo2_suite_mode2_data_req;                                      // for Wellness library report.

//ss_data_req whrm_mode1_data_req;
ss_data_req agc_mode1_data_req;
ss_data_req bpt_mode1_2_data_req;

#ifdef ENABLE_WHRM_AND_SP02
ss_data_req whrm_wspo2_suite_mode1_data_req;
#endif

bool agc_enabled = true;

static const char* sensor_type;
static uint8_t     vis;

/* PRIVATE CONST VARIABLES */
#define SSMAX30101_REG_SIZE                  1
#define SSMAX30101_MODE1_DATASIZE            12 	         //Taken from API doc
#define SSWHRM_MODE1_DATASIZE                6			       //Taken from API doc
#define SSACCEL_MODE1_DATASIZE               6		         //Taken from API doc
#define SSAGC_MODE1_DATASIZE                 0			       //Taken from API doc

#define SSWHRM_WSPO2_SUITE_MODE1_DATASIZE    21            // Taken from API doc
#define SSWHRM_WSPO2_SUITE_MODE2_DATASIZE    53

#if defined(BPT_OLDER_THAN_40_2_7)
	static const int SSBPT_MODE1_2_DATASIZE = 11;		         //Taken from API doc
#elif defined(BPT_OLDER_THAN_40_4_0)
	static const int SSBPT_MODE1_2_DATASIZE = 14;		         //Taken from API doc
#else
	static const int SSBPT_MODE1_2_DATASIZE = 17;
#endif

static const char* const cmd_tbl[] = {
    "never used",
    "read bpt 0",                                     // For MAX32664D, read ppg 0 for Calibration by Jason
    "read bpt 1",                                     // For MAX32664D, read ppg 1 for measurement by Jason, /* Only Algo Mode */
    "read ppg 9",                                     // For MAX32664A(10.4.1), read ppg 9 for HRV....measurement by Jason
    //"bptv30",                                          // For MAX32664D, read ppg data for V3 process by Jason
    //"bptv31",                                          // For MAX32664D, read ppg data for V3 process by Jason
    "set_cfg ppg agc 0",
    "set_cfg ppg agc 1",
    "set_cfg bpt med",
    "set_cfg bpt sys_bp",
    "set_cfg bpt dia_bp",
	  "get_cfg bpt cal_result",
    "set_cfg bpt cal_result",
	  "set_cfg bpt date_time",
    "set_cfg bpt spo2_coefs",
	  "self_test ppg os24",
    "self_test ppg acc",
    "set_cfg bpt nonrest",
    "backup_shut_down",
    "backup_wake_up",
    "set_cfg bpt cal_index",                                       // Added by Jason
    "set_cfg bpt sys_dia"                                          // Added by Jason
};

SSMAX30101Comm ssMAX30101;

void accel_data_rx(uint8_t* data_ptr)                                                             // for Acclerometer X, Y ,X        by Jason
{
	//See API doc for data format
	accel_mode1_data sample;
	sample.x = (data_ptr[0] << 8) | data_ptr[1];
	sample.y = (data_ptr[2] << 8) | data_ptr[3];
	sample.z = (data_ptr[4] << 8) | data_ptr[5];

//pr_info("accX= %d , accY=%d, accZ=%d  \r\n", sample.x, sample.y , sample.z);                                   // Commented by Jason
  ssmax_log("accX= %d , accY=%d, accZ=%d  \r\n", sample.x, sample.y , sample.z);                                 // Added by Jason, 2022.05.04
  enqueue(&accel_queue, &sample);
}

void max30101_data_rx(uint8_t* data_ptr)                                                                     // for Led1,Led2, Led3, Led4, wellness library API report ppg 9 needs it for streaming format by Jason
{
	max30101_mode1_data sample;
	sample.led1 = (data_ptr[0] << 16) | (data_ptr[1] << 8) | data_ptr[2];
	sample.led2 = (data_ptr[3] << 16) | (data_ptr[4] << 8) | data_ptr[5];
	sample.led3 = (data_ptr[6] << 16) | (data_ptr[7] << 8) | data_ptr[8];
	sample.led4 = (data_ptr[9] << 16) | (data_ptr[10] << 8) | data_ptr[11];

  pr_info("led1=%.6X led2=%.6X led3=%.6X led4=%.6X\r\n", sample.led1, sample.led2, sample.led3, sample.led4);            // Commented by Jason
  enqueue(&max30101_queue, &sample);
}

void whrm_wspo2_suite_data_rx_mode2(uint8_t* data_ptr)                                               // for wellness library API report by Jason
{
	whrm_wspo2_suite_modeX_data sample;

	sample.current_operating_mode =  data_ptr[0];
	//whrm
	sample.hr 					          = (data_ptr[1] << 8) | data_ptr[2];
	sample.hr_conf 				        =  data_ptr[3];
	sample.rr 					          = (data_ptr[4] << 8) | data_ptr[5];
	sample.rr_conf 				        =  data_ptr[6];
	sample.activity_class 		    =  data_ptr[7];
	//activity
	sample.walk_steps 			      = (data_ptr[8]  << 24) | (data_ptr[9]  << 16) | (data_ptr[10] << 8) | data_ptr[11];
	sample.run_steps              = (data_ptr[12] << 24) | (data_ptr[13] << 16) | (data_ptr[14] << 8) | data_ptr[15];
	sample.kcal 				          = (data_ptr[16] << 24) | (data_ptr[17] << 16) | (data_ptr[18] << 8) | data_ptr[19];
	sample.totalActEnergy         = (data_ptr[20] << 24) | (data_ptr[21] << 16) | (data_ptr[22] << 8) | data_ptr[23];

	sample.is_led_cur1_adj        =  data_ptr[24];
	sample.adj_led_cur1           = (data_ptr[25] << 8) | data_ptr[26];
	sample.is_led_cur2_adj        =  data_ptr[27];
	sample.adj_led_cur2           = (data_ptr[28] << 8) | data_ptr[29];
	sample.is_led_cur3_adj        =  data_ptr[30];
	sample.adj_led_cur3           = (data_ptr[31] << 8) | data_ptr[32];

	sample.is_int_time_adj        =  data_ptr[33];
	sample.t_int_code             =  data_ptr[34];
	sample.is_f_smp_adj           =  data_ptr[35];
	sample.adj_f_smp 			        =  data_ptr[36];
	sample.smp_ave                =  data_ptr[37];
	sample.hrm_afe_state          =  data_ptr[38];
	sample.is_high_motion         =  data_ptr[39];
	sample.scd_contact_state      =  data_ptr[40];
	//wspo2
	sample.r 					            = (data_ptr[41] << 8) | data_ptr[42];
	sample.spo2_conf 			        =  data_ptr[43];
	sample.spo2 				          = (data_ptr[44] << 8) | data_ptr[45];
	sample.percentComplete 		    =  data_ptr[46];
	sample.lowSignalQualityFlag   =  data_ptr[47];
	sample.motionFlag 			      =  data_ptr[48];
	sample.lowPiFlag 			        =  data_ptr[49];
	sample.unreliableRFlag 		    =  data_ptr[50];
	sample.spo2State 			        =  data_ptr[51];
	sample.ibi_offset             =  data_ptr[52];

	enqueue(&whrm_wspo2_suite_queue, &sample);
  //pr_info("hr=%.1f conf=%d rr=%.1f rr_conf=%d status=%d\r\n", (float)sample.hr / 10.0, sample.hr_conf, (float)sample.rr/10, sample.rr_conf,sample.activity_class);
    pr_info("hr=%.1f spo2=%0.1f rr=%.1f rr_conf=%d status=%d\r\n", (float)sample.hr / 10.0, (float)sample.spo2/10.0, (float)sample.rr/10, sample.rr_conf,sample.activity_class);
#if 0//DEBUG_LEVEL
    uint16_t hr21 = sample.hr / 10;

    uint16_t spo21 = sample.spo2/10;
    uint16_t spo22 = sample.spo2 - spo21 * 10;

    uint16_t rr21 = sample.rr/10;
    uint16_t rr22 = sample.rr - rr21 * 10;

    ssmax_log("Mode 2 hr=%3d spo2=%3d.%d rr=%4d.%d rr_conf=%3d status=%d\r\n", hr21, spo21, spo22, rr21, rr22, sample.rr_conf,sample.activity_class);                         // Added by Jason
#endif
}

void bpt_data_rx(uint8_t* data_ptr)
{
	bpt_mode1_2_data sample;
	sample.status  = data_ptr[0];
	sample.prog    = data_ptr[1];
	sample.hr      = (data_ptr[2] << 8) | data_ptr[3];
	sample.sys_bp  = data_ptr[4];
	sample.dia_bp  = data_ptr[5];
	sample.spo2    = (data_ptr[6] << 8) | data_ptr[7];
	sample.r_value = (data_ptr[8] << 8) | data_ptr[9];
#if defined(BPT_OLDER_THAN_40_2_7)
	sample.hr_excthresh = data_ptr[10];
#else
	sample.pulse_flag = data_ptr[10];
	sample.ibi = (data_ptr[11] << 8) | data_ptr[12];
	sample.spo2_conf = data_ptr[13];

#endif
	enqueue(&bpt_queue, &sample);

#if 0//DEBUG_LEVEL
    uint16_t hr21 = sample.hr / 10;

    uint16_t spo21 = sample.spo2/10;
    uint16_t spo22 = sample.spo2 - spo21 * 10;

	ssmax_log("[MAX30101:%d] status=%d prog=%d hr=%d spo2=%d.%d sys=%d dia=%d ibi=%d\r\n", __LINE__, sample.status, sample.prog, hr21, spo21, spo22, sample.sys_bp, sample.dia_bp, sample.ibi);
#endif
}

#ifdef ENABLE_WHRM_AND_SP02
void whrm_data_rx(uint8_t* data_ptr)
{
	//See API doc for data format
	whrm_mode1_data sample;
	sample.hr      = (data_ptr[0] << 8) | data_ptr[1];
	sample.hr_conf = data_ptr[2];
	sample.spo2    = (data_ptr[3] << 8) | data_ptr[4];
	sample.status  = data_ptr[5];

	pr_info("hr=%.1f conf=%d spo2=%d status=%d\r\n", (float)sample.hr / 10.0, sample.hr_conf, sample.spo2, sample.status);
	enqueue(&whrm_queue, &sample);
}
#endif

void agc_data_rx(uint8_t* data_ptr)
{
  (void)data_ptr;
	//NOP: AGC does not collect data
}

void whrm_wspo2_suite_data_rx_mode1(uint8_t* data_ptr)
{
	whrm_wspo2_suite_modeX_data sample;
	sample.current_operating_mode = data_ptr[0];
	//whrm
	sample.hr                     = (data_ptr[1] << 8) | data_ptr[2];
	sample.hr_conf                = data_ptr[3];
	sample.rr                     = (data_ptr[4] << 8) | data_ptr[5];
	sample.rr_conf                = data_ptr[6];
	sample.activity_class         = data_ptr[7];
	//wspo2
	sample.r                      = (data_ptr[8] << 8) | data_ptr[9];  // already x1000
	sample.spo2_conf              = data_ptr[10];
	sample.spo2                   = (data_ptr[11] << 8) | data_ptr[12]; // already x10
	sample.percentComplete        = data_ptr[13];
	sample.lowSignalQualityFlag   = data_ptr[14];
	sample.motionFlag             = data_ptr[15];
	sample.lowPiFlag              = data_ptr[16];
	sample.unreliableRFlag        = data_ptr[17];
	sample.spo2State              = data_ptr[18];
	sample.scd_contact_state 	  = data_ptr[19];
	sample.ibi_offset             = data_ptr[20];  // in place of ibi offset!

	//printf("WHRM DATA PARSER \r\n");

	enqueue(&whrm_wspo2_suite_queue, &sample);
  //printf("Mode 1 hr=%.1f conf=%d rr=%.1f rr_conf=%d status=%d  mot=%d\r\n", (float)sample.hr / 10.0, sample.hr_conf, (float)sample.rr/10, sample.rr_conf,sample.activity_class, sample.motionFlag);
	ssmax_log("Mode 1 hr=%d spo2=%d rr=%d rr_conf=%d mot=%d\r\n", (int)sample.hr / 10, (int)sample.spo2/10, (int)sample.rr/10, sample.rr_conf,sample.activity_class);                         // Added by Jason
}

static const char* get_type_00(void);
static uint8_t     is_visible_00(void);
static int         get_part_info_00(uint8_t *part_id, uint8_t *rev_id);
static uint8_t     is_enabled(void);
static void        SensorComm_Set_Ble_Status_00(bool status, uint8_t idx);
static const char* get_part_name_00(void);
static const char* get_algo_ver_00(void);
static void        stop_00(void);
static uint8_t     parse_command_00(const char* cmd);
static int         data_report_execute_00(char* buf, int size);

//void set_sensorhub_accel_tt(void);


void SSMAX30101Comm_init(void)                                                  //:SensorComm("ppg", true), m_USB(USB), ss_int(ssInterface), ds_int(dsInterface), agc_enabled(true)
{
	//sensor = NULL;
	sensor_type = "ppg";
	vis = true;

	ssMAX30101.get_type            = (get_type_callback)&get_type_00;
	ssMAX30101.is_visible          = (is_visible_callback)&is_visible_00;
	ssMAX30101.get_part_info       = (get_part_info_callback)&get_part_info_00;
//ssMAX30101.is_enabled          = (is_enabled_callback)&is_enabled_00;
	ssMAX30101.get_part_name       = (get_part_name_callback)&get_part_name_00;
	ssMAX30101.get_algo_ver        = (get_algo_ver_callback)&get_algo_ver_00;
	ssMAX30101.stop                = (stop_callback)&stop_00;
	ssMAX30101.parse_command       = (parse_command_callback)&parse_command_00;
	ssMAX30101.data_report_execute = (data_report_execute_callback)&data_report_execute_00;

	ssMAX30101.SensorComm_Set_Ble_Status    = (SensorComm_Set_Ble_Status_callback)&SensorComm_Set_Ble_Status_00;

	max30101_mode1_data_req.data_size              = SSMAX30101_MODE1_DATASIZE;
	max30101_mode1_data_req.rx_data_parser         = (rx_data_callback)&max30101_data_rx;                                  // for Led1,Led2, Led3, Led4       by Jason

	accel_mode1_data_req.data_size                 = SSACCEL_MODE1_DATASIZE;
	accel_mode1_data_req.rx_data_parser            = (rx_data_callback)&accel_data_rx;                                     // for Acclerometer X, Y ,X        by Jason

	whrm_wspo2_suite_mode2_data_req.data_size      = SSWHRM_WSPO2_SUITE_MODE2_DATASIZE;
	whrm_wspo2_suite_mode2_data_req.rx_data_parser = (rx_data_callback)&whrm_wspo2_suite_data_rx_mode2;                    // for wellness library API report by Jason

#ifdef ENABLE_WHRM_AND_SP02
	whrm_mode1_data_req.data_size      = SSWHRM_MODE1_DATASIZE;
	whrm_mode1_data_req.rx_data_parser = (rx_data_callback)&whrm_data_rx;

	whrm_wspo2_suite_mode1_data_req.data_size      = SSWHRM_WSPO2_SUITE_MODE1_DATASIZE;
	whrm_wspo2_suite_mode1_data_req.rx_data_parser = (rx_data_callback)&whrm_wspo2_suite_data_rx_mode1;
#endif

	agc_mode1_data_req.data_size        = SSAGC_MODE1_DATASIZE;
	agc_mode1_data_req.rx_data_parser   = (rx_data_callback)&agc_data_rx;

	bpt_mode1_2_data_req.data_size      = SSBPT_MODE1_2_DATASIZE;
	bpt_mode1_2_data_req.rx_data_parser = (rx_data_callback)&bpt_data_rx;

	queue_init(&max30101_queue, max30101_queue_buf, sizeof(max30101_mode1_data), sizeof(max30101_queue_buf));

#ifdef ENABLE_WHRM_AND_SP02
	queue_init(&whrm_queue, whrm_queue_buf, sizeof(whrm_mode1_data), sizeof(whrm_queue_buf));
#endif

	queue_init(&accel_queue, accel_queue_buf, sizeof(accel_mode1_data), sizeof(accel_queue_buf));
	queue_init(&bpt_queue, bpt_queue_buf, sizeof(bpt_mode1_2_data), sizeof(bpt_queue_buf));

	queue_init(&whrm_wspo2_suite_queue, whrm_wspo2_suite_queue_buf, sizeof(whrm_wspo2_suite_modeX_data), sizeof(whrm_wspo2_suite_queue_buf));

	//sensor_algo_en_dis_.sensorhub_accel = 1;   // enable sensor hub accel by default

	send_battery_flag = false;
}

static const char* get_part_name_00(void)
{
  return "max30101";
}

void set_sensorhub_accel_00(void)
{

}

int read_ppg_mode_9(const char* cmd){

  (void)cmd;
  sample_count = 0;sample_count32 = 0;
	SS_STATUS status;

	ss_disable_irq();
	//wait_ms(1000);
	status = set_data_type(SS_DATATYPE_BOTH, false);                         // cmd: 0x10 00 0x03
	if (status != SS_SUCCESS) {
	    pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
	    pr_info("FAILED at line %d\n", __LINE__);
	    return -1;
	}

	status = set_fifo_thresh(5);                                             // cmd: 0x10 01 0x05
	if (status != SS_SUCCESS) {
	    pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
	    pr_info("FAILED at line %d\n", __LINE__);
	    return -1;
	}

#if 0
	if (agc_enabled) {
	    status = ss_int->(SS_ALGOIDX_AGC, 1, &agc_mode1_data_req);
	} else {
	    status = ss_int->disable_algo(SS_ALGOIDX_AGC);
	}
	if (status != SS_SUCCESS) {
	    pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
	    pr_info("FAILED at line %d - agc_enabled: %d\n", __LINE__, agc_enabled);
	    ss_int->enable_irq();
	    return -1;
	}
#endif                                                                          // Move from line 1039 to here by Jason

	ss_disable_irq();

#ifdef ENABLE_SS_MAX30101
	status = enable_sensor(SS_SENSORIDX_MAX30101, 1, &max30101_mode1_data_req);   // cmd: 0x44, 03, 01 for Led1~Led4 Reading by Jason
	if (status != SS_SUCCESS) {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("                : %s MAX30301 enable fail,  MAX30101, line=%d\r\n", __LINE__);
#endif
	  //pr_info("FAILED at line %d\n", __LINE__);
	    ss_enable_irq();
	    return -1;
	}
#endif

  status = enable_sensor_A(SS_SENSORIDX_ACCEL, &accel_mode1_data_req);                      // cmd: 0x44, 04, 01, 01 for enabling host-side Accerometer by Jason
  pipe_accel_FIFO();
	if (status != SS_SUCCESS) {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("                : %s accel enable fail, MAX30101, line=%d\r\n", cmd, __LINE__);
#endif
	  //pr_info("FAILED at line %d\n", __LINE__);
	    ss_enable_irq();
	    return -1;
	}

#ifdef ENABLE_WHRM_AND_SP02_MODE2

	status = enable_algo(SS_ALGOIDX_WHRM_WSPO2_SUITE, 2, &whrm_wspo2_suite_mode2_data_req);//cmd: 0x52,07,02, mode 2 for HR,SPO2, RR ....Reading for Wellness library report by Jason
	if (status != SS_SUCCESS) {
	    pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
	    pr_info("FAILED at line %d\n", __LINE__);
	    ss_enable_irq();
	    return -1;
	}

#endif

	data_report_mode = read_ppg_9;
	ssmax_log("\r\n                : %s err=%d, MAX30101:Line:%d\r\n", cmd, COMM_SUCCESS, __LINE__);         // Modified by Jason
	ss_enable_irq();

	send_battery_flag = true;
	return  0;
}



#if 0
bool self_test_result_evaluate(const char *message, uint8_t result){
	// check i2c response status
	if(result != 0x00){
	    app_log_error("%s has failed %02X err<-1>\r\n", message, result);
	    if((result & FAILURE_COMM))
	      app_log_error("%s communication has failed err<-1>\r\n", message);
	    if(result & FAILURE_INTERRUPT)
	      app_log_error("%s interrupt pin check has failed err<-1>\r\n", message);
	    return false;
	}
	return true;
}

// TODO: convert this to PPG sensor test
int selftest_max30101(void){
  int ret;
	uint8_t test_result;
	bool test_failed = false;
	ssmax_log("starting selftest_max30101\r\n");
	// configure mfio pin for self test
	ss_mfio_selftest();
	ret = self_test(SS_SENSORIDX_MAX30101, &test_result, 500);
	if(ret != SS_SUCCESS){
	    app_log_error("self_test(SS_SENSORIDX_MAX30101, &test_result) has failed err = %d\r\n", ret);
	    test_failed = true;
	}
	// reset mfio pin to old state
	if(!ss_reset_mfio_irq()){
	    app_log_error("smart sensor reset_mfio_irq has failed err<-1>\r\n");
	    test_failed = true;
	}
	// reset the sensor to turn off the LED
	ret = sensorHub_reset();
	if(test_failed | !self_test_result_evaluate("selftest_max30101", test_result)){
	    return -1;
	}else{
	    return SS_SUCCESS;
	}
}

int selftest_accelerometer(void){
	int ret;
	uint8_t test_result;
	bool test_failed = false;
	ssmax_log("starting selftest_accelerometer\r\n");
	ret = self_test(SS_SENSORIDX_ACCEL, &test_result, 1000);
	if(ret != SS_SUCCESS){
	    app_log_error("self_test(SS_SENSORIDX_ACCEL, &test_result) has failed err = %d\r\n", test_result);
	    test_failed = true;
	}
	// reset the sensor to turn off the LED
	ret = sensorHub_reset();
	if(ret != SS_SUCCESS){
	    app_log_error("smart sensor reset has failed err = %d\r\n", ret);
	    test_failed = true;
	}
	if(test_failed | !self_test_result_evaluate("selftest_accelerometer", test_result)){
	    return -1;
	}else{
	    return SS_SUCCESS;
	}
}
#endif

static const char* get_type_00(void)
{
  return sensor_type;
}

static uint8_t is_visible_00(void)
{
  return vis;
}

static int get_part_info_00(uint8_t *part_id, uint8_t *rev_id)
{
  //if (sensor != NULL)
    //{
	//	return sensor->get_part_info(part_id, rev_id);
	//}
	//else
	{
	  *part_id = 0xFF;
		*rev_id = 0xFF;
		return -1;
	}
}

static uint8_t is_enabled(void)
{
  return (data_report_mode != 0);
}


static void SensorComm_Set_Ble_Status_00(bool status, uint8_t idx)
{
  (void)idx;
  pr_debug("Setting ble status: %d %10s     ", status, sensor_type);
	m_sensorcomm_ble_interface_exists_ = status;
}

static const char* get_algo_ver_00(void)
{
	return get_ss_algo_version();
}

uint8_t cal_index_falg = 0;
uint8_t index_val      = 0xFF;
char cmd_bptv31_call[] = "bptv31 20220105";
static uint8_t parse_command_00(const char* cmd)
{
  int ret,i;
	int data_len = 0;
	bool ble_response = false;
//char cal_str_to_be_set[650];
  SS_STATUS status = 0;
  bool recognizedCmd = false;

  for (i = 0; i < NUM_CMDS_VALUE; i++)
  {
      if (starts_with(cmd, cmd_tbl[i]))
      {
          cmd_state_t user_cmd = (cmd_state_t)i;
          recognizedCmd = true;

          switch (user_cmd)
          {
            /* BP Calibration */
            case read_bpt_0:
            {
              sample_count = 0;sample_count32 = 0;
              ok_to_sleep = true;                                               // Jason 2021.12.09
              BLE_reset_queue();

              //pr_info("___DEBUG: Calibration init started \r\n");

              status = set_data_type(SS_DATATYPE_BOTH, false);
              if (status != SS_SUCCESS) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                  pr_info("FAILED at line %d\n", __LINE__);
                  break;
              }

              status = set_fifo_thresh(bptFifoThreh);
              if (status != SS_SUCCESS) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                  pr_info("FAILED at line %d\n", __LINE__);
                  break;
              }

    #if 0
              if (agc_enabled) {
                  status = ss_int->enable_algo(SS_ALGOIDX_AGC, 1, &agc_mode1_data_req);
              } else {
                  status = ss_int->disable_algo(SS_ALGOIDX_AGC);
              }

              if (status != SS_SUCCESS) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                  pr_info("FAILED at line %d - agc_enabled: %d\n", __LINE__, agc_enabled);
                  ss_enable_irq();
                  break;
              }
    #endif
              ss_disable_irq();

    #ifdef ENABLE_SS_MAX30101
              status = enable_sensor(SS_SENSORIDX_MAX30101, 1, &max30101_mode1_data_req);
              if (status != SS_SUCCESS) {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
              app_log_error("                : %s MAX30301 enable fail err=%d, line=%d\r\n", cmd, COMM_GENERAL_ERROR, __LINE__);
#endif
                  pr_info("FAILED at line %d\n", __LINE__);
                  ss_enable_irq();
                break;
              }
    #endif

    #ifdef ENABLE_BPT
              status = enable_algo(SS_ALGOIDX_BPT, 1, &bpt_mode1_2_data_req);
              if (status != SS_SUCCESS) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                  pr_info("FAILED at line %d\n", __LINE__);
                  ss_enable_irq();
                  break;
              }
    #endif
              data_report_mode = read_bpt_0;
              pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);

              send_battery_flag = true;

              ss_enable_irq();
              ble_response = true;
              break;
            }

            /* BP Estimation */
            case read_bpt_1:
            {
              sample_count = 0; sample_count32 = 0;
              ok_to_sleep = true;                                               // Jason 2021.12.09
              BLE_reset_queue();

              //pr_info("___DEBUG: Estimation init started \r\n");

              status = set_data_type(SS_DATATYPE_BOTH, false);
              if (status != SS_SUCCESS) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);                  // Added by Jason
                  break;
              }

              status = set_fifo_thresh(bptFifoThreh);
              if (status != SS_SUCCESS) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);                  // Added by Jason
                  break;
              }

              if (agc_enabled) {
                  status = enable_algo(SS_ALGOIDX_AGC, 1, &agc_mode1_data_req);
              } else {
                  status = disable_algo(SS_ALGOIDX_AGC);
              }

              if (status != SS_SUCCESS) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);                   // Added by Jason
                  ss_enable_irq();
                break;
              }

              ss_disable_irq();
    #ifdef ENABLE_SS_MAX30101
              status = enable_sensor(SS_SENSORIDX_MAX30101, 1, &max30101_mode1_data_req);
              if (status != SS_SUCCESS) {
     #if APP_LOG_ERROR_ENABLE                                                   //Added by Jason Chen, 2022.1102
              app_log_error("                : %s MAX30301 enable fail err=%d, line=%d\r\n", cmd, COMM_GENERAL_ERROR, __LINE__);
     #endif
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);                   // Added by Jason
                  ss_enable_irq();
                  break;
              }
    #endif

    #ifdef ENABLE_BPT
              status = enable_algo(SS_ALGOIDX_BPT, 2, &bpt_mode1_2_data_req);                                // bptExecModeEstimation = 2, by Jason
              if (status != SS_SUCCESS) {
                  pr_info("\r\n%s err=%d\r\n", cmd, status);
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);               // Added by Jason
                  ss_enable_irq();
                  break;
              }
    #endif
              data_report_mode = read_bpt_1;
            //ssmax_log("%s err=%d\r\n", cmd, COMM_SUCCESS);
              data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);                  // Added by Jason

              send_battery_flag = true;

              ss_enable_irq();

              ble_response = true;

            } break;

            case read_ppg_9:
            {
              int ret = read_ppg_mode_9(cmd);
    #if 0
              if(ret == 0)
                pr_info("%s err=%d, Max30101:Line=%d\r\n", cmd, COMM_SUCCESS, __LINE__);                 // Modified by Jason
              else
                pr_info("%s err=%d, Max30101:Line=%d\r\n", cmd, COMM_GENERAL_ERROR, __LINE__);           // Modified by Jason
    #else
                if (ret == SS_SUCCESS)
                {
                    //for (i = 0; i < sizeof(rxBuff)-1; i++)
                    //  str_idx += snprintf(outstr + str_idx, sizeof(outstr) - str_idx - 1, "%02X", rxBuff[i+1]);
                    data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                } else{
                    data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                }
    #endif
                ble_response = true;
                break;
            }

            case get_cfg_bpt_cal_data:
            {
              char cal_str[2*512+1];
              int ii,str_idx = 0;

              data_report_mode = get_cfg_bpt_cal_data;                                                           // Nov18 by Jason Chen, 2021.11.18

              status = get_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, (uint8_t*)&charbuf[0], 513);
              if (status != SS_SUCCESS) {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
                  app_log_error("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
#endif
                  data_len = snprintf(charbuf, sizeof(charbuf),"\r\n%s err=%d\r\n", cmd,COMM_GENERAL_ERROR);
                  break;
              }

               for (ii = 0; ii < 512; ii++)
                 str_idx += snprintf(cal_str + str_idx, sizeof(cal_str) - str_idx, "%02X", charbuf[ii+1]);

              //cal_data_flag  = true;

              data_len = snprintf(charbuf, sizeof(charbuf),"\r\n%s value=%s err=%d\r\n", cmd, cal_str, COMM_SUCCESS);
              snprintf(cal_buf, sizeof(cal_buf),"\r\n%s value=%s err=%d\r\n", cmd, cal_str, COMM_SUCCESS);             // Nov18 by Jason Chen, 2021.11.18

              ble_response = false;

            } break;

            case set_cfg_bpt_cal_data:
            {
              uint8_t cal_data[512+1];
              int k = i;
              //static bool cal_data_flag = false;

              memset(cal_buf, 0, sizeof(cal_buf));                                                                             // Nov18 by Jason Chen
              data_report_mode = 0;//set_cfg_bpt_cal_data;

              ret = parse_cmd_hexstr(cmd, cmd_tbl[k], cal_data, 512);//sizeof(cal_data));
              if (ret) {
                  data_len = snprintf(cal_buf,sizeof(cal_buf),"%s err=%d\r\n", cmd, COMM_INVALID_PARAM);                       // Nov18 by Jason Chen
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
              app_log_error("Len=%4d, %s.... err=%d, Line:%d\r\n\r\n", data_len, charbuf, COMM_INVALID_PARAM, __LINE__);
#endif
                  data_len = 0;
                  break;
              }

              if(!(cal_index_falg & (1 << index_val)))
              {
                  status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, &cal_data[0], 512);//sizeof(cal_data));
                  cal_index_falg |= (1 << index_val);
              }
              else
              {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
              app_log_error("---->The calibration data (index=%d) exists..., Line=%d\r\n", index_val, __LINE__);
#endif
                  status = SS_SUCCESS;
              }
              if (status == SS_SUCCESS)
              {
                //data_len = snprintf(charbuf,sizeof(charbuf),"\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
                  data_len = snprintf(charbuf,sizeof(charbuf),"\r\nset_cfg bpt cal_result err=0\r\n");
              }
              else
              {
                //snprintf(charbuf,sizeof(charbuf),"\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
                  data_len = snprintf(charbuf,sizeof(charbuf),"\r\nset_cfg bpt cal_result err=-1\r\n");
              }
              //ble_response = false;
              ble_response = true;

            } break;

            case set_cfg_bpt_date_time:
            {
              // Date: yymmdd  Time: hhmmss
              uint32_t date_time[2];

              ret = (parse_cmd_data32(cmd, cmd_tbl[i], date_time, 2, false) != 2);
              if (ret) {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
                  app_log_error("%s err=%d\r\n\r\n", cmd, COMM_INVALID_PARAM);
#endif
                  break;
              }

              status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_EST_DATE, (uint8_t*)date_time, sizeof(date_time));
    #if 1
              if (status == SS_SUCCESS)
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d, MAX30101:Line=%d\r\n", cmd, COMM_SUCCESS, __LINE__);
              else
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d, MAX30101:Line=%d\r\n", cmd, COMM_GENERAL_ERROR, __LINE__);
              ble_response = false;
    #else
              if (status == SS_SUCCESS){
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              } else{
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
              }
              ble_response = true;
    #endif
            } break;

            case set_cfg_bpt_spo2_coefs:
            {
              uint32_t coefs[3];
              if(3 != parse_cmd_data32(cmd, cmd_tbl[i], coefs, 3, true)){
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
                  app_log_error("%s err=%d\r\n\r\n", cmd, COMM_INVALID_PARAM);
#endif
                  break;
              }
              // Reformat for MSB first
              uint8_t CalCoef[12] = { (uint8_t)((coefs[0] >> (3*8)) & 0xFF),  (uint8_t)((coefs[0] >> (2*8)) & 0xFF), (uint8_t)((coefs[0] >> (1*8)) & 0xFF), (uint8_t)((coefs[0] >> (0*8)) & 0xFF), // A
                                      (uint8_t)((coefs[1] >> (3*8)) & 0xFF),  (uint8_t)((coefs[1] >> (2*8)) & 0xFF), (uint8_t)((coefs[1] >> (1*8)) & 0xFF), (uint8_t)((coefs[1] >> (0*8)) & 0xFF), // B
                                      (uint8_t)((coefs[2] >> (3*8)) & 0xFF),  (uint8_t)((coefs[2] >> (2*8)) & 0xFF), (uint8_t)((coefs[2] >> (1*8)) & 0xFF), (uint8_t)((coefs[2] >> (0*8)) & 0xFF)  // C
                                    };

              status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_SPO2_COEFS, &CalCoef[0], sizeof(CalCoef));
    #if 0
              if (status == SS_SUCCESS)
              {
                  ssmax_log("%s err=%d\r\n\r\n", cmd, COMM_SUCCESS);
              }
              else
              {
                  ssmax_log("%s err=%d\r\n\r\n", cmd, COMM_GENERAL_ERROR);
              }
    #else
              if (status == SS_SUCCESS){
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              } else{
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
              }
              ble_response = true;
    #endif
            } break;

            case set_cfg_bpt_sys_dia:                                                                                        // Added by Jason
            {
              uint8_t val[3];
              ret = (parse_cmd_data(cmd, cmd_tbl[i], val, 3, ' ', false) != 3);
              if (ret == SS_SUCCESS){
                  ;//data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              } else{
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
                  break;
              }

              status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_SYS_DIA, &val[0], 3);
              if (status == SS_SUCCESS){                                                                                   // Added by Jason
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);                   // Added by Jason
              } else{
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);             // Added by Jason
              }

              ble_response = true;

            }break;

            case set_cfg_bpt_cal_index:
            {
              uint8_t val;

              ret = (parse_cmd_data(cmd, cmd_tbl[i], &val, 1, ' ', false) != 1);
              if (ret == SS_SUCCESS){
                  ;//data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);

              } else{
                  //ssmax_log("%s err=%d\r\n\r\n", cmd, COMM_INVALID_PARAM);
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
                  break;
              }
              if(val > 4)
              {
                  //ssmax_log("%s err=%d\r\n\r\n", cmd, COMM_INVALID_PARAM);
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
                  break;
              }

              index_val = val;
              if(!(cal_index_falg & (1 << val)))
              {
                  status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_INDEX, &val, 1);                                  // Added by Jason
              }
              else
              {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
                  app_log_error("--->The index value = %d exists...\r\n", val);
#endif
                  status = SS_SUCCESS;
              }

              if (status == SS_SUCCESS){                                                                                   // Added by Jason
                  //ssmax_log("%s err=%d\r\n\r\n", cmd, COMM_SUCCESS);
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);                   // Added by Jason
              } else{
                  //ssmax_log("%s err=%d\r\n\r\n", cmd, COMM_INVALID_PARAM);
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);             // Added by Jason
              }
              ble_response = true;
            }break;
#if 0
            case set_agc_dis:
            {
              agc_enabled = false;
              pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
            } break;

            case set_agc_en:
            {
              agc_enabled = true;
              pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
            } break;

            case set_cfg_bpt_med:
            {
              uint8_t val;
              ret = (parse_cmd_data(cmd, cmd_tbl[i], &val, 1, false) != 1);
              if (ret) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
                  break;
              }

              status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_USE_MED, &val, 1);
              if (status == SS_SUCCESS)
              {
                ssmax_log("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              }
              else
              {
                app_log_error("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
              }

            } break;

            case set_cfg_bpt_sys_bp:
            {
              uint8_t val[3];
              ret = (parse_cmd_data(cmd, cmd_tbl[i], &val[0], 3, false) != 3);
              if (ret) {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
                  break;
              }

              status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_SYS_BP_CAL, &val[0], 3);
              if (status == SS_SUCCESS)
                pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              else
              {
                  pr_info("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
              }

            } break;


            case set_cfg_bpt_dia_bp:
            {
              uint8_t val[3];
              ret = (parse_cmd_data(cmd, cmd_tbl[i], &val[0], 3, false) != 3);
              if (ret) {
                  app_log_error("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
                  break;
              }

              status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_DIA_BP_CAL, &val[0], 3);
              if (status == SS_SUCCESS)
                ssmax_log("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              else
              {
                  app_log_error("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
              }
            } break;

            case set_cfg_bpt_nonrest:
            {
              uint8_t val;
              ret = (parse_cmd_data(cmd, cmd_tbl[i], &val, 1, false) != 1);
              if (ret) {
                  app_log_error("\r\n%s err=%d\r\n", cmd, COMM_INVALID_PARAM);
                  break;
              }

              status = set_algo_cfg(SS_ALGOIDX_BPT, SS_CFGIDX_BP_EST_NONREST, &val, 1);
              if (status == SS_SUCCESS)
              {
                  ssmax_log("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              }
              else
              {
                  app_log_error("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
              }

            } break;

            case self_test_ppg_os24:
            {
              ret = selftest_max30101();
              pr_info("%s selftest_max30101: err=<%d>\r\n", cmd, ret);
            } break;

            case self_test_ppg_acc:
            {
              ret = selftest_accelerometer();
              pr_info("%s selftest_accelerometer: err=<%d>\r\n", cmd, ret);
            } break;

            case backup_shut_down:
            {
              status = set_shut_down_enter();
              if (status == SS_SUCCESS)
                pr_info("\r\n%s err=%d\r\n", cmd, COMM_SUCCESS);
              else
              {
                pr_err("\r\n%s err=%d\r\n", cmd, COMM_GENERAL_ERROR);
              }
            } break;

            case backup_wake_up:
            {
              set_shut_down_exit();
            } break;
#endif
            default:
            {
                assert_msg(false, "Invalid switch case!");
            } break;
          }
			    break;
      }
    }

    if (recognizedCmd && data_len)                                                                         // Added by Jason
    {
        if((data_report_mode == read_bpt_0)||(data_report_mode == read_bpt_1)||(data_report_mode == read_ppg_9))
        {
            sl_EN5V_enable();
        }
#if 1//def ENABLE_BLE
        if(ble_response)
        {
            if(BLE_Interface_Exists())                                                                     // Added by Jason
			      {
                BLE_AddtoQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, bio_sensor_type);          // Added by Jason
#if 0
                if(bio_sensor_type == BPT_SENSOR)
                  BLE_AddtoBPTQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);          // Added by Jason
                else if(bio_sensor_type == HRV_SENSOR)
                  BLE_AddtoHRVQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);          // Added by Jason
#endif
			      }
            ble_response = false;
            bio_packet.sec_flag = 0;
        }
#endif
#if SSMAX30101_LOG_ENABLE
        charbuf[data_len - 1] = 0;                                                                  // Added by Jason
        charbuf[data_len - 2] = 0;                                                                  // Added by Jason
        charbuf[1] = '\r';                                                                          // Added by Jason
        ssmax_log("len=%4d        : ", data_len);                                                   // Added by Jason
        if(data_len > (64+15))
        {
          charbuf[64+15] = 0x00;
          ssmax_log(&charbuf[2]);                                                                // Added by Jason
          if (status == SS_SUCCESS){
              ssmax_log(".... err=%d, MAX30101:Line=%d\r\n\r\n", COMM_SUCCESS, __LINE__);        // Added by Jason
          }
          else
          {
              ssmax_log(".... err=%d, MAX30101:Line=%d\r\n\r\n", COMM_GENERAL_ERROR, __LINE__);  // Added by Jason
          }
        }
        else
        {
            ssmax_log(&charbuf[2]);                                                             // Added by Jason
            ssmax_log(", MAX30101:Line=%d\r\n\r\n", __LINE__);                                  // Added by Jason
        }
#endif
    }
    return recognizedCmd;
}

bool charging_state = false;
void send_battery_info(uint8_t charging_flag)
{
  uint16_t batlevel = 0;
  int data_len      = 0;
  char charbuf1[512];

  if(bio_packet.batsoc&0x80)
    batlevel = 100;
  else
    batlevel = (bio_packet.batsoc)&0x7F;

  data_len  = snprintf(charbuf1, sizeof(charbuf1), "\r\ngetbattery percent=%d volt%d=%2.2fV", batlevel, charging_flag, (float)(bio_packet.batvcell * 78.125 / (1000.0 * 1000.0)));
  data_len += snprintf(charbuf1 + data_len, sizeof(charbuf1) - data_len, " T=%2.1f err=0\r\n", (float)(bio_packet.temp0 / 200.0));

  if (BLE_Interface_Exists())
  {
      BLE_AddtoQueue((uint8_t *)charbuf1, (int32_t)sizeof(charbuf1), data_len, bio_sensor_type);
  }
}

uint32_t report_count_prev = 0;
uint8_t bat_sending_rate = 0;
int execute_read_ppg_9(char* const buf, const int size)                                                          // sensor streaming @ 25Hz, wellness library API report ppg 9 for streaming format by Jason
{
  (void)size;
  max30101_mode1_data         max30101_sample               = { 0 };
  whrm_wspo2_suite_modeX_data whrm_wspo2_suite_sample_modeX = { 0 };
  accel_mode1_data            accel_sample                  = { 0 };
  int16_t data_len = 0;
    //int i = 0;
  if (1
      && (queue_len(&max30101_queue) > 0 )
      && (queue_len(&accel_queue) > 0 )
      && (queue_len(&whrm_wspo2_suite_queue) > 0 )
    )
  {
      dequeue(&max30101_queue,         &max30101_sample);
      dequeue(&accel_queue,            &accel_sample);
      dequeue(&whrm_wspo2_suite_queue, &whrm_wspo2_suite_sample_modeX);

      //mxm_assert_msg(((uint32_t)size > sizeof(ds_pkt_data_mode1_whrm_wspo2_suite)), "data_report_execute buffer too small");
      ds_pkt_data_mode_maxim_sensors_app* data_packet = (ds_pkt_data_mode_maxim_sensors_app*) buf;

      data_packet->start_byte             = DS_BINARY_PACKET_START_BYTE;
      data_packet->sample_cnt             = sample_count++;
      data_packet->sampleTime             = 1;

      data_packet->irCnt                  = max30101_sample.led1;
      data_packet->redCnt                 = max30101_sample.led2;
      data_packet->grnCnt                 = max30101_sample.led1;
      data_packet->grn2Cnt                = max30101_sample.led1;

      data_packet->x                      = accel_sample.x;
      data_packet->y                      = accel_sample.y;
      data_packet->z                      = accel_sample.z;

      data_packet->curr_opmode            = whrm_wspo2_suite_sample_modeX.current_operating_mode;
      data_packet->hr                     = whrm_wspo2_suite_sample_modeX.hr * 0.1;
      data_packet->hr_confidence          = whrm_wspo2_suite_sample_modeX.hr_conf;
      data_packet->rr                     = whrm_wspo2_suite_sample_modeX.rr;
      data_packet->rr_confidence          = whrm_wspo2_suite_sample_modeX.rr_conf;
      data_packet->activity               = whrm_wspo2_suite_sample_modeX.activity_class;
      data_packet->r                      = whrm_wspo2_suite_sample_modeX.r;
      data_packet->spo2_confidence        = whrm_wspo2_suite_sample_modeX.spo2_conf;
      data_packet->spo2                   = whrm_wspo2_suite_sample_modeX.spo2;
      data_packet->percentComplete        = whrm_wspo2_suite_sample_modeX.percentComplete;
      data_packet->lowSignalQualityFlag   = whrm_wspo2_suite_sample_modeX.lowSignalQualityFlag;
      data_packet->motionFlag             = whrm_wspo2_suite_sample_modeX.motionFlag;
      data_packet->lowPiFlag              = whrm_wspo2_suite_sample_modeX.lowPiFlag;
      data_packet->unreliableRflag        = whrm_wspo2_suite_sample_modeX.unreliableRFlag;
      data_packet->spo2State              = whrm_wspo2_suite_sample_modeX.spo2State;
      data_packet->scdState               = whrm_wspo2_suite_sample_modeX.scd_contact_state;

      data_packet->walk_steps             = whrm_wspo2_suite_sample_modeX.walk_steps;
      data_packet->run_steps              = whrm_wspo2_suite_sample_modeX.run_steps;
      data_packet->kcal                   = whrm_wspo2_suite_sample_modeX.kcal;
      data_packet->totalActEnergy         = whrm_wspo2_suite_sample_modeX.totalActEnergy;
      data_packet->ibiOffset              = whrm_wspo2_suite_sample_modeX.ibi_offset;
      data_packet->sample_count           = sample_count32++;
      data_packet->temp                   = bio_packet.temp0;                                                           // Added biosensor temperature value, 16bit,     20220111
      if((bio_packet.batsoc)&0x80) data_packet->batsoc = 100;
      else data_packet->batsoc = (bio_packet.batsoc)&0x7F;
    //data_packet->batsoc                 = bio_packet.batsoc;                                                          // Added biosensor battery voltage value, 16bit, 20220111
    //data_packet->crc8 = crc8((uint8_t*)data_packet, sizeof(*data_packet) - sizeof(uint8_t));
      //data_len = sizeof(*data_packet);
#if MAXIM_BLE_OUT_RATE
      //data_packet->sample_cnt             = sample_count++;
      //data_packet->sample_count           = sample_count32++;
      data_packet->crc8 = crc8((uint8_t*)data_packet, sizeof(*data_packet) - sizeof(uint8_t));
      data_len = sizeof(*data_packet);
#else
#if 0
      if(bio_packet.sec_flag)
    //if(bio_packet.other_sec_flag)
      {
          bio_packet.sec_flag = 0;
        //bio_packet.other_sec_flag = 0;
          data_packet->crc8 = crc8((uint8_t*)data_packet, sizeof(*data_packet) - sizeof(uint8_t));
          data_len = sizeof(*data_packet);
      }
      else
#endif
      if((whrm_wspo2_suite_sample_modeX.rr > 0) || (data_packet->sample_count - report_count_prev) > 25)
      {
          data_packet->crc8 = crc8((uint8_t*)data_packet, sizeof(*data_packet) - sizeof(uint8_t));
          data_len = sizeof(*data_packet);
          report_count_prev = data_packet->sample_count;
      }
      else
      {
          data_len = 0;
      }
      bio_packet.sec_flag = 0;
      bat_sending_rate = 0;
#endif
      //ssmax_log("ds_pkt_data_mode_maxim_sensors_app len=%d, line=%d\r\n", data_len, __LINE__);
  }
  else
  {
      if(bio_packet.sec_flag)
      {
          bio_packet.sec_flag = 0;
          bat_sending_rate++;
      }

      if(send_battery_flag && (bat_sending_rate > 3))
      {
          bat_sending_rate = 0;
          bio_packet.sec_flag = 0;
          send_battery_info(charging_state);
      }
  }

  return data_len;
}

static int data_report_execute_00(char* buf, int size)
{
  uint8_t tmp_report_mode;
	max30101_mode1_data max30101_sample = { 0 };
	//whrm_mode1_data whrm_sample = { 0 };
	//accel_mode1_data accel_sample = { 0 };
	bpt_mode1_2_data bpt_sample = { 0 };
	int16_t data_len = 0;
	if (size <= 0)
	  return 0;

	if (!is_enabled()) return 0;

	ss_execute_once();

	tmp_report_mode = data_report_mode;

	switch (tmp_report_mode)
	{
		case read_ppg_9:
		{
			data_len = execute_read_ppg_9(buf,size);
		}break;

		case read_bpt_0:
		case read_bpt_1:
		{
			if (1
#ifdef ENABLE_SS_MAX30101
			    && queue_len(&max30101_queue) > 0
#endif
#ifdef ENABLE_BPT
			    && queue_len(&bpt_queue) > 0
#endif
				)
			{
#ifdef ENABLE_SS_MAX30101
			    dequeue(&max30101_queue, &max30101_sample);
#endif
#ifdef ENABLE_BPT
			    dequeue(&bpt_queue, &bpt_sample);
#endif
			    {
			        assert_msg(((uint32_t)size > sizeof(ds_pkt_bpt_data)), "data_report_execute buffer too small");
              ds_pkt_bpt_data *pkt = (ds_pkt_bpt_data *)buf;
              pkt->start_byte   = DS_BINARY_PACKET_START_BYTE;
              pkt->status       = bpt_sample.status;
              pkt->irCnt        = max30101_sample.led1; reportCnt++;
              pkt->redCnt       = (max30101_sample.led2 >> 8);
              pkt->hr           = bpt_sample.hr / 10;
              pkt->prog         = bpt_sample.prog;
              pkt->sys_bp       = bpt_sample.sys_bp;
              pkt->dia_bp       = bpt_sample.dia_bp;
              pkt->spo2         = bpt_sample.spo2 / 10;
              pkt->hr_excthresh = bpt_sample.pulse_flag;
              pkt->r            = bpt_sample.r_value;
              pkt->ibi          = (bpt_sample.pulse_flag)? (bpt_sample.ibi):0;
              pkt->spo2_conf    = bpt_sample.spo2_conf;
              pkt->temp         = bio_packet.temp0;                                // Added biosensor temperature value, 16bit, 20220111
              if((bio_packet.batsoc)&0x80) pkt->batsoc = 100;
              else pkt->batsoc  = (bio_packet.batsoc)&0x7F;
#if BPT_ACCEL_ENABLE
              accel_data_TT accel_data;                                         // Added by Jason Chen, 2022.05.16
              get_accel_data(&accel_data);
              pkt->accl_x = accel_data.accl_data.x;
              pkt->accl_y = accel_data.accl_data.y;
              pkt->accl_z = accel_data.accl_data.z;
#endif
#if MAXIM_BLE_OUT_RATE
              pkt->sample_count = sample_count32++;
              pkt->crc8         = crc8((uint8_t*)pkt, sizeof(ds_pkt_bpt_data) - sizeof(uint8_t));
              data_len          = sizeof(ds_pkt_bpt_data);
#else
              if(bio_packet.sec_flag)
              {
                  bio_packet.sec_flag = 0;
                  pkt->sample_count = sample_count32++;
                  pkt->crc8 = crc8((uint8_t*)pkt, sizeof(ds_pkt_bpt_data) - sizeof(uint8_t));
                  data_len = sizeof(ds_pkt_bpt_data);
              }
              else
              {
                  data_len = 0;
              }
#endif
              bio_packet.sec_flag = 0;
			    }

          if(bpt_sample.prog >= 100)
          {
              if( bpt_sample.status == 0x2 )
              {
                  //calidata.hr     = bpt_sample.hr/10;
                  //calidata.sys_bp = bpt_sample.sys_bp;
                  //calidata.dia_bp = bpt_sample.dia_bp;
                  //calidata.spo2   = bpt_sample.spo2/10;
              }
          }
			}
		  else
		  {
		      if(send_battery_flag && bio_packet.sec_flag)
		      {
		          bio_packet.sec_flag = 0;
		          send_battery_info(charging_state);
		      }
		  }
		} break;

		case get_cfg_bpt_cal_data:
		{
		  sprintf(buf, "%s", cal_buf);
		  data_len = strlen(cal_buf);
			data_report_mode = 0;
		}break;

		case set_cfg_bpt_cal_data:
		{
		  sprintf(buf, "%s", cal_buf);                                            // Nov18 by Jason Chen
			data_len = strlen(cal_buf);                                             // Nov18 by Jason Chen
			data_report_mode = 0;
		}break;

		default:
			return 0;
	}

	if (data_len < 0) {
	    pr_err("snprintf console_tx_buf failed");
	} else if (data_len > size) {
	    pr_err("buffer is insufficient to hold data");
	  }

	return data_len;
}

void stop_00(void)
{
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
  SS_STATUS ret;
#endif
	ss_disable_irq();
  //ssmax_log("\r\nData_report_mode = %d, MAX30101:Line=%d\r\n", data_report_mode, __LINE__);           // Modified by Jason
	//sample_count = 0;sample_count32 = 0;

	sl_EN5V_disable();

#ifdef ENABLE_SS_MAX30101
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
	ret =
#endif
	    disable_sensor(SS_SENSORIDX_MAX30101);
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
	if(ret != SS_SUCCESS)
	{
	    app_log_error("                : MAX30101 disable fails, MAX30101:Line=%d\r\n", __LINE__);
	}
#endif
  queue_reset(&max30101_queue);
    /*__DEBUG*/
  memset( max30101_queue.base, 0, max30101_queue.buffer_size);
#endif

  if(data_report_mode == read_ppg_9)                                            // Added by Jason Chen, 2022.10.13
  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      ret =
#endif
          disable_sensor_A();//SS_SENSORIDX_ACCEL);
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      if(ret != SS_SUCCESS)
      {
          app_log_error("                : ACCEL disable fails, MAX30101:Line=%d\r\n", __LINE__);
      }
#endif
      queue_reset(&accel_queue);
      memset( accel_queue.base, 0, accel_queue.buffer_size);
  }

#ifdef ENABLE_WHRM_AND_SP02_MODE2
  if(data_report_mode == read_ppg_9)
  {
      disable_algo(SS_ALGOIDX_WHRM_WSPO2_SUITE);
    	queue_reset(&whrm_wspo2_suite_queue);
    	memset( whrm_wspo2_suite_queue.base, 0, whrm_wspo2_suite_queue.buffer_size);
    	ssmax_log("\r\nALGO WHRM_WSPO2 Disabled, Data_report_mode=%d, MAX30101:Line=%d\r\n", data_report_mode, __LINE__);           // Modified by Jason
  }
#endif

#ifdef ENABLE_BPT
  if((data_report_mode == read_bpt_0)||(data_report_mode == read_bpt_1))
  {
      disable_algo(SS_ALGOIDX_BPT);
    	queue_reset(&bpt_queue);
    	/*__DEBUG*/
    	memset( bpt_queue.base, 0, bpt_queue.buffer_size);
    	ssmax_log("\r\nALGO BPT Disabled, Data_report_mode=%d, MAX30101:Line=%d\r\n", data_report_mode, __LINE__);                  // Modified by Jason
  }

  BLE_reset_queue();

  disable_algo(SS_ALGOIDX_AGC);

#endif

  ssmax_log("\r\n                  All Queue reset on stop cmd, MAX30101:Line=%d\r\n", __LINE__);           // Modified by Jason
	ss_clear_interrupt_flag();
	ss_enable_irq();
#if(!MAXIM_BLE_OUT_RATE)
	ok_to_sleep       = false;                                                    // Added by Jason 2022.10.28
#endif
	data_report_mode  = 0;                                                                   //bpt_init();
	send_battery_flag = false;
}

