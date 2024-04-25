/***************************************************************************
 *
 *            Copyright (c) 2021 by Artafelx INC.
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

#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "app_log.h"
#include "em_cmu.h"
#include "em_wdog.h"
#include "sl_iostream.h"
#include "mscflash.h"
#include "bio_sm.h"
#include "DSInterface.h"
#include "peripherals.h"
#include "utils.h"
#include "sl_status.h"
#include "assert.h"
#include "MAX14676E.h"
#include "LSM6DSOX.h"                                                           // Added by Jason Chen, 2022.05.05
#include "queue.h"

#define MAXIM_BLE_OUT_RATE           0
#define WATCHDOG_ENABLE              1
#define ENABLE_HOSTSIDE_ACCEL        1                                          // just for MAX62664A/HRV control, using external accelerometer
#define BPT_ACCEL_ENABLE             1                                          // Added by Jason Chen, 2022.05.16
#define CMD_POWERON_RESET            0                                          // Added by Jason Chen, 2022.10.03

#if  APP_LOG_ENABLE
#define DEBUG_LEVEL                  1
#else
#define DEBUG_LEVEL                  0
#endif
#define USING_BLE_SOFTTIMER          0
#define USING_SLEEPTIMER             !USING_BLE_SOFTTIMER

typedef enum {
  SENSOR_MODE_ERROR = -1,
  SENSOR_MODE_IDLE,
  SENSOR_MODE_DETECTION,
  SENSOR_MODE_BPT_OR_HRV,
//SENSOR_MODE_MAX30101_RESET,
//SENSOR_MODE_ACCELEROMETER_START,
  SENSOR_MODE_READY,
  SENSOR_MODE_RUNNING,
  SENSOR_MODE_RUNNING1,
//SENSOR_MODE_MAX30101_STATUS,
  SENSOR_MODE_EXIT_BOOOTLOADER,
  SENSOR_MODE_WAITING,
//SENSOR_MODE_MAX32664_STATUS
}demo_appstate_t;


extern demo_appstate_t appState;
extern bool ok_to_sleep;

#if DEBUG_LEVEL
#define app_printfLog(...)                              \
  do {                                                   \
      sl_iostream_printf(app_log_iostream, __VA_ARGS__); \
  } while (0)
#else
#define app_printfLog(...)
#endif
#if APP_LOG_ENABLE
  #define APP_MYLOG_ENABLE          0
  #define BIO_INIT_LOG_ENABLE       0
  #define LSM6DSOX_LOG_ENABLE       0
  #define MSCFLASH_LOG_ENABLE       0
  #define DSINTERFACE_LOG_ENABLE    0
  #define SSINTERFACE_LOG_ENABLE    0
  #define SSGENERIC_LOG_ENABLE      0
  #define SSMAX30101_LOG_ENABLE     0
  #define AESCRYPT_LOG_ENABLE       0
  #define DE_CRYPT_LOG_ENABLE       0
  #define APP_LOG_ERROR_ENABLE      0
#else
  #define APP_MYLOG_ENABLE          0
  #define BIO_INIT_LOG_ENABLE       0
  #define LSM6DSOX_LOG_ENABLE       0
  #define MSCFLASH_LOG_ENABLE       0
  #define DSINTERFACE_LOG_ENABLE    0
  #define SSITERFACE_LOG_ENABLE     0
  #define SSGENERIC_LOG_ENABLE      0
  #define AESCRYPT_LOG_ENABLE       0
  #define DE_CRYPT_LOG_ENABLE       0
  #define APP_LOG_ERROR_ENABLE      0
#endif
#if APP_LOG_ENABLE
#if 0
    #define app_mylog_info(...)                              \
      do {                                                   \
          app_log_print_background(APP_LOG_LEVEL_DEBUG);     \
          app_log_print_color(APP_LOG_LEVEL_DEBUG);          \
          sl_iostream_printf(app_log_iostream, "%s:%4d   : (%s)  ", __FILE__, __LINE__, __func__);\
          sl_iostream_printf(app_log_iostream, __VA_ARGS__); \
      } while (0)

#elif 0
#define app_mylog_info(...)                              \
  do {                                                   \
      app_log_print_background(APP_LOG_LEVEL_DEBUG);     \
      app_log_print_color(APP_LOG_LEVEL_DEBUG);          \
      sl_iostream_printf(app_log_iostream, "%s:%4d   : ", __FILE__, __LINE__);\
      sl_iostream_printf(app_log_iostream, __VA_ARGS__); \
  } while (0)
#elif 1
    #define app_mylog_info(...)         app_printfLog(__VA_ARGS__)
#else
    #define app_mylog_info(...)
#endif
    #if SSGENERIC_LOG_ENABLE
      #define ssg_log(...)              app_printfLog(__VA_ARGS__)
    #else
      #define ssg_log(...)
    #endif

    #if SSMAX30101_LOG_ENABLE
      #define ssmax_log(...)            app_printfLog(__VA_ARGS__)
    #else
      #define ssmax_log(...)
    #endif

    #if LSM6DSOX_LOG_ENABLE
      #define lsm6d_log(...)            app_printfLog(__VA_ARGS__)
    #else
      #define lsm6d_log(...)
    #endif

    #if APP_MYLOG_ENABLE
      #define app_my_log(...)           app_printfLog(__VA_ARGS__)
    #else
      #define app_my_log(...)
    #endif

    #if BIO_INIT_LOG_ENABLE
      #define bioinit_log(...)          app_printfLog(__VA_ARGS__)
    #else
      #define bioinit_log(...)
    #endif

    #if MSCFLASH_LOG_ENABLE
      #define mscflash_log(...)         app_printfLog(__VA_ARGS__)
    #else
      #define mscflash_log(...)
    #endif

    #if DSINTERFACE_LOG_ENABLE
      #define dsinterface_log(...)      app_printfLog(__VA_ARGS__)
    #else
      #define dsinterface_log(...)
    #endif

    #if SSINTERFACE_LOG_ENABLE
      #define ssinterface_log(...)      app_printfLog(__VA_ARGS__)
    #else
      #define ssinterface_log(...)
    #endif

    #if AESCRYPT_LOG_ENABLE
      #define mbedtls_log(...)          app_printfLog(__VA_ARGS__)
    #else
      #define mbedtls_log(...)
    #endif
#else
    #define app_mylog_info(...)
    #define ssg_log(...)
    #define ssmax_log(...)
    #define lsm6d_log(...)
    #define app_my_log(...)
    #define bioinit_log(...)
    #define mscflash_log(...)
    #define dsinterface_log(...)
    #define ssinterface_log(...)
    #define mbedtls_log(...)
#endif

typedef enum {
  ERR_SENSOR = -1,
  BPT_SENSOR = 0,
  HRV_SENSOR = 1
}bio_sensor_type_t;
extern bio_sensor_type_t bio_sensor_type;

typedef enum {
  EXIT_BOOTLOADER_MODE  = 0,
  SENSOR_HUB_SHUTDOWM   = 1,
  SENSOR_HUB_RESET      = 2,
  ENTER_BOOTLOADER_MODE = 8
}bio_sensor_mode_t;

extern int BLE_AddtoQueue(uint8_t *data_transfer, int32_t buf_size, int32_t data_size, bio_sensor_type_t sensor_type);
#if 0
extern int BLE_AddtoBPTQueue(uint8_t *data_transfer, int32_t buf_size, int32_t data_size, int32_t line);
extern int BLE_AddtoHRVQueue(uint8_t *data_transfer, int32_t buf_size, int32_t data_size, int32_t line);
#endif

extern void BLE_reset_queue(void);
extern bool BLE_Interface_Exists(void);

typedef struct {
    uint8_t addr;
    uint32_t val;
} addr_val_pair;

typedef const char* (*get_type_callback)(void);
typedef const char* (*get_part_name_callback)(void);
typedef const char* (*get_algo_ver_callback)(void);
typedef int         (*get_part_info_callback)(uint8_t*, uint8_t*);
typedef int         (*data_report_execute_callback)(char*, int);
typedef void        (*SensorComm_Set_Ble_Status_callback)(bool, uint8_t);
typedef void        (*stop_callback) (void);
typedef uint8_t     (*parse_command_callback)(const char*);
typedef uint8_t     (*is_visible_callback)(void);


typedef struct
{
  get_type_callback                  get_type;
  get_part_name_callback             get_part_name;
  get_algo_ver_callback              get_algo_ver;
  get_part_info_callback             get_part_info;
  is_visible_callback                is_visible;
  stop_callback                      stop;
  parse_command_callback             parse_command;
  data_report_execute_callback       data_report_execute;
  SensorComm_Set_Ble_Status_callback SensorComm_Set_Ble_Status;
} SensorComm;

#define DS_MAX_NUM_SENSORCOMMS  2

extern void LED_GRN_init(uint8_t led_level);
extern void LED_BLUE_init(uint8_t led_level);

extern void LED_RED(uint8_t led_level);
extern void LED_GRN(uint8_t led_level);
extern void LED_BLUE(uint8_t led_level);

/****************************************************************************
 * Application Init.
 *****************************************************************************/
extern const char *bio_fw_version;
extern int bio_mode;
extern int bio_mode_type;
extern uint16_t led_flash_timer;
extern volatile uint8_t data_report_mode;

extern bool sleep_flag;
extern bool connect_flag;
extern bool charging_state;                                                     // Added by Jason Chen, 2022.10.24

void bio_init(void);
void app_init(void);
void bio_ble_init(void);
void bio_maintain(void);
void sl_EN5V_enable(void);
void sl_EN5V_disable(void);
void preSleep_config(void);
void postSleep_config(void);
void postSleep_exit(void);

#define wait_msx sl_sleeptimer_delay_millisecond
#ifdef wait_ms
#else
void wait_ms(uint16_t wait_ms);
#endif

/*static */void app_one_shot_timer_start(int ss_minutes);                       // Added by Jason Chen, 2022.10.13

void sl_bioreset_highlow(uint8_t outputlevel);                                  // Added by Jason Chen, 2022.09.20
void TemperatureReset(void);                                                    // Added by Jason Chen, 2022.09.19
void send_battery_info(uint8_t charging_flag);                                  // Added by Jason Chen, 2022.10.24

void mpc_pwm_init(void);
void mpc_pwm_stop(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);
#define ENABLE_I2C_DEINIT                                                       1
#if ENABLE_I2C_DEINIT
void sl_i2cspm_deinit_instances(void);                                          // Added by Jason Chen, 2023.11.28
#endif
#endif // APP_H
