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
#include "app.h"
#include "gatt_db.h"
#include "sl_bluetooth.h"
#include "power_manager_app.h"
#include "sl_emlib_gpio_init_MFIO_config.h"
#include "sl_emlib_gpio_init_EN5V_config.h"
#include "sl_udelay.h"
#include "app_assert.h"
#include "em_rmu.h"
#include "LSM6DSOX.h"
#include "SSMAX30101Comm.h"
#include "app_aescrypt.h"

#include "sl_pwm.h"
#include "sl_pwm_instances.h"
#include "em_i2c.h"
#include "sl_i2cspm_sensor_config.h"
#include "sl_emlib_gpio_init_G_INT1_config.h"
#include "sl_emlib_gpio_init_bio_reset_config.h"

/* soft timer handles */
#define TIC_TIMER_HANDLE 1
#define SL_TIMER_HANDLE  2
#define TIC_TIMER_CONST  32768
#define TIC_TIMER_PERSEC     10
#define TIC_TIMER_PERHALFSEC 5                                                  // Added by Jason Chen, 2022.10.18

static int _main_state;
static uint8_t _conn_handle = 0xFF;

struct queue_t BLE_BPT_QUEUE;
static uint8_t BLEBPTBuffer[BLE_BPT_NOTIFY_CHAR_ARR_SIZE * MAX_BLE_QUEUE];

struct queue_t BLE_HRV_QUEUE;
static uint8_t BLEHRVBuffer[BLE_HRV_NOTIFY_CHAR_ARR_SIZE * MAX_BLE_QUEUE];

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

uint8_t tic_count     = TIC_TIMER_PERSEC;
uint8_t tic_halfcount = TIC_TIMER_PERHALFSEC;                                   // Added by Jason Chen, 2022.10.18

#if(!USING_BLE_SOFTTIMER)
static sl_sleeptimer_timer_handle_t app_periodic_timer;
static sl_sleeptimer_timer_handle_t app_one_shot_timer;
void initWDOG(void);
static void periodic_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data);
static void BLE_TransferMesSamplesFromQueue(bio_sensor_type_t sensor_type);
static void reset_variables(void);
static void set_device_systemID_and_name(void);//bd_addr *addr);
#if APP_LOG_ENABLE
char name[20];
#endif
/**************************************************************************//**
 * Timer callback
 * Called periodically to time periodic temperature measurements and indications.
 *****************************************************************************/
//static void app_periodic_timer_cb(sl_simple_timer_t *timer, void *data)
static void periodic_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data)
{
  (void)data;
  (void)timer;
  bio_maintain();

  tic_halfcount++;                                                              // Added by Jason Chen, 2022.10.18
  if(tic_halfcount > TIC_TIMER_PERHALFSEC-1)                                    // Added by Jason Chen, 2022.10.18
  {
      tic_halfcount = 0;
      bio_packet.other_sec_flag = 1;
  }

  tic_count++;
  if(tic_count > TIC_TIMER_PERSEC-1)
  {
      tic_count = 0;
      bio_packet.sec_flag = 1;
      bio_sm_state();
#if APP_LOG_ENABLE&&APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      static uint8_t tic_output_cn = 9;

      tic_output_cn++;
      if(tic_output_cn > 9)
      {
          tic_output_cn = 0;
        //app_mylog_info("(%s)%d(%.2fV), B=0x%2x, A=0x%2x, %2d, a=%4d, o=%2d\r\n",
          app_mylog_info("(%s)%d(%.2fV), 0x%X, 0x%X, %d, %d, %d\r\n",
                             ((bio_packet.batsoc)&0x80)?"1":"0",
                             (bio_packet.batsoc)&0x7F,
                             (float)(bio_packet.batvcell * 78.125 / (1000.0 * 1000.0)),
                             bio_packet.statusb,
                             bio_packet.statusa,
                             retrigger_charging,
                           //bio_packet.awakesec, bio_packet.temp0);                                                                              // Added by Jason, 2022.01.06
                             bio_packet.awakesec, charging_voltage_settings);                                                                     // Added by Jason, 2022.01.06
      }
#endif
  }
}

static void app_one_shot_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data)
{
  (void)data;
  (void)timer;

  NVIC_SystemReset();
}

static void app_periodic_timer_start(void)
{
  sl_status_t status;

  uint32_t timer_timeout = sl_sleeptimer_ms_to_tick(100);                            // 100ms
  status = sl_sleeptimer_start_periodic_timer(&app_periodic_timer,
                                              timer_timeout,
                                              periodic_timer_cb,
                                              (void *)NULL,
                                              0,
                                              0);
  if(status != SL_STATUS_OK)
  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("Timer start fails....\r\n");
#endif
  }
}

/*static */void app_one_shot_timer_start(int ss_minutes)
{
  sl_status_t status;

  uint32_t timer_timeout = sl_sleeptimer_ms_to_tick(ss_minutes * 1000);         // 1 minute
  status = sl_sleeptimer_restart_timer(&app_one_shot_timer,
                                       timer_timeout,
                                       app_one_shot_timer_cb,
                                       (void *)NULL,
                                       0,
                                       0);
  if(status != SL_STATUS_OK)
  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("One Shot Timer start fails....\r\n");
#endif
  }
}

//void app_periodic_timer_stop(void)
//{
//  sl_status_t status = sl_sleeptimer_stop_timer(&app_periodic_timer);
//  if(status != SL_STATUS_OK)
//  {
//      app_log_error("Timer stop fails....\r\n");
//  }
//}

void app_timer_stop(sl_sleeptimer_timer_handle_t *app_timer)
{
  bool running = false;

  sl_status_t status = sl_sleeptimer_is_timer_running(app_timer, &running);

  if(status != SL_STATUS_OK)
  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("The one shot timer error.......%d\r\n", status);
#endif
      return;
  }

  if(running == true)
  {
      sl_status_t status = sl_sleeptimer_stop_timer(app_timer);
      if(status != SL_STATUS_OK)
      {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
          app_log_error("Timer stop fails....%d\r\n", status);
#endif
          return;
      }
      else
      {
          app_mylog_info("Timer stopped successfully!\r\n");
          return;
      }
  }
  app_mylog_info("Timer already stop!\r\n\r\n");
}
#endif

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  app_my_log("wireless Bio-sensor initializing......\r\n");
#if (APP_MYLOG_ENABLE)
  // Store the cause of the last reset, and clear the reset cause register
  uint32_t resetCause = RMU_ResetCauseGet();
  // Clear Reset causes so we know which reset occurs the next time
  RMU_ResetCauseClear();

  app_mylog_info("Reset cause: 0x%x\r\n", resetCause);
  if(resetCause&EMU_RSTCAUSE_WDOG0)
    app_mylog_info("Watch Dog Reset ....\r\n");
  else if(resetCause&EMU_RSTCAUSE_PIN)
    app_mylog_info("Pin Reset ....\r\n");
  else if(resetCause&EMU_RSTCAUSE_DVDDBOD)
    app_mylog_info("DVDDBOD Reset ....\r\n");
  else if(resetCause&EMU_RSTCAUSE_SYSREQ)
    app_mylog_info("SYSREQ Reset ....\r\n");
  else
    app_mylog_info("Unknow Reset ....\r\n");
#endif
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  max14676_init();
  SensorComm_create();
  //bio_sm_init();
  mpc_pwm_init();
  bio_init();

#if 0//((DEBUG_LEVEL)&&(APP_MYLOG_ENABLE))
  uint8_t accID =
#endif
      lsm6dsox_GetID();
  if(!lsm6dsox_init(true))
  {
    app_my_log("lsm6dsox init fail....\r\n");
  }

  power_manager_app_init();
  flash_init(false);

#if(!USING_BLE_SOFTTIMER)
  sl_status_t status_t = sl_bt_system_start_bluetooth();
  if(status_t != SL_STATUS_OK)
  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("ble stack starting.....fail......\r\n");
#endif
  }
#endif
#if WATCHDOG_ENABLE
  initWDOG();
#endif
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////

  data_report_execute();
  if(_main_state == STATE_SPP_MODE)
  {
      /* If SPP data mode is active, use non-blocking gecko_peek_event() */
      bool status_b = sl_bt_event_pending();

      if(!status_b){
          /* No stack events to be handled -> send data from local TX buffer */
          BLE_TransferMesSamplesFromQueue(bio_sensor_type);
        //return;
      }
  }

  if(acc_interrupt_flag())
  {
      acc_clear_interrupt_flag();
    //acc_intEnable(false);
    //postSleep_config();
      app_mylog_info("Single tap happening...........Bat=%0.2fV{%d}\r\n", (float)(bio_packet.batvcell * 78.125 / (1000.0 * 1000.0)), (bio_packet.batsoc&0x80)?100:(bio_packet.batsoc&0x7F));
    //NVIC_SystemReset();
      bio_packet.awakesec   = BIO_AWAKE_TIME;
      bio_packet.offsec     = BIO_OFF_TIME;

      if(!connect_flag)
      {
          LED_GRN(97);
          led_flash_timer = 1;
      }

      sleep_flag = false;
  }

  power_manager_app_process_action();
  
#if WATCHDOG_ENABLE
  WDOGn_Feed(DEFAULT_WDOG);                                                     // WDOG_Feed();
#endif
}

#define BATT_MEASUREMENT_INTERVAL_MS 5000
#define TEMP_MEASUREMENT_INTERVAL_MS 1000
static sl_sleeptimer_timer_handle_t batt_timer;
static sl_sleeptimer_timer_handle_t temp_timer;
static uint8_t batt_connection = 0;
static uint8_t temp_connection = 0;

static uint8_t sl_gatt_service_battery_get_level(void)
{
  uint8_t bat_level = (bio_packet.batsoc&0x80)?100:(bio_packet.batsoc&0x7F);//max14676_GetBatteryCharge();
  return bat_level;
}

static void batt_measurement_notify(void)
{
  sl_status_t sc;
  uint8_t value = sl_gatt_service_battery_get_level();
  sc = sl_bt_gatt_server_send_notification(
    batt_connection,
    gattdb_battery_level_0,
    1,
    &value);
  app_assert_status(sc);
}

static void temp_measurement_indication(void)
{
  sl_status_t sc;
  uint8_t value[17];

  float temp0 = ((float)(bio_packet.temp0 / 200.0));//*10.0;
  uint16_t temp1 = (int)(temp0 * 100.0);
  uint8_t  byte3 = temp1/(256*256);
  uint16_t temp2 = temp1 - byte3 * 256 * 256;
  uint8_t  byte2 = temp2/256;
  uint8_t  byte1 = temp2 - byte2 * 256;

  value[0] = 0x00;
  value[1] = byte1;//0x68;//(temp1 & 0x00FF) >> 0;
  value[2] = byte2;//0x01;//(temp1 & 0xFF00) >> 8;
  value[3] = byte3;
  value[4] = 0xFE;

  sc = sl_bt_gatt_server_send_indication(
    temp_connection,
    gattdb_temperature_measurement_0,
    17,
    value);
#if APP_LOG_ENABLE
  if(sc != SL_STATUS_OK)
  {
      app_mylog_info("send_indication fail-->sc=%4X\r\n", (unsigned int)sc);
  }
  else
  {
      app_mylog_info("send_indication OK....temperature = %2.2f C\r\n", temp0);
  }
#else
  app_assert_status(sc);
#endif
}

static void batt_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data)
{
  (void)data;
  (void)timer;
  // send temperature measurement indication to connected client
  batt_measurement_notify();
}

static void temp_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data)
{
  (void)data;
  (void)timer;
  // send temperature measurement indication to connected client
  temp_measurement_indication();
}

static void battery_level_changed_cb(sl_bt_evt_gatt_server_characteristic_status_t *data)
{
  sl_status_t status;

  batt_connection = data->connection;
  if (sl_bt_gatt_disable != data->client_config_flags)
  {
      uint32_t timer_timeout = sl_sleeptimer_ms_to_tick(BATT_MEASUREMENT_INTERVAL_MS);                            // 10000ms
      status = sl_sleeptimer_start_periodic_timer(&batt_timer,
                                                  timer_timeout,
                                                  batt_timer_cb,
                                                  (void *)NULL,
                                                  0,
                                                  0);
      if(status != SL_STATUS_OK)
      {
  #if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
          app_log_error("Timer start fails....\r\n");
  #endif
      }

      // Send the first notification
      batt_measurement_notify();
  }
  // indication and notifications disabled
  else {
    // stop timer used for periodic notifications
      app_timer_stop(&batt_timer);
  }
}

static void temp_measurement_changed_cb(sl_bt_evt_gatt_server_characteristic_status_t *data)
{
  sl_status_t status;

  temp_connection = data->connection;
  if (sl_bt_gatt_disable != data->client_config_flags)
  {
      uint32_t timer_timeout = sl_sleeptimer_ms_to_tick(TEMP_MEASUREMENT_INTERVAL_MS);                            // 10000ms
      status = sl_sleeptimer_start_periodic_timer(&temp_timer,
                                                  timer_timeout,
                                                  temp_timer_cb,
                                                  (void *)NULL,
                                                  0,
                                                  0);
      if(status != SL_STATUS_OK)
      {
  #if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
          app_log_error("Temp Timer start fails....\r\n");
  #endif
      }

      // Send the first notification
      temp_measurement_indication();
  }
  // indication and notifications disabled
  else {
    // stop timer used for periodic notifications
      app_timer_stop(&temp_timer);
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      app_my_log("Bluetooth stack booted: v%d.%d.%d-b%d\n",
                   evt->data.evt_system_boot.major,
                   evt->data.evt_system_boot.minor,
                   evt->data.evt_system_boot.patch,
                   evt->data.evt_system_boot.build);

      set_device_systemID_and_name();

    //Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

    //Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(advertising_set_handle,
                                       160,                                        // min. adv. interval (milliseconds * 1.6)
                                       160,                                        // max. adv. interval (milliseconds * 1.6)
                                       0,                                          // adv. duration
                                       0);                                         // max. num. adv. events
      app_assert_status(sc);

    //Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(advertising_set_handle, advertiser_general_discoverable, advertiser_connectable_scannable);
      app_assert_status(sc);
#if APP_LOG_ENABLE
      app_mylog_info("Started advertising......%s\r\n\r\n", name);
#endif

      appState  = SENSOR_MODE_IDLE;//SENSOR_MODE_DETECTION;

#if(!USING_BLE_SOFTTIMER)
      app_periodic_timer_start();
#endif

#if(USING_BLE_SOFTTIMER)
      sl_status_t status1 = sl_bt_system_set_soft_timer(TIC_TIMER_CONST/TIC_TIMER_PERSEC,  SL_TIMER_HANDLE, false);
      if(status1 != SL_STATUS_OK)
      {
          app_log_error("ble soft timer set fails......\r\n");
      }
#endif
      reset_variables();connect_flag = false;
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      _main_state    = STATE_CONNECTED;
      bio_packet.spp = STATE_CONNECTED;
      app_timer_stop(&app_one_shot_timer);                                      // app_one_shot_timer_stop();
      _conn_handle   = evt->data.evt_connection_opened.connection;
    //gecko_cmd_le_connection_set_timing_parameters(_conn_handle, 24, 40, 0, 200, 0, 0xFFFF);
      bio_packet.batvcell = max14676_GetBatteryChargeVoltage();
      bio_packet.batsoc   = max14676_GetBatteryCharge();
    //app_mylog_info("bluetooth Connected.....Bat=%0.2fV{%d}\r\n", (float)(bio_packet.batvcell * 78.125 / (1000.0 * 1000.0)), bio_packet.batsoc);
      app_mylog_info("bluetooth Connected.....\r\n");
      connect_flag          = true;

    //sc = sl_bt_connection_set_parameters(_conn_handle,//evt->data.evt_connection_parameters.connection,
    //                                     24,
    //                                     40,
    //                                     0,
    //                                     200,
    //                                     0,
    //                                     0xFFFF);

      break;

    case sl_bt_evt_connection_parameters_id:
      break;

    case sl_bt_evt_gatt_mtu_exchanged_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_mylog_info("Connection closed\n");
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(advertising_set_handle,
                                  advertiser_general_discoverable,
                                  advertiser_connectable_scannable);
      app_assert_status(sc);
    //app_mylog_info("Started advertising......\r\n\r\n");
      app_mylog_info("Started advertising.....Bat=%0.2fV{%d}\r\n\r\n", (float)(bio_packet.batvcell * 78.125 / (1000.0 * 1000.0)), (bio_packet.batsoc&0x80)?100:(bio_packet.batsoc&0x7F));
      _main_state    = STATE_ADVERTISING;
      bio_packet.spp = STATE_ADVERTISING;
      //app_one_shot_timer_start(60);
      //reset_variables();
      //wait_ms(500);
      NVIC_SystemReset();
      connect_flag = false;
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
    {
      struct sl_bt_evt_gatt_server_characteristic_status_s *pStatus;
      pStatus = &(evt->data.evt_gatt_server_characteristic_status);

      if (pStatus->characteristic == gattdb_notifyChar)
      {
          if (pStatus->status_flags == gatt_server_client_config) {

              bio_packet.batvcell = max14676_GetBatteryChargeVoltage();
              bio_packet.batsoc   = max14676_GetBatteryCharge();
           // Characteristic client configuration (CCC) for spp_data has been changed
              if (pStatus->client_config_flags == gatt_notification) {
                  _main_state    = STATE_SPP_MODE;
                  bio_packet.spp = STATE_SPP_MODE;
                  app_mylog_info("SPP Mode ON.............Bat=%0.2fV{%d}\r\n\r\n", (float)(bio_packet.batvcell * 78.125 / (1000.0 * 1000.0)), (bio_packet.batsoc&0x80)?100:(bio_packet.batsoc&0x7F));
              } else {
                  _main_state    = STATE_CONNECTED;
                  bio_packet.spp = STATE_CONNECTED;
                  app_mylog_info("SPP Mode OFF.....\r\n");
             }
          }
      }

#if defined(gattdb_battery_level_0)
      // Battery service notification
      if (pStatus->characteristic == gattdb_battery_level_0)
      {
          if(pStatus->status_flags == sl_bt_gatt_server_client_config)
          {
              battery_level_changed_cb(&evt->data.evt_gatt_server_characteristic_status);
          }
      }
#endif
#if defined(gattdb_temperature_measurement_0)
      // Battery service notification
      if (pStatus->characteristic == gattdb_temperature_measurement_0)
      {
          if(pStatus->status_flags == sl_bt_gatt_server_client_config)
          {
              temp_measurement_changed_cb(&evt->data.evt_gatt_server_characteristic_status);
          }
      }
#endif

    }
    break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_gatt_server_attribute_value_id:
    {
      if(evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_configRWChar)
      {
          int x, len;

          len = evt->data.evt_gatt_server_attribute_value.value.len;
          if (evt->data.evt_gatt_server_attribute_value.value.data[0] == 0xFF)
          {
              if (evt->data.evt_gatt_server_attribute_value.value.data[1] == 0xFF)
              {
                  app_my_log("Power off wireless sensor......\r\n");
                  sl_udelay_wait(300*1000);
                  max14676_poweroff();
              }
              else if (evt->data.evt_gatt_server_attribute_value.value.data[1] == 0xFE)
              {
                  app_my_log("Serial Number erased......\r\n");
                  flash_init(true);
                  wait_ms(500);
                  NVIC_SystemReset();
              }
#if ENABLE_I2C_DEINIT
              ////////////////////////////////////////////////////////////////////////////////   // Added by Jason Chen, 2023.11.28
              else if (evt->data.evt_gatt_server_attribute_value.value.data[1] == 0xFD)
              {
                  preSleep_config();
                  sl_udelay_wait(300*1000);
                  sl_i2cspm_deinit_instances();
              }
              else if (evt->data.evt_gatt_server_attribute_value.value.data[1] == 0xFC)
              {
                  app_my_log("Power cycle wireless sensor......\r\n");
                  wait_ms(500);
                  NVIC_SystemReset();
              }
              ////////////////////////////////////////////////////////////////////////////////
#endif
          }
          for(x=0; x < len; x++)
          {
            if (evt->data.evt_gatt_server_attribute_value.value.data[x] != 0)
            {
              Build_Command((char)evt->data.evt_gatt_server_attribute_value.value.data[x]);
            }
          }
      }
#if APP_MYLOG_ENABLE
      else
      {
          app_my_log("Unknown_config: 0x%x, line:%d\r\n", evt->data.evt_gatt_server_attribute_value.value.data[0], __LINE__);
      }
#endif
    }break;

#if(USING_BLE_SOFTTIMER)
    case sl_bt_evt_system_soft_timer_id:
    {
      switch (evt->data.evt_system_soft_timer.handle)
      {
        case SL_TIMER_HANDLE:
        {
          bio_maintain();

          tic_count++;
          if(tic_count > TIC_TIMER_PERSEC-2)
          {
              tic_count = 0;
              bio_packet.sec_flag = 1;
              bio_sm_state();
              app_my_log("(%s)%d(%.2fV), B=%x, A=%x, %d, a=%d, o=%d\r\n", ((bio_packet.batsoc)&0x80)?"1":"0", (bio_packet.batsoc)&0x7F, (float)(bio_packet.batvcell * 78.125 / (1000.0 * 1000.0)), bio_packet.statusb, bio_packet.statusa, retrigger_charging, bio_combo.awakesec, bio_combo.offsec);                              // Added by Jason, 2022.01.06
          }
        }break;

        default:
        {
          app_log_error("Unkonw soft timer even-->%d\r\n", SL_BT_MSG_ID(evt->header));
        }break;
      }
    }break;
#endif
    // -------------------------------
    // Default event handler.
    default:
    {
      //app_log_error("Unkonw even-->%d\r\n", SL_BT_MSG_ID(evt->header));
    }
    break;
  }
}

/**************************************************************************//**
 * EN5V Control
 * Button state changed callback
 * @param[in] handle Button event handle
 *****************************************************************************/
void sl_EN5V_enable(void)
{
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_EN5V_PORT,
                  SL_EMLIB_GPIO_INIT_EN5V_PIN,
                  gpioModeInput,
                  SL_EMLIB_GPIO_INIT_EN5V_DOUT);
}

void sl_EN5V_disable(void)
{
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_EN5V_PORT,
                  SL_EMLIB_GPIO_INIT_EN5V_PIN,
                  gpioModeWiredAnd,
                  SL_EMLIB_GPIO_INIT_EN5V_DOUT);
}

void sl_bioreset_highlow(uint8_t outputlevel)
{
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_BIO_RESET_PORT,
                  SL_EMLIB_GPIO_INIT_BIO_RESET_PIN,
                  SL_EMLIB_GPIO_INIT_BIO_RESET_MODE,
                  outputlevel);
}

void disable_i2c0(void)
{
  I2C_Reset(SL_I2CSPM_SENSOR_PERIPHERAL);

  GPIO_PinModeSet(SL_I2CSPM_SENSOR_SCL_PORT,
                  SL_I2CSPM_SENSOR_SCL_PIN,
                  gpioModeWiredOr,
                  1);

  GPIO_PinModeSet(SL_I2CSPM_SENSOR_SDA_PORT,
                  SL_I2CSPM_SENSOR_SDA_PIN,
                  gpioModeWiredOr,
                  1);
#if 0
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_G_INT1_PORT,
                  SL_EMLIB_GPIO_INIT_G_INT1_PIN,
                  SL_EMLIB_GPIO_INIT_G_INT1_MODE,                 // or gpioModeInputPullFilter, or gpioModeInput
                  SL_EMLIB_GPIO_INIT_G_INT1_DOUT);

  GPIO_ExtIntConfig(SL_EMLIB_GPIO_INIT_G_INT1_PORT,
                    SL_EMLIB_GPIO_INIT_G_INT1_PIN,
                    SL_EMLIB_GPIO_INIT_G_INT1_PIN,          // Interrupt Number
                    false,                                  // RisingEdge  Disable
                    true,                                   // FallingEdge Enable
                    true                                    // interrupt   Disable
          );

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  // EM4 Init
  EMU_EM4Init_TypeDef em4_init = EMU_EM4INIT_DEFAULT;
  em4_init.pinRetentionMode = emuPinRetentionLatch;
  EMU_EM4Init(&em4_init);
#endif
  //EMU_UnlatchPinRetention();
}

void preSleep_config(void)
{
  //bool deep_enable = true;
  //ssMAX30101.stop();

  TemperatureReset();
  set_shut_down_enter();

  GPIO_IntDisable(1 << SL_EMLIB_GPIO_INIT_MFIO_PIN);
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);

  LED_RED(0);
  LED_GRN_init(0);
  LED_BLUE_init(0);
  mpc_pwm_stop();

  sl_bt_advertiser_stop(advertising_set_handle);
#if APP_LOG_ENABLE
  int ret =
#endif
      sl_bt_system_stop_bluetooth();
#if APP_LOG_ENABLE
  if(ret)
  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("ble stack stop fails!....\r\n");
#endif
  }
#endif
  //app_periodic_timer_stop();
  NVIC_ClearPendingIRQ(RTCC_IRQn);
  NVIC_DisableIRQ(RTCC_IRQn);

  //acc_intEnable(true);
  //WDOG_Enable(false);
}

void postSleep_config(void)
{
#if(!USING_BLE_SOFTTIMER)
  app_timer_stop(&app_periodic_timer);                                          // app_periodic_timer_stop();
#endif
  NVIC_ClearPendingIRQ(RTCC_IRQn);
  NVIC_EnableIRQ(RTCC_IRQn);
  //sl_sleeptimer_init();

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  GPIO_IntEnable(1 << SL_EMLIB_GPIO_INIT_MFIO_PIN);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  int ret = sl_bt_system_start_bluetooth();
  if(ret)
  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("ble stack start fails!.....\r\n");
#endif
  }

  bio_packet.bio_state = BIO_STATE_IDLE;
  bio_packet.awakesec   = BIO_AWAKE_TIME;//???
  bio_packet.offsec     = BIO_OFF_TIME;//???
  bio_sm               = BIO_SM_IDLE;
}

static void reset_variables(void)
{
  int ret;

  //_conn_handle = 0xFF;
  _main_state = STATE_ADVERTISING;
  bio_packet.spp = STATE_ADVERTISING;
  ret  = queue_init(&BLE_BPT_QUEUE, BLEBPTBuffer, BLE_BPT_NOTIFY_CHAR_ARR_SIZE, BLE_BPT_NOTIFY_CHAR_ARR_SIZE * MAX_BLE_QUEUE);
  ret |= queue_init(&BLE_HRV_QUEUE, BLEHRVBuffer, BLE_HRV_NOTIFY_CHAR_ARR_SIZE, BLE_HRV_NOTIFY_CHAR_ARR_SIZE * MAX_BLE_QUEUE);
  if(ret != 0)
  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
      app_log_error("queue_init has failed\r\n");
#endif
  }
}

static void BLE_TransferMesSamplesFromQueue(bio_sensor_type_t sensor_type)
{
  int ret;
  sl_status_t result;
  int BLE_NOTIFY_CHAR_ARR_SIZE;
  struct queue_t *BLE_QUEUE;
  uint8_t data_transfer[BLE_A_NOTIFY_CHAR_ARR_SIZE];

  if(sensor_type == BPT_SENSOR)
  {
      BLE_NOTIFY_CHAR_ARR_SIZE = BLE_BPT_NOTIFY_CHAR_ARR_SIZE;
      BLE_QUEUE = (struct queue_t *)&BLE_BPT_QUEUE;
  }
  else if(sensor_type == HRV_SENSOR)
  {
      BLE_NOTIFY_CHAR_ARR_SIZE = BLE_HRV_NOTIFY_CHAR_ARR_SIZE;
      BLE_QUEUE = (struct queue_t *)&BLE_HRV_QUEUE;
  }
  else
    return;

  if (BLE_QUEUE->num_item >= 1)
  {
      ret = dequeue(BLE_QUEUE, data_transfer);
      if(ret < 0) return;
      do
      {
        //result = sl_bt_gatt_server_notify_all(gattdb_notifyChar, BLE_NOTIFY_CHAR_ARR_SIZE, data_transfer);
          result = sl_bt_gatt_server_send_notification(_conn_handle, gattdb_notifyChar, BLE_NOTIFY_CHAR_ARR_SIZE, data_transfer);
      } while(result == SL_STATUS_BT_CTRL_MEMORY_CAPACITY_EXCEEDED);

      if (result != 0) {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
          app_log_error("spp Unexpected error: 0x%4x\r\n", result);
#endif
      }
      else
      {
          sl_udelay_wait(200);
      }
  }
}

bool BLE_Interface_Exists(void)
{
  //return (bio_packet.spp == STATE_SPP_MODE);
  return (_main_state == STATE_SPP_MODE);
}

int BLE_AddtoQueue(uint8_t *data_transfer, int32_t buf_size, int32_t data_size, bio_sensor_type_t sensor_type)
{
  //(void)line;
  int ret = 0;
  int BLE_NOTIFY_CHAR_ARR_SIZE;
  struct queue_t *BLE_QUEUE;

  if(sensor_type == BPT_SENSOR)
  {
      BLE_NOTIFY_CHAR_ARR_SIZE = BLE_BPT_NOTIFY_CHAR_ARR_SIZE;
      BLE_QUEUE = (struct queue_t *)&BLE_BPT_QUEUE;
  }
  else if(sensor_type == HRV_SENSOR)
  {
      BLE_NOTIFY_CHAR_ARR_SIZE = BLE_HRV_NOTIFY_CHAR_ARR_SIZE;
      BLE_QUEUE = (struct queue_t *)&BLE_HRV_QUEUE;
  }
  else
    return -1;

  while ((data_size % BLE_NOTIFY_CHAR_ARR_SIZE) && data_size < buf_size)
    data_transfer[data_size++] = 0;
  //mxm_assert_msg(!(data_size % BLE_NOTIFY_CHAR_ARR_SIZE), "BLE packet size must be multiple of 32 bytes");

  while(data_size > 0)
  {
      ret = enqueue(BLE_QUEUE, data_transfer);
      data_size     -= BLE_NOTIFY_CHAR_ARR_SIZE;
      data_transfer += BLE_NOTIFY_CHAR_ARR_SIZE;
  }
#if APP_MYLOG_ENABLE
  if(ret != 0)
  {
      app_my_log("BLE_AddtoQueue has failed, ret=%d, Line:%d\r\n", ret, (int)sensor_type);
  }
#endif
  return ret;
}

void BLE_reset_queue(void)
{
  queue_reset(&BLE_BPT_QUEUE);
  memset( BLE_BPT_QUEUE.base, 0, BLE_BPT_QUEUE.buffer_size);                                      // Added by Jason

  queue_reset(&BLE_HRV_QUEUE);
  memset( BLE_HRV_QUEUE.base, 0, BLE_HRV_QUEUE.buffer_size);                                      // Added by Jason
}

/**************************************************************************//**
 * @brief Watchdog initialization
 *****************************************************************************/
void initWDOG(void)
{

  // Enable clock for the WDOG module; has no effect on xG21
  CMU_ClockEnable(cmuClock_WDOG0, true);

  // Watchdog Initialize settings
  WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
  CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_ULFRCO); /* ULFRCO 1000Hz as clock source */
  wdogInit.enable   = true;//false;
  wdogInit.debugRun = true;
  wdogInit.em2Run   = true;
//wdogInit.em3Run   = true;
  wdogInit.em4Block = false;
  wdogInit.perSel   = wdogPeriod_4k; //~4.2 seconds // 2049 clock cycles of a 1kHz clock  ~2 seconds period

  // Initializing watchdog with chosen settings
  WDOGn_Init(DEFAULT_WDOG, &wdogInit);                                          // WDOG_Init(&wdogInit);
}

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] addr  Pointer to Bluetooth address.
 ******************************************************************************/
char hexstring[128];
//int data_len2 = 0;
static int hex_to_hexstring(uint8_t *hexinput, int len, int dataLen)
{
  int i;
  int data_len2 = dataLen;

  for(i = 0; i < len; i++)
  {
     if(hexinput[i] == 0)
     {
         data_len2 += snprintf(hexstring + data_len2, sizeof(hexstring) - data_len2, "%s", "00");
     }
     else if((hexinput[i] & 0xF0) == 0)
     {
         data_len2 += snprintf(hexstring + data_len2, sizeof(hexstring) - data_len2, "0%X", hexinput[i] & 0x0F);
     }
     else if((hexinput[i] & 0x0F) == 0)
     {
         data_len2 += snprintf(hexstring + data_len2, sizeof(hexstring) - data_len2, "%X%d", (hexinput[i] >> 4) & 0x0F, 0);
     }
     else
     {
         data_len2 += snprintf(hexstring + data_len2, sizeof(hexstring) - data_len2, "%2X", hexinput[i]);
     }
  }
  return data_len2;
}

static void set_device_systemID_and_name(void)//bd_addr *addr)
{
#if (!APP_LOG_ENABLE)
  char name[20];
#endif
  char fw_revision_string[25];
  int data_len;
  uint8_t system_id[8];
  sl_status_t result;
  bd_addr address;
  //uint8_t address_type;

//Extract unique ID from BT Address.
  result = sl_bt_system_get_identity_address(&address, &system_id[0]);//address_type);
  app_assert_status(result);
  app_my_log("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n", system_id[0]?"static random":"public device",
                                                                      address.addr[5], address.addr[4], address.addr[3],
                                                                      address.addr[2], address.addr[1], address.addr[0]);

  // Pad and reverse unique ID to get System ID.
  system_id[0] = address.addr[5];system_id[1] = address.addr[4];system_id[2] = address.addr[3];system_id[3] = 0xFF;
  system_id[4] = 0xFE;           system_id[5] = address.addr[2];system_id[6] = address.addr[1];system_id[7] = address.addr[0];
  result = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
  app_assert_status(result);

  // Create unique device name using the last two bytes of the Bluetooth address
#if MAXIM_BLE_OUT_RATE
  snprintf(name, sizeof(name), "MAX30101%02X%02X_Sensor", address.addr[1], address.addr[0]);
#else
  snprintf(name, sizeof(name), "ARTA%02X%02X", address.addr[1], address.addr[0]);
  //snprintf(name, sizeof(name), "ARTAFLEX%02X%02X_Sensor", address.addr[1], address.addr[0]);
#endif
  result = sl_bt_gatt_server_write_attribute_value(gattdb_device_name,                                                   0,
                                                   strlen(name),
                                                   (uint8_t *)name);
  //app_mylog_info("Device name: %s\r\n", name);
  app_assert_status(result);

  data_len = 0;
  data_len  = snprintf(hexstring + data_len, sizeof(hexstring) - data_len, "%s", "Serial: ");
  data_len  = hex_to_hexstring(calidata.serial, 6, data_len);
  data_len += snprintf(hexstring + data_len, sizeof(hexstring) - data_len, "%s", "\r\nAuth:  0x");
  data_len  = hex_to_hexstring(calidata.auth_value, 16, data_len);
  data_len += snprintf(hexstring + data_len, sizeof(hexstring) - data_len, "%s", "...");
  app_mylog_info("%s[%d]\r\n", hexstring, data_len);
  result = sl_bt_gatt_server_write_attribute_value(gattdb_serial_number_string_0,
                                                   0,
                                                   data_len,
                                                   (uint8_t *)hexstring);
  app_assert_status(result);


  const char *bio_fw_version = get_ss_fw_version();
  snprintf(fw_revision_string, sizeof(fw_revision_string), "Hub%s  %sBIO%d.%d", bio_fw_version, MAXIM_BLE_OUT_RATE?"Max":"Arta", BIO_FW_MAJOR, BIO_FW_MINOR);
  result = sl_bt_gatt_server_write_attribute_value(gattdb_firmware_revision_string_0,
                                                   0,
                                                   strlen(fw_revision_string),
                                                   (uint8_t *)fw_revision_string);
  app_assert_status(result);

  if((calidata.serial[5] != address.addr[0])&&(calidata.serial[4] != address.addr[1]))
  {
      //calidata.serial[0] = address.addr[5];  calidata.serial[1] = address.addr[4];                                                  // don't need it, the serial number assigned by APP, 2022.08.25, asked by Jared
      //calidata.serial[2] = address.addr[3];  calidata.serial[3] = address.addr[2];                                                  // don't need it, the serial number assigned by APP, 2022.08.25, asked by Jared
      //calidata.serial[4] = address.addr[1];  calidata.serial[5] = address.addr[0];                                                  // don't need it, the serial number assigned by APP, 2022.08.25, asked by Jared
      //flash_write();                                                                                                                // don't need it, the serial number assigned by APP, 2022.08.25, asked by Jared
  }
}

#if ENABLE_I2C_DEINIT
#include "sl_i2cspm.h"
#include "em_cmu.h"
// Include instance config
#include "sl_i2cspm_sensor_config.h"

#ifndef SL_I2CSPM_SCL_HOLD_TIME_US
#define SL_I2CSPM_SCL_HOLD_TIME_US 100
#endif

#define EM4ENABLEPIN_MASK               0x00010000
#define EM4ENABLEPIN_SHIFT              3                                       // Select GPIO.EM4WU3 (PB01)

extern I2CSPM_Init_TypeDef init_sensor;

/*******************************************************************************
 *   Initalize I2C peripheral
 ******************************************************************************/
void I2CSPM_DeInit(I2CSPM_Init_TypeDef *init)
{
  I2C_Init_TypeDef i2cInit;

  /* Set emlib init parameters */
  i2cInit.enable = false;
  i2cInit.master = true; /* master mode only */
  i2cInit.freq = init->i2cMaxFreq;
  i2cInit.refFreq = init->i2cRefFreq;
  i2cInit.clhr = init->i2cClhr;

  //I2C_Init(init->port, &i2cInit);
  I2C_Enable(init->port, i2cInit.enable);

  GPIO_PinModeSet(init->sclPort,
                  init->sclPin,
                  gpioModeInputPull, 1);
  GPIO_PinModeSet(init->sdaPort,
                  init->sdaPin,
                  gpioModeInputPull, 1);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_MFIO_PORT,
                  SL_EMLIB_GPIO_INIT_MFIO_PIN,
                  gpioModeInputPull, 1);                                        // SL_EMLIB_GPIO_INIT_MFIO_DOUT);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_BIO_RESET_PORT,
                  SL_EMLIB_GPIO_INIT_BIO_RESET_PIN,
                  gpioModeInputPull, 1);                                        // SL_EMLIB_GPIO_INIT_BIO_RESET_DOUT);

  uint32_t em4enablepin = GPIO->EM4WUEN  | (EM4ENABLEPIN_MASK << EM4ENABLEPIN_SHIFT);
  uint32_t em4enablepol = GPIO->EM4WUPOL | (EM4ENABLEPIN_MASK << EM4ENABLEPIN_SHIFT);
  uint32_t em4enableEN  = GPIO->IEN      | (EM4ENABLEPIN_MASK << EM4ENABLEPIN_SHIFT);

  //em_mode = SL_POWER_MANAGER_EM4;

// Enable GPIO_EM4WU3 (PB01) wake up evnt from deep sleep mode EM4
  GPIO_EM4EnablePinWakeup(em4enablepin, em4enablepol);                               // Set GPIO.EM4WU3 (PB01) as wakeup of CPU from EM4
  GPIO_IntEnable(em4enableEN);                                                       // GPIO.EM4WU3 interrupt enabled
  //NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  //NVIC_EnableIRQ(GPIO_ODD_IRQn);
  EMU_EnterEM4();
  NVIC_SystemReset();
}

void sl_i2cspm_reset(void)
{
  I2C_Reset(init_sensor.port);
}

void sl_i2cspm_deinit_instances(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  I2CSPM_DeInit(&init_sensor);
}
#endif

