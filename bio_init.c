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
#include "SSMAX30101Comm.h"
#include "sl_pwm.h"
#include "sl_pwm_instances.h"

#define LEDS_FLASH_RATE_LOW           74
#define LEDS_FLASH_RATE_HIGH          92
//#include "power_manager_app.h"

#include "em_gpio.h"

demo_appstate_t appState  = SENSOR_MODE_IDLE;
const char *bio_fw_version;
int bio_mode = -1;
bio_sensor_type_t bio_sensor_type = ERR_SENSOR;

uint8_t pwm_lut[] = {
  0,  1,  1,  1,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  4,  4,  4,  4,
  5,  5,  5,  5,  6,  6,  6,  7,  7,  7,
  8,  8,  8,  9,  9,  10, 10, 10, 11, 11,
  12, 12, 13, 13, 14, 15, 15, 16, 17, 17,
  18, 19, 19, 20, 21, 22, 23, 23, 24, 25,
  26, 27, 28, 29, 30, 31, 32, 34, 35, 36,
  37, 39, 40, 41, 43, 44, 46, 48, 49, 51,
  53, 54, 56, 58, 60, 62, 64, 66, 68, 71,
  73, 75, 78, 80, 83, 85, 88, 91, 94, 97,
  100,
};

void mpc_pwm_init(void)
{
  // Enable PWM output
  sl_pwm_start(&sl_pwm_MPC0);
  sl_pwm_start(&sl_pwm_MPC1);
}

void LED_GRN(uint8_t led_level)
{
  //sl_pwm_set_duty_cycle(&sl_pwm_MPC0, led_level);
  sl_pwm_set_duty_cycle(&sl_pwm_MPC0, pwm_lut[led_level]);
}

void LED_BLUE(uint8_t led_level)
{
  sl_pwm_set_duty_cycle(&sl_pwm_MPC1, pwm_lut[led_level]);
}

void mpc_pwm_stop(void)
{
  LED_GRN(0);
  LED_BLUE(0);

  sl_pwm_stop(&sl_pwm_MPC0);
  sl_pwm_stop(&sl_pwm_MPC1);
}

 void bio_init(void)
{
   LED_GRN_init(12);
   LED_BLUE_init(14);
   LED_BLUE(97);
   wait_ms(250);
   LED_BLUE(0);
   LED_GRN(95);
   wait_ms(250);
   LED_GRN(0);
   LED_RED(8);
   wait_ms(100);
   LED_RED(0);
   wait_ms(100);

   ss_init_hwcomm_interface();
   ss_disable_irq();
   ss_clear_interrupt_flag();
   ss_enable_irq();
}

uint16_t led_flash_timer = 0xFFFF;
uint16_t measure_count   = 0;
bool sleep_flag          = false;
bool connect_flag        = false;
SS_STATUS status         = 0xFF;
void bio_maintain(void)
{
  switch(appState)
  {
    case SENSOR_MODE_IDLE:
#if 1
    {
      if(measure_count++ > 10 )
      {
          measure_count = 0;
          appState = SENSOR_MODE_DETECTION;
          sl_bioreset_highlow(1);
      }
    }
    break;
#endif
    case SENSOR_MODE_DETECTION:
    {
      if(measure_count++ > 3)
      {
          measure_count = 0;

          //bio_packet.awakesec   = BIO_AWAKE_TIME;//???                       // Keep wakeup, added by Jason Chen, 2022.09.30
          //bio_packet.offsec     = BIO_OFF_TIME;//???
          //sleep_flag            = false;

          bio_mode = ss_in_bootldr_mode();
          if(bio_mode == EXIT_BOOTLOADER_MODE)
          {
              measure_count = 0;
              LED_RED(0);
              appState = SENSOR_MODE_BPT_OR_HRV;
              bioinit_log("Hub is working mode....\r\n\r\n");
          }
#if 1
          else if(bio_mode == ENTER_BOOTLOADER_MODE)
          {
              static bool exit_boadloader = true;

              LED_GRN(97);
              appState = SENSOR_MODE_WAITING;   //SENSOR_MODE_READY;//
              measure_count = 0;
              if(exit_boadloader)
              {
                  //exit_boadloader = false;
                  status = ss_set_hub_mode(ENTER_BOOTLOADER_MODE);
                  //status = ss_set_hub_mode(EXIT_BOOTLOADER_MODE);
                  //status = ss_set_hub_mode(SENSOR_HUB_RESET);
                  if(status != SS_SUCCESS)
                  {
                      LED_BLUE(97);
                  }
                  appState = SENSOR_MODE_EXIT_BOOOTLOADER;
              }
          }
          else if(bio_mode == SENSOR_HUB_RESET)
          {
              LED_BLUE(97);
              appState = SENSOR_MODE_EXIT_BOOOTLOADER;
          }
#endif
          else
          {
              LED_RED(8);
              appState = SENSOR_MODE_WAITING;
              measure_count = 0;
          }
      }
    }
    break;

    case SENSOR_MODE_EXIT_BOOOTLOADER:
    {
      if(measure_count++ > 20)
      {
          measure_count = 0;
          LED_GRN(0);
          LED_BLUE(0);
          //ss_set_hub_mode(EXIT_BOOTLOADER_MODE);
          set_shut_down_exit();
          appState = SENSOR_MODE_IDLE;
      }
    }
    break;

    case SENSOR_MODE_WAITING:
    {
      if(measure_count++ > 1)
      {
          measure_count = 0;
          LED_RED(0);
          LED_GRN(0);
          appState = SENSOR_MODE_DETECTION;
      }
    }
    break;

    case SENSOR_MODE_BPT_OR_HRV:
    {
      if(measure_count++ > 5)
      {
          //measure_count = 0;
          bio_fw_version = ssMAX30101.get_algo_ver();
          bioinit_log("Hub algo:      %s\r\n", bio_fw_version);
          bio_fw_version = get_ss_fw_version();

          if((starts_with(bio_fw_version, "10.4.2") == true)
            ||(starts_with(bio_fw_version, "10.4.1") == true))
          {
              bio_sensor_type = HRV_SENSOR;
              appState = SENSOR_MODE_READY;//SENSOR_MODE_MAX30101_RESET; // SENSOR_MODE_ACCELEROMETER_START;//
              LED_GRN(99);
              measure_count = 0;
            //reset_to_bootloader();
              BIO_OFF_TIME = BIO_OFF_TIME_HRV;
          }
          else if(starts_with(bio_fw_version, "40.6.0") == true)
          {
              bio_sensor_type = BPT_SENSOR;
              appState = SENSOR_MODE_READY;//SENSOR_MODE_MAX30101_RESET;
              LED_BLUE(99);
              BIO_OFF_TIME = BIO_SLEEP_TIME_BPT;
			    }
          else
		      {
              bio_sensor_type = ERR_SENSOR;
              max32664_soft_reset();
              appState = SENSOR_MODE_WAITING;//SENSOR_MODE_DETECTION;
              BIO_OFF_TIME = BIO_OFF_TIME_HRV;
		      }
          bioinit_log("Hub FW:        %s\r\n\r\n", bio_fw_version);
      }
    }
    break;

    case SENSOR_MODE_READY:
    {
      if(measure_count++ > 5)
      {
          measure_count = 0;
          set_fw_platform(get_ss_platform_name());
          set_fw_version(get_ss_fw_version());
          appState = SENSOR_MODE_RUNNING;
          LED_GRN(0);
          LED_BLUE(0);
          LED_RED(0);
      }
    }
    break;

    case SENSOR_MODE_RUNNING:
    {
      uint8_t leds_flash_rate;

      if((bio_packet.batsoc)&0x80)
      {
          if(connect_flag) {LED_BLUE(97);LED_GRN(0);}
          else             {LED_BLUE(0);LED_GRN(97);}
          LED_RED(0);
      }
      else if(((bio_packet.batsoc)&0x7F) < 34)
      {
          if(measure_count++ > 20)
          {
              LED_BLUE(0);LED_GRN(0);
              if(sleep_flag)
              {
                  LED_RED(0);
                  break;
              }

              measure_count = 0;
              LED_RED(8);
              appState = SENSOR_MODE_RUNNING1;
          }
      }
      else
      {
          static uint8_t level = LEDS_FLASH_RATE_LOW;
          static bool updownFlag = true;

          if(sleep_flag)
          {
              LED_GRN(0);
              LED_BLUE(0);
              //LED_RED(0);
              break;
          }

          if ((bio_packet.spp == STATE_CONNECTED)
           || (bio_packet.spp == STATE_SPP_MODE)) {leds_flash_rate = LEDS_FLASH_RATE_HIGH;}
          else                                    {leds_flash_rate = LEDS_FLASH_RATE_LOW; }

          if(updownFlag)
          {
              if(connect_flag) {LED_GRN(0);LED_BLUE(level++);}
              else{
                  if(((bio_packet.batsoc)&0x7F) < 85){LED_GRN(level++);LED_BLUE(0);}
                  else                               {LED_GRN(level++);LED_BLUE(0);}
              }
              if(level >= 100) updownFlag = false;
          }
          else
          {
              if(connect_flag) {LED_GRN(0);LED_BLUE(level--);}
              else{
                  if(((bio_packet.batsoc)&0x7F) < 85){LED_GRN(level--);LED_BLUE(0);}
                  else                               {LED_GRN(level--);LED_BLUE(0);      }
              }
              if(level <= leds_flash_rate) updownFlag = true;
          }
      }
    }
    break;

		case SENSOR_MODE_RUNNING1:
		{
		  LED_RED(0);
		  appState = SENSOR_MODE_RUNNING;
		}
		break;

		case SENSOR_MODE_ERROR:
		default:
		  break;

  }

  if(led_flash_timer-- == 0)
  {
      LED_GRN(0);
      //LED_RED(0);
  }
}

