/*
 * bio_sm.c
 *
 *  Created on: 01/04/2022
 *      Author: Jason chen
 */
#include "app.h"
#include "bio_sm.h"
#include "power_manager_app.h"
//#include "led.h"

uint8_t retrigger_charging = 10;    // retart charging trigger
uint16_t BIO_OFF_TIME = BIO_OFF_TIME_HRV;//BIO_SLEEP_TIME_BPT;

// zeroing of global structures and variables happens automatically in cinit on power on, but expicit here
struct bio_ble_packet bio_packet = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,BIO_AWAKE_TIME,BIO_SLEEP_TIME_BPT, 0};

void bio_sm_init(void)
{
//initialize the status message
   bio_packet.bio_state  = BIO_STATE_IDLE;
   bio_packet.inithandle = BIO_NO_HANDLE;    // No handle
}

// v3 main state machine
uint8_t bio_sm =  BIO_SM_IDLE;
void bio_sm_state(void)
{
  static uint8_t beat, up = 1;
  static uint8_t trigger_cn = 5;
  static bool statusA, statusB;
  static uint16_t timer;
  static uint8_t wait = 0;

  bio_packet.time++;

  bio_packet.statusb  = max14676_GetStatusB();  // Get PMIC registers for reporting
  bio_packet.batvcell = max14676_GetBatteryChargeVoltage();
  bio_packet.statusa  = max14676_GetStatusA() & MAX_STATA_CHGIN;
  bio_packet.crate    = max14676_GetBatteryCRATE();

//When in maint timer done state, will not exit until battery drains 0.070V from reg point (4.2V)
//So set charge = 100 until state is exited
  statusA = ((bio_packet.statusa >= 2)&&(bio_packet.statusa<=6));
  statusB = ((bio_packet.statusb & MAX_STATB_CHGIN)!=0);
  bio_packet.batsoc = max14676_GetBatteryCharge();
  charging_state = statusB;
  if (statusA || statusB)
  {
      if(bio_packet.batsoc < 90)
      {
          if(charging_voltage_settings != MAX_VSET_4_25)//MAX_VSET_4_30)
            max14676_ChargerOn(MAX_VSET_4_25);
      }
      //else if((bio_packet.batsoc < 99))
      //{
      //    if(charging_voltage_settings != MAX_VSET_4_25)
      //      max14676_ChargerOn(MAX_VSET_4_25);
      //}
      else
      {
          if(charging_voltage_settings != MAX_VSET_4_20)
            max14676_ChargerOn(MAX_VSET_4_20);
      }
  }
  if ((bio_packet.statusa & 0x07) == 0x06)
  {
      if(bio_packet.batsoc >= 95) bio_packet.batsoc |= 0x80;
  }
//else
//  bio_packet.batsoc = max14676_GetBatteryCharge();

  if ((bio_packet.spp == STATE_CONNECTED) || (bio_packet.spp == STATE_SPP_MODE))
  {
      bio_packet.awakesec   = BIO_AWAKE_TIME;//???
      bio_packet.offsec     = BIO_OFF_TIME;//???
      sleep_flag            = false;
  }

//override state machine sequencing for 1) charging, 2) low battery
//if (bio_packet.bio_state < BIO_STATE_CHRG)    // See if PMIC CHRGIN active, change state globally on charge OR power button
  {
    if (statusA&&statusB)                              //detect charge in, notify app , wait timeout before charging if in a treatment
    {
        sleep_flag               = false;
        bio_packet.awakesec  = BIO_AWAKE_TIME;//???
        bio_packet.offsec    = BIO_OFF_TIME;//???
        if(bio_packet.bio_state <= BIO_STATE_EQUILIB)  // in a treatment
        {
          //bio_sm_save          = bio_sm;
          //bio_state_save       = bio_packet.bio_state;
            bio_packet.bio_state = BIO_STATE_CHRGW;
            wait                 = BIO_CHRG_PAUSE;
            bio_sm               = BIO_SM_PARK;
        }
        else                                           // not in a treatment, begin charging
        {
            bio_sm               = BIO_SM_CHARGE;
            bio_packet.bio_state = BIO_STATE_CHRG;
            //bio_packet.awakesec  = BIO_AWAKE_TIME;//???
            //bio_packet.offsec    = BIO_OFF_TIME;//???
            trigger_cn           = 5;
        }
    }
    else if(statusB&&(!statusA))
    {
        bio_packet.awakesec  = BIO_AWAKE_TIME;//???
        bio_packet.offsec    = BIO_OFF_TIME;//???
        if(retrigger_charging == 0)
        {
            bio_sm               = BIO_SM_IDLE;
            bio_packet.bio_state = BIO_STATE_IDLE;
            retrigger_charging   = 10;
        }
        else
        {
            if(!statusA&&trigger_cn)
            {
                max14676_ChargerOn(MAX_VSET_CURR);
                trigger_cn--;
                return;
            }
        }
        sleep_flag           = false;                                           // Added by jason Chen, 2022.10.20
    }
    else if (bio_packet.batsoc <= 5)
    {
        bio_sm = BIO_SM_OFF;        // shutdown
    }
  }

  switch (bio_sm)
  {
    case  BIO_SM_IDLE:
    {
      if (bio_packet.awakesec)
      {
          bio_packet.awakesec--;                                                // decrement power down reporting timers when idle
          if(bio_packet.awakesec == 0)
            sleep_flag = true;
      }
      else
      {
          if( bio_packet.inithandle != BIO_NO_HANDLE)
            bio_sm_init();                                                      // set init handle and other init when v3combo.awakesec terminates
          if (bio_packet.offsec)
            bio_packet.offsec--;
      }

      if (bio_packet.bio_state == BIO_STATE_COMBO)
      {
          beat = 0;
          timer = bio_packet.time;
          bio_sm = BIO_SM_COMBO;
      }

      if (!bio_packet.offsec)
      {
          //if(bio_sensor_type == BPT_SENSOR)
            bio_sm = BIO_SM_SLEEP;      // Deep Sleep
          //else
          //  bio_sm = BIO_SM_OFF;        // shutdown
      }
    }
    break;

    case BIO_SM_CHARGE:
    {
      if ( !(/*(bio_packet.statusb & MAX_STATB_CHGIN)||*/statusA))
      {
        //ledseq_set(LEDPOWEROFF,1);                                         // kick off LED
        //led_sm = LED_SM_OFF;                                               // signal to LED state machine
        //bio_sm = BIO_SM_OFF;                                               // shutdown
          if(statusB)
          {
              if (bio_packet.batsoc < 95)
              {
                  if(retrigger_charging)
                  {
                      retrigger_charging--;
                      max14676_ChargerOn(MAX_VSET_CURR);                        // debounce charging, each rising edge of CHRGIN resets registers
                  }
              }
          }
          else
          {
              bio_sm               = BIO_SM_IDLE;
              bio_packet.bio_state = BIO_STATE_IDLE;
              bio_packet.awakesec  = BIO_AWAKE_TIME;
              bio_packet.offsec    = BIO_OFF_TIME;
              retrigger_charging   = 10;
          }
      }
      else
      {
          if (bio_packet.batsoc < 95)
          {
              if(retrigger_charging)
              {
                  retrigger_charging--;
                  max14676_ChargerOn(MAX_VSET_CURR);                            // debounce charging, each rising edge of CHRGIN resets registers
              }
          }
      }
    //max14676_ChargerOn();                                                  // debounce charging, each rising edge of CHRGIN resets registers
    }
    break;

    case BIO_SM_OFF:                                                         //  shutdown after led, beep and haptic  (LED must be longer time)
    {
        max14676_poweroff();  //does not return                              // power off , wait for LED sequence to complete
    }
    break;

    case BIO_SM_SLEEP:
    {
      preSleep_config();

      start_test     = true;
      selected_emode = EMODE_4;
      bio_sm         = BIO_SM_PAUSE;
    }
    break;

    case BIO_SM_COMBO:
    {
      if (up) beat++; else beat--;
      // make sine variable change atomic  (look at doing add(s) before disable to reduce disable time)

      if (beat >=10 ) up = 0;  //count down
      if (beat == 0) up = 1;   //count up
   // set intensity

      if (!timer-- || (bio_packet.bio_state != BIO_STATE_COMBO))
      {
          //SinOff();
          bio_sm = BIO_SM_IDLE;
          bio_packet.bio_state = BIO_STATE_IDLE;
      }
    }
    break;

    case BIO_SM_PARK:  // indiciate to app that charger is plugged in until timeout
    {
      wait--;
      if (!wait)   // transition to charging after timeout
      {
          bio_sm = BIO_SM_CHARGE;
          bio_packet.bio_state = BIO_STATE_CHRG;
      }

      if ( !(/*(bio_packet.statusb & MAX_STATB_CHGIN)||*/statusA))  // charger unplugged before timeout, resume
      {
        //bio_sm               = bio_sm_save;
        //if (v3_sm_save == V3_SM_COMBO) SinOn(v3combomod.inten);
        //bio_packet.bio_state = bio_state_save;
          bio_sm               = BIO_SM_IDLE;
          bio_packet.bio_state = BIO_STATE_IDLE;
          bio_packet.awakesec  = BIO_AWAKE_TIME;
          bio_packet.offsec    = BIO_OFF_TIME;
      }
    }
    break;

    case BIO_SM_PAUSE:
    {
      //bio_packet.bio_state = BIO_STATE_IDLE;
      //bio_combo.awakesec   = BIO_AWAKE_TIME;
      //bio_combo.offsec     = BIO_OFF_TIME;
      //bio_sm               = BIO_SM_IDLE;
    }
    break;

    default:
    {
      bio_sm               = BIO_SM_IDLE;
      bio_packet.bio_state = BIO_STATE_IDLE;
    }
    break;
  }

  bio_packet.temp0 = Temperature(MAX30208A_ADR); // read temperature sensor A
}

