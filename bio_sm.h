 /*
 * bio_sm.h
 *
 *  Created on: 01/04/2022
 *      Author: Jason chen
 */
#ifndef __BIO_SM__
#define __BIO_SM__

typedef enum
{
  BIO_SM_IDLE     = 0,
  BIO_SM_COMBO,
  BIO_SM_PAUSE,
  BIO_SM_CHARGE,
  BIO_SM_OFF,
  BIO_SM_SLEEP,
  BIO_SM_PARK,
  BIO_SM_CURRENT,
  BIO_SM_CURRENT1
}BIO_SM_TypeDef;
extern uint8_t bio_sm;

#define BIO_AWAKE_TIME                 (10)                    // initial/ default seconds of LED flash
#define BIO_SLEEP_TIME_BPT             (1*60)                 // initial/ default seconds of BPT before sleeping
#define BIO_OFF_TIME_HRV               (1*60)                 // initial/ default seconds of HRV before Power off

extern uint16_t BIO_OFF_TIME;                                  // Added by Jason Chen, 2022.10.11

#define BIO_CHRG_PAUSE                 30                      // number of seconds which charge connected can interrupt a treatment
extern uint8_t retrigger_charging;
extern void bio_sm_init(void);
extern void bio_sm_state(void);
extern void bio_info_init(void);

// BLE SPP connection state set in spp_server_main.c
#define STATE_POWERON	                 0
#define STATE_ADVERTISING              1
#define STATE_CONNECTED                2
#define STATE_SPP_MODE                 3

#define BIO_NO_HANDLE                  0xFFFF                  // token sent for message handle when there is no handle.

typedef enum  // order is important for VM state machine override code
{
   BIO_STATE_COMBO,
   BIO_STATE_COMBOPAUSE,
   BIO_STATE_EQUILIB,
   BIO_STATE_SLEEP,
   BIO_STATE_LOWPOWER,
   BIO_STATE_IDLE,
   BIO_STATE_CUR,
   BIO_STATE_CHRG,
   BIO_STATE_CHRGW,
   BIO_STATE_OTA
} BIO_STATE_TypeDef;

struct bio_ble_packet                  // rearrange and pack later adding to the end for now
{
   uint32_t time;                      // report device time
   uint16_t inithandle;                // handle of the init
   uint8_t  batsoc;                    // 8 bit state of charge value read from PMIC fuel Gauge - 0 to 100
   uint8_t  statusb;                   // PMIC StatusB register
   uint16_t batvcell;                  // 16 bit battery voltage value read from PMIC fuel Gauge, 1 LSb = 78.125uV
   uint16_t temp0;                     // Tethered sensor 0 (Left) = deg C *200
   uint8_t  sec_flag;                  // Second Flag;
   uint8_t  spp;                       // BLE connection state
   uint16_t crate;                     // Rate of battery charge or discharge, 1 LSb - 0.208% per hour
   uint8_t  statusa;                   // PMIC StatusA register
   uint8_t  bio_status;
   uint8_t  bio_sys_bp;
   uint8_t  bio_dia_bp;
   uint8_t  bio_prog;
   uint16_t bio_hr;
   uint16_t bio_spo2;
   uint8_t  bio_state;
   uint16_t awakesec;                  // time to report after combo completed , seconds
   uint16_t offsec;                    // time to stay on, not reporting, seconds
   uint8_t  other_sec_flag;            // flex second flag,                     by Jason Chen, 2022.10.18
};
extern struct bio_ble_packet bio_packet;

#endif /* bio_sm.h */
