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
#ifndef _MAX14676_H_
#define _MAX14676_H_

#include "bio_sm.h"

#define MAX14676E_ADR50	0x50                 //PMIC 8 bit (shifted) I2C address
#define MAX14676E_ADR6C	0x6C                 //PMIC fuel guage 8 bit I2C address

//MAX initialization values
#define LEDSTRT         0x24
#define MAX_LEDN_CFG    0x20
#define MAX_LEDN_MPC0   0xA0                // MPC0 for PWM control
#define MAX_LEDN_MPC1   0xE0                // MPC1 for PWM Control
#define MAX_BST_ON      0x49                // Enable BOOST at +14V
#define MAX_BST_OFF     0x09                // Turn off BOOST supply
#define MAX_CDET_INIT   0x84                // CDETCNTLB reg init - Charger Auto Restart
#define MAX_VSET_INIT   0x00
#define MAX_VSET_4_20   0x00
#define MAX_VSET_4_25   MAX_VSET_4_20//0x10
#define MAX_VSET_4_30   0x20
#define MAX_VSET_4_35   0x30

#define MAX_VSET_CURR   MAX_VSET_4_20//MAX_VSET_4_25

//MAX 0x50 addressed registers
#define MAX_CHIP_ID         0x00
#define MAX_STATUSA         0x02
#define MAX_STATUSB         0x03
#define MAX_ILIMCNTL_ADDR   0x0A
#define MAX_CHGCNTLA        0x0B
#define MAX_CHGCNTLB        0x0C
#define MAX_CHGTMR          0x0D
#define MAX_CHGVSET         0x0E
#define MAX_PCHGCNTL        0x10
#define MAX_CDETCNTLB       0x11
#define MAX_BUCKCFG         0x15
#define MAX_LDOCFG          0x16
#define MAX_BST_CFG         0x19
#define MAX_LED_CFG         0x1A
#define MAX_LED0_GRN        0x1B
#define MAX_LED1_BLU        0x1C
#define MAX_LED2_RED        0x1D
#define MAX_PWR_CFG_ADDR    0x1E

//MAX registerA masks
#define MAX_STATB_CHGIN     0x08  //Charge is connected
//MAX registerA masks
#define MAX_STATA_CHGIN     0x07  //Charge status

//MAX 0x6C addressed registers
#define MAX_VCELL	          0x02
#define MAX_SOC		          0x04
#define MAX_CRATE	          0x16
#define MAX_POWERON         0xFE

#define POWERON_RESET       0x5400
#define POWERON_DEFAULT     0xFFFF

extern uint8_t charging_voltage_settings;
extern bool max14676_init(void);
extern uint8_t max14676_GetBatteryCharge(void);
extern uint16_t max14676_GetBatteryChargeVoltage( void);
extern uint16_t max14676_GetBatteryCRATE( void);
extern uint8_t max14676_GetStatusB(void);
extern uint8_t max14676_GetStatusA(void);
extern void max14676_ChargerOn(uint8_t charging_bat);
extern void max14676_Boost(uint8_t state);
extern void max14676_poweroff(void);
extern void max14676_PowerOnReset(bool enableR);

// MAX30208 temperatuer sensor defines
#define MAX30208A_ADR       0xA2      // wireless Temp sensor Address

#define MAX30208_ID             0xFF      // Part Identifier
#define MAX30208_SETUP          0x14      // part Setup/ initiate register TEMP_SENSOR_SETUP
#define MAX30208_CFG2           0x0A      // FIFO confiuration 2 - FIFO CONFIGURATION 2
#define MAX30208_DATA           0x08      // FIFO Data register FIFO_DATA
#define MAX30208_COUNT          0x07      // FIFO count FIFO_DATA_COUNT
#define MAX30208_STATUS         0x00      // Part Staus register
#define MAX30208_SYSTEM_CONTROL 0x0C      // System Control                                         // Added by Jason Chen, 2022.09.19

#define MAX30208_STATUS_RDY 0x01      // Status bit sample ready mask
#define MAX30208_SETUP_GO   0xC1      // Value to write to initiate a conversion
#define MAX30208_ID_VAL     0x30      // Value that should be in register MAX30208_ID
#define MAX30208_CFG2_FLUSH 0x10      // Value to write to CFG2 to flush FIFO
#define MAX30208_RESET      0x01                                                                    // Added by Jason Chen, 2022.09.19

#define MAX30208_WAIT       0xFFFF    // temperature return value when present, waiting on first conversion
#define MAX30208_ERROR      0xDEAD    // temperature sensor does not respond

extern uint16_t Temperature(uint8_t addr);

#endif
