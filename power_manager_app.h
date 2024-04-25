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

#ifndef POWER_MANAGER_APP_H
#define POWER_MANAGER_APP_H

#include <stdbool.h>
#include "sl_power_manager.h"

// Energy mode enumerations
typedef enum {
  EMODE_0,
  EMODE_1,
  EMODE_2,
  EMODE_3,
  EMODE_4,
  NUM_EMODES
} energy_mode_enum_t;

// Flag to indicate to power manager if application can sleep
// The application will start in EM0
extern /*static*/ bool ok_to_sleep;
// Flag to indicate to power manager if the application should return to sleep
// after an interrupt
extern /*static*/ sl_power_manager_on_isr_exit_t isr_ok_to_sleep;
// User selected energy mode
extern /*static*/ volatile energy_mode_enum_t selected_emode;
// Start selected energy mode test.
extern /*static*/ volatile int start_test;


/***************************************************************************//**
 * Initialize example
 ******************************************************************************/
void power_manager_app_init(void);

/***************************************************************************//**
 * Ticking function
 ******************************************************************************/
void power_manager_app_process_action(void);

#endif  // POWER_MANAGER_APP_H
