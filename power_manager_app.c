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
#define CURRENT_MODULE_NAME "power_manager_app"

#include <stdio.h>
#include "power_manager_app.h"
#include "sl_power_manager.h"
#include "sl_power_manager_debug.h"
#include "sl_sleeptimer.h"
#include "sl_bt_api.h"
#include "em_emu.h"
#include "app_assert.h"

#include "em_gpio.h"
#include "sl_emlib_gpio_init_G_INT1_config.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#define EM_EVENT_MASK_ALL  (SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM0   \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM0  \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM1 \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM1  \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM2 \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM2  \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM3 \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM3)

#ifndef BUTTON_INSTANCE_0
#define BUTTON_INSTANCE_0   sl_button_btn0
#endif

#ifndef BUTTON_INSTANCE_1
#define BUTTON_INSTANCE_1   sl_button_btn1
#endif
#define NO_OF_EMODE_TESTS 5

/*******************************************************************************
 *******************  LOCAL FUNCTION DECLARATIONS   ****************************
 ******************************************************************************/

// Emode transition callback
static void transition_callback(sl_power_manager_em_t from,
                                sl_power_manager_em_t to);

// sleeptimer callback
//static void timer_callback(sl_sleeptimer_timer_handle_t *handle,
//                           void *data);

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

static sl_power_manager_em_transition_event_handle_t event_handle;
// energy mode transition callback settings
static sl_power_manager_em_transition_event_info_t event_info = {
  .event_mask = EM_EVENT_MASK_ALL,     // subscribe to all em transitions
  .on_event = transition_callback      // callback triggered on transition event
};

//static sl_sleeptimer_timer_handle_t my_timer;

// Flag to indicate to power manager if application can sleep
// The application will start in EM0
/*static*/ bool ok_to_sleep = false;
// Flag to indicate to power manager if the application should return to sleep
// after an interrupt
/*static*/ sl_power_manager_on_isr_exit_t isr_ok_to_sleep = SL_POWER_MANAGER_IGNORE;
// Target energy mode
static sl_power_manager_em_t em_mode = SL_POWER_MANAGER_EM0;
// User selected energy mode
/*static*/ volatile energy_mode_enum_t selected_emode;
// Start selected energy mode test.
/*static*/ volatile int start_test;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

/***************************************************************************//**
 * Function called when energy mode changes.
 ******************************************************************************/
static void transition_callback(sl_power_manager_em_t from,
                                sl_power_manager_em_t to)
{
  // Set breakpoint to monitor transitions
  (void)from;
  (void)to;
}

/***************************************************************************//**
 * Function called when sleep timer expires.
 ******************************************************************************/
//static void timer_callback(sl_sleeptimer_timer_handle_t *handle,
//                           void *data)
//{
//  (void)handle;
//  (void)data;
  // when the timer expires, prevent return to sleep
//  ok_to_sleep = false;
//  isr_ok_to_sleep = SL_POWER_MANAGER_WAKEUP;
  // Clear requirements
//  if (em_mode == SL_POWER_MANAGER_EM1
//      || em_mode == SL_POWER_MANAGER_EM2) {
//    sl_power_manager_remove_em_requirement(em_mode);
//  }
//}

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * Hook for power manager
 ******************************************************************************/
bool app_is_ok_to_sleep(void)
{
  // return false to prevent sleep mode and force EM0
  // return true to indicate to power manager that application can sleep
  return ok_to_sleep;
}

/***************************************************************************//**
 * Hook for power manager
 ******************************************************************************/
sl_power_manager_on_isr_exit_t app_sleep_on_isr_exit(void)
{
  // flag used by power manager to determine if device can return to sleep
  // following interrupt
  return isr_ok_to_sleep;
}

/***************************************************************************//**
 * Callback on button change.
 *
 * This function overrides a weak implementation defined in the simple_button
 * module. It is triggered when the user activates one of the buttons.
 *
 ******************************************************************************/
#if 0
void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED)
  {
    if (&BUTTON_INSTANCE_0 == handle)
    {
      start_test = true;
    }
    else if (&BUTTON_INSTANCE_1 == handle)
    {
      selected_emode = (energy_mode_enum_t)((selected_emode + 1) % NO_OF_EMODE_TESTS);
    }
    isr_ok_to_sleep = SL_POWER_MANAGER_WAKEUP;
  }
}
#else
void postSleep_exit(void)
{
  ok_to_sleep = false;
  isr_ok_to_sleep = SL_POWER_MANAGER_WAKEUP;
  // Clear requirements
  if (em_mode == SL_POWER_MANAGER_EM1
      || em_mode == SL_POWER_MANAGER_EM2) {
    sl_power_manager_remove_em_requirement(em_mode);
  }
}
#endif
/***************************************************************************//**
 * Initialize example.
 ******************************************************************************/
void power_manager_app_init(void)
{
  // Subscribe to energy mode transitions
  // The callback set in event_info will be called when the energy mode changes,
  // based on the transition mask
  sl_power_manager_subscribe_em_transition_event(&event_handle, &event_info);
}

/***************************************************************************//**
 * Ticking function.
 ******************************************************************************/
#define EM4ENABLEPIN_MASK               0x00010000
#define EM4ENABLEPIN_SHIFT              3                                       // Select GPIO.EM4WU3 (PB01)
extern void disable_i2c0(void);
extern void lsm6dsox_g_int1_cfg(bool mode_sel);
void power_manager_app_process_action(void)
{
  energy_mode_enum_t emode;

  if (start_test) {
    start_test = false;
    emode = selected_emode;
    selected_emode = EMODE_0;

    switch (emode) {
      case EMODE_0:
        // flag returned by app_is_ok_to_sleep()
        // returning false will prevent sleep
        ok_to_sleep = false;
        em_mode = SL_POWER_MANAGER_EM0;
        break;
      case EMODE_1:
        // Set flag returned by app_is_ok_to_sleep() to indicate
        // to power manager that application can sleep
        ok_to_sleep = true;
        // Set flag returned by app_sleep_on_isr_exit() to indicate
        // to power manager that application can return to sleep after
        // interrupt
        isr_ok_to_sleep = SL_POWER_MANAGER_SLEEP;
        em_mode = SL_POWER_MANAGER_EM1;
        // Ensure app stays in selected energy mode
        sl_power_manager_add_em_requirement(em_mode);
        // Sleep timer functions in EM1
        //sl_sleeptimer_start_timer_ms(&my_timer,
        //                             5000,
        //                             timer_callback,
        //                             (void *)NULL,
        //                             0,
        //                             0);
        break;
      case EMODE_2:
        // Set flag returned by app_is_ok_to_sleep() to indicate
        // to power manager that application can sleep
        ok_to_sleep = true;
        // Set flag returned by app_sleep_on_isr_exit() to indicate
        // to power manager that application can return to sleep after
        // interrupt
        isr_ok_to_sleep = SL_POWER_MANAGER_SLEEP;
        em_mode = SL_POWER_MANAGER_EM2;
        // Ensure app stays in selected energy mode
        //sl_status_t sc = sl_bt_system_halt(1);
        //app_assert_status(sc);
        sl_power_manager_add_em_requirement(em_mode);
        // Sleep timer functions in EM2
        //sl_sleeptimer_start_timer_ms(&my_timer,
        //                             5000,
        //                             timer_callback,
        //                             (void *)NULL,
        //                             0,
        //                             SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
        break;
      case EMODE_3:
        // Set flag returned by app_is_ok_to_sleep() to indicate
        // to power manager that application can sleep
        ok_to_sleep = true;
        // Set flag returned by app_sleep_on_isr_exit() to indicate
        // to power manager that application can return to sleep after
        // interrupt
        isr_ok_to_sleep = SL_POWER_MANAGER_SLEEP;
        em_mode = SL_POWER_MANAGER_EM3;
        // Sleep timer does not function in EM3
        // Application will be woken from sleep in button interrupt
        break;
      case EMODE_4:
        // EM4 is not supported by Power Manager
#if 0
        em_mode = SL_POWER_MANAGER_EM4;
        EMU_EnterEM4();
#else
        {
        lsm6dsox_g_int1_cfg(false);
        uint32_t em4enablepin = GPIO->EM4WUEN  | (EM4ENABLEPIN_MASK << EM4ENABLEPIN_SHIFT);
        uint32_t em4enablepol = GPIO->EM4WUPOL | (EM4ENABLEPIN_MASK << EM4ENABLEPIN_SHIFT);
        uint32_t em4enableEN  = GPIO->IEN      | (EM4ENABLEPIN_MASK << EM4ENABLEPIN_SHIFT);

        em_mode = SL_POWER_MANAGER_EM4;

     // Enable GPIO_EM4WU3 (PB01) wake up evnt from deep sleep mode EM4
        GPIO_EM4EnablePinWakeup(em4enablepin, em4enablepol);                               // Set GPIO.EM4WU3 (PB01) as wakeup of CPU from EM4
        GPIO_IntEnable(em4enableEN);                                                       // GPIO.EM4WU3 interrupt enabled
        //NVIC_EnableIRQ(GPIO_EVEN_IRQn);
        //NVIC_EnableIRQ(GPIO_ODD_IRQn);
        disable_i2c0();
        EMU_EnterEM4();
        NVIC_SystemReset();
        }
#endif
        break;
      default:
        EFM_ASSERT(false);
        break;
    }
  }
}
