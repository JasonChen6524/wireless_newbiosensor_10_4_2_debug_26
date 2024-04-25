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
#include "LSM6DSOX.h"
#include "sl_i2cspm.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "sl_emlib_gpio_init_G_INT1_config.h"

#define MAXTRIES 20000  // for I2C read and write checking if HW stuck

//extern void wait_ms(uint16_t wait_ms);
static int I2C_WRITE(uint8_t addr, uint8_t reg , uint8_t *data, uint16_t len)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = addr;
  seq.flags = I2C_FLAG_WRITE_WRITE;

  seq.buf[0].len = 1;
  seq.buf[1].len = len;
  seq.buf[0].data = &reg;
  seq.buf[1].data = data;

    // Do a polled transfer
  ret = I2CSPM_Transfer(I2C0, &seq);
  if (ret != i2cTransferDone) {
    return ret;//false;
  }
// Debug code for maxtries testing
   //if (i2ccounts[1]< maxtries) i2ccounts[1] = maxtries;

  return ((int) ret);
}

static int I2C_READ(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{

  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = addr;
  seq.flags = I2C_FLAG_WRITE_READ;

  seq.buf[0].len = 1;
  seq.buf[1].len = len;
  seq.buf[0].data = &reg;
  seq.buf[1].data = data;

    // Do a polled transfer
  ret = I2CSPM_Transfer(I2C0, &seq);
  if (ret != i2cTransferDone) {
    return ret;//false;
  }
// Debug code for maxtries testing
   //if (i2ccounts[0]< maxtries) i2ccounts[0] = maxtries;

  return ((int) ret);
}

static volatile bool g_int1_received = false;

static void acc_irq_handler(uint8_t intNo)
{
  (void)intNo;
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  g_int1_received = true;
  //ok_to_sleep = false;
  //postSleep_exit();
}

void acc_clear_interrupt_flag(void){
  g_int1_received = false;
}

bool acc_interrupt_flag(void){
  bool int1_g = g_int1_received;
  return int1_g;//g_int1_received;
}

/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
static void accGPIOSetup(void)
{
  /* Configure GPIO Clock */
//CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure Button PC4 as input and enable interrupt */
//GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_MFIO_PORT, SL_EMLIB_GPIO_INIT_MFIO_PIN, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(SL_EMLIB_GPIO_INIT_G_INT1_PORT,
                    SL_EMLIB_GPIO_INIT_G_INT1_PIN,
                    SL_EMLIB_GPIO_INIT_G_INT1_PIN,          // Interrupt Number
                    false,                                  // RisingEdge  Disable
                    true,                                   // FallingEdge Enable
                    true                                    // interrupt   Enable
          );

  /* Enable ODD interrupt to catch button press that changes slew rate */
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
//NVIC_EnableIRQ(GPIO_ODD_IRQn);
  NVIC_DisableIRQ(GPIO_ODD_IRQn);

  GPIOINT_CallbackRegister(SL_EMLIB_GPIO_INIT_G_INT1_PIN, (GPIOINT_IrqCallbackPtr_t)&acc_irq_handler);
//GPIO_IntEnable(1 << SL_EMLIB_GPIO_INIT_G_INT1_PIN);
  GPIO_IntDisable(1 << SL_EMLIB_GPIO_INIT_G_INT1_PIN);
}

void acc_intEnable(bool intEnable)
{
  if(intEnable)
  {
      NVIC_EnableIRQ(GPIO_ODD_IRQn);
      GPIO_IntEnable(1 << SL_EMLIB_GPIO_INIT_G_INT1_PIN);
  }
  else
  {
      NVIC_DisableIRQ(GPIO_ODD_IRQn);
      GPIO_IntDisable(1 << SL_EMLIB_GPIO_INIT_G_INT1_PIN);
  }
}

uint8_t lsm6dsox_GetID(void)
{
  uint8_t result;
  uint8_t var[1];
  
  result =  (uint8_t)I2C_READ(LSM6DSOX_ADR_READ,WHO_AM_I,var,1);

  
  if((result == 0 )&&(var[0] == 0x6C))
    return var[0];
  else
    return(0);
}
#if 0
static void lsm6dsox_g_int1_cfg(void)                                           // See AN5272
{
  uint8_t var[1];
#if LSM6DSOX_LOG_ENABLE
  I2C_TransferReturn_TypeDef ret;
#endif
#if MODE_SINGLE_TAP
      //The basic SW routine for single-tap detection is given below.
      // 1. Write 60h to CTRL1_XL // Turn on the accelerometer                      // ODR_XL = 417 Hz, FS_XL = ±2 g
      var[0]=0x60;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif

      // 2. Write 0Eh to TAP_CFG0                                                   // Enable tap detection on X, Y, Z-axis
      var[0]=0x0E;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG0, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      // 3. Write 09h to TAP_CFG1                                                   // Set X-axis threshold and axes priority
      var[0]=0x09;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG1, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      // 4. Write 89h to TAP_CFG2                                                   // Set Y-axis threshold and enable interrupt
      var[0]=0x89;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG2, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      // 5. Write 09h to TAP_THS_6D                                                 // Set Z-axis threshold
      var[0]=0x09;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_THS_6D, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      // 6. Write 06h to INT_DUR2                                                   // Set Quiet and Shock time windows
      var[0]=0x06;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, INT_DUR2, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      // 7. Write 00h to WAKE_UP_THS                                                // Only single-tap enabled (SINGLE_DOUBLE_TAP = 0)
      var[0]=0x00;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_THS , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      // 8. Write 40h to MD1_CFG                                                    // Single-tap interrupt driven to INT1 pin
      var[0]=0x40;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, MD1_CFG, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
#endif

#if MODE_DOUBLE_TAP
   //A basic SW routine for double-tap detection is given below.
   //1. Write 60h to CTRL1_XL            // Turn on the accelerometer  // ODR_XL = 417 Hz, FS_XL = ±2 g
      var[0]=0x60;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //2. Write 0Eh to TAP_CFG0            // Enable tap detection on X, Y, Z-axis
      var[0]=0x0E;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG0 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //3. Write 0Ch to TAP_CFG1            // Set X-axis threshold and axes priority
      var[0]=0x0C;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG1 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //4. Write 8Ch to TAP_CFG2            // Set Y-axis threshold and enable interrupt
      var[0]=0x8C;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG2 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //5. Write 0Ch to TAP_THS_6D          // Set Z-axis threshold
      var[0]=0x0C;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_THS_6D , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //6. Write 7Fh to INT_DUR2            // Set Duration, Quiet and Shock time windows
      var[0]=0x7F;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, INT_DUR2 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //7. Write 80h to WAKE_UP_THS         // Single-tap and double-tap enabled (SINGLE_DOUBLE_TAP = 1)
      var[0]=0x80;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_THS , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //8. Write 08h to MD1_CFG             // Double-tap interrupt driven to INT1 pin
      var[0]=0x08;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, MD1_CFG , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
#endif

#if MODE_WAKE_UP
    //A basic SW routine for wake-up event recognition using the high-pass digital filter is given below
    //1. Write 60h to CTRL1_XL // Turn on the accelerometer   // ODR_XL = 417 Hz, FS_XL = ±2 g
      var[0]=0x60;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //2. Write 51h to TAP_CFG0 // Enable latch mode with reset on read and digital high-pass filter
      var[0]=0x51;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG0, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //3. Write 80h to TAP_CFG2 // Enable interrupt function
      var[0]=0x80;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG2, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //4. Write 00h to WAKE_UP_DUR // No duration and selection of wake-up threshold weight (1 LSB = FS_XL / 26)
      var[0]=0x00;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_DUR, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //5. Write 02h to WAKE_UP_THS // Set wake-up threshold
      var[0]=0x02;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_THS, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //6. Write 20h to MD1_CFG // Wake-up interrupt driven to INT1 pin
      var[0]=0x20;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE,  MD1_CFG, var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
#endif

#if MODE_ACTIVITY_INACTIVITY
    //A basic SW routine for Activity/Inactivity detection is as follows:
    //1. Write 50h to CTRL1_XL            // Turn on the accelerometer                         // ODR_XL = 208 Hz, FS_XL = ±2 g
      var[0]=0x50;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //2. Write 40h to CTRL2_G             // Turn on the gyroscope                             // ODR_G = 104 Hz, FS_G = ±250 dps
      var[0]=0x40;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL2_G , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //3. Write 02h to WAKE_UP_DUR         // Set duration for Inactivity detection             // Select Activity/Inactivity threshold resolution and duration
      var[0]=0x02;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_DUR , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //4. Write 02h to WAKE_UP_THS         // Set Activity/Inactivity threshold
      var[0]=0x02;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_THS , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //5. Write 00h to TAP_CFG0            // Select sleep-change notification                  // Select slope filter
      var[0]=0x00;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG0 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //6. Write E0h to TAP_CFG2            // Enable interrupt                                  // Inactivity configuration: accelerometer to 12.5 Hz (LP mode), // Gyroscope to Power-Down mode
      var[0]=0xE0;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG2 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif

      //7. Write 80h to MD1_CFG // Activity/Inactivity interrupt driven to INT1 pin
      var[0]=0x80;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, MD1_CFG , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
#endif

#if MODE_WAKE_UP_WAITTIME
    //wake up
    //1. Write 00h to WAKE_UP_DUR         // No duration and selection of wake-up threshold weight (1 LSB = FS_XL / 26)
      var[0]=0x00;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_DUR , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //2. Write 02h to WAKE_UP_THS         // Set wake-up threshold
      var[0]=0x02;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_THS , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //3. Write 51h to TAP_CFG0            // Enable interrupts and apply slope filter; latch mode disabled
      var[0]=0x61;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG0 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //4. Write 80h to TAP_CFG2            // Enable interrupt function
      var[0]=0x80;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG2 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //5. Write 70h to CTRL1_XL            // Turn on the accelerometer,  // ODR_XL = 833 Hz, FS_XL = ±2 g
      var[0]=0x70;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //6. Wait 4 ms                        // Insert (reduced) wait time
      wait_ms(4);

      //7. Write 10h to CTRL1_XL            // ODR_XL = 12.5 Hz
      var[0]=0x10;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //8. Write 20h to MD1_CFG             // Wake-up interrupt driven to INT1 pin
      var[0]=0x20;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, MD1_CFG , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
#endif

#if MODE_6D_ORIENTATION
    //A basic SW routine for 6D orientation detection is as follows.
    //1. Write 60h to CTRL1_XL            // Turn on the accelerometer, // ODR_XL = 417 Hz, FS_XL = ±2 g
      var[0]=0x60;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //2. Write 41h to TAP_CFG0            // Enable latch mode with reset on read
      var[0]=0x41;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG0 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //3. Write 80h to TAP_CFG2            // Enable interrupt function
      var[0]=0x80;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG2 , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //4. Write 40h to TAP_THS_6D          // Set 6D threshold (SIXD_THS[1:0] = 10b = 60 degrees)
      var[0]=0x40;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_THS_6D , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //5. Write 01h to CTRL8_XL            // Enable LPF2 filter to 6D functionality
      var[0]=0x01;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL8_XL , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //6. Write 04h to MD1_CFG             // 6D interrupt driven to INT1 pin
      var[0]=0x04;
#if LSM6DSOX_LOG_ENABLE
      ret =
#endif
      I2C_WRITE(LSM6DSOX_ADR_WRITE, MD1_CFG , var,1);
#if LSM6DSOX_LOG_ENABLE
      if(ret){lsm6d_log("lsm6dsox write fails...ret=%d\r\n", ret);}
#endif
      //Stationary/Motion detection is a particular case of the Activity/Inactivity functionality in which no ODR / power
      //mode changes occur when a sleep condition (equivalent to Stationary condition) is detected. Stationary/Motion
      //detection is activated by setting the INACT_EN[1:0] bits of the TAP_CFG2 register to 00b.
#endif
}
#else
void lsm6dsox_g_int1_cfg(bool mode_sel)
{
  uint8_t var[1];

  if(!mode_sel)
  {
   //A basic SW routine for double-tap detection is given below.
   //1. Write 60h to CTRL1_XL            // Turn on the accelerometer  // ODR_XL = 417 Hz, FS_XL = ±2 g
      var[0]=0x60;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL , var,1);
      //2. Write 0Eh to TAP_CFG0            // Enable tap detection on X, Y, Z-axis
      var[0]=0x0E;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG0 , var,1);
      //3. Write 0Ch to TAP_CFG1            // Set X-axis threshold and axes priority
      var[0]=0x0C;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG1 , var,1);
      //4. Write 8Ch to TAP_CFG2            // Set Y-axis threshold and enable interrupt
      var[0]=0x8C;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG2 , var,1);
      //5. Write 0Ch to TAP_THS_6D          // Set Z-axis threshold
      var[0]=0x0C;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_THS_6D , var,1);
      //6. Write 7Fh to INT_DUR2            // Set Duration, Quiet and Shock time windows
      var[0]=0x7F;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, INT_DUR2 , var,1);
      //7. Write 80h to WAKE_UP_THS         // Single-tap and double-tap enabled (SINGLE_DOUBLE_TAP = 1)
      var[0]=0x80;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_THS , var,1);
      //8. Write 08h to MD1_CFG             // Double-tap interrupt driven to INT1 pin
      var[0]=0x08;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, MD1_CFG , var,1);
  }
  else
  {
    //A basic SW routine for Activity/Inactivity detection is as follows:
    //1. Write 50h to CTRL1_XL            // Turn on the accelerometer                         // ODR_XL = 208 Hz, FS_XL = ±2 g
      var[0]=0x50;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL1_XL , var,1);
      //2. Write 40h to CTRL2_G             // Turn on the gyroscope                           // ODR_G = 104 Hz, FS_G = ±250 dps
      var[0]=0x40;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, CTRL2_G , var,1);
      //3. Write 02h to WAKE_UP_DUR         // Set duration for Inactivity detection           // Select Activity/Inactivity threshold resolution and duration
      var[0]=0x02;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_DUR , var,1);
      //4. Write 02h to WAKE_UP_THS         // Set Activity/Inactivity threshold
      var[0]=0x02;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, WAKE_UP_THS , var,1);
      //5. Write 00h to TAP_CFG0            // Select sleep-change notification                // Select slope filter
      var[0]=0x00;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG0 , var,1);
      //6. Write E0h to TAP_CFG2            // Enable interrupt                                // Inactivity configuration: accelerometer to 12.5 Hz (LP mode), // Gyroscope to Power-Down mode
      var[0]=0xE0;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, TAP_CFG2 , var,1);

      //7. Write 80h to MD1_CFG // Activity/Inactivity interrupt driven to INT1 pin
      var[0]=0x80;
      I2C_WRITE(LSM6DSOX_ADR_WRITE, MD1_CFG , var,1);
  }
}
#endif
// return true if exists
bool lsm6dsox_init(bool irq_enable)
{
  if(!lsm6dsox_GetID()) return(false);

  lsm6dsox_g_int1_cfg(true);

  accGPIOSetup();
  acc_intEnable(irq_enable);
  return(true);
   
}

bool get_accel_data(accel_data_TT *accel_data)
{
  uint8_t result;
  float acc_d;

  result =  (uint8_t)I2C_READ(LSM6DSOX_ADR_READ, OUTX_L_A, accel_data->accelByte, 6);

  acc_d = accel_data->accl_data.x * (float)0.061;
  accel_data->accl_data.x = (int16_t)acc_d;
  acc_d = accel_data->accl_data.y * (float)0.061;
  accel_data->accl_data.y = (int16_t)acc_d;
  acc_d = accel_data->accl_data.z * (float)0.061;
  accel_data->accl_data.z = (int16_t)acc_d;

  return result;
}
