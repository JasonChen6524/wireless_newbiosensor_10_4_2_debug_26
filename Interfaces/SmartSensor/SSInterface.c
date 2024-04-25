/***************************************************************************
* Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
****************************************************************************
*/
#include "app.h"
#include "SSInterface.h"
#include "em_i2c.h"                                            // Added by Jason Chen, 2021.11.15
#include "gpiointerrupt.h"
#include "sl_emlib_gpio_init_MFIO_config.h"
#include "sl_sleeptimer.h"

#include "sl_emlib_gpio_init_bio_reset_config.h"

//#define DEBUG_TIMING_PROFILE

static char fw_version[128];
static char algo_version[128];
static const char* plat_name;

static bool in_bootldr;
static bool sc_en;
static int data_type;

static volatile bool m_irq_received_;
static volatile bool mfio_int_happened;

static int sensor_enabled_mode[SS_MAX_SUPPORTED_SENSOR_NUM];
static ss_data_req* sensor_data_reqs[SS_MAX_SUPPORTED_SENSOR_NUM];

static void fifo_sample_size(int data_type, int* sample_size1);
static int algo_enabled_mode[SS_MAX_SUPPORTED_ALGO_NUM];
static ss_data_req* algo_data_reqs[SS_MAX_SUPPORTED_ALGO_NUM];

#ifdef wait_ms
#else
void wait_ms(uint16_t wait_ms)
{
#if 1
	uint32_t delay = sl_sleeptimer_ms_to_tick(wait_ms);
	uint32_t curren_tick1 = sl_sleeptimer_get_tick_count();
	uint32_t diff = 0;
	while(1)
	{
		diff = sl_sleeptimer_get_tick_count() - curren_tick1;
		if(diff > delay)
			return;
	}
#else
	sl_sleeptimer_delay_millisecond(wait_ms);
#endif
}
#endif

static int m_i2cBus_read(uint8_t addr, uint8_t *data, uint16_t len)   // Added by Jason for Biosensor read
{
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;

	seq.addr = addr;
	seq.flags = I2C_FLAG_READ;

	seq.buf[1].len = 0;
	seq.buf[1].data = NULL;
	seq.buf[0].len = len;
	seq.buf[0].data = data;

	// Do a polled transfer
	ret = I2C_TransferInit(I2C0, &seq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}
	return ((int) ret);
}

static int m_i2cBus_write(uint8_t addr, uint8_t *cmd_bytes, int cmd_bytes_len, bool flag)               // Added by Jason for Biosensor Write
{
  (void)flag;
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;

	seq.addr = addr;
	seq.flags = I2C_FLAG_WRITE_WRITE;

	seq.buf[0].len = cmd_bytes_len;
	seq.buf[0].data = cmd_bytes;
	seq.buf[1].len = 0;
	seq.buf[1].data = NULL;

	// Do a polled transfer
	ret = I2C_TransferInit(I2C0, &seq);
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}
	return ((int) ret);
}

void ss_clear_interrupt_flag(void){
	m_irq_received_ = false;
}

SS_STATUS ss_set_hub_mode(bio_sensor_mode_t mode)
{
  uint8_t cmd_bytes[] = { SS_FAM_W_MODE, SS_CMDIDX_MODE };
  uint8_t data[] = { mode };

  SS_STATUS status = write_cmd2(
      &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
      &data[0], ARRAY_SIZE(data), SS_ENABLE_SENSOR_SLEEP_MS);

  in_bootldr = (status == SS_SUCCESS) ? true : false;
  return status;
}
#if 0
SS_STATUS ss_stay_in_bootloader(void)
{
  uint8_t cmd_bytes[] = { SS_FAM_W_MODE, SS_CMDIDX_MODE };
  uint8_t data[] = { SS_MASK_MODE_BOOTLDR };

  SS_STATUS status = write_cmd2(
      &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
      &data[0], ARRAY_SIZE(data), SS_DEFAULT_CMD_SLEEP_MS);

  in_bootldr = (status == SS_SUCCESS) ? true : false;
  return status;
}
#endif
void cfg_mfio(PinDirection pinvalue)
{
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_MFIO_PORT,
                  SL_EMLIB_GPIO_INIT_MFIO_PIN,
                  pinvalue?gpioModePushPull:SL_EMLIB_GPIO_INIT_MFIO_MODE,
                  SL_EMLIB_GPIO_INIT_MFIO_DOUT);
}

void mfio_pin_write(uint8_t outputvalue)
{
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_MFIO_PORT,
                  SL_EMLIB_GPIO_INIT_MFIO_PIN,
                  gpioModePushPull,
                  outputvalue);
}

void reset_pin_input(PinDirection pinvalue)
{
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_BIO_RESET_PORT,
                  SL_EMLIB_GPIO_INIT_BIO_RESET_PIN,
                  pinvalue?SL_EMLIB_GPIO_INIT_BIO_RESET_MODE:gpioModeInputPull,
                  SL_EMLIB_GPIO_INIT_BIO_RESET_DOUT);
}

SS_STATUS reset_to_main_app(void)
{
	ss_disable_irq();
#if defined(BOOTLOADER_USES_MFIO)
	reset_pin_input(PIN_OUTPUT);
	cfg_mfio(PIN_OUTPUT);
	sl_bioreset_highlow(0);                                                       // Added by Jason Chen, 2022.09.29
	wait_ms(SS_RESET_TIME);
	mfio_pin_write(1);
	sl_bioreset_highlow(1);                                                       // Added by Jason Chen, 2022.09.29
	wait_ms(SS_STARTUP_TO_MAIN_APP_TIME);
	cfg_mfio(PIN_INPUT);
	reset_pin_input(PIN_INPUT);
	ss_enable_irq();
	// Verify we exited bootloader mode
	if (ss_in_bootldr_mode() == 0)
		return SS_SUCCESS;
	else
		return SS_ERR_UNKNOWN;
#else
	SS_STATUS status = ss_set_hub_mode(EXIT_BOOTLOADER_MODE);
	ss_enable_irq();
	return status;
#endif
}

SS_STATUS reset_to_bootloader(void)
{
	ss_disable_irq();
#if defined(BOOTLOADER_USES_MFIO)
	reset_pin_input(PIN_OUTPUT);
	cfg_mfio(PIN_OUTPUT);
	sl_bioreset_highlow(0);                                                       // Added by Jason Chen, 2022.09.29
	wait_ms(SS_RESET_TIME);
	mfio_pin_write(0);
	sl_bioreset_highlow(1);                                                       // Added by Jason Chen, 2022.09.29
	wait_ms(SS_STARTUP_TO_BTLDR_TIME);
	cfg_mfio(PIN_INPUT);
	reset_pin_input(PIN_INPUT);
	ss_enable_irq();
	//stay_in_bootloader();

	// Verify we entered bootloader mode
	if (ss_in_bootldr_mode() < 0)
		return SS_ERR_UNKNOWN;
	return SS_SUCCESS;
#else
	/*SS_STATUS status = */ss_set_hub_mode(ENTER_BOOTLOADER_MODE);
	ss_enable_irq();
	return SS_SUCCESS;
#endif
}

SS_STATUS MAX30101_reset(void)                                                                       // Start MAX30101 reset Shutdown MAX32664A/B/C/D Added by Jason Chen, 2022.09.26
{
  SS_STATUS status;
  uint8_t cmd_bytes[] = { SS_FAM_W_WRITEREG, SS_SENSORIDX_MAX30101, 0x09, 0x40};

  status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, SS_ENABLE_SENSOR_SLEEP_MS);
#if APP_LOG_ENABLE
  if (status)
  {
    app_mylog_info("max30101 reset eror....., Line:%d\r\n", __LINE__);
  }
#endif
#if 0
  uint8_t cmd_bytes1[] = { 0x40, 0x04, 0x18, 0x80 };                                                        // KX122 Operating mode Added by Jason Chen, 2022.09.26
  status = write_cmd2(&cmd_bytes1[0], ARRAY_SIZE(cmd_bytes1), 0, 0, SS_ENABLE_SENSOR_SLEEP_MS);
#if APP_LOG_ENABLE
  if (status)
  {
    app_mylog_info("Accelerometer standby mode  error, Line:%d\r\n", __LINE__);
  }
#endif
#endif
  return status;

}

int ss_in_bootldr_mode(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_MODE, SS_CMDIDX_MODE };
	uint8_t rxbuf[2] = { 0 };

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
			0, 0,
			&rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	if (status != SS_SUCCESS)
		return -1;

	return (rxbuf[1]);// & SS_MASK_MODE_BOOTLDR);                                 // ?? 2022.09.22
}

#if 0
SS_STATUS self_test(int idx, uint8_t *result, int sleep_ms){
    uint8_t cmd_bytes[] = { SS_FAM_R_SELFTEST, (uint8_t)idx };
    uint8_t rxbuf[2];
    SS_STATUS ret;

	result[0] = 0xFF;
	ret = read_cmd(cmd_bytes, 2, (uint8_t *)0, 0, rxbuf, ARRAY_SIZE(rxbuf), sleep_ms);
	result[0] = rxbuf[1];
	return ret;
}
#endif

static void irq_handler(uint8_t intNo)
{
  (void)intNo;
	m_irq_received_ = true;
}

/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
void mfioGPIOSetup(void)
{
  cfg_mfio(PIN_OUTPUT);
  mfio_pin_write(1);
  cfg_mfio(PIN_INPUT);

  /* Configure GPIO Clock */
//CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure Button PC4 as input and enable interrupt */
  //GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_MFIO_PORT, SL_EMLIB_GPIO_INIT_MFIO_PIN, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(SL_EMLIB_GPIO_INIT_MFIO_PORT,
                    SL_EMLIB_GPIO_INIT_MFIO_PIN,
                    SL_EMLIB_GPIO_INIT_MFIO_PIN,          // Interrupt Number
                    false,                                // RisingEdge  Disable
                    true,                                 // FallingEdge Enable
                    true                                  // interrupt   Enable
					);

  /* Enable EVEN interrupt to catch button press that changes slew rate */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);

  GPIOINT_CallbackRegister(SL_EMLIB_GPIO_INIT_MFIO_PIN, (GPIOINT_IrqCallbackPtr_t)&irq_handler);
  GPIO_IntEnable(1 << SL_EMLIB_GPIO_INIT_MFIO_PIN);
}

void set_shut_down_exit(void)
{
#if 0
  //reset_pin_input(PIN_OUTPUT);
  sl_bioreset_highlow(1);                                                       // Added by Jason Chen, 2022.09.20
  wait_ms(10);
  sl_bioreset_highlow(0);                                                       // Added by Jason Chen, 2022.09.20
  wait_ms(100);
  sl_bioreset_highlow(1);                                                       // Added by Jason Chen, 2022.09.20
  wait_ms(2000);
  //reset_pin_input(PIN_INPUT);
#else
  cfg_mfio(PIN_OUTPUT);
  sl_bioreset_highlow(0);                                                       // Added by Jason Chen, 2022.09.20
  mfio_pin_write(1);
  wait_ms(SS_RESET_TIME);
  sl_bioreset_highlow(1);                                                       // Added by Jason Chen, 2022.09.20
  wait_ms(SS_STARTUP_TO_MAIN_APP_TIME);
  cfg_mfio(PIN_INPUT);
#endif
}

void ss_init_hwcomm_interface(void)
{
  //SS_STATUS status1;
#if 0
  set_shut_down_exit();
#elif 1
  // Enter application Mode
  //reset_pin_input(PIN_OUTPUT);
  cfg_mfio(PIN_OUTPUT);
  sl_bioreset_highlow(0);                                                       // Added by Jason Chen, 2022.09.20
  mfio_pin_write(1);
  sl_sleeptimer_delay_millisecond(SS_RESET_TIME);
  sl_bioreset_highlow(1);                                                       // Added by Jason Chen, 2022.09.20
  sl_sleeptimer_delay_millisecond(SS_STARTUP_TO_MAIN_APP_TIME);
  cfg_mfio(PIN_INPUT);
#elif 1
  // Enter Bootloader Mode
  //reset_pin_input(PIN_OUTPUT);
  cfg_mfio(PIN_OUTPUT);
  sl_bioreset_highlow(0);                                                       // Added by Jason Chen, 2022.09.20
  mfio_pin_write(0);
  sl_sleeptimer_delay_millisecond(SS_RESET_TIME);
  sl_bioreset_highlow(1);                                                       // Added by Jason Chen, 2022.09.20
  sl_sleeptimer_delay_millisecond(SS_STARTUP_TO_MAIN_APP_TIME);
  mfio_pin_write(1);
  cfg_mfio(PIN_INPUT);
  //reset_pin_input(PIN_INPUT);
#endif
  mfioGPIOSetup();
	return;
}

SS_STATUS set_shut_down_enter(void)
{
  SS_STATUS status;

  ssinterface_log(" ----> entering shut-down backup \r\n ");
#if 1
  uint8_t cmd_bytes0[] = { SS_FAM_W_WRITEREG, 0x03, 0x09, 0x80};                                             // Shutdowm MAX30101 added by Jason Chen, 2022.09.19
  status = write_cmd2(&cmd_bytes0[0], ARRAY_SIZE(cmd_bytes0), 0, 0, SS_ENABLE_SENSOR_SLEEP_MS);
#if APP_LOG_ENABLE
  if (status)
  {
    app_mylog_info("Shut Dowm Command error, Line:%d\r\n", __LINE__);
  }
#endif

  uint8_t cmd_bytes1[] = { SS_FAM_W_WRITEREG, 0x04, 0x18, 0x00 };                                            // Standby KX122 Added by Jason Chen, 2022.09.19
  status = write_cmd2(&cmd_bytes1[0], ARRAY_SIZE(cmd_bytes1), 0, 0, SS_ENABLE_SENSOR_SLEEP_MS);
#if APP_LOG_ENABLE
  if (status)
  {
    app_mylog_info("Shut Dowm Command error, Line:%d\r\n", __LINE__);
  }
#endif
#endif

//uint8_t cmd_bytes[] = { SS_FAM_W_MODE, 0x00, SENSOR_HUB_SHUTDOWM };                                        // Shutdown MAX32664A/B/C/D Added by Jason Chen, 2022.09.19
//if(bio_sensor_type != BPT_SENSOR) cmd_bytes[2] = SENSOR_HUB_RESET;                                         // Reset    MAX32664A/B/C/D Added by Jason Chen, 2022.09.28
//status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, SS_ENABLE_SENSOR_SLEEP_MS);
  status = ss_set_hub_mode(SENSOR_HUB_SHUTDOWM);
#if APP_LOG_ENABLE
  if (status)
  {
    app_mylog_info("Shut Dowm Command error, Line:%d\r\n", __LINE__);
  }
#endif

#if CMD_POWERON_RESET
  max14676_PowerOnReset(POWERON_RESET);
#endif

  return status;
}

/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/

void ss_enable_irq(void)
{
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  //GPIO_IntEnable(1 << SL_EMLIB_GPIO_INIT_MFIO_PIN);
}
void ss_disable_irq(void)
{
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	//GPIO_IntDisable(1 << SL_EMLIB_GPIO_INIT_MFIO_PIN);
}

void ss_mfio_selftest(void){
	ss_disable_irq();
	ss_enable_irq();
}

const char* get_ss_fw_version(void)
{
  uint8_t cmd_bytes[2];
  uint8_t rxbuf[4];

	int bootldr = ss_in_bootldr_mode();
	bootldr = bootldr & SS_MASK_MODE_BOOTLDR;                                 // ?? 2022.09.22
	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_FWVERSION;
	} else {
		return plat_name;
	}

    SS_STATUS status = read_cmd(
             &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
             0, 0,
             &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

    if (status == SS_SUCCESS) {
        snprintf(fw_version, sizeof(fw_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
		pr_info("fw_version:%s\r\n", fw_version);
    }

    return &fw_version[0];
}

const char* get_ss_algo_version(void)
{
    uint8_t cmd_bytes[3];
    uint8_t rxbuf[4];

	int bootldr = ss_in_bootldr_mode();
	bootldr = bootldr & SS_MASK_MODE_BOOTLDR;                                 // ?? 2022.09.22

	if (bootldr > 0) {
		cmd_bytes[0] = SS_FAM_R_BOOTLOADER;
		cmd_bytes[1] = SS_CMDIDX_BOOTFWVERSION;
		cmd_bytes[2] = 0;
	} else if (bootldr == 0) {
		cmd_bytes[0] = SS_FAM_R_IDENTITY;
		cmd_bytes[1] = SS_CMDIDX_ALGOVER;
		cmd_bytes[2] = SS_CMDIDX_AVAILSENSORS;
	} else {
		return plat_name;
	}

    SS_STATUS status = read_cmd(
             &cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
             0, 0,
             &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

    if (status == SS_SUCCESS) {
        snprintf(algo_version, sizeof(algo_version),
            "%d.%d.%d", rxbuf[1], rxbuf[2], rxbuf[3]);
		pr_info("algo_version:%s\r\n", fw_version);
    }

    return &algo_version[0];
}

const char* get_ss_platform_name(void)
{
  uint8_t cmd_bytes[] = { SS_FAM_R_IDENTITY, SS_CMDIDX_PLATTYPE };
  uint8_t rxbuf[2];

  SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
                              0, 0,
                              &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

  if (status == SS_SUCCESS)
  {
      int bootldr = ss_in_bootldr_mode();
      bootldr = bootldr & SS_MASK_MODE_BOOTLDR;                                 // ?? 2022.09.22
      if (rxbuf[1] == SS_PLAT_MAX3263X)
      {
          if (bootldr > 0) plat_name = SS_BOOTLOADER_PLATFORM_MAX3263X;
          else plat_name = SS_PLATFORM_MAX3263X;
      }
      else if (rxbuf[1] == SS_PLAT_MAX32660)
      {
          if (bootldr > 0) plat_name = SS_BOOTLOADER_PLATFORM_MAX32660;
          else plat_name = SS_PLATFORM_MAX32660;
      }
      else {
          plat_name = "SS_ERR_UNKNOWN";                                        // Added by Jason Chen
      }
  }
  else {
      plat_name = "read_cmd: error!";                                        // Added by Jason Chen
  }

    return plat_name;
}

static SS_STATUS write_cmd_small(uint8_t *cmd_bytes, int cmd_bytes_len,
                       uint8_t *data, int data_len,
                       int sleep_ms)
{
  uint8_t write_buf[SS_SMALL_BUF_SIZE];
  memcpy(write_buf, cmd_bytes, cmd_bytes_len);
  memcpy(write_buf + cmd_bytes_len, data, data_len);

	SS_STATUS status = write_cmd(write_buf, cmd_bytes_len + data_len, sleep_ms);
	return status;
}

static SS_STATUS write_cmd_medium(uint8_t *cmd_bytes, int cmd_bytes_len,
                       uint8_t *data, int data_len,
                       int sleep_ms)
{
  uint8_t write_buf[SS_MED_BUF_SIZE];
  memcpy(write_buf, cmd_bytes, cmd_bytes_len);
  memcpy(write_buf + cmd_bytes_len, data, data_len);

	SS_STATUS status = write_cmd(write_buf, cmd_bytes_len + data_len, sleep_ms);
	return status;
}

static SS_STATUS write_cmd_large(uint8_t *cmd_bytes, int cmd_bytes_len,
                       uint8_t *data, int data_len,
                       int sleep_ms)
{
  uint8_t write_buf[SS_LARGE_BUF_SIZE];

  memcpy(write_buf, cmd_bytes, cmd_bytes_len);
  memcpy(write_buf + cmd_bytes_len, data, data_len);

	SS_STATUS status = write_cmd(write_buf, cmd_bytes_len + data_len, sleep_ms);
	return status;
}

SS_STATUS write_cmd2(uint8_t *cmd_bytes, int cmd_bytes_len, uint8_t *data, int data_len, int sleep_ms)
{
  int total_len = data_len + cmd_bytes_len;

  if (total_len <= SS_SMALL_BUF_SIZE) {
      return write_cmd_small(cmd_bytes, cmd_bytes_len, data, data_len, sleep_ms);
  } else if (total_len <= SS_MED_BUF_SIZE) {
      return write_cmd_medium(cmd_bytes, cmd_bytes_len, data, data_len, sleep_ms);
  } else if (total_len <= SS_LARGE_BUF_SIZE) {
      return write_cmd_large(cmd_bytes, cmd_bytes_len, data, data_len, sleep_ms);
  } else {
      assert_msg(true, "Tried to send I2C tx larger than maximum allowed size\n");
      return SS_ERR_DATA_FORMAT;
  }
}

SS_STATUS write_cmd(uint8_t *tx_buf, int tx_len, int sleep_ms)
{
  pr_info("write_cmd: ");
	for (int i = 0; i < tx_len; i++) {
	    pr_info("0x%02X ", tx_buf[i]);
	}
	pr_info("\r\n");

	int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, tx_buf, tx_len, false);

	int retries = 4;
	while (ret != 0 && retries-- > 0) {
	    pr_err("i2c wr retry\r\n");
	    wait_ms(1);
    	ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, tx_buf, tx_len, false);
	}

	if (ret != 0) {
	    pr_err("m_i2cBus->write returned %d\r\n", ret);
      return SS_ERR_UNAVAILABLE;
	}

	wait_ms(sleep_ms);

	uint8_t status_byte;
  ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte, 1);
	bool try_again = (status_byte == SS_ERR_TRY_AGAIN);
	retries = 4;                                                                  // Added by Jason Chen, 2022.05.04
	while ((ret != 0 || try_again) && retries-- > 0) {
	    pr_info("i2c rd retry\r\n");
	    wait_ms(sleep_ms);
    	ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte, 1);
    	try_again = (status_byte == SS_ERR_TRY_AGAIN);
	}

	if (ret != 0 || try_again) {
	    pr_err("m_i2cBus->read returned %d, ss status_byte %d\r\n", ret, status_byte);
	    return SS_ERR_UNAVAILABLE;
	}

	pr_info("status_byte: %d\r\n", status_byte);

	return (SS_STATUS)status_byte;
}

SS_STATUS write_cmd_F(uint8_t *tx_buf, int tx_len, int sleep_ms)
{
  pr_info("write_cmd: ");
  for (int i = 0; i < tx_len; i++) {
      pr_info("0x%02X ", tx_buf[i]);
  }
  pr_info("\r\n");

  int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, tx_buf, tx_len, false);

  int retries = 4;
  while (ret != 0 && retries-- > 0) {
      pr_err("i2c wr retry\r\n");
      wait_ms(1);
      ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, tx_buf, tx_len, false);
  }

  if (ret != 0) {
      pr_err("m_i2cBus->write returned %d\r\n", ret);
      return SS_ERR_UNAVAILABLE;
  }

  wait_ms(sleep_ms);

  uint8_t status_byte[3];
  ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte[0], 3);              // Pipe line accel data into sensor, it will return 3 bytes response, 2022.05.06
  bool try_again = (status_byte[0] == SS_ERR_TRY_AGAIN);
  retries = 4;                                                                  // Added by Jason Chen, 2022.05.04
  while ((ret != 0 || try_again) && retries-- > 0) {
      pr_info("i2c rd retry\r\n");
      wait_ms(sleep_ms);
      ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, &status_byte[0], 3);
      try_again = (status_byte[0] == SS_ERR_TRY_AGAIN);
  }

  if (ret != 0 || try_again) {
      pr_err("m_i2cBus->read returned %d, ss status_byte %d\r\n", ret, status_byte);
      return SS_ERR_UNAVAILABLE;
  }

  pr_info("status_byte: %d\r\n", status_byte);
  if((status_byte[0] == 0)&&(status_byte[1] == 0)&&(status_byte[2] == (tx_len - 2)))
  {
      //app_printfLog("status_byte: %2X%2X%2X\r\n", status_byte[0], status_byte[1], status_byte[2]);
      return (SS_STATUS)status_byte[0];
  }
  else
  {
      app_printfLog("status_byte: %2X%2X%2X\r\n", status_byte[0], status_byte[1], status_byte[2]);
  }

  return (SS_STATUS)(SS_ERR_COMMAND);
}

SS_STATUS read_cmd(uint8_t *cmd_bytes,
                   int cmd_bytes_len,
                   uint8_t *data, int data_len,
                   uint8_t *rxbuf,
                   int rxbuf_sz,
                   int sleep_ms)
{
#if 0
  pr_info("read_cmd: ");                                // by Jason   Begin
  for (int i = 0; i < cmd_bytes_len; i++) {
		pr_info("0x%02X ", cmd_bytes[i]);
	}
	pr_info("\r\n");                                      // by Jason   End
#endif

	int retries = 4;

  int ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, cmd_bytes, cmd_bytes_len, (data_len != 0));
#ifdef SHOW_I2C_DEBUG_MESSAGES
  printf("ret1 : %d\rt\n",ret);
#endif
  if (data_len != 0) {
      ret |= m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, data, data_len, false);
#ifdef SHOW_I2C_DEBUG_MESSAGES
      printf("ret2 : %d\rt\n",ret);
#endif
  }

	while (ret != 0 && retries-- > 0) {

	    pr_err("i2c wr retry\r\n");
	    wait_ms(1);
		  ret = m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, cmd_bytes, cmd_bytes_len, (data_len != 0));
#ifdef SHOW_I2C_DEBUG_MESSAGES
		  printf("ret3 : %d\rt\n",ret);
#endif
		  if (data_len != 0) {
		      ret |= m_i2cBus_write(SS_I2C_8BIT_SLAVE_ADDR, data, data_len, false);
#ifdef SHOW_I2C_DEBUG_MESSAGES
		      printf("ret4 : %d\rt\n",ret);
#endif
		  }
	}

  if (ret != 0) {
      pr_err("m_i2cBus->write returned %d\r\n", ret);
      return SS_ERR_UNAVAILABLE;
  }

  wait_ms(sleep_ms);

  ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, rxbuf, rxbuf_sz);
	bool try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	while ((ret != 0 || try_again) && retries-- > 0) {
	    pr_info("i2c rd retry\r\n");
	    wait_ms(sleep_ms);
	    ret = m_i2cBus_read(SS_I2C_8BIT_SLAVE_ADDR, rxbuf, rxbuf_sz);
	    try_again = (rxbuf[0] == SS_ERR_TRY_AGAIN);
	}

	if (ret != 0 || try_again) {
	    pr_err("m_i2cBus->read returned %d, ss status_byte %d\r\n", ret, rxbuf[0]);
      return SS_ERR_UNAVAILABLE;
  }

	pr_info("status_byte: %d\r\n", rxbuf[0]);
	pr_info("data:");                                            
	for (int i = 1; i < rxbuf_sz; i++) {
	    pr_info("0x%02X ", rxbuf[i]);
	}

	pr_info("\r\n");

  return (SS_STATUS)rxbuf[0];
}

SS_STATUS get_reg(int idx, uint8_t addr, uint32_t *val)
{
  assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_R_REGATTRIBS, (uint8_t)idx };
	uint8_t rx_reg_attribs[3] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            &rx_reg_attribs[0], ARRAY_SIZE(rx_reg_attribs), SS_DEFAULT_CMD_SLEEP_MS);

	if (status != SS_SUCCESS)
	  return status;

	int reg_width = rx_reg_attribs[1];

	uint8_t cmd_bytes2[] = { SS_FAM_R_READREG, (uint8_t)idx, addr };
	uint8_t rxbuf[5] = {0};

	status = read_cmd(&cmd_bytes2[0], ARRAY_SIZE(cmd_bytes2),
	                  0, 0,
	                  &rxbuf[0], reg_width + 1, SS_DEFAULT_CMD_SLEEP_MS);

	if (status == SS_SUCCESS) {
	    *val = 0;
	    for (int i = 0; i < reg_width; i++) {
	        *val = (*val << 8) | rxbuf[i + 1];
	    }
	}

	return status;
}

SS_STATUS set_reg(int idx, uint8_t addr, uint32_t val, int byte_size)
{
  assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_W_WRITEREG, (uint8_t)idx, addr };
	uint8_t data_bytes[4];
	for (int i = 0; i < byte_size; i++) {
	    data_bytes[i] = (val >> (8 * (byte_size - 1)) & 0xFF);
	}

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                              &data_bytes[0], byte_size, SS_ENABLE_SENSOR_SLEEP_MS);
	return status;
}

SS_STATUS dump_reg(int idx, addr_val_pair* reg_vals, int reg_vals_sz, int* num_regs)
{
  assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_R_REGATTRIBS, (uint8_t)idx };
	uint8_t rx_reg_attribs[3] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            &rx_reg_attribs[0], ARRAY_SIZE(rx_reg_attribs), SS_DEFAULT_CMD_SLEEP_MS);

	if (status != SS_SUCCESS)
	  return status;

	int reg_width = rx_reg_attribs[1];
	*num_regs = rx_reg_attribs[2];
	assert_msg((*num_regs <= reg_vals_sz), "Need to increase reg_vals array to hold all dump_reg data");
	assert_msg(((size_t)reg_width <= sizeof(uint32_t)), "IC returned register values greater than 4 bytes in width");

	int dump_reg_sz = (*num_regs) * (reg_width + 1) + 1; //+1 to reg_width for address, +1 for status byte

	uint8_t rxbuf[512];
	assert_msg(((size_t)dump_reg_sz <= sizeof(rxbuf)), "Need to increase buffer size to receive dump_reg data");

	cmd_bytes[0] = SS_FAM_R_DUMPREG;
	status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                  0, 0,
	                  &rxbuf[0], dump_reg_sz, SS_DUMP_REG_SLEEP_MS);

	if (status != SS_SUCCESS)
	  return status;

	//rxbuf format is [status][addr0](reg_width x [val0])[addr1](reg_width x [val1])...
	for (int reg = 0; reg < *num_regs; reg++) {
	    reg_vals[reg].addr = rxbuf[(reg * (reg_width + 1)) + 1];
	    uint32_t *val = &(reg_vals[reg].val);
	    *val = 0;
	    for (int byte = 0; byte < reg_width; byte++) {
	        *val = (*val << 8) | rxbuf[(reg * (reg_width + 1)) + byte + 2];
	    }
	}

	return SS_SUCCESS;
}

SS_STATUS enable_sensor(int idx, int mode, ss_data_req *data_req)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");
	assert_msg((mode <= SS_MAX_SUPPORTED_MODE_NUM), "mode must be < SS_MAX_SUPPORTED_MODE_NUM, or update code to handle variable length mode values");
	assert_msg((mode != 0), "Tried to enable sensor to mode 0, but mode 0 is disable");

	uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, (uint8_t)mode };

//SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, 5 * SS_ENABLE_SENSOR_SLEEP_MS);
  SS_STATUS status = write_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 100);                                //modified by Jason Chen from 100ms to 110ms, 2022.05.03
	if (status == SS_SUCCESS) {
	    sensor_enabled_mode[idx] = mode;
	    sensor_data_reqs[idx] = data_req;
	}
#if SSINTERFACE_LOG_ENABLE
	else{
	    ssinterface_log("Sensor Enable Error:%s, SSI:line:%d\r\n", (idx==3)?"MAX30101":"ACCEL", __LINE__);
	}
#endif
	return status;
}

SS_STATUS enable_sensor_A(int idx, ss_data_req *data_req)
{
  assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");
  //assert_msg((mode <= SS_MAX_SUPPORTED_MODE_NUM), "mode must be < SS_MAX_SUPPORTED_MODE_NUM, or update code to handle variable length mode values");
  //assert_msg((mode != 0), "Tried to enable sensor to mode 0, but mode 0 is disable");

#if ENABLE_HOSTSIDE_ACCEL
  uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, 1, 1};                                 //Enable host-side accelerometer (extern accelerometer LSM6DSOC), 2022.05.05
#else
  uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, 1, 0};                                 //Enable host-side accelerometer (extern accelerometer LSM6DSOC), 2022.09.23
#endif

//SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, 5 * SS_ENABLE_SENSOR_SLEEP_MS);
  SS_STATUS status = write_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 100);                                    //modified by Jason Chen from 100ms to 110ms, 2022.05.03
  if (status == SS_SUCCESS) {
      sensor_enabled_mode[idx] = 1;
      sensor_data_reqs[idx]    = data_req;
  }
#if SSINTERFACE_LOG_ENABLE
  else{
      ssinterface_log("Sensor Enable Error:%s, SSI:line:%d\r\n", (idx==3)?"MAX30101":"ACCEL", __LINE__);
  }
#endif
  return status;
}

SS_STATUS disable_sensor(int idx)
{
  assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");
  uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, 0 };

  SS_STATUS status = write_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 2 * SS_ENABLE_SENSOR_SLEEP_MS);                                // Modified by Jason
	if (status == SS_SUCCESS) {
	    sensor_enabled_mode[idx] = 0;
	    sensor_data_reqs[idx] = 0;
	}
#if (SSINTERFACE_LOG_ENABLE&&APP_LOG_ERROR_ENABLE)
	else{
	    app_log_error("Sensor Disable Error:%s\r\n", (idx==SS_SENSORIDX_MAX30101)?"MAX30101":"ACCEL");
	}
#endif
	return status;
}

SS_STATUS disable_sensor_A(void)
{
  //assert_msg((idx <= SS_MAX_SUPPORTED_SENSOR_NUM), "idx must be < SS_MAX_SUPPORTED_SENSOR_NUM, or update code to handle variable length idx values");
  //uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)idx, 0, 1};                                                                 //Disable host-side accelerometer (extern accelerometer LSM6DSOC), 2022.05.05
#if (ENABLE_HOSTSIDE_ACCEL)
  uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)0x04, 0, 1 };
#else
  uint8_t cmd_bytes[] = { SS_FAM_W_SENSORMODE, (uint8_t)0x04, 0, 0 };
#endif

  SS_STATUS status = write_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 2 * SS_ENABLE_SENSOR_SLEEP_MS);                                // Modified by Jason
  if (status == SS_SUCCESS) {
      sensor_enabled_mode[0x04] = 0;
      sensor_data_reqs[0x04] = 0;
  }
#if (SSINTERFACE_LOG_ENABLE&&APP_LOG_ERROR_ENABLE)
  else{
      app_log_error("Sensor Disable Error:ACCEL\r\n");
  }
#endif
  return status;
}

SS_STATUS enable_algo(int idx, int mode, ss_data_req *data_req)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_ALGO_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_NUM, or update code to handle variable length idx values");
	assert_msg((mode <= SS_MAX_SUPPORTED_MODE_NUM), "mode must be < SS_MAX_SUPPORTED_MODE_NUM, or update code to handle variable length mode values");
	assert_msg((mode != 0), "Tried to enable algo to mode 0, but mode 0 is disable");

	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOMODE, (uint8_t)idx, (uint8_t)mode };

	SS_STATUS status = write_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 25 * SS_ENABLE_SENSOR_SLEEP_MS);
	if (status == SS_SUCCESS) {
		algo_enabled_mode[idx] = mode;
		algo_data_reqs[idx] = data_req;
	}
#if (SSINTERFACE_LOG_ENABLE&&APP_LOG_ERROR_ENABLE)
	else{
	    app_log_error("Algo Enable Error:%s\r\n", (idx==SS_ALGOIDX_BPT)?"BPT":"Other");
	}
#endif
	return status;
}

SS_STATUS disable_algo(int idx)
{
	assert_msg((idx <= SS_MAX_SUPPORTED_ALGO_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_NUM, or update code to handle variable length idx values");
	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOMODE, (uint8_t)idx, 0 };

	SS_STATUS status = write_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), SS_ENABLE_SENSOR_SLEEP_MS);
	if (status == SS_SUCCESS) {
	    algo_enabled_mode[idx] = 0;
	    algo_data_reqs[idx] = 0;
	}
#if (SSINTERFACE_LOG_ENABLE&&APP_LOG_ERROR_ENABLE)
	else{
	    app_log_error("Algo Disable Error:%s\r\n", (idx==SS_ALGOIDX_BPT)?"BPT":"Other");
	}
#endif
	return status;
}

SS_STATUS set_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz)
{
	assert_msg((algo_idx <= SS_MAX_SUPPORTED_ALGO_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_NUM, or update code to handle variable length idx values");
	assert_msg((cfg_idx <= SS_MAX_SUPPORTED_ALGO_CFG_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_CFG_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_W_ALGOCONFIG, (uint8_t)algo_idx, (uint8_t)cfg_idx };
	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), cfg, cfg_sz, 30);

	return status;
}

SS_STATUS get_algo_cfg(int algo_idx, int cfg_idx, uint8_t *cfg, int cfg_sz)
{
	assert_msg((algo_idx <= SS_MAX_SUPPORTED_ALGO_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_NUM, or update code to handle variable length idx values");
	assert_msg((cfg_idx <= SS_MAX_SUPPORTED_ALGO_CFG_NUM), "idx must be < SS_MAX_SUPPORTED_ALGO_CFG_NUM, or update code to handle variable length idx values");

	uint8_t cmd_bytes[] = { SS_FAM_R_ALGOCONFIG, (uint8_t)algo_idx, (uint8_t)cfg_idx };
	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, cfg, cfg_sz, SS_DEFAULT_CMD_SLEEP_MS);

	return status;
}

SS_STATUS set_data_type(int data_type1, bool sc_en1)
{
	assert_msg((data_type1 >= 0) && (data_type1 <= 3), "Invalid value for data_type");
	uint8_t cmd_bytes[] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_OUTPUTMODE };
	uint8_t data_bytes[] = { (uint8_t)((sc_en1 ? SS_MASK_OUTPUTMODE_SC_EN : 0) |
							((data_type1 << SS_SHIFT_OUTPUTMODE_DATATYPE) & SS_MASK_OUTPUTMODE_DATATYPE)) };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), &data_bytes[0], ARRAY_SIZE(data_bytes), SS_DEFAULT_CMD_SLEEP_MS);                  // CMD: 10 0 3
	data_type = data_type1;
	sc_en     = sc_en1;

	return status;
}


SS_STATUS get_data_type(int *data_type1, bool *sc_en1)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_COMMCHAN, SS_CMDIDX_OUTPUTMODE };
	uint8_t rxbuf[2]    = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	if (status == SS_SUCCESS) {
	    *data_type1 = (rxbuf[1] & SS_MASK_OUTPUTMODE_DATATYPE) >> SS_SHIFT_OUTPUTMODE_DATATYPE;
	    *sc_en1     = (bool)((rxbuf[1] & SS_MASK_OUTPUTMODE_SC_EN) >> SS_SHIFT_OUTPUTMODE_SC_EN);
	}

	return status;
}

SS_STATUS set_fifo_thresh(int thresh)
{
	assert_msg((thresh > 0 && thresh <= 255), "Invalid value for fifo a full threshold");
	uint8_t cmd_bytes[]  = { SS_FAM_W_COMMCHAN, SS_CMDIDX_FIFOAFULL };
	uint8_t data_bytes[] = { (uint8_t)thresh };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                              &data_bytes[0], ARRAY_SIZE(data_bytes), SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

SS_STATUS get_fifo_thresh(int *thresh)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_COMMCHAN, SS_CMDIDX_FIFOAFULL };
	uint8_t rxbuf[2]    = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

	if (status == SS_SUCCESS) {
	    *thresh = rxbuf[1];
	}

	return status;
}

SS_STATUS ss_comm_check(void)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_IDENTITY, SS_CMDIDX_PLATTYPE };
	uint8_t rxbuf[2];

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);

	int tries = 4;
	while (status == SS_ERR_TRY_AGAIN && tries--) {
	    wait_ms(1000);
	    status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                      0, 0,
	                      &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	}

	return status;
}

static void fifo_sample_size(int data_type1, int *sample_size1)
{
  *sample_size1 = 0;

	if (data_type1 == SS_DATATYPE_RAW || data_type1 == SS_DATATYPE_BOTH) {
	    for (int i = 0; i < SS_MAX_SUPPORTED_SENSOR_NUM; i++) {
	        if (sensor_enabled_mode[i])
	        {
	            assert_msg(sensor_data_reqs[i], "no ss_data_req found for enabled sensor");
	            *sample_size1 += sensor_data_reqs[i]->data_size;
	        }
	    }
	}

	if (data_type1 == SS_DATATYPE_ALGO || data_type1 == SS_DATATYPE_BOTH) {
	    for (int i = 0; i < SS_MAX_SUPPORTED_ALGO_NUM; i++) {
	        if(algo_enabled_mode[i])
	        {
	            assert_msg(algo_data_reqs[i], "no ss_data_req found for enabled algo");
	            *sample_size1 += algo_data_reqs[i]->data_size;
	        }
	    }
	}
}

SS_STATUS num_avail_samples(int *num_samples)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_OUTPUTFIFO, SS_CMDIDX_OUT_NUMSAMPLES };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            &rxbuf[0], ARRAY_SIZE(rxbuf), 1);

	if (status == SS_SUCCESS) {
	    *num_samples = rxbuf[1];
	}

	return status;
}

SS_STATUS get_log_len(int *log_len)
{
	uint8_t cmd_bytes[] = { SS_FAM_R_LOG, SS_CMDIDX_R_LOG_LEN };
	uint8_t rxbuf[2] = {0};

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            &rxbuf[0], ARRAY_SIZE(rxbuf), 1);

	if (status == SS_SUCCESS) {
	    *log_len = (rxbuf[1] << 8) | rxbuf[0];
	}

	return status;
}

SS_STATUS read_fifo_data(int num_samples, int sample_size1,
                         uint8_t* databuf, int databuf_sz)
{
	int bytes_to_read = num_samples * sample_size1 + 1; //+1 for status byte
	assert_msg((bytes_to_read <= databuf_sz), "databuf too small");

	uint8_t cmd_bytes[] = { SS_FAM_R_OUTPUTFIFO, SS_CMDIDX_READFIFO };

	pr_info("[reading %d bytes (%d samples)\r\n", bytes_to_read, num_samples);

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            databuf, bytes_to_read, 10);
	return status;
}

SS_STATUS read_ss_log(int num_bytes, uint8_t *log_buf, int log_buf_sz)
{
	int bytes_to_read = num_bytes + 1; //+1 for status byte
	assert_msg((bytes_to_read <= log_buf_sz), "log_buf too small");

	uint8_t cmd_bytes[] = { SS_FAM_R_LOG, SS_CMDIDX_R_LOG_DATA };

	pr_info("[reading %d bytes (%d samples)\r\n", bytes_to_read, bytes_to_read);

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            log_buf, bytes_to_read, 5);
	return status;
}

#if(ENABLE_HOSTSIDE_ACCEL)
void pipe_accel_FIFO(void)
{
  accel_data_TT accel_data;
  uint8_t cmd_bytesA[] = {SS_FAM_W_INPUTFIFO, SS_CMDIDN_WRITEFIFO,
                          0x84, 0xFE, 0x1C, 0x00, 0x55, 0xFC, 0x85, 0xFE, 0x1B, 0x00, 0x57, 0xFC,
                          0x84, 0xFE, 0x1A, 0x00, 0x56, 0xFC, 0x85, 0xFE, 0x19, 0x00, 0x54, 0xFC,
                          0x85, 0xFE, 0x19, 0x00, 0x54, 0xFC, 0x85, 0xFE, 0x19, 0x00, 0x54, 0xFC,
                          0x85, 0xFE, 0x19, 0x00, 0x54, 0xFC, 0x85, 0xFE, 0x19, 0x00, 0x54, 0xFC,
                          0x87, 0xFE, 0x1B, 0x00, 0x57, 0xFC};

  get_accel_data(&accel_data);
  memcpy(&cmd_bytesA[2],accel_data.accelByte, 6);memcpy(&cmd_bytesA[8],accel_data.accelByte, 6);

  //get_accel_data(&accel_data);
  memcpy(&cmd_bytesA[14],accel_data.accelByte, 6);memcpy(&cmd_bytesA[20],accel_data.accelByte, 6);

  get_accel_data(&accel_data);
  memcpy(&cmd_bytesA[26],accel_data.accelByte, 6);memcpy(&cmd_bytesA[32],accel_data.accelByte, 6);

  //get_accel_data(&accel_data);
  memcpy(&cmd_bytesA[38],accel_data.accelByte, 6);memcpy(&cmd_bytesA[44],accel_data.accelByte, 6);

  get_accel_data(&accel_data);
  memcpy(&cmd_bytesA[50],accel_data.accelByte, 6);

#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
  SS_STATUS status1 =
#endif
      write_cmd_F(&cmd_bytesA[0], ARRAY_SIZE(cmd_bytesA), 125 );                                //Added by Jason Chen, 2022.05.04
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
  if (status1 != SS_SUCCESS) {
      app_log_error("Couldn't write accel data into FIFO....\r\n");
  }
#endif
}
#endif

static uint8_t databuf[1024];
void ss_execute_once(void){

  if(m_irq_received_ == false) return;

#if DEBUG_LEVEL&APP_LOG_ERROR_ENABLE
	uint8_t sample_count;
#endif

	m_irq_received_     = false;
	//LED_RED(16);                                                             // Red led flashing when sensor interrupt happens, 2022.05.09
	//led_flash_timer = 20;

#if(ENABLE_HOSTSIDE_ACCEL)
	if(data_report_mode == 3/*read_ppg_9*/) pipe_accel_FIFO();                    // read_ppg_9 = 3, 2022.05.12
#endif

	uint8_t cmd_bytes[] = { SS_FAM_R_STATUS, SS_CMDIDX_STATUS };
	uint8_t rxbuf[2]    = {0};

	ss_disable_irq();

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            &rxbuf[0], ARRAY_SIZE(rxbuf), SS_DEFAULT_CMD_SLEEP_MS);
	if (status != SS_SUCCESS) {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
	    app_log_error("Couldn't read status byte of SmartSensor!");
#endif
	    ss_enable_irq();
	    ssinterface_log("SSI:%d\r\n", __LINE__);

	    return;
	}
#if (SSINTERFACE_LOG_ENABLE&&APP_LOG_ERROR_ENABLE)
	if (rxbuf[1] & SS_MASK_STATUS_ERR) {
	    app_log_error("SmartSensor status error: %d\r\n", rxbuf[1] & SS_MASK_STATUS_ERR);
	}
	if (rxbuf[1] & SS_MASK_STATUS_FIFO_OUT_OVR) {
	    app_log_error("SmartSensor Output FIFO overflow!\r\n");
	}
	if (rxbuf[1] & SS_MASK_STATUS_FIFO_IN_OVR) {
	    app_log_error("SmartSensor Input FIFO overflow!\r\n");
	}

	if (rxbuf[1] & SS_MASK_STATUS_LOG_OVR) {
	    app_log_error("SmartSensor log overflow!\r\n");
	}
#endif
	if (rxbuf[1] & SS_MASK_STATUS_LOG_RDY) {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
	    app_log_error("SmartSensor Log ready");
#endif
		  unsigned int log_len;
		  status = get_log_len((int*)&log_len);
		  if (status != SS_SUCCESS)
		  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
		      app_log_error("Couldn't read log lenght\r\n");
#endif
			    ss_enable_irq();
          return;
		  }
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
		  app_log_error("log_len: %d\n", log_len);
#endif
		  assert_msg((log_len <= sizeof(databuf)), "log size in SS longer than buffer.\n");
		  status = read_ss_log(log_len, &databuf[0], sizeof(databuf));
		  if (status != SS_SUCCESS)
		  {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
		      app_log_error("Couldn't read from SmartSensor Log\r\n");
#endif
			    ss_enable_irq();
			    return;
		  }

		  databuf[log_len] = 0;
		  ssinterface_log("\r\n--->%s", (char *)databuf);
	}

	if (rxbuf[1] & SS_MASK_STATUS_DATA_RDY) {

	    int num_samples = 1;
	    status = num_avail_samples(&num_samples);
	    if (status != SS_SUCCESS)
	    {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
	        app_log_error("Couldn't read number of available samples in SmartSensor Output FIFO\r\n");
#endif
	        ss_enable_irq();
	        return;
	    }

	    int sample_size2;
	    fifo_sample_size(data_type, &sample_size2);

	    int bytes_to_read = num_samples * sample_size2 + 1; //+1 for status byte
	    if ((uint32_t)bytes_to_read > sizeof(databuf)) {
	        //Reduce number of samples to read to fit in buffer
	        num_samples = (sizeof(databuf) - 1) / sample_size2;
	    }

	    //wait_ms(5);

	    //printf("nbytes to read  = %d \r\n", bytes_to_read);
	    pr_info("Num of sample = %d, size = %d SSI:%d\r\n", num_samples, sample_size2, __LINE__);
	    status = read_fifo_data(num_samples, sample_size2, &databuf[0], sizeof(databuf));
	    if (status != SS_SUCCESS)
	    {
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
	        app_log_error("Couldn't read from SmartSensor Output FIFO");
#endif
	        ss_enable_irq();
	        return;
	    }

	    //Skip status byte
	    uint8_t *data_ptr = &databuf[1];

	    int i = 0;
	    for (i = 0; i < num_samples; i++) {
	        if (sc_en) {
#if DEBUG_LEVEL&APP_LOG_ERROR_ENABLE
	            sample_count = *data_ptr++;
#if APP_LOG_ERROR_ENABLE                                                        //Added by Jason Chen, 2022.1102
	            app_log_error("Received sample #%d", sample_count);
#endif
#endif
	        }

	        //Chop up data and send to modules with enabled sensors
	        if (data_type == SS_DATATYPE_RAW || data_type == SS_DATATYPE_BOTH) {
	            for (int i = 0; i < SS_MAX_SUPPORTED_SENSOR_NUM; i++)
	            {
	                if (sensor_enabled_mode[i])
	                {
	                    assert_msg(sensor_data_reqs[i], "no ss_data_req found for enabled sensor");
	                    sensor_data_reqs[i]->rx_data_parser(data_ptr);
	                    data_ptr += sensor_data_reqs[i]->data_size;
	                }
	            }
	        }
	        if (data_type == SS_DATATYPE_ALGO || data_type == SS_DATATYPE_BOTH) {
	            for (int i = 0; i < SS_MAX_SUPPORTED_ALGO_NUM; i++)
	            {
	                if (algo_enabled_mode[i]) {
	                    assert_msg(algo_data_reqs[i], 	"no ss_data_req found for enabled algo");
	                    algo_data_reqs[i]->rx_data_parser(data_ptr);
	                    data_ptr += algo_data_reqs[i]->data_size;
	                }
	            }
	        }
	    }
	}
	else
  {
  }
	ss_enable_irq();
}

void ss_irq_handler_selftest(void){
  mfio_int_happened = true;
}

bool ss_reset_mfio_irq(void){
  bool ret = mfio_int_happened;
	mfio_int_happened = false;
	ss_disable_irq();
	ss_enable_irq();
	return ret;
}

#if 1
SS_STATUS max32664_soft_reset(void)
{
  SS_STATUS status;
  uint8_t cmd_bytes[] = { SS_FAM_W_MODE, 0x00, 0x02 };                                                     // MAX32664 soft reset Added by Jason Chen, 2022.09.22
  status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes), 0, 0, SS_ENABLE_SENSOR_SLEEP_MS);
#if APP_LOG_ENABLE
  if (status)
  {
    app_mylog_info("max32664 soft reset eeror....., Line:%d\r\n", __LINE__);
  }
#endif
  return status;
}
#endif

SS_STATUS get_dhparams( uint8_t *response, int response_sz ){

  uint8_t cmd_bytes[] = { SH_FAM_R_DHPARAMS , (uint8_t) 0x00 };

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

SS_STATUS set_dhlocalpublic(  uint8_t *publicKey , int public_sz ){

	uint8_t cmd_bytes[] = { SH_FAM_W_DHPUBLICKEY, (uint8_t) 0x00 };
	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                              publicKey, public_sz, SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

SS_STATUS get_dhremotepublic( uint8_t *response, int response_sz ){

	uint8_t cmd_bytes[] = { SH_FAM_R_DHPUBLICKEY , (uint8_t) 0x00 };

	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

SS_STATUS get_authentication( uint8_t *response, int response_sz )
{

//	const int auth_cfg_sz = 32; // fixed to 32 bytes
	uint8_t cmd_bytes[] = { SH_FAM_R_SHADIGEST , (uint8_t) 0x00 };
	SS_STATUS status = read_cmd(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                            0, 0,
	                            response, response_sz, SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}

SS_STATUS set_i2c_addr(uint8_t addr)
{
  uint8_t cmd_bytes[] = { SS_FAM_W_COMMCHAN, SS_CMDIDX_I2C_ADDR };
	uint8_t data_bytes[] = { (uint8_t)addr };

	SS_STATUS status = write_cmd2(&cmd_bytes[0], ARRAY_SIZE(cmd_bytes),
	                              &data_bytes[0], ARRAY_SIZE(data_bytes), SS_DEFAULT_CMD_SLEEP_MS);
	return status;
}
