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
 * Modified by Jason Chen, 2022.04.28
 * Tel:   (905) 470-0109
 * http://www.artaflex.com
 *
 ***************************************************************************/
#include "app.h"
#include "MAX14676E.h"
#include "sl_i2cspm.h"
#include "SSInterface.h"                                                        // Added by Jason Chen, 2022.09.19

#define MAXTRIES 20000  // for I2C read and write checking if HW stuck

uint8_t charging_voltage_settings = MAX_VSET_4_20;

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
#if 1
  ret = I2CSPM_Transfer(I2C0, &seq);
  if (ret != i2cTransferDone) {
    return ret;//false;
  }
#else
  uint16_t maxtries = 0;
  ret = I2C_TransferInit(I2C0, &seq);
  while (ret == i2cTransferInProgress)
  {
    ret = I2C_Transfer(I2C0);
    maxtries++;
    if (maxtries> MAXTRIES) break;
  }
#endif
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
#if 1
  ret = I2CSPM_Transfer(I2C0, &seq);
  if (ret != i2cTransferDone) {
    return ret;//false;
  }
#else
  ret = I2C_TransferInit(I2C0, &seq);
  while (ret == i2cTransferInProgress)
  {
    ret = I2C_Transfer(I2C0);
    maxtries++;
    if (maxtries> MAXTRIES) break;
  }
#endif
// Debug code for maxtries testing
   //if (i2ccounts[0]< maxtries) i2ccounts[0] = maxtries;

  return ((int) ret);
}

uint8_t max14676_GetVer(void);

// return true if exists
bool max14676_init(void)
{
  uint8_t var[1];
 
  if(!max14676_GetVer()) return(false);

  var[0]=0x40;
  I2C_WRITE(MAX14676E_ADR50,MAX_LDOCFG,var,1);            // Enable LDO

  var[0] = 0;
  I2C_WRITE(MAX14676E_ADR50,MAX_LED_CFG,var,1);           // 0.6mA per LED step
  
  var[0] = 0x80;
  I2C_WRITE(MAX14676E_ADR50, MAX_PWR_CFG_ADDR, var,1);    // Stay ON
   
  var[0] =0x17;
  I2C_WRITE(MAX14676E_ADR50,MAX_ILIMCNTL_ADDR, var,1);    // Custom Input Current Limit 1000mA

//var[0] = 0x14;                                          // Charge Current Setting 250mA
  var[0] = 0x08;                                          // Charge Current Setting 100mA
//var[0] = 0x04;                                          // Charge Current Setting 75mA
  I2C_WRITE(MAX14676E_ADR50,MAX_CHGCNTLA, var, 1);

  var[0] =  0x6E;
  I2C_WRITE(MAX14676E_ADR50,MAX_PCHGCNTL,var,1);          // Precharge current setting 40mA, Precharge voltage threshold setting 3.00V

  var[0] =  MAX_CDET_INIT; //0x84 ->Charger Auto restart
  I2C_WRITE(MAX14676E_ADR50,MAX_CDETCNTLB,var,1);

  charging_voltage_settings = MAX_VSET_CURR;
  var[0] =  MAX_VSET_CURR;                                // MAX_VSET_INIT; //00 -> Voltage settings
  I2C_WRITE(MAX14676E_ADR50,MAX_CHGVSET,var,1);           // Battery Regulation Threshold 4.20V

  var[0] = 0x90;                                          // Thermistor and charger enabled       Charge Done Threshold Setting 7.5mA
  I2C_WRITE(MAX14676E_ADR50,MAX_CHGCNTLB,var,1);

#if CMD_POWERON_RESET
  max14676_PowerOnReset(POWERON_DEFAULT);                 // CMD Register default value Added by Jason Chen, 2022.10.03
#endif

//var[0] =  MAX_BST_ON;   // 0x49 -> Enable BOOST at +14V
//I2C_WRITE(MAX14676E_ADR50,MAX_BST_CFG,var,1);
  return(true);
}

void LED_RED(uint8_t led_level)
{
  uint8_t var[1];

  led_level &=0x1F;

  if(led_level) led_level |= MAX_LEDN_CFG;
  else led_level = 0;

// MAX14676E does not have Auto inc I2C register address mode, write each register individually
  var[0] = led_level;
  I2C_WRITE(MAX14676E_ADR50,MAX_LED2_RED,var,1);
}

void LED_GRN_init(uint8_t led_level)
{
  uint8_t var[1];

  led_level &=0x1F;

  if(led_level) led_level |= MAX_LEDN_MPC0;
  else led_level = 0;

  var[0] = led_level;
  I2C_WRITE(MAX14676E_ADR50,MAX_LED0_GRN,var,1);
}

void LED_BLUE_init(uint8_t led_level)
{
  uint8_t var[1];

  led_level &=0x1F;

  if(led_level) led_level |= MAX_LEDN_MPC1;
  else led_level = 0;

  var[0] = led_level;
  I2C_WRITE(MAX14676E_ADR50,MAX_LED1_BLU,var,1);
}

//return the percentage 
uint8_t max14676_GetBatteryCharge(void)
{
  uint8_t var[2];
	
  I2C_READ(MAX14676E_ADR6C,MAX_SOC,var,2);
  return (var[0]) ;
}

uint16_t max14676_GetBatteryChargeVoltage(void)
{
  uint16_t data;
  uint8_t var[2];

  I2C_READ(MAX14676E_ADR6C, MAX_VCELL, var,2);

  data = ((uint16_t)var[1]) + ((uint16_t)var[0] <<8);
	
	return data;
}

uint16_t max14676_GetBatteryCRATE(void)
{
  uint16_t data;
  uint8_t  var[2];

  I2C_READ(MAX14676E_ADR6C,MAX_CRATE, var,2);

  data = ((uint16_t)var[1]) + ((uint16_t)var[0] <<8);
	
	return data;
}

uint8_t max14676_GetStatusA(void)
{
  uint8_t var[1];
	
  I2C_READ(MAX14676E_ADR50,MAX_STATUSA,var,1);
  return (var[0]) ;
}

uint8_t max14676_GetStatusB(void)
{
  uint8_t var[1];
	
  I2C_READ(MAX14676E_ADR50,MAX_STATUSB,var,1);
  return (var[0]) ;
}

uint8_t max14676_GetVer(void)
{
  uint8_t result;
  uint8_t var[1];
  
  result =  (uint8_t)I2C_READ(MAX14676E_ADR50,MAX_CHIP_ID,var,1);
  
  if((result == 0 )&&(var[0] == 0x2E)) return (1);
  else return(0);
}

void max14676_ChargerOn(uint8_t charging_bat)
{
  uint8_t var[1];

  var[0] =0x17;
  I2C_WRITE(MAX14676E_ADR50,MAX_ILIMCNTL_ADDR, var,1);              // Custom Input Current Limit 1000mA
   
//var[0] = 0x14;                                                    // Charge Current Setting 250mA
  var[0] = 0x08;                                                    // Charge Current Setting 100mA
  I2C_WRITE(MAX14676E_ADR50,MAX_CHGCNTLA, var, 1);

  var[0] =  0x6E;
  I2C_WRITE(MAX14676E_ADR50,MAX_PCHGCNTL,var,1);                    // Precharge current setting 40mA, Precharge voltage threshold setting 3.00V

  var[0] =  MAX_CDET_INIT; // 0x84 -> Charger Auto restart
  I2C_WRITE(MAX14676E_ADR50,MAX_CDETCNTLB,var,1);

  charging_voltage_settings = charging_bat;//MAX_VSET_4_30;
  var[0] =  charging_bat;//MAX_VSET_4_30;                           // 00 -> Voltage settings                // Battery Regulation Threshold 4.20V
  I2C_WRITE(MAX14676E_ADR50,MAX_CHGVSET,var,1);

  var[0] = 0x90;  // Thermistor and charger enabled                 // Charge Done Threshold Setting 7.5mA
  I2C_WRITE(MAX14676E_ADR50,MAX_CHGCNTLB,var,1);
}

void max14676_Boost(uint8_t state)
{
  uint8_t var[1];

  if (state)
    var[0] =  MAX_BST_ON;                               // 0x49 -> Enable BOOST at +14V
  else
    var[0] =  MAX_BST_OFF;                              // 0x09 -> Disable Boost

  I2C_WRITE(MAX14676E_ADR50,MAX_BST_CFG,var,1);
}

void max14676_poweroff(void)
{
  uint8_t var[1];

  var[0] = 0x05;
	
	for(;;)
	{
      I2C_WRITE(MAX14676E_ADR50,MAX_PWR_CFG_ADDR,var,1);
	}
}

void max14676_PowerOnReset(bool enableR)
{
  uint16_t  var;
  if(enableR) var = 0x5400;
  else        var = 0xFFFF;

  I2C_WRITE(MAX14676E_ADR6C,MAX_POWERON, (uint8_t*)&var,2);
}

//Read MAX30208 temperature sensor
uint16_t Temperature(uint8_t addr)
{
  uint8_t  var[2];
  uint16_t sample;

//See if temperature sensor is present
  I2C_READ(addr, MAX30208_ID,var,1 );
  if (var[0] != MAX30208_ID_VAL) return(MAX30208_ERROR);

  I2C_READ(addr, MAX30208_STATUS,var,1 );

  if (!(var[0] & MAX30208_STATUS_RDY)) // no sample waiting
  {
      var[0] = MAX30208_SETUP_GO;
      I2C_WRITE(addr, MAX30208_SETUP, var, 1);  // initiate a conversion for next temperature read
      return(MAX30208_WAIT);        // RETURN not ready
  }

//GET FIFO level
  I2C_READ(addr, MAX30208_COUNT,var,1 );
  if (var[0] != 0x01)  // a single sample is expected
  {
      var[0] = MAX30208_CFG2_FLUSH;             // Flush FIFO
      I2C_WRITE(addr, MAX30208_CFG2, var,0);
      var[0] = MAX30208_SETUP_GO;
      I2C_WRITE(addr, MAX30208_SETUP, var, 1);  // initiate a conversion for next temperature read
      return(MAX30208_WAIT);        // RETURN not ready
  }

// now actually get sample
  I2C_READ(addr,MAX30208_DATA,var,2);
  sample = (uint16_t)var[1]+((uint16_t)var[0]<<8);
  var[0] = MAX30208_SETUP_GO;
  I2C_WRITE(addr, MAX30208_SETUP, var, 1);  // initiate a conversion for next temperature read
  return(sample);
}

void TemperatureReset(void)
{
  uint8_t var[2];
  //uint8_t status = 0;

  var[0] = MAX30208_RESET;
  I2C_WRITE(MAX30208A_ADR, MAX30208_SYSTEM_CONTROL, var, 1);                    // Reset Temperature sensor to reset status
}
