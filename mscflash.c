/******************************************************************************
 * @file
 * @brief V3 PROCESSOR FLASH routines
 * @author Ron Graczyk
 * @version 
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2020 Artaflex
 *******************************************************************************
 *  05/1/2020
 *
 ******************************************************************************/
#include "app.h"
#include "mscflash.h"
#include "em_msc.h"
 
#define USER_DATA     USERDATA_BASE                            // flash area for user storage
#define USERDATA      ((uint32_t*)USERDATA_BASE)

uint32_t *puserdata  = (uint32_t*) USER_DATA;


const struct cali_data flashrom = MSC_INIT_DEFAULT;

struct cali_data calidata;

static MSC_Status_TypeDef UserData_write(void *data, uint32_t numBytes)
{
  MSC_Status_TypeDef status;

  MSC_Init();
  status = MSC_WriteWord(USERDATA, data, numBytes);
  MSC_Deinit();
  return status;
}

void flash_init(bool wipe)
{
  MSC_Status_TypeDef status;

  struct cali_data *pflashdata;

  pflashdata = (struct cali_data *) puserdata;
	if ((pflashdata->key != MSC_KEY) || wipe )  // init user data page if first time or wipe set
	{
	    status  = MSC_ErasePage(USERDATA);

	    status += UserData_write((uint8_t*)&flashrom, (uint32_t)sizeof(flashrom));
#if MSCFLASH_LOG_ENABLE
	    if(status == mscReturnOk)
	    {
	        mscflash_log("flash_init() .....OK.....\r\n");
	    }
	    else
	    {
	        mscflash_log("flash_init() .....fail...\r\n");
	    }
#endif
	}
	
	memcpy(&calidata, puserdata, sizeof (flashrom));   // copy flash to ram
//mscflash_log("\r\ncalibration date: %d ",      calidata.date);
//mscflash_log("\r\n      heart rate: %d ",      calidata.hr);
//mscflash_log("\r\n            SPO2: %d ",      calidata.spo2);
//mscflash_log("\r\n   blood presure: %d  %d ",  calidata.sys_bp, calidata.dia_bp);
//mscflash_log("\r\n    offset Index: %d ",      calidata.data_cal[0]);
//mscflash_log("\r\n           Index: 0x%2x\r\n",calidata.index);
	mscflash_log("\r\n        fw_major: %x.%x",    calidata.fw_major, calidata.fw_minor);
	mscflash_log("\r\n          serial: ");
#if MSCFLASH_LOG_ENABLE
  for(int i = 0; i < 6; i++)
  {
      if(calidata.serial[i] == 0)
      {
          mscflash_log("00 ");
      }
      else if((calidata.serial[i] & 0xF0) == 0)
      {
          mscflash_log("0%x ", calidata.serial[i] & 0x0F);
      }
      else if((calidata.serial[i] & 0x0F) == 0)
      {
          mscflash_log("%x%d ", (calidata.serial[i] >> 4) & 0x0F, 0);
      }
      else
      {
          mscflash_log("%2X ", calidata.serial[i]);
      }
  }

  mscflash_log("\r\n      auth_value: ");
  for(int i = 0; i < 32; i++)
  {
      if(calidata.auth_value[i] == 0)
      {
          mscflash_log("00 ");
      }
      else if((calidata.auth_value[i] & 0xF0) == 0)
      {
          mscflash_log("0%x ", calidata.auth_value[i] & 0x0F);
      }
      else if((calidata.auth_value[i] & 0x0F) == 0)
      {
          mscflash_log("%x%d ", (calidata.auth_value[i] >> 4) & 0x0F, 0);
      }
      else
      {
          mscflash_log("%2x ", calidata.auth_value[i]);
      }
  }
  mscflash_log("\r\n    auth_allowed: %s", calidata.auth_allowed?"TRUE":"FALSE");
#endif

#if 0
  mscflash_log("\r\n\r\n");
	for(int i = 1; i < 513; i++)
	{
      if(calidata.data_cal[i] == 0)
      {
          mscflash_log("00 ");
      }
      else if((calidata.data_cal[i] & 0xF0) == 0)
      {
          mscflash_log("0%x ", calidata.data_cal[i] & 0x0F);
      }
      else if((calidata.data_cal[i] & 0x0F) == 0)
      {
          mscflash_log("%x%d ", (calidata.data_cal[i] >> 4) & 0x0F, 0);
      }
      else
      {
          mscflash_log("%2x ", calidata.data_cal[i]);
      }

      if(i%32 == 0)
      {
          mscflash_log("\r\n");
      }

	}
#endif


	mscflash_log("\r\n\r\n");
}

void flash_write(void)
{
  MSC_Status_TypeDef status;

  status  = MSC_ErasePage(USERDATA);
  status |= UserData_write(&calidata, (uint32_t)sizeof(calidata));
#if MSCFLASH_LOG_ENABLE
  if(status == mscReturnOk)
  {
      mscflash_log("flash_writet() .....OK.....\r\n");
  }
  else
  {
      mscflash_log("flash_writet() .....fail...\r\n");
  }
#endif
}


