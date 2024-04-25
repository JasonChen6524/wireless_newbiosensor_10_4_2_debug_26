/****************************************************************************
*
*  flash.h
*
******************************************************************************/

#ifndef MSCFLASH_H_
#define MSCFLASH_H_

#define BIO_FW_MAJOR 01                                                         // Changed from 00 to 01 by Jason Chen, 2023.11.28
#define BIO_FW_MINOR 44                                                         // Changed from 00 to 01 by Jason Chen, 2024.01.23

#include "sl_sleeptimer.h"
extern struct cali_data calidata;;

extern void flash_init(bool wipe);
void flash_write(void);

#define MSC_KEY 0xDEAD

#define MSC_INIT_DEFAULT                              \
{                                                     \
   MSC_KEY,                          /* KEY */        \
   0x55,                             /*index*/        \
   20220106,                         /*old date*/     \
   {25, 30, 10, 7, 0, 2022, 0, 0,0}, /*date*/         \
   0,                                                 \
   {0},                              /*data_cal*/     \
   {0},                              /*serial*/       \
   {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11,          \
    0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,          \
    0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11,          \
    0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99},         \
   0,                                /*HW Major*/     \
   0,                                /*HW Minjor*/    \
   BIO_FW_MAJOR,                     /*FW Major*/     \
   BIO_FW_MINOR,                     /*FW Minor*/     \
   0                                 /*auth_allowed*/ \
}

struct cali_data               // data to save
{
  uint16_t key;
  uint16_t index;
  uint32_t date;
  sl_sleeptimer_date_t      cli_date;
  sl_sleeptimer_timestamp_t time_stamp;
  uint8_t  data_cal[513];

  uint8_t  serial[6];
  uint8_t  auth_value[32];
  uint8_t  hw_major;
  uint8_t  hw_minor;
  uint8_t  fw_major;
  uint8_t  fw_minor;
  uint8_t  auth_allowed;
//uint8_t  hr;
//uint8_t  sys_bp;
//uint8_t  dia_bp;
//uint8_t  spo2;
};

#define EM_MSC_RUN_FROM_FLASH

#endif /* MSCFLASH_H_ */
