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
 * Richmond Hill, ON, L4B 1B4
 *
 * Tel:   (905) 470-0109
 * http:  www.artaflex.com
 *
 ***************************************************************************/
#ifndef _LSM6DSOX_H_
#define _LSM6DSOX_H_

#define LSM6DSOX_ADR_WRITE	0xD4
#define LSM6DSOX_ADR_READ  	0xD5
#define LSM6DSOX_ADR_7BIT   0x4A

#define FUNC_CFG_ACCESS   0x01                              // 00000000
#define PIN_CTRL          0x02                              // 00111111
#define S4S_TPH_L         0x04                              // 00000000
#define S4S_TPH_H         0x05                              // 00000000
#define S4S_RR            0x06                              // 00000000
#define FIFO_CTRL1        0x07                              // 00000000
#define FIFO_CTRL2        0x08                              // 00000000
#define FIFO_CTRL3        0x09                              // 00000000
#define FIFO_CTRL4        0x0A                              // 00000000
#define COUNTER_BDR_REG1  0x0B                              // 00000000
#define COUNTER_BDR_REG2  0x0C                              // 00000000
#define INT1_CTRL         0x0D                              // 00000000
#define INT2_CTRL         0x0E                              // 00000000
#define WHO_AM_I          0x0F                              // 01101100 R (SPI2)
#define CTRL1_XL          0x10                              // 00000000 R (SPI2)
#define CTRL2_G           0x11                              // 00000000 R (SPI2)
#define CTRL3_C           0x12                              // 00000100 R (SPI2)
#define CTRL4_C           0x13                              // 00000000 R (SPI2)
#define CTRL5_C           0x14                              // 00000000 R (SPI2)
#define CTRL6_C           0x15                              // 00000000 R (SPI2)
#define CTRL7_G           0x16                              // 00000000 R (SPI2)
#define CTRL8_XL          0x17                              // 00000000 R (SPI2)
#define CTRL9_XL          0x18                              // 11100000 R (SPI2)
#define CTRL10_C          0x19                              // 00000000 R (SPI2)
#define ALL_INT_SRC       0x1A                              // output
#define WAKE_UP_SRC       0x1B                              // output
#define TAP_SRC           0x1C                              // output
#define D6D_SRC           0x1D                              // output
#define STATUS_REG        0x1E                              // output

#define OUT_TEMP_L        0x20                              // output
#define OUT_TEMP_H        0x21                              // output
#define OUTX_L_G          0x22                              // output
#define OUTX_H_G          0x23                              // output
#define OUTY_L_G          0x24                              // output
#define OUTY_H_G          0x25                              // output
#define OUTZ_L_G          0x26                              // output
#define OUTZ_H_G          0x27                              // output
#define OUTX_L_A          0x28                              // output
#define OUTX_H_A          0x29                              // output
#define OUTY_L_A          0x2A                              // output
#define OUTY_H_A          0x2B                              // output
#define OUTZ_L_A          0x2C                              // output
#define OUTZ_H_A          0x2D                              // output

#define EMB_FUNC_STATUS_MAINPAGE  0x35                      // output
#define FSM_STATUS_A_MAINPAGE     0x36                      // output
#define FSM_STATUS_B_MAINPAGE     0x37                      // output
#define MLC_STATUS_MAINPAGE       0x38                      // output
#define STATUS_MASTER_MAINPAGE    0x39                      // output
#define FIFO_STATUS1              0x3A                      // output
#define FIFO_STATUS2              0x3B                      // output

#define TIMESTAMP0                0x40                      // output R (SPI2)
#define TIMESTAMP1                0x41                      // output R (SPI2)
#define TIMESTAMP2                0x42                      // output R (SPI2)
#define TIMESTAMP3                0x43                      // output R (SPI2)

#define UI_STATUS_REG_OIS         0x49                       // output
#define UI_OUTX_L_G_OIS           0x4A                       // output
#define UI_OUTX_H_G_OIS           0x4B                       // output
#define UI_OUTY_L_G_OIS           0x4C                       // output
#define UI_OUTY_H_G_OIS           0x4D                       // output
#define UI_OUTZ_L_G_OIS           0x4E                       // output
#define UI_OUTZ_H_G_OIS           0x4F                       // output
#define UI_OUTX_L_A_OIS           0x50                       // output
#define UI_OUTX_H_A_OIS           0x51                       // output
#define UI_OUTY_L_A_OIS           0x52                       // output
#define UI_OUTY_H_A_OIS           0x53                       // output
#define UI_OUTZ_L_A_OIS           0x54                       // output
#define UI_OUTZ_H_A_OIS           0x55                       // output
#define TAP_CFG0                  0x56                       // 00000000
#define TAP_CFG1                  0x57                       // 00000000
#define TAP_CFG2                  0x58                       // 00000000
#define TAP_THS_6D                0x59                       // 00000000
#define INT_DUR2                  0x5A                       // 00000000
#define WAKE_UP_THS               0x5B                       // 00000000
#define WAKE_UP_DUR               0x5C                       // 00000000
#define FREE_FALL                 0x5D                       // 00000000
#define MD1_CFG                   0x5E                       // 00000000
#define MD2_CFG                   0x5F                       // 00000000
#define S4S_ST_CMD_CODE           0x60                       // 00000000
#define S4S_DT_REG                0x61                       // 00000000
#define I3C_BUS_AVB RW            0x62                       // 00000000
#define INTERNAL_FREQ_FINE        0x63                       // output

#define UI_INT_OIS        0x6F                               // 00000000
#define UI_CTRL1_OIS      0x70                               // 00000000
#define UI_CTRL2_OIS      0x71                               // 00000000
#define UI_CTRL3_OIS      0x72                               // 00000000
#define X_OFS_USR RW      0x73                               // 00000000
#define Y_OFS_USR RW      0x74                               // 00000000
#define Z_OFS_USR RW      0x75                               // 00000000

#define FIFO_DATA_OUT_TAG 0x78                               // output
#define FIFO_DATA_OUT_X_L 0x79                               // output
#define FIFO_DATA_OUT_X_H 0x7A                               // output
#define FIFO_DATA_OUT_Y_L 0x7B                               // output
#define FIFO_DATA_OUT_Y_H 0x7C                               // output
#define FIFO_DATA_OUT_Z_L 0x7D                               // output
#define FIFO_DATA_OUT_Z_H 0x7E                               // output

#define MODE_SINGLE_TAP                          0
#define MODE_DOUBLE_TAP                          0
#define MODE_WAKE_UP_WAITTIME                    0
#define MODE_WAKE_UP                             0
#define MODE_ACTIVITY_INACTIVITY                 1
#define MODE_6D_ORIENTATION                      0

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} accel_data;

typedef union {
  accel_data accl_data;
  uint8_t accelByte[6];
} accel_data_TT;

uint8_t lsm6dsox_GetID(void);
bool lsm6dsox_init(bool irq_enable);
void acc_clear_interrupt_flag(void);
bool acc_interrupt_flag(void);
void acc_intEnable(bool intEnable);
bool get_accel_data(accel_data_TT *accel_data);

#endif
