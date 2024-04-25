/***************************************************************************//**
 * @file
 * @brief mbedTLS AES example functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_AESCRYPT_H
#define APP_AESCRYPT_H

#include "mbedtls/aes.h"

#define CBC_AES_ENCRYPT  MBEDTLS_AES_ENCRYPT   /**< AES encryption. */
#define CBC_AES_DECRYPT  MBEDTLS_AES_DECRYPT   /**< AES decryption. */

//extern uint8_t key256[32];
extern uint8_t IV16[16];
extern uint8_t plain_data16[16];
extern uint8_t cipherData16[16];

void bin2hextext(char* hexstr, uint8_t* binbuf, unsigned int binbuflen);
void print_hexstring(uint8_t* char_auth, int len);

/***************************************************************************//**
 * Initialize AESCRYPT example
 ******************************************************************************/
void app_aescrypt_init(void);

/***************************************************************************//**
 * AESCRYPT process function
 ******************************************************************************/
int generate_random_key(uint8_t *Se_Key, size_t len);
void aes_crypt_cbc(uint8_t *securityKey, uint8_t *IV, uint8_t *input, uint8_t *cryptOutput, int mode);

#endif // APP_AESCRYPT_H
