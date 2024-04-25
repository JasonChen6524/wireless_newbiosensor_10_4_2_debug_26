/***************************************************************************//**
 * @file
 * @brief mbedTLS AES examples functions
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app.h"
#include "app_aescrypt.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
//#include "mbedtls/aes.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
#define AES_BLOCK_SIZE                (16)
//#define IV_SIZE                     (16)

/*******************************************************************************
 ****************************  Local variables  ********************************
 ******************************************************************************/
//uint8_t key256[32];//     = {0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81,0x1f,0x35,0x2c,0x07,0x3b,0x61,0x08,0xd7,0x2d,0x98,0x10,0xa3,0x09,0x14,0xdf,0xf4};
uint8_t IV16[16]            = {0xF5, 0x8C, 0x4C, 0x04, 0xD6, 0xE5, 0xF1, 0xBA, 0x77, 0x9E, 0xAB, 0xFB, 0x5F, 0x7B, 0xFB, 0xD6};
uint8_t plain_data16[16];// = {0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51};
uint8_t cipherData16[16];// = {0x9c, 0xfc, 0x4e, 0x96, 0x7e, 0xdb, 0x80, 0x8d, 0x67, 0x9f, 0x77, 0x7b, 0xc6, 0x70, 0x2c, 0x7d};

/*******************************************************************************
 ****************************  Local functions  ********************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief Convert ascii hexadecimal text into binary
 ******************************************************************************/
int hextext2bin(uint8_t *binbuf, unsigned int binbuflen, const char *hexstr)
{
  uint32_t ret = 0;
  int      i;
  uint8_t  tmp;
  uint8_t  val;

  while (ret < binbuflen) {
    val = 0;
    for (i = 1; i >= 0; i--) {
      tmp = *(hexstr++);
      // Skip spaces
      while (tmp == ' ') {
        tmp = *(hexstr++);
      }
      // Reached end of string?
      if (!tmp) {
        goto done;
      }

      if (tmp > '9') {
        // Ensure uppercase hex
        tmp &= ~0x20;

        val |= ((tmp - 'A') + 10) << (4 * i);
      } else {
        val |= (tmp - '0') << (4 * i);
      }
    }
    *(binbuf++) = val;
    ret++;
  }
  done:
  return ret;
}

/***************************************************************************//**
 * @brief Convert binary data to ascii hexadecimal text string
 ******************************************************************************/
void bin2hextext(char* hexstr, uint8_t* binbuf, unsigned int binbuflen)
{
  uint32_t i;
  uint8_t nibble;

  for (i = 0; i < binbuflen; i++) {
    nibble = (binbuf[i] >> 4) & 0xF;
    *hexstr++ = nibble > 9 ? nibble - 10 + 'A' : nibble + '0';
    nibble = (binbuf[i] >> 0) & 0xF;
    *hexstr++ = nibble > 9 ? nibble - 10 + 'A' : nibble + '0';
  }
  // Null terminate at end of string.
  *hexstr = 0;
}

/***************************************************************************//**
 * @brief  Initialize application
 ******************************************************************************/
void app_aescrypt_init(void)
{
  return;
}

void print_hexstring(uint8_t* char_auth, int len)
{
  //mbedtls_log("    aes%d :  ", len);
  for(int i = 0; i < len; i++)
  {
      if(char_auth[i] == 0)
      {
          mbedtls_log("00");
      }
      else if((char_auth[i] & 0xF0) == 0)
      {
          mbedtls_log("0%X", char_auth[i] & 0x0F);
      }
      else if((char_auth[i] & 0x0F) == 0)
      {
          mbedtls_log("%X%d", (char_auth[i] >> 4) & 0x0F, 0);
      }
      else
      {
          mbedtls_log("%2X", char_auth[i]);
      }
  }
}

int generate_random_key(uint8_t *Se_Key, size_t len)
{
  mbedtls_ctr_drbg_context ctr_drbg;
  mbedtls_entropy_context entropy;
  char *pers = "aes generate random number";
  int ret;

  //printf("The personalization string needs to be unique to your application to add randomness to your random sources.\r\n");
  //printf("Creating the AES key\r\n");
  //printf("You need to initialize the entropy pool and the random source and extract data for your key. In this case we generate 32 bytes (256 bits) of random data.\r\n");

  mbedtls_entropy_init( &entropy );
  mbedtls_ctr_drbg_init( &ctr_drbg );

  ret = mbedtls_ctr_drbg_seed( &ctr_drbg,
                               mbedtls_entropy_func,
                               &entropy,
                               (unsigned char *) pers,
                               strlen( pers ) );
  if(ret != 0)
  {
      mbedtls_log( " failed\n ! mbedtls_ctr_drbg_init returned -0x%04x\n", -ret );
      return ret;
  }


  ret = mbedtls_ctr_drbg_random( &ctr_drbg, Se_Key, len );
  if(ret !=0 )
  {
      mbedtls_log( " failed\n ! mbedtls_ctr_drbg_random returned -0x%04x\n", -ret );
      return ret;
  }

  mbedtls_entropy_free(&entropy);
  mbedtls_ctr_drbg_free(&ctr_drbg);
  return 0;
}

void aes_crypt_cbc(uint8_t *securityKey,
                   uint8_t *IV,
                   uint8_t *input,
                   uint8_t *cryptOutput,
                   int mode)
{
  //MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH
  mbedtls_aes_context aes_ctx;
  mbedtls_aes_init(&aes_ctx);

  mbedtls_aes_setkey_enc(&aes_ctx, securityKey, 256);

  int ret = mbedtls_aes_crypt_cbc(&aes_ctx,
                                  mode,                                         // MBEDTLS_AES_ENCRYPT, MBEDTLS_AES_DECRYPT
                                  AES_BLOCK_SIZE,
                                  IV,
                                  input,
                                  cryptOutput);
#if 0//AESCRYPT_LOG_ENABLE
  mbedtls_log("SecurityKey%d: ", aes_ctx.keybits);
  print_hexstring(aes_ctx.key, aes_ctx.keybits/8);mbedtls_log("\r\n");
  mbedtls_log("plain_data16  : ");
  print_hexstring(input, 16);                     mbedtls_log("\r\n");
  mbedtls_log("CBC encrypt%d: ", aes_ctx.keybits);
  print_hexstring(cryptOutput, AES_BLOCK_SIZE);
#endif
  if(ret == MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH)
  {
    mbedtls_log("-->MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH");
  }
  //mbedtls_log("\r\n");

  mbedtls_aes_free(&aes_ctx);
}

