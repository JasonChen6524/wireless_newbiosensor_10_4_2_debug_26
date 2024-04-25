/*
 * DSInterface.h
 *
 *  Created on: May 28, 2021
 *      Author: jason Chen Local
 */
#ifndef DEVICESTUDIO_DSINTERFACE_H_
#define DEVICESTUDIO_DSINTERFACE_H_


//#include "SensorComm.h"

#define COMM_SUCCESS         0
#define COMM_GENERAL_ERROR  -1
#define COMM_AUTH_ERROR     -253
#define COMM_INVALID_PARAM  -254
#define COMM_NOT_RECOGNIZED -255

#define FLASH_ERR_GENERAL   -1
#define FLASH_ERR_CHECKSUM  -2
#define FLASH_ERR_AUTH      -3

//#define DS_MAX_NUM_SENSORCOMMS	8

#define DS_BINARY_PACKET_START_BYTE	0xAA

void DSInterface_init(void);

extern void SensorComm_create(void);

void Build_Command(char ch);

void data_report_execute(void);

//extern bool calibration_success;
//void DSInterface_BuildCommand_itself(void);

/**
* @brief    Set the fw version which DSInterface will replay with for "get_device_info" command
*
* @param[in]    fw_version Firmware version number.
*/
void set_fw_version(const char *fw_version);

/**
* @brief    Set the fw platform which DSInterface will replay with for "get_device_info" command
*
* @param[in]    fw_platform Firmware platform name.
*/
void set_fw_platform(const char* platform);


#endif /* DEVICESTUDIO_DSINTERFACE_H_ */
