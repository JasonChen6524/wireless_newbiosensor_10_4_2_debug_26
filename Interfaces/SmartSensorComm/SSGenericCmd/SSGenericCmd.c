/*******************************************************************************
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
 *******************************************************************************
 */
#include "app.h"
#include "SSInterface.h"
#include "SSGenericCmd.h"
#include "CRC8.h"
//#include "mxm_assert.h"

static const char* cmdStr;
//static char charbuf[1024];
extern char charbuf[1792];
static int ridx;
static int data_len;
static uint8_t cmd_index;

#define NUM_OF_SSGENERICCMD_COMMANDS (sizeof(CMDTABLE_SSGENERICCMD) / sizeof(*CMDTABLE_SSGENERICCMD))

static const char* sensor_type;
static uint8_t     vis;

SSGenericCmd ssGenericCmd;

static void write_to_ss(void);
static void read_from_ss(void);
static void set_i2c_address(void);
static void read_sh_dhparams(void);
static void send_sh_dhlocalpublic(void);
static void read_sh_dhremotepublic(void);
static void read_sh_authentication(void);

//Define the command table
const cmdSSGenericCmd_t CMDTABLE_SSGENERICCMD[] = {
		{ "ss_write"                                  ,     &write_to_ss                               ,     "Writes to ss."                                                 },
		{ "ss_read"                                   ,     &read_from_ss                              ,     "Reads from ss."                                                },
		{ "set_cfg i2c_addr"                          ,     &set_i2c_address                           ,     "Changes I2C address of ME11."                                  },
//	{ "set_cfg sh_mode"                           ,     &set_sensor_hub_mode                       ,     "Changes Sensor Hub mode."                                      },
//	{ "get_cfg sh_counter"                        ,     &read_sh_counter                           ,     "Gets Sensor Hub counter."                                      },
		{ "get_cfg sh_dhparams"                       ,     &read_sh_dhparams                          ,     "calculate secret nonce."                                       },
		{ "set_cfg sh_dhlpublic"                      ,     &send_sh_dhlocalpublic                     ,     "calculate secret nonce."                                       },
		{ "get_cfg sh_dhrpublic"                      ,     &read_sh_dhremotepublic                    ,     "calculate secret nonce."                                       },
		{ "get_cfg sh_auth"                           ,     &read_sh_authentication                    ,     "get challange responce from sshub."                            }

};

static const char* get_type_02(void);
static uint8_t is_visible_02(void);
static uint8_t parse_command_02(const char* cmd);
static void stop_02(void);

int data_report_execute_02(char* buf, int size)
{
  (void)buf;
  (void)size;

	return 0;
}

void SSGenericCmd_int(void)
{
	//sensor = NULL;
	sensor_type = "test";
	vis = false;

	ssGenericCmd.get_type      = (get_type_callback)&get_type_02;
	ssGenericCmd.is_visible    = (is_visible_callback)&is_visible_02;
//ssGenericCmd.is_enabled    = is_enabled_02;
  ssGenericCmd.stop          = (stop_callback)&stop_02;
	ssGenericCmd.parse_command = (parse_command_callback)&parse_command_02;
	ssGenericCmd.data_report_execute = (data_report_execute_callback)&data_report_execute_02;

	ridx = 0;
}

static void stop_02(void)
{

}

static const char* get_type_02(void)
{
	return sensor_type;
}

static uint8_t is_visible_02(void)
{
	return vis;
}

void write_to_ss(void){
	uint8_t cmd_bytes[256];
	int num_cmd_bytes = parse_cmd_data(cmdStr, CMDTABLE_SSGENERICCMD[cmd_index].command, &cmd_bytes[0], sizeof(cmd_bytes), ' ', true);
	if (num_cmd_bytes <= 0) {
		ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "\r\n%s err=%d", cmdStr, num_cmd_bytes);
		return;
	}

	SS_STATUS status = write_cmd2(&cmd_bytes[0], num_cmd_bytes, 0, 0, 500);
	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "\r\n%s err=0\r\n", cmdStr);

	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "\r\nWrote: { ");
	for (int i = 0; i < num_cmd_bytes; i++) {
		ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "%02X ", cmd_bytes[i]);
	}
	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "}\r\n");
	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "Status: { %02X }\r\n", (uint8_t)status);
}

void read_from_ss(void){
	uint8_t cmd_bytes[256];
	uint8_t read_bytes[256];


	int num_cmd_bytes = parse_cmd_data(cmdStr, CMDTABLE_SSGENERICCMD[cmd_index].command, &cmd_bytes[0], sizeof(cmd_bytes), ' ', true);
	if (num_cmd_bytes <= 0) {
		ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "\r\n%s err=%d", cmdStr, num_cmd_bytes);
		return;
	}

	//Last space separated value is the number of bytes to read
	uint16_t num_rx_bytes = cmd_bytes[num_cmd_bytes - 1];                        // Modified by Jason from uint8_t to uint16_t
	if (num_rx_bytes > sizeof(read_bytes)) {
		pr_err("Can read up to %d bytes", num_rx_bytes);
		ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "\r\n%s err=-1", cmdStr);
		return;
	}

	SS_STATUS status = read_cmd(&cmd_bytes[0], num_cmd_bytes - 1,
										0, 0,
										&read_bytes[0], num_rx_bytes,
										500);

	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "\r\n%s err=0\r\n", cmdStr);

	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "\r\nWrote: { ");
	for (int i = 0; i < num_cmd_bytes; i++) {
		ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "%02X ", cmd_bytes[i]);
	}
	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "}\r\n");
	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "Read: { ");
	for (int i = 0; i < num_rx_bytes; i++) {
		ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "%02X ", read_bytes[i]);
	}
	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "}\r\n");
	ridx += snprintf(charbuf + ridx, sizeof(charbuf) - ridx - 1, "Status: { %02X }\r\n", (uint8_t)status);
}

void set_i2c_address(){
	uint8_t val;
	int ret;
	SS_STATUS status;

	ret = (parse_cmd_data(cmdStr, CMDTABLE_SSGENERICCMD[cmd_index].command, &val, 1, ' ', true) != 1);
	if (ret) {
		data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmdStr, COMM_INVALID_PARAM);
		pr_info(charbuf);
		return;
	}

	status = set_i2c_addr(val);

	if (status == SS_SUCCESS){
		data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmdStr, COMM_SUCCESS);
		pr_info(charbuf);
	}
	else{
		data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmdStr, COMM_GENERAL_ERROR);
		pr_info(charbuf);
	}
}


static uint8_t parse_command_02(const char* cmd)
{
//char* ptrChar;
  uint8_t i;
  bool recognizedCmd = false;
//ridx = 0;

//if (!ss_int) {
//    pr_err("No SmartSensor Interface defined!");
//    return false;
//}
//if (!ds_int) {
//    pr_err("No DeviceStudio Interface defined!");
//    return false;
//}

  cmdStr = cmd;

  for (i = 0; i < NUM_OF_SSGENERICCMD_COMMANDS; i++) {
      if (starts_with(cmd, CMDTABLE_SSGENERICCMD[i].command)) {
          cmd_index=i;
        	(*CMDTABLE_SSGENERICCMD[i].functionPtr)();
      recognizedCmd = true;
			break;
      }
  }

	if (recognizedCmd) {
	    //ptrChar = (char*)&charbuf[0];
	    //m_USB->printf(charbuf);                                          // Commented by Jason
        if(cmd_index == 3)                                               // Added by Jason
        {
            //memcpy(charbuf, "get_cfg sh_dhparams value=39C161914B7C err=0", 44 );
            //charbuf[44] = '\0';          
        }
#if 1//def ENABLE_BLE
        if(BLE_Interface_Exists())
		    {
            BLE_AddtoQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, HRV_SENSOR);
          //BLE_AddtoHRVQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);
		    }
#endif
        charbuf[data_len - 1] = 0;                                                                  // Added by Jason
        charbuf[data_len - 2] = 0;                                                                  // Added by Jason
        charbuf[1] = '\r';                                                                          // Added by Jason
        ssg_log("len=%4d        : ", data_len);                                                     // Added by Jason, 2021.11.19
        ssg_log(&charbuf[2]);                                                                       // Added by Jason
        ssg_log(", SSG:Line=%d\r\n\r\n", __LINE__);                                                 // Added by Jason
	}

	return recognizedCmd;
}

int parse_public_str(const char *ptr_ch, const char *cmd, uint8_t *data, int data_sz)
{
	char ascii_byte[] = { 0, 0, 0 };
	const char* sptr = ptr_ch + strlen(cmd);
	int found = 0;
	int ssfound;
	unsigned int val32;

	//Eat spaces after cmd
	while (*sptr == ' ') { sptr++; }
	if (*sptr == '\0')
		return -1;

	while (found < data_sz) {
		if (*sptr == '\0')
			break;
		ascii_byte[0] = *sptr++;
		ascii_byte[1] = *sptr++;
		ssfound = sscanf(ascii_byte, "%x", &val32);
		if (ssfound != 1)
			break;
		*(data + found) = (uint8_t)val32;
		found++;
	}

	if (found < data_sz)
		return -1;
	return 0;
}

void read_sh_dhparams(void) {

	SS_STATUS status;
	uint8_t rxBuff[6+1];  // first byte is status
	char outstr[2*sizeof(rxBuff)];
	int str_idx = 0;
    uint32_t i;

	status = get_dhparams( &rxBuff[0], sizeof(rxBuff) );
    if (status == SS_SUCCESS){

    	for (i = 0; i < sizeof(rxBuff)-1; i++)
    		str_idx += snprintf(outstr + str_idx, sizeof(outstr) - str_idx - 1, "%02X", rxBuff[i+1]);

    	data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s value=%s err=%d\r\n", cmdStr, outstr, COMM_SUCCESS);

	} else{
		data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmdStr, COMM_GENERAL_ERROR);
	}

}

void send_sh_dhlocalpublic(void) {


	int ret;
	SS_STATUS status;
	uint8_t LOCALPUBLIC[12];

	ret = parse_public_str(cmdStr, CMDTABLE_SSGENERICCMD[cmd_index].command, LOCALPUBLIC, sizeof(LOCALPUBLIC));
	if (ret) {
		pr_info("\r\n%s err=%d\r\n", cmdStr, COMM_INVALID_PARAM);
		snprintf(charbuf,sizeof(charbuf),"%s err=%d\r\n", cmdStr,COMM_INVALID_PARAM);
		return;
	}

	status = set_dhlocalpublic( &LOCALPUBLIC[0] , sizeof(LOCALPUBLIC) );
    if (status == SS_SUCCESS){
		data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmdStr, COMM_SUCCESS);
	}
	else{
		data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmdStr, COMM_GENERAL_ERROR);
	}

}

void read_sh_dhremotepublic(void) {

	SS_STATUS status;
	uint8_t rxBuff[12+1];  // first byte is status
	char outstr[2*sizeof(rxBuff)];
	int str_idx = 0;
    uint32_t i;

	status = get_dhremotepublic( &rxBuff[0], sizeof(rxBuff) );
    if (status == SS_SUCCESS){
    	for (i = 0; i < sizeof(rxBuff)-1; i++)
    		str_idx += snprintf(outstr + str_idx, sizeof(outstr) - str_idx - 1, "%02X", rxBuff[i+1]);

    	data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s value=%s err=%d\r\n", cmdStr, outstr, COMM_SUCCESS);

	}else{

		data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmdStr, COMM_GENERAL_ERROR);
	}
}

void read_sh_authentication(void)
{

	uint8_t rxBuff[32+1];  // first byte is status
	char outstr[2*sizeof(rxBuff)];
	int str_idx = 0;
	SS_STATUS status;
    uint32_t i;

	status = get_authentication( &rxBuff[0], sizeof(rxBuff) );
	if(status == SS_SUCCESS) {

    	for (i = 0; i < sizeof(rxBuff)-1; i++)
    		str_idx += snprintf(outstr + str_idx, sizeof(outstr) - str_idx - 1, "%02X", rxBuff[i+1]);

    	data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s value=%s err=%d\r\n", cmdStr, outstr, COMM_SUCCESS);

    }else{
			data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmdStr, COMM_GENERAL_ERROR);
    }

}

#if 0

SSGenericCmd::SSGenericCmd(USBSerial *USB, SSInterface* ssInterface, DSInterface* dsInterface)
    :SensorComm("ss_test", false), m_USB(USB), ss_int(ssInterface), ds_int(dsInterface)
{
	ridx = 0;
}
#endif
