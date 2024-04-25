/*
 * DSInterface.c
 *
 *  Created on: May 28, 2021
 *      Author: jason Chen
 */
#include "app.h"
#include "DSInterface.h"
#include "SSMAX30101Comm.h"
#include "SSGenericCmd.h"
#include "bio_version.h"
#include "app_aescrypt.h"

static const char *glbl_cmds[] = {
  "never use",
	"stop",
	"get_device_info",
	"assign",                                                // Jared need it
	"auth",                                                  // Jared need it
  "challenge",                                             // Jared need it as one new authentication
  "power off",
  "getbattery"
};

typedef enum {
  never_use = 0,
	stop,
	get_device_info,
	assign,
  auth,
  challenge,
  power_off,
  getbattery,
	NUM_CMDS,
} glbl_cmd_state;

int num_sensors;
SensorComm sensor_list_t[DS_MAX_NUM_SENSORCOMMS];
SensorComm* sensor_list = (SensorComm*)(&sensor_list_t[0]);

//bool calibration_success = false;
volatile uint8_t ds_console_interface_exists_;
//volatile uint8_t ds_ble_interface_exists_;

const char* platform_name;
const char* firmware_version;

#define CONSOLE_STR_BUF_SZ                    (1024 + 64) //(2048)

char cmd_str[CONSOLE_STR_BUF_SZ];

int cmd_idx;
bool silent_mode;
bool pause_mode;

void DSInterface_init(void)
{
	cmd_idx = 0;
	silent_mode = false;
	pause_mode = false;

	memset(&sensor_list[0], 0, DS_MAX_NUM_SENSORCOMMS * sizeof(SensorComm*));
	num_sensors = 0;
}

void SensorComm_create(void)
{
	SSMAX30101Comm_init();
//SSBootloaderComm_init();
	SSGenericCmd_int();
#if 1
	int i = 0;
		sensor_list_t[i].get_type                  = (get_type_callback)ssMAX30101.get_type;
		sensor_list_t[i].get_part_info             = (get_part_info_callback)ssMAX30101.get_part_info;     //
		sensor_list_t[i].is_visible                = (is_visible_callback)ssMAX30101.is_visible;
		//sensor_list_t[i].is_enabled                 = (is_enabled_callback)ssMAX30101.is_enabled;
		sensor_list_t[i].SensorComm_Set_Ble_Status = (SensorComm_Set_Ble_Status_callback)ssMAX30101.SensorComm_Set_Ble_Status;

		sensor_list_t[i].get_part_name             = (get_part_name_callback)ssMAX30101.get_part_name;
		sensor_list_t[i].get_algo_ver              = (get_algo_ver_callback)ssMAX30101.get_algo_ver;
		sensor_list_t[i].stop                      = (stop_callback)ssMAX30101.stop;
		sensor_list_t[i].parse_command             = (parse_command_callback)ssMAX30101.parse_command;
		sensor_list_t[i].data_report_execute       = (data_report_execute_callback)ssMAX30101.data_report_execute;

	i = 1;
		sensor_list_t[i].get_type                  = (get_type_callback)ssGenericCmd.get_type;
		sensor_list_t[i].is_visible                = (is_visible_callback)ssGenericCmd.is_visible;
	//sensor_list_t[i].is_enable                 = (is_enable_callback)&is_enabled_t;
	//sensor_list_t[i].get_part_info             = (get_part_info_callback)ssGenericCmd.get_part_info;
  //sensor_list_t[i].SensorComm_Set_Ble_Status = (SensorComm_Set_Ble_Status_callback)&SensorComm_Set_Ble_Status_t;

	//sensor_list_t[i].get_part_name             = (get_part_name_callback)&get_part_name_t;
	//sensor_list_t[i].get_algo_ver              = (get_algo_ver_callback)&get_algo_ver_t;
	  sensor_list_t[i].stop                      = (stop_callback)ssGenericCmd.stop;
		sensor_list_t[i].parse_command             = (parse_command_callback)ssGenericCmd.parse_command;
	//sensor_list_t[i].data_report_execute       = (data_report_execute_callback)&data_report_execute_t;
		sensor_list_t[i].data_report_execute       = (data_report_execute_callback)ssGenericCmd.data_report_execute;

	num_sensors = i + 1;
#endif
}


static void Parse_command(void)
{
  int i;
	glbl_cmd_state cmd;
	char charbuf[512];
	int data_len = 0;
	int ret;
	bool parsed_cmd = true;

	//If top level command, then handle it
	for (i = 0; i < NUM_CMDS; i++)
	{
	    if (starts_with(&cmd_str[0], glbl_cmds[i]))
	    {
	        cmd = (glbl_cmd_state)i;

	        switch (cmd)
	        {
	          case (stop):
	          {
	            for (int i = 0; i < num_sensors; i++)
	            {
	                sensor_list[i].stop();
	            }
	            data_len += snprintf(charbuf, sizeof(charbuf) - 1, "\r\n%s err=0\r\n", cmd_str);
	          }
	          break;

	          case (get_device_info):
	          {
	          //data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s platform=%s firmware_ver=%s sensors=", cmd_str, platform_name, FIRMWARE_VERSION);
	            data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s platform=%s firmware_ver=SW%d.%d sensors=", cmd_str, platform_name, BIO_FW_MAJOR, BIO_FW_MINOR);

	            //Add list of sensors
	            for (int i = 0; i < num_sensors; i++)
	            {
	                if (sensor_list[i].is_visible())
	                {
	                    data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, "%s", sensor_list[i].get_type());
	                    if (i < (num_sensors - 1))
	                      data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, ",");
	                }
	            }

	            for (int i = 0; i < num_sensors; i++)
	            {
	                //SensorComm *s = &sensor_list[i];
	                if (!sensor_list[i].is_visible())
	                  continue;

	                //Add algo_ver
	                data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " algo_ver_%s=%s", sensor_list[i].get_type(), sensor_list[i].get_algo_ver());

	                //Add part name
	              //data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " part_name_%s=%s", sensor_list[i].get_type(), sensor_list[i].get_part_name());
	                data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " serialNo=%02X%02X%02X%02X%02X%02X", calidata.serial[0], calidata.serial[1], calidata.serial[2], calidata.serial[3], calidata.serial[4], calidata.serial[5]);

	                uint8_t part_id, part_rev;
	                ret = sensor_list[i].get_part_info(&part_id, &part_rev);

	                if (ret == 0)
	                {
	                    //Add part id
	                    data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " part_id_%s=%d", sensor_list[i].get_type(), part_id);

	                    //Add rev id
	                    data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " part_rev_%s=%d", sensor_list[i].get_type(), part_rev);
	                }
	            }

	            if(firmware_version)
	            {
	                data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " hub_firm_ver=%s", firmware_version);
	            }

	            data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " err=0\r\n");

	          }
	          break;

            case assign:
            {
              uint8_t auth_data[32 + 1] = {0};

              // assign hw_major=H hw_minor=H serial=HHHHHHHHHHHH auth=HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
              ret = parse_cmd_data(&cmd_str[7], "hw_major", &calidata.fw_major, 1, '=', false);
              if(ret != 1)
              {
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_INVALID_PARAM);
                  break;
              }

              ret = parse_cmd_data(&cmd_str[18], "hw_minor", &calidata.fw_minor, 1, '=', false);
              if(ret != 1)
              {
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_INVALID_PARAM);
                  break;
              }

              ret = parse_cmd_hexstr(&cmd_str[29], "serial", calidata.serial, 6);
              if(ret)
              {
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_INVALID_PARAM);
                  break;
              }

              ret = parse_cmd_hexstr(&cmd_str[49], "auth", auth_data/*calidata.auth_value*/, 32);
              if(ret)
              {
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_INVALID_PARAM);
                  break;
              }

              if(!compare_uint8(auth_data, calidata.auth_value, 32))
              {
                  memcpy(calidata.auth_value, auth_data, sizeof(calidata.auth_value));
                  calidata.auth_allowed = 0;                                        // new auth value applied
                  flash_write();
                  dsinterface_log(" .......New auth value, DSI:Line=%d\r\n\r\n", __LINE__);                                       // Added by Jason
              }
#if DSINTERFACE_LOG_ENABLE
              else
              {
                  dsinterface_log(" .......The auth value is same as the old auth value, DSI:Line=%d\r\n\r\n", __LINE__);          // Added by Jason
              }
#endif
              data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_SUCCESS);
              parsed_cmd = true;
         #if DSINTERFACE_LOG_ENABLE
              flash_init(false);
         #endif
            }break;

            case auth:
            {
              uint8_t auth_data[16 + 1] = {0};
              uint8_t IV16_1[16] = {0};
              uint8_t de_cipherData[16] = {0};

              ret = parse_cmd_hexstr(cmd_str, "auth", auth_data, 16);
              if(ret)
              {
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_INVALID_PARAM);
                  calidata.auth_allowed = 0;                                        // auth failed
                  break;
              }

#if DE_CRYPT_LOG_ENABLE
              mbedtls_log("\r\nFirst time Decrpt......");
              mbedtls_log("\r\n16byte IV Value: ");
              print_hexstring(IV16, 16);
#endif
              memcpy(IV16_1, IV16, 16);
              memset(de_cipherData, 0 , sizeof(de_cipherData));
              aes_crypt_cbc(calidata.auth_value, IV16_1, (uint8_t*)auth_data, de_cipherData, CBC_AES_DECRYPT);

#if DE_CRYPT_LOG_ENABLE
              mbedtls_log("\r\nSecurityKey256 : ");
              print_hexstring(calidata.auth_value, 32);
              mbedtls_log("\r\nplain_data16   : ");
              print_hexstring(auth_data, 16);
              mbedtls_log("\r\nCBC decrypt256 : ");
              print_hexstring(de_cipherData, 16);
              mbedtls_log("\r\n");
#endif

#if DE_CRYPT_LOG_ENABLE
              mbedtls_log("\r\nSecond time Decrpt......");
              mbedtls_log("\r\n16byte IV Value: ");
              print_hexstring(IV16, 16);
#endif
              memcpy(IV16_1, IV16, 16);
              memset(auth_data, 0 , sizeof(de_cipherData));
              aes_crypt_cbc(calidata.auth_value, IV16_1, (uint8_t*)de_cipherData, auth_data, CBC_AES_DECRYPT);

#if AESCRYPT_LOG_ENABLE
    #if DE_CRYPT_LOG_ENABLE
              mbedtls_log("\r\nSecurityKey256 : ");
              print_hexstring(calidata.auth_value, 32);
              mbedtls_log("\r\nplain_data16   : ");
              print_hexstring(de_cipherData, 16);
    #endif
              mbedtls_log("\r\nCBC decrypt256  : ");
              print_hexstring(auth_data, 16);
              mbedtls_log(", DSI:Line=%d\r\n", __LINE__);
#endif
              if(!compare_uint8(plain_data16, auth_data, 16))
              {
                  data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_AUTH_ERROR);
                  dsinterface_log("\r\n.....Auth fail!.............Line=%d\r\n", __LINE__);
                  calidata.auth_allowed = 0;                                        // auth failed
                  break;
              }
              dsinterface_log("\r\n                : Authentication pass...., DSI:Line=%d\r\n", __LINE__);
              calidata.auth_allowed = 1;                                        // auth passed!
#if 0
              data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_SUCCESS);
#else
              data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s", "auth ");
              bin2hextext(&charbuf[data_len], auth_data, 16);   data_len +=32;
              data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " err=%d\r\n", COMM_SUCCESS);
#endif
              parsed_cmd = true;
            }
            break;

            case challenge:
            {
              uint8_t IV16_1[16] = {0};

              memset(plain_data16, 0, sizeof(plain_data16));
              ret = parse_cmd_hexstr(cmd_str, "challenge", plain_data16, 16);        // 8 or 16?????
              if(ret)
              {
                  mbedtls_log("\r\n****The random number length < 16 bytes, will pad 8 zeros into it****\r\n");
                  //data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_INVALID_PARAM);
                  //calidata.auth_allowed = 0;                                        // auth failed
                  //break;
              }

              generate_random_key(IV16,   16);
#if AESCRYPT_LOG_ENABLE
              mbedtls_log("\r\n16byte IV Value : ");
              print_hexstring(IV16, 16);
#endif
              memcpy(IV16_1, IV16, 16);
              aes_crypt_cbc(calidata.auth_value, IV16_1, (uint8_t*)plain_data16, cipherData16, CBC_AES_ENCRYPT);

#if AESCRYPT_LOG_ENABLE
              mbedtls_log("\r\nSecurityKey256  : ");
              print_hexstring(calidata.auth_value, 32);
              mbedtls_log(", DSI:Line=%d\r\nplain_data16    : ", __LINE__);
              print_hexstring(plain_data16, 16);
              mbedtls_log("\r\nCBC encrypt256  : ");
              print_hexstring(cipherData16, 16);
              mbedtls_log("\r\n\r\n");
#endif
#if 0
              data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s err=%d\r\n", cmd_str, COMM_SUCCESS);
#else
              data_len = snprintf(charbuf, sizeof(charbuf), "\r\n%s", "challenge ");
              bin2hextext(&charbuf[data_len], calidata.serial, 6); data_len +=12;
              charbuf[data_len++] = ' ';
              bin2hextext(&charbuf[data_len], cipherData16, 16);   data_len +=32;
              charbuf[data_len++] = ' ';
              bin2hextext(&charbuf[data_len], IV16, 16);         data_len +=32;
              data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " err=%d\r\n", COMM_SUCCESS);
#endif
              parsed_cmd = true;
            }break;

            case power_off:
            {
              pr_info("----power off..., Line=%d\r\n", __LINE__);
              wait_ms(300);
              max14676_poweroff();  //does not return
            }break;

            case getbattery:
            {
              uint16_t batlevel = 0;
              if(bio_packet.batsoc&0x80)
                batlevel = 100;
              else
                batlevel = (bio_packet.batsoc)&0x7F;

              data_len  = snprintf(charbuf, sizeof(charbuf), "\r\n%s percent=%d volt%d=%2.2fV", cmd_str, batlevel, charging_state, (float)(bio_packet.batvcell * 78.125 / (1000.0 * 1000.0)));
              data_len += snprintf(charbuf + data_len, sizeof(charbuf) - data_len, " T=%2.1f err=0\r\n", (float)(bio_packet.temp0 / 200.0));                                                 // temperature added
            }
            break;

            default:
	          {
	            parsed_cmd = false;
	          }
	          break;
	        }

	        if (parsed_cmd)
	        {
	            if (BLE_Interface_Exists())
	            {
	                BLE_AddtoQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, bio_sensor_type);
#if 0
	                if(bio_sensor_type == BPT_SENSOR)
	                  BLE_AddtoBPTQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);
	                else if(bio_sensor_type == HRV_SENSOR)
	                  BLE_AddtoHRVQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);
#endif
	            }
	        }
#if DSINTERFACE_LOG_ENABLE
	        if(data_len)
	        {
              charbuf[data_len - 1] = 0;                                                        // Added by Jason
              charbuf[data_len - 2] = 0;                                                        // Added by Jason
              charbuf[1] = '\r';                                                                // Added by Jason
              dsinterface_log("len=%4d        : ", data_len);                                   // Added by Jason, 2021.11.19
              dsinterface_log(&charbuf[2]);                                                     // added by Jason
              dsinterface_log(", DSI(%s):Line=%d\r\n\r\n", __func__, __LINE__);                 // Added by Jason
	        }
#endif
	        return;
	    }
	}

	if(calidata.auth_allowed == 0)
	{
	    //dsinterface_log("Auth fail!.............Line=%d\r\n", __LINE__);
	    //return;
	}
	//Loop through each sensor in sensorList
	//If sensor.get_type() is contained in cmd_str, pass cmd_str to that sensor's parser
	for (int i = 0; i < num_sensors; i++)
	{
	    if (strstr(&cmd_str[0], sensor_list[i].get_type()))
	    {
	        if (sensor_list[i].parse_command(cmd_str))
	        {
	            return;
	        }
	        break;
	    }
	}

	//If we still haven't found a way to parse the command,
	//send it to every sensor until one handles it
	for (int i = 0; i < num_sensors; i++)
	{
	    if (sensor_list[i].parse_command(cmd_str))
	    {
	        return;
	    }
	}

	//No one could handle the command, print cmd not recognized string
	data_len += snprintf(charbuf, sizeof(charbuf) - 1, "\r\n%s err=-255\r\n", cmd_str);

	//m_USB->printf(charbuf);                                                           // Commented by Jason
	if (BLE_Interface_Exists())
	{
	    BLE_AddtoQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, bio_sensor_type);
#if 0
      if(bio_sensor_type == BPT_SENSOR)
        BLE_AddtoBPTQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);
      else if(bio_sensor_type == HRV_SENSOR)
        BLE_AddtoHRVQueue((uint8_t *)charbuf, (int32_t)sizeof(charbuf), data_len, __LINE__);
#endif
	}

#if DSINTERFACE_LOG_ENABLE
	charbuf[data_len - 1] = 0;                                                        // Added by Jason
	charbuf[data_len - 2] = 0;                                                        // Added by Jason
	charbuf[1] = '\r';                                                                // Added by Jason
	dsinterface_log("len=%4d        : ", data_len);                                    // Added by Jason, 2021.11.19
	dsinterface_log(&charbuf[2]);                                                     // Added by Jason
	dsinterface_log(", DSI:Line=%d\r\n\r\n", __LINE__);                               // Added by Jason
#endif
}

#if DSINTERFACE_LOG_ENABLE
char cmd_str_log[54 + 23 + 1];
#endif
void Build_Command(char ch)
{
  static int count_c = 0;
	count_c++;
	if(count_c < (54 + 23 - 1))
	{
		//dsinterface_log("%c", ch);
	}

	if (ch == 0x00) {
		pr_err("Ignored char 0x00");
		return;
	}

	if ((ch == '\n') || (ch == '\r')) {
		if (cmd_idx < (int)CONSOLE_STR_BUF_SZ)
           cmd_str[cmd_idx++] = '\0';
#if DSINTERFACE_LOG_ENABLE
		memcpy( &cmd_str_log[0], cmd_str , 54 + 23);
		dsinterface_log("Len=%4d        : ", count_c);
		if(count_c >= (54 + 23))
		{
			cmd_str_log[54 + 23] = '\0';
			dsinterface_log("%s....", cmd_str_log);
		}
		else
		{
		    dsinterface_log("%s", cmd_str_log);
		}
		dsinterface_log(", DSI(%s):Line=%d\r\n", __func__, __LINE__);
#endif
		Parse_command();
		count_c = 0;

		//Clear cmd_str
		while (cmd_idx > 0) /* BUG: POTENTIAL BUG for multiple port access */
			cmd_str[--cmd_idx] = '\0';

	} else if ((ch == 0x08 || ch == 0x7F) && cmd_idx > 0) {
		//Backspace character
		if (cmd_idx > 0)
			cmd_str[--cmd_idx] = '\0';
	} else {
		/* BUG: POTENTIAL BUG for multiple port access */
		if (cmd_idx < (int)CONSOLE_STR_BUF_SZ)
			cmd_str[cmd_idx++] = ch;
	}
}

void data_report_execute(void)
{
	static char buffer[1024 + 64] __attribute__((aligned(4)));
	int data_len = 0;

	buffer[0] = '\0';

	for (int i = 0; i < num_sensors; i++)
	{
	    data_len = sensor_list[i].data_report_execute(buffer, sizeof(buffer));

	    if (data_len > 0)
	    {
	        if(BLE_Interface_Exists())
	        {
	            BLE_AddtoQueue((uint8_t *)buffer, (int32_t)sizeof(buffer), data_len, bio_sensor_type);
#if 0
	            if(bio_sensor_type == BPT_SENSOR)
	              BLE_AddtoBPTQueue((uint8_t *)buffer, (int32_t)sizeof(buffer), data_len, __LINE__);
	            else if(bio_sensor_type == HRV_SENSOR)
	              BLE_AddtoHRVQueue((uint8_t *)buffer, (int32_t)sizeof(buffer), data_len, __LINE__);
#endif
	        }
	        data_len = 0;
	    }
	}
}

void set_fw_version(const char* version)
{
    if (version && *version)
        firmware_version = version;
}

void set_fw_platform(const char* platform)
{
    if (platform && *platform)
        platform_name = platform;
}
