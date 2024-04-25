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

#ifndef _SSGENERICCMD_H_
#define _SSGENERICCMD_H_

#include "SSInterface.h"

//Struct to hold the commands, their functions, and information about themselves
typedef struct {
    const char* command;
    void (*functionPtr)();
    const char* help;
}cmdSSGenericCmd_t;

/*
 * Include this class to use "ss_read" and "ss_write" commands to read and write arbitrary
 * SmartSensor data:
 *
 * ss_write [cmdByte0] [cmdByte1] ... [cmdByteN]
 * ss_read [cmdByte0] [cmdByte1] ... [cmdByteN] <bytes_to_read_in_hex>
 *
 * Example: Read SS firmware version
 * "ss_read FF 03 4"
 *
 * Example: Set MAX30101 register 8 to 1F
 * "ss_write 40 03 8 1F"
 *
 * Example: Read MAX30101 register 1F
 * "ss_read 40 03 8 2"
 *
 */

void SSGenericCmd_int(void);

/**
 * @brief	SSMAX30101Comm Command handler class for communication with MAX30101 on SmartSensor board
 * @details
 */
typedef struct//class SSGenericCmd:	public SensorComm
{

	get_type_callback            get_type;
	is_visible_callback          is_visible;
	stop_callback                stop;
	parse_command_callback       parse_command;
	data_report_execute_callback data_report_execute;
#if 0

//	void set_sensor_hub_mode();
//	void read_sh_counter();
	uint8_t SHLOCALPUBLIC[12];
	uint8_t SHREMOTEPUBLIC[12];
#endif
}SSGenericCmd;

extern SSGenericCmd ssGenericCmd;

#endif /* _SSMAX30101COMM_H_ */
