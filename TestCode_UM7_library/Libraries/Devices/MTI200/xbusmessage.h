/*!
 * \file
 * \copyright Copyright (C) Xsens Technologies B.V., 2015.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy
 * of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef __XBUSMESSAGE_H
#define __XBUSMESSAGE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Xbus message IDs. */
enum XsMessageId
{
	XMID_Undefined			= 0xFF,
	XMID_Wakeup             = 0x3E,
	XMID_WakeupAck          = 0x3F,
	XMID_ReqDid             = 0x00,
	XMID_DeviceId           = 0x01,
	XMID_GotoConfig         = 0x30,
	XMID_GotoConfigAck      = 0x31,
	XMID_GotoMeasurement    = 0x10,
	XMID_GotoMeasurementAck = 0x11,
	XMID_MtData2            = 0x36,
	XMID_ReqOutputConfig    = 0xC0,
	XMID_SetOutputConfig    = 0xC0,
	XMID_OutputConfig       = 0xC1,
	XMID_Reset              = 0x40,
	XMID_ResetAck           = 0x41,
	XMID_Error              = 0x42,
	/* This is for UM7 */
	/* Configuration registers */
	CREG_COM_SETTINGS 			= 0x00,
	CREG_COM_RATES1					= 0x01,
	CREG_COM_RATES2					= 0x02,
	CREG_COM_RATES3					= 0x03,
	CREG_COM_RATES4					= 0x04,
	CREG_COM_RATES5					= 0x05,
	CREG_COM_RATES6					= 0x06,
	CREG_COM_RATES7					= 0x07,
	CREG_MISC_SETTINGS			= 0x08,
	CREG_HOME_NORTH					= 0x09,
	CREG_HOME_EAST					= 0x0A,
	CREG_HOME_UP						= 0x0B,
	CREG_GYRO_TRIM_X				= 0x0C,
	CREG_GYRO_TRIM_Y				= 0x0D,
	CREG_GYRO_TRIM_Z				= 0x0E,
	CREG_MAG_CAL1_1					= 0x0F,
	CREG_MAG_CAL1_2					= 0x10,
	CREG_MAG_CAL1_3					= 0x11,
	CREG_MAG_CAL2_1					= 0x12,
	CREG_MAG_CAL2_2					= 0x13,
	CREG_MAG_CAL2_3					= 0x14,
	CREG_MAG_CAL3_1					= 0x15,
	CREG_MAG_CAL3_2					= 0x16,
	CREG_MAG_CAL3_3					= 0x17,
	CREG_MAG_BIAS_X					= 0x18,
	CREG_MAG_BIAS_Y					= 0x19,
	CREG_MAG_BIAS_Z					= 0x1A,
  /* Data registers */
	DREG_HEALTH							= 0x55,
	DREG_GYRO_RAW_XY				= 0x56,
	DREG_GYRO_RAW_Z					= 0x57,
	DREG_GYRO_TIME					= 0x58,
	DREG_ACCEL_RAW_XY				= 0x59,
	DREG_ACCEL_RAW_Z				= 0x5A,
	DREG_ACCEL_TIME					= 0x5B,
	DREG_MAG_RAW_XY					= 0x5C,
	DREG_MAG_RAW_Z					= 0x5D,
	DREG_MAG_RAW_TIME				= 0x5E,
	DREG_TEMPERATURE				= 0x5F,
	DREG_TEMPERATURE_TIME		= 0x60,
	DREG_GYRO_PROC_X				= 0x61,
	DREG_GYRO_PROC_Y				= 0x62,
	DREG_GYRO_PROC_Z	      = 0x63,
	DREG_GRYO_PROC_TIME			= 0x64,
	DREG_ACCEL_PROC_X				= 0x65,
	DREG_ACCEL_PROC_Y				= 0x66,
	DREG_ACCEL_PROG_Z				= 0x67,
	DREG_ACCEL_PROC_TIME		= 0x68,
	DREG_MAG_PROC_X					= 0x69,
	DREG_MAG_PROC_Y					= 0x6A,
	DREG_MAG_PROC_Z					= 0x6B,
	DREG_MAG_PROC_TIME			= 0x6C,
	DREG_QUAT_AB						= 0x6D,
	DREG_QUAT_CD						= 0x6E,
	DREG_QUAT_TIME		  		= 0x6F,
	DREG_EULER_PHI_THETA		= 0x70,
	DREG_EULER_PSI					= 0x71,
	DREG_EULER_PHI_THETA_DOT= 0x72,
	DREG_EULER_PSI_DOT			= 0x73,
	DREG_EULER_TIME					= 0x74,
	DREG_POSITION_NORTH			= 0x75,
	DREG_POSITION_EAST			= 0x76,
	DREG_POSITION_UP				= 0x77,
	DREG_POSITION_TIME			= 0x78,
	DREG_VELOCITY_NORTH			= 0x79,
	DREG_VELOCITY_EAST			= 0x7A,
	DREG_VELOCITY_UP				= 0x7B,
	DREG_VELOCITY_TIME      = 0x7C,
	DREG_GPS_LATITUDE				= 0x7D,
	DREG_GPS_LONGITUDE			= 0x7E,
	DREG_GPS_ALTITUDE				= 0x7F,
	DREG_GPS_COURSE					= 0x80,
	DREG_GPS_SPEED					= 0x81,
	DREG_GPS_TIME						= 0x82,
	DREG_GPS_SAT_1_2				= 0x83,
	DREG_GPS_SAT_3_4				= 0x84,
	DREG_GPS_SAT_5_6				= 0x85,
	DREG_GPS_SAT_7_8				= 0x86,
	DREG_GPS_SAT_9_10				= 0x87,
	DREG_GPS_SAT_11_12			= 0x88,
	DREG_GYRO_BIAS_X				= 0x89,
	DREG_GYRO_BIAS_Y				= 0x8A,
	DREG_GYRO_BIAS_Z				= 0x8B,
	/* Commands */
  GET_FW_REVISION         = 0xAA,
	FLASH_COMMIT						= 0xAB,
	RESET_TO_FACTORY				= 0xAC,
	ZERO_GYROS							= 0xAD,
	SET_HOME_POSITION				= 0xAE,
	RESERVED								= 0xAF,
	SET_MAG_REFERENCE 			= 0xB0,
	RESERVED_2              = 0xB1,
	RESERVED_3              = 0xB2,
	RESET_EKF								= 0xB3
};

/*! \brief Xbus data message type IDs. */
enum XsDataIdentifier
{
	XDI_PacketCounter  = 0x1020,
	XDI_SampleTimeFine = 0x1060,
	XDI_Quaternion     = 0x2010, // estimated orientation
	XDI_DeltaV         = 0x4010, // estimated acceleration
	XDI_Acceleration   = 0x4020, // calibrated accelerometer values
	XDI_RateOfTurn     = 0x8020, // calibrated gyro values
	XDI_DeltaQ         = 0x8030, // estimated quaternion derivative
	XDI_RawAccGyrMagTemp = 0xA010, // raw sensors
	XDI_MagneticField  = 0xC020,
	XDI_StatusWord     = 0xE020
};

/*!
 * \brief Low level format to use when formating Xbus messages for transmission.
 */
enum XbusLowLevelFormat
{
	/*! \brief Format for use with I2C interface. */
	XLLF_I2c,
	/*! \brief Format for use with SPI interface. */
	XLLF_Spi,
	/*! \brief Format for use with UART interface. */
	XLLF_Uart,
	/*! \brief Format for use with UART interface on UM7 . */
	XLLF_UM7
};

/*!
 * \brief An Xbus message structure with optional payload.
 */
struct XbusMessage
{
	/*! \brief The message ID of the message. */
	enum XsMessageId mid;
	/*!
	 * \brief The length of the payload.
	 *
	 * \note The meaning of the length is message dependent. For example,
	 * for XMID_OutputConfig messages it is the number of OutputConfiguration
	 * elements in the configuration array.
	 */
	uint16_t length;
	/*! \brief Pointer to the payload data. */
	void* data;

	/*!
	 * \brief The writing or reading of the operation, also if it is batch and batch length
	 */
	uint8_t read_write;
	uint8_t is_batch;
	uint8_t batch_length;


};

/*!
 * \brief Output configuration structure.
 */
struct OutputConfiguration
{
	/*! \brief Data type of the output. */
	enum XsDataIdentifier dtype;
	/*!
	 * \brief The output frequency in Hz, or 65535 if the value should be
	 * included in every data message.
	 */
	uint16_t freq;
};

size_t XbusMessage_format(uint8_t* raw, struct XbusMessage const* message, enum XbusLowLevelFormat format);
bool XbusMessage_getDataItem(void* item, enum XsDataIdentifier id, struct XbusMessage const* message);
char const* XbusMessage_dataDescription(enum XsDataIdentifier id);

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __XBUSMESSAGE_H
