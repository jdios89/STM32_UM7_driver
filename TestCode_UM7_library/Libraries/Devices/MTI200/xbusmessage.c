/* Copyright (C) 2019-2020 Juan de Dios Flores Mendez. All rights reserved.
 * Based on code by Xsens
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Juan de Dios Flores Mendez
 * e-mail   :  juan.dios.flores@gmail.com
 * ------------------------------------------
 */

#include "xbusmessage.h"
#include "xbusdef.h"
#include "xbusutility.h"

/*!
 * \brief Calculate the number of bytes needed for \a message payload.
 */
static uint16_t messageLength(struct XbusMessage const* message)
{
	switch (message->mid)
	{
		case XMID_SetOutputConfig:
			return message->length * 2 * sizeof(uint16_t);

		default:
			return message->length;
	}
}

/*!
 * \brief Format a message with a pointer to an array of OutputConfiguration elements.
 */
static void formatOutputConfig(uint8_t* raw, struct XbusMessage const* message)
{
	struct OutputConfiguration* conf = message->data;
	for (int i = 0; i < message->length; ++i)
	{
		raw = XbusUtility_writeU16(raw, conf->dtype);
		raw = XbusUtility_writeU16(raw, conf->freq);
		++conf;
	}
}

/*!
 * \brief Format the payload of a message from a native data type to
 * raw bytes.
 */
static void formatPayload(uint8_t* raw, struct XbusMessage const* message)
{
	switch (message->mid)
	{
		case XMID_SetOutputConfig:
			formatOutputConfig(raw, message);
			break;

		default:
			for (int i = 0; i < message->length; ++i)
			{
				*raw++ = ((uint8_t*)message->data)[i];
			}
			break;
	}
}

/*!
 * \brief Format a message into the raw Xbus format ready for transmission to
 * a motion tracker.
 */
size_t XbusMessage_format(uint8_t* raw, struct XbusMessage const* message, enum XbusLowLevelFormat format)
{
	uint8_t* dptr = raw;
	switch (format)
	{
		case XLLF_I2c:
			{
				*dptr++ = XBUS_CONTROL_PIPE;
			}
			break;

		case XLLF_Spi:
			{
				*dptr++ = XBUS_CONTROL_PIPE;
				// Fill bytes required to allow MT to process data
				*dptr++ = 0;
				*dptr++ = 0;
				*dptr++ = 0;
			}
			break;

		case XLLF_Uart:
			{
				*dptr++ = XBUS_PREAMBLE;
				*dptr++ = XBUS_MASTERDEVICE;
			}
			break;
		case XLLF_UM7:
		  {
		  	uint16_t checksum = 's' + 'n' + 'p';
			  *dptr++ = 's'; /* Complete header of the message */
			  *dptr++ = 'n';
			  *dptr++ = 'p';
			  uint8_t address_type = message->mid;
			  if (address_type < 0xAA) {
			  	uint8_t packet_type = 0x0;
			  	if(message->read_write)
			  		packet_type = (0x01 << 7);
			  	if(message->is_batch)
			  		packet_type = packet_type | (0x01 << 6);

			  	packet_type = packet_type | (message->batch_length << 2);
			  	*dptr++ = packet_type;
			  	checksum += packet_type;
			  }
			  else if (address_type >= 0xAA) {
            *dptr++ = 0x0;
            checksum += 0x0;
			  }
			  /* Write the address */
			  *dptr++ = (uint8_t) message->mid;
			  checksum += (uint8_t) message->mid;
			  uint8_t datalength = 0;
			  if (message->read_write) { // if packet has data
			  	if(message->is_batch) {
			  		datalength = 4*message->batch_length;
			  	}
			  	else {
			  		datalength = 4;
			  	}
			  }
			  for (int i = 0; i < datalength; ++i)
			  {
			    *dptr++ = ((uint8_t*) message->data)[i];
			    /* Complete checksum */
			    checksum += ((uint8_t*)message->data)[i];
			  }
			  // formatPayload(dptr, message);

			  uint8_t checksum1 = (uint8_t) ((checksum >> 8) & 0xFF);
			  uint8_t checksum0 = (uint8_t) (checksum & 0xFF);
			  *dptr++ = checksum1;
			  *dptr++ = checksum0;
			  return dptr - raw;
		  }
		  /* return 7; */
		  break;
	}


	uint8_t checksum = (uint8_t)(-XBUS_MASTERDEVICE);
  /*//// No change for UM7, add address */
	*dptr = message->mid;
	checksum -= *dptr++;

	uint16_t length = messageLength(message);

	if (length < XBUS_EXTENDED_LENGTH)
	{
		*dptr = length;
		checksum -= *dptr++;
	}
	else
	{
		*dptr = XBUS_EXTENDED_LENGTH;
		checksum -= *dptr++;
		*dptr = length >> 8;
		checksum -= *dptr++;
		*dptr = length & 0xFF;
		checksum -= *dptr++;
	}

	formatPayload(dptr, message);
	for (int i = 0; i < length; ++i)
	{
		checksum -= *dptr++;
	}
	*dptr++ = checksum;

	return dptr - raw;
}


/*!
 * \brief Get a pointer to the data corresponding to \a id.
 * \param id The data identifier to find in the message.
 * \param data Pointer to the raw message payload.
 * \param dataLength The length of the payload in bytes.
 * \returns Pointer to data item, or NULL if the identifier is not present in
 * the message.
 */
static uint8_t const* getPointerToData(enum XsDataIdentifier id, uint8_t const* data, uint16_t dataLength)
{
	uint8_t const* dptr = data;
	while (dptr < data + dataLength)
	{
		uint16_t itemId;
		uint8_t itemSize;
		dptr = XbusUtility_readU16(&itemId, dptr);
		dptr = XbusUtility_readU8(&itemSize, dptr);

		if (id == itemId)
			return dptr;

		dptr += itemSize;
	}
	return NULL;
}

/*!
 * \brief Read a number of floats from a message payload.
 * \param out Pointer to where to output data.
 * \param raw Pointer to the start of the raw float data.
 * \param floats The number of floats to read.
 */
static void readFloats(float* out, uint8_t const* raw, uint8_t floats)
{
	for (int i = 0; i < floats; ++i)
	{
		raw = XbusUtility_readU32((uint32_t*)&out[i], raw);
	}
}

/*!
 * \brief Get a data item from an XMID_MtData2 Xbus message.
 * \param item Pointer to where to store the data.
 * \param id The data identifier to get.
 * \param message The message to read the data item from.
 * \returns true if the data item is found in the message, else false.
 */
bool XbusMessage_getDataItem(void* item, enum XsDataIdentifier id, struct XbusMessage const* message)
{
	uint8_t const* raw = getPointerToData(id, message->data, message->length);
	if (raw)
	{
		switch (id)
		{
			case XDI_PacketCounter:
				raw = XbusUtility_readU16(item, raw);
				break;

			case XDI_SampleTimeFine:
			case XDI_StatusWord:
				raw = XbusUtility_readU32(item, raw);
				break;

			case XDI_Quaternion:
			case XDI_DeltaQ:
				readFloats(item, raw, 4);
				break;

			case XDI_DeltaV:
			case XDI_Acceleration:
			case XDI_RateOfTurn:
			case XDI_MagneticField:
				readFloats(item, raw, 3);
				break;

			default:
				return false;
		}
		return true;
	}
	else
	{
		return false;
	}
}

/*!
 * \brief Get a string description for the passed data identifier.
 */
char const* XbusMessage_dataDescription(enum XsDataIdentifier id)
{
	switch (id)
	{
		case XDI_PacketCounter:
			return "Packet counter";

		case XDI_SampleTimeFine:
			return "Sample time fine";

		case XDI_Quaternion:
			return "Quaternion";

		case XDI_DeltaV:
			return "Velocity increment";

		case XDI_Acceleration:
			return "Acceleration";

		case XDI_RateOfTurn:
			return "Rate of turn";

		case XDI_DeltaQ:
			return "Orientation increment";

		case XDI_MagneticField:
			return "Magnetic field";

		case XDI_StatusWord:
			return "Status word";

		default:
			return "Unknown data type";
	}
}
