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

#include "xbusparser.h"
#include "xbusdef.h"
#include "xbusutility.h"
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
//#include "Debug.h"

/*! \brief XbusParser states. */
enum XbusParserState
{
	XBPS_Preamble,          /*!< \brief Looking for preamble. */
	XBPS_BusId,             /*!< \brief Waiting for bus ID. */
	XBPS_MessageId,         /*!< \brief Waiting for message ID. */
	XBPS_Length,            /*!< \brief Waiting for length. */
	XBPS_ExtendedLengthMsb, /*!< \brief Waiting for extended length MSB*/
	XBPS_ExtendedLengthLsb, /*!< \brief Waiting for extended length LSB*/
	XBPS_Payload,           /*!< \brief Reading payload. */
	XBPS_Checksum           /*!< \brief Waiting for checksum. */
};

/*! \brief UM7 states. */
enum XbusParserStateUM7
{
	STATE_ZERO,
	STATE_S,
	STATE_SN,
	STATE_SNP,
	STATE_PT,
	STATE_DATA,
	STATE_CHK1,
	STATE_CHK0
};


/*!
 * \brief Xbus Parser state structure.
 */
struct XbusParser
{
	/*! \brief Callbacks for memory management, and message handling. */
	struct XbusParserCallback callbacks;
	/*! \brief Storage for the current message being received. */
	struct XbusMessage currentMessage;
	/*! \brief The number of bytes of payload received for the current message. */
	uint16_t payloadReceived;
	/*! \brief The calculated checksum for the current message. */
	uint8_t checksum;
	/*! \brief The state of the parser. */
	enum XbusParserState state;
	/* Extra info for UM7 */
	uint8_t packet_type;
	bool packet_is_batch;
	bool packet_has_data;
	uint8_t batch_length;
	uint8_t checksum1;		// First byte of checksum
	uint8_t checksum0;
	unsigned short checksum10;
	unsigned short computed_checksum;
};

/*!
 * \brief Get the amount of memory needed for the XbusParser structure.
 */
size_t XbusParser_mem(void)
{
	return sizeof(struct XbusParser);
}

/*!
 * \brief Create a new XbusParser object.
 * \param callback Pointer to callback structure containing callback functions
 * for memory management and handling received messages.
 * \returns Pointer the new XbusParser structure.
 *
 * Uses malloc to allocate the memory required for the parser.
 */
struct XbusParser* XbusParser_create(struct XbusParserCallback const* callback)
{
	void* mem = pvPortMalloc(XbusParser_mem());
	if (mem)
	{
		return XbusParser_init(mem, callback);
	}
	return NULL;
}

/*!
 * \brief Frees an XbusParser structure allocated by XbusParser_create().
 */
void XbusParser_destroy(struct XbusParser* parser)
{
	vPortFree(parser);
}

/*!
 * \brief Initializes an XbusParser in the passed memory location.
 * \param parserMem Pointer to memory to use for storing parser state. Should
 * be at least as big as the value returned by XbusParser_mem().
 * \param callback Pointer to callback structure containing callback functions
 * for memory management and handling received messages.
 * \returns Initialized XbusParser structure.
 */
struct XbusParser* XbusParser_init(void* parserMem, struct XbusParserCallback const* callback)
{
	struct XbusParser* parser = (struct XbusParser*)parserMem;
	parser->state = XBPS_Preamble;
	parser->callbacks.allocateBuffer = callback->allocateBuffer;
	parser->callbacks.deallocateBuffer = callback->deallocateBuffer;
	parser->callbacks.handleMessage = callback->handleMessage;
	parser->callbacks.parameter = callback->parameter;
	return parser;
}

/*!
 * \brief Parse an XMID_DeviceId message to extract the device ID value.

 * Replaces the raw Xbus message data with the device ID.
 */
static void parseDeviceId(struct XbusParser* parser, uint8_t const* rawData)
{
	uint32_t* deviceId = parser->callbacks.allocateBuffer(sizeof(uint32_t));
	if (deviceId)
	{
		XbusUtility_readU32(deviceId, rawData);
		parser->currentMessage.data = deviceId;
		parser->currentMessage.length = 1;
	}
	else
	{
		parser->currentMessage.data = NULL;
	}
}

static void parseDefault(struct XbusParser* parser, uint8_t const* rawData)
{
	uint32_t* deviceId = parser->callbacks.allocateBuffer(sizeof(uint32_t));
	if (deviceId)
	{
		XbusUtility_readU32(deviceId, rawData);
		parser->currentMessage.data = deviceId;
		parser->currentMessage.length = 1;
	}
	else
	{
		parser->currentMessage.data = NULL;
	}
}

/*!
 * \brief Parse an XMID_OutputConfig message.
 *
 * Replaces the raw Xbus message data with an array of OutputConfiguration
 * structures.
 */
static void parseOutputConfig(struct XbusParser* parser, uint8_t const* rawData)
{
	uint8_t fields = parser->currentMessage.length / 4;
	struct OutputConfiguration* conf = parser->callbacks.allocateBuffer(fields * sizeof(struct OutputConfiguration));
	if (conf)
	{
		parser->currentMessage.data = conf;
		parser->currentMessage.length = fields;

		for (int i = 0; i < fields; ++i)
		{
			rawData = XbusUtility_readU16((uint16_t*)&conf->dtype, rawData);
			rawData = XbusUtility_readU16(&conf->freq, rawData);
			++conf;
		}
	}
	else
	{
		parser->currentMessage.data = NULL;
	}
}

/*!
 * \brief Converts raw Xbus payload data to native structures if possible.
 *
 * Raw data payloads are converted to native data structures and the
 * message data pointer is changed to point to the native structure.
 * The raw data is automatically deallocated.
 */
static void parseMessagePayload(struct XbusParser* parser)
{
	uint8_t const* const rawData = parser->currentMessage.data;
	switch (parser->currentMessage.mid)
	{
		default:
			;
			parseDefault(parser, rawData);
			// Leave parsing and memory management to user code
			return;

		case XMID_DeviceId:
			parseDeviceId(parser, rawData);
			break;

		case XMID_OutputConfig:
			parseOutputConfig(parser, rawData);
			break;
	}

	if (rawData)
		parser->callbacks.deallocateBuffer(rawData);
}

/*!
 * \brief Prepare for receiving a message payload.
 *
 * Requests a memory area to store the received data to using the
 * registered callbacks.
 */
void prepareForPayload(struct XbusParser* parser)
{
	parser->payloadReceived = 0;
	parser->currentMessage.data = parser->callbacks.allocateBuffer(parser->currentMessage.length);
}

/*!
 * \brief Parse a byte of data from a motion tracker.
 *
 * When a complete message is received the user will be notified by a call
 * to the handleMessage() callback function.
 */
void XbusParser_parseByte(struct XbusParser* parser, const uint8_t byte)
{
	/* Parsing an UM7 message */
	switch (parser->state)
	{
	  case STATE_ZERO:
	  	if (byte == 's')
	  	{
	  		parser->computed_checksum = byte;
	  		parser->state = STATE_S;
	  	}
	  	else
	  	{
        parser->computed_checksum = 0;
        parser->state = STATE_ZERO;
	  	}
	  	break;
	  case STATE_S:
	  	if (byte == 'n')
	  	{
	  		parser->computed_checksum += byte;
	  		parser->state = STATE_SN;
	  	}
	  	else
	  	{
	  		parser->computed_checksum = 0;
	  		parser->state = STATE_ZERO;
	  	}
	  	break;
	  case STATE_SN:
	  	if (byte == 'p')
	  	{
	  		parser->computed_checksum += byte;
	  		parser->state = STATE_SNP;
	  	}
	  	else
	  	{
	  		parser->computed_checksum = 0;
	  		parser->state = STATE_ZERO;
	  	}
	  	break;
	  case STATE_SNP:
	  	parser->computed_checksum += byte;
	  	parser->state = STATE_PT;
      parser->packet_type = byte;
      parser->packet_has_data = (parser->packet_type >> 7) & 0x01;
      parser->packet_is_batch = (parser->packet_type >> 6) & 0x01;
      parser->batch_length    = (parser->packet_type >> 2) & 0x0F;
      parser->currentMessage.read_write = parser->packet_has_data;
      parser->currentMessage.is_batch = parser->packet_is_batch;
      parser->currentMessage.batch_length = parser->batch_length;
		  if (parser->packet_has_data) {
			  if (parser->packet_is_batch) {
			  	parser->currentMessage.length = 4 * parser->batch_length;	// Each data packet is 4 bytes long
			  } else {
			  	parser->currentMessage.length = 4;
			  }
		  } else {
			  parser->currentMessage.length = 0;
			  parser->currentMessage.data = NULL;
		  }
		  break;
	  case STATE_PT:
	  	parser->computed_checksum += byte;
	  	/* Added UM7 registers on XsMessageId */
	  	parser->currentMessage.mid = (enum XsMessageId)byte;
		  if (parser->currentMessage.length == 0) {
			  parser->state = STATE_CHK1;
		  } else {
			  prepareForPayload(parser);
			  parser->state = STATE_DATA;
		  }
		  break;
	  case STATE_DATA:
	  	parser->computed_checksum += byte;
	  	if (parser->currentMessage.data)
	  	{
	  	  ((uint8_t*)parser->currentMessage.data)[parser->payloadReceived] = byte;
	  	}
	  	if (++parser->payloadReceived == parser->currentMessage.length)
	  	{
	  	  parser->state = STATE_CHK1;
	  	}
	  	break;
	  case STATE_CHK1:
	  	parser->state = STATE_CHK0;
	  	parser->checksum1 = byte;
	  	break;
	  case STATE_CHK0:
	  	parser->checksum0 = byte;
	  	parser->checksum10  = parser->checksum1 << 8;	// Combine checksum1 and checksum0
	  	parser->checksum10 |= parser->checksum0;
	  	if (parser->checksum10 == parser->computed_checksum)
	  	{
	  	  // parseMessagePayload(parser);
	  		parser->callbacks.handleMessage(parser->callbacks.parameter, &parser->currentMessage);
	    }
	  	else if (parser->currentMessage.data)
	  	{
	  		parser->callbacks.deallocateBuffer(parser->currentMessage.data);
	  	}
	  	parser->state = STATE_ZERO;
	  	break;
	}
#if 0 // Original message parser
	switch (parser->state)
	{
		case XBPS_Preamble:
			if (byte == XBUS_PREAMBLE)
			{
				parser->checksum = 0;
				parser->state = XBPS_BusId;
			}
			break;

		case XBPS_BusId:
			parser->checksum += byte;
			parser->state = XBPS_MessageId;
			break;

		case XBPS_MessageId:
			parser->checksum += byte;
			parser->currentMessage.mid = (enum XsMessageId)byte;
			parser->state = XBPS_Length;
			break;

		case XBPS_Length:
			parser->checksum += byte;
			if (byte == XBUS_NO_PAYLOAD)
			{
				parser->currentMessage.length = byte;
				parser->currentMessage.data = NULL;
				parser->state = XBPS_Checksum;
			}
			else if (byte < XBUS_EXTENDED_LENGTH)
			{
				parser->currentMessage.length = byte;
				prepareForPayload(parser);
				parser->state = XBPS_Payload;
			}
			else
			{
				parser->state = XBPS_ExtendedLengthMsb;
			}
			break;

		case XBPS_ExtendedLengthMsb:
			parser->checksum += byte;
			parser->currentMessage.length = ((uint16_t)byte) << 8;
			parser->state = XBPS_ExtendedLengthLsb;
			break;

		case XBPS_ExtendedLengthLsb:
			parser->checksum += byte;
			parser->currentMessage.length |= byte;
			prepareForPayload(parser);
			parser->state = XBPS_Payload;
			break;

		case XBPS_Payload:
			parser->checksum += byte;
			if (parser->currentMessage.data)
			{
				((uint8_t*)parser->currentMessage.data)[parser->payloadReceived] = byte;
			}
			if (++parser->payloadReceived == parser->currentMessage.length)
			{
				parser->state = XBPS_Checksum;
			}
			break;

		case XBPS_Checksum:
			parser->checksum += byte;
			if ((parser->checksum == 0) &&
					((parser->currentMessage.length == 0) ||
					 parser->currentMessage.data))
			{
				parseMessagePayload(parser);
				parser->callbacks.handleMessage(parser->callbacks.parameter, &parser->currentMessage);
			}
			else if (parser->currentMessage.data)
			{
				parser->callbacks.deallocateBuffer(parser->currentMessage.data);
			}
			parser->state = XBPS_Preamble;
			break;
	}
#endif
}

/*!
 * \brief Parse a buffer of data received from a motion tracker.
 */
void XbusParser_parseBuffer(struct XbusParser* parser, uint8_t const* buf, size_t bufSize)
{
	for (size_t i = 0; i < bufSize; ++i)
	{
		XbusParser_parseByte(parser, buf[i]);
	}
}

