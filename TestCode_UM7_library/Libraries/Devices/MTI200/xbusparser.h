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

#ifndef __XBUSPARSER_H
#define __XBUSPARSER_H

#include <stddef.h>
#include <stdint.h>
#include "xbusmessage.h"

#ifdef __cplusplus
extern "C" {
#endif

struct XbusParser;

/*!
 * \brief Callback function structure for use with the XbusParser.
 */
struct XbusParserCallback
{
	/*!
	 * \brief Allocate a buffer for message reception.
	 * \param bufSize The size of the buffer to allocate.
	 * \returns Pointer to buffer to use for message reception, or NULL if
	 * a buffer cannot be allocated.
	 *
	 * \note It is the resposibility of the user to deallocate the message
	 * data buffers pointed to by XbusMessage structures passed to the
	 * handleMessage() callback function.
	 */
	void* (*allocateBuffer)(size_t bufSize);

	/*!
	 * \brief Deallocate a buffer that was previously allocated by a call to
	 * allocateBuffer.
	 */
	void (*deallocateBuffer)(void const* buffer);

	/*!
	 * \brief Handle a received message.
	 *
	 * \note If the passed XbusMessage structure has a non-null data pointer
	 * then it is the responsibility of the user to free this once handling
	 * of the message is complete.
	 */
	void (*handleMessage)(void * param, struct XbusMessage const* message);
	void * parameter;
};

size_t XbusParser_mem(void);
struct XbusParser* XbusParser_create(struct XbusParserCallback const* callback);
void XbusParser_destroy(struct XbusParser* parser);
struct XbusParser* XbusParser_init(void* parserMem, struct XbusParserCallback const* callback);

void XbusParser_parseByte(struct XbusParser* parser, uint8_t byte);
void XbusParser_parseBuffer(struct XbusParser* parser, uint8_t const* buf, size_t bufSize);

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __XBUSPARSER_H
