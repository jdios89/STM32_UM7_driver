/* Copyright (C) 2019-2020 Juan de Dios Flores Mendez. All rights reserved.
 * Based on code by Thomas Jespersen
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
 
#ifndef DEVICES_MTI200_H
#define DEVICES_MTI200_H

#include "IMU.h"
#include "UART.h"

/* Include Xsens library */
#include "xbusparser.h"
#include "xbusmessage.h"
#include "xsdeviceid.h"
#include "xbusdef.h"


class MTI200 : public IMU
{
	private:
		const int RESPONSE_QUEUE_LENGTH = 4;
		const int MESSAGE_QUEUE_LENGTH = 4;

	public:
		typedef struct LastMeasurement_t {
			uint16_t PackageCounter;
			float Time;
			float dt;
			float Quaternion[4];
			float DeltaQ[4];
			float Accelerometer[3];
			float Gyroscope[3];
			float Magnetometer[3];
			uint32_t Status;
		};

	public:
		MTI200(UART * uart);
		~MTI200();

		uint32_t WaitForNewData(uint32_t xTicksToWait = portMAX_DELAY);
		void Get(Measurement_t& measurement);
		void GetEstimates(Estimates_t& estimates);

		bool Configure(uint32_t SampleRate);
		bool ConfigureUM7(uint32_t SampleRate);
		LastMeasurement_t GetLastMeasurement();

	private:
		UART * _uart;
		SemaphoreHandle_t _interruptSemaphore;
		SemaphoreHandle_t _resourceSemaphore;
		SemaphoreHandle_t _dataSemaphore;
		XbusParser * _xbusParser;
		QueueHandle_t _messageQueue;
		QueueHandle_t _responseQueue;
		LastMeasurement_t LastMeasurement;

		static void UART_Callback(void * param, uint8_t * buffer, uint32_t bufLen);

		static void* allocateMessageData(size_t bufSize);
		static void deallocateMessageData(void const* buffer);
		static void mtMessageHandler(void * param, struct XbusMessage const* message);
	
		void sendMessage(XbusMessage const* m);
		void sendMessageUM7(XbusMessage const* m);
		XbusMessage const * doTransaction(XbusMessage const* m);
		XbusMessage const * doTransactionUM7(XbusMessage const* m);
		void dumpResponse(XbusMessage const* response);

		bool sendCommand(XsMessageId cmdId);
		bool sendCommandUM7(XsMessageId cmdId);
		uint32_t readDeviceId(void);
	public: /*for testing make public */
		uint32_t getFirmwareRevisionUM7(void);
		bool resetEKFUM7(void);
	private:
		bool setOutputConfiguration(OutputConfiguration const* conf, uint8_t elements);
		bool setOutputRateUM7(uint8_t rate);
		bool configureMotionTracker(uint32_t SampleRate);
		bool configureMotionTrackerUM7(uint8_t SampleRate);

		bool waitForWakeup(uint32_t timeout);
		void sendWakeupAck(void);

		XbusMessage GetMessage(uint32_t timeout = portMAX_DELAY);
		void printMessageData(XbusMessage const* message);

		void parseMTData2Message(XbusMessage const* message);
		void parseMTData2MessageUM7(XbusMessage const* message);

};
	
	
#endif
