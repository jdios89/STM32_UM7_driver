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

 
#include "MTI200.h"
#include "UART.h"
#include "Debug.h"
#include "cmsis_os.h"
#include "Quaternion.h"
#include <algorithm>

MTI200::MTI200(UART * uart) : _uart(uart)
{
	_interruptSemaphore = xSemaphoreCreateBinary();
	if (_interruptSemaphore == NULL) {
		ERROR("Could not create MTI200 interrupt semaphore");
		return;
	}
	vQueueAddToRegistry(_interruptSemaphore, "MTI200 Interrupt");

	_resourceSemaphore = xSemaphoreCreateBinary();
	if (_resourceSemaphore == NULL) {
		ERROR("Could not create MTI200 resource semaphore");
		return;
	}
	vQueueAddToRegistry(_resourceSemaphore, "MTI200 Resource");
	xSemaphoreGive( _resourceSemaphore ); // give the semaphore the first time

	_dataSemaphore = xSemaphoreCreateBinary();
	if (_dataSemaphore == NULL) {
		ERROR("Could not create MTI200 data semaphore");
		return;
	}
	vQueueAddToRegistry(_dataSemaphore, "MTI200 Data");
	xSemaphoreGive( _dataSemaphore ); // give the semaphore the first time

	_messageQueue = xQueueCreate( MESSAGE_QUEUE_LENGTH, sizeof(XbusMessage *) );
	if (_messageQueue == NULL) {
		ERROR("Could not create MTI200 message queue");
		return;
	}
	vQueueAddToRegistry(_messageQueue, "MTI200 Messages");

	_responseQueue = xQueueCreate( RESPONSE_QUEUE_LENGTH, sizeof(XbusMessage *) );
	if (_responseQueue == NULL) {
		ERROR("Could not create MTI200 response queue");
		return;
	}
	vQueueAddToRegistry(_responseQueue, "MTI200 Response");

	XbusParserCallback xbusCallback = {};
	xbusCallback.allocateBuffer = &allocateMessageData;
	xbusCallback.deallocateBuffer = &deallocateMessageData;
	xbusCallback.handleMessage = &mtMessageHandler;
	xbusCallback.parameter = (void *)this;

	_xbusParser = XbusParser_create(&xbusCallback);

	if (_uart) {
		_uart->RegisterRXcallback(&UART_Callback, (void *)this);
	}

	/* Reset last measurement and make the quaternion valid if requested before getting first message */
	memset(&LastMeasurement, 0, sizeof(LastMeasurement_t));
	LastMeasurement.Quaternion[0] = 1;
}

MTI200::~MTI200()
{
	if (_interruptSemaphore) {
		vQueueUnregisterQueue(_interruptSemaphore);
		vSemaphoreDelete(_interruptSemaphore);
	}
	if (_resourceSemaphore) {
		vQueueUnregisterQueue(_resourceSemaphore);
		vSemaphoreDelete(_resourceSemaphore);
	}
	if (_dataSemaphore) {
		vQueueUnregisterQueue(_dataSemaphore);
		vSemaphoreDelete(_dataSemaphore);
	}
	if (_messageQueue) {
		vQueueUnregisterQueue(_messageQueue);
		vQueueDelete(_messageQueue);
	}
	if (_responseQueue) {
		vQueueUnregisterQueue(_responseQueue);
		vQueueDelete(_responseQueue);
	}

	if (_uart) {
		_uart->DeregisterCallback();
	}
}

bool MTI200::Configure(uint32_t SampleRate)
{
	int tries = 3; // try 3 times to go into config - if no response, we might already be in config
	while (tries-- > 0) if (sendCommand(XMID_GotoConfig)) break;

	if (tries <= 0) { // if we did not get any config response, check if it was because we were already in config mode
		tries = 3;
		while (tries-- > 0) if (sendCommand(XMID_ReqDid)) break;
	} else {
		tries = 3;
	}

	/*while (tries-- > 0) if (sendCommand(XMID_Reset)) break;
	if (waitForWakeup(10000))
	{
		sendWakeupAck();
	}
	while (tries-- > 0) if (sendCommand(XMID_GotoConfig)) break;*/

	while (tries-- > 0) if (configureMotionTracker(SampleRate)) break;
	while (tries-- > 0) if (sendCommand(XMID_GotoMeasurement)) break;

	if (tries <= 0) {
		Debug::print("Failed configuring MTI200 IMU\n");
		return false;
	}
	return true;
}

bool MTI200::ConfigureUM7(uint32_t SampleRate)
{
	/* Test a send command to UM7 */
	int tries = 3; // try 3 times to go into config - if no response, we might already be in config
	// while (tries-- > 0) if (getFirmwareRevisionUM7()) break;
	while (tries-- > 0) if (configureMotionTrackerUM7(SampleRate)) break;

	if (tries <= 0) {
		Debug::print("Failed configuring UM7 IMU\n");
		return false;
	}
	return true;
}

uint32_t MTI200::WaitForNewData(uint32_t xTicksToWait) // blocking call
{
	if (!_interruptSemaphore)
		return pdFALSE;

	return xSemaphoreTake( _interruptSemaphore, ( TickType_t ) xTicksToWait );
}

void MTI200::Get(Measurement_t& measurement)
{
	memcpy(measurement.Accelerometer, LastMeasurement.Accelerometer, sizeof(LastMeasurement.Accelerometer));
	memcpy(measurement.Gyroscope, LastMeasurement.Gyroscope, sizeof(LastMeasurement.Gyroscope));
	memcpy(measurement.Magnetometer, LastMeasurement.Magnetometer, sizeof(LastMeasurement.Magnetometer));
}

void MTI200::UART_Callback(void * param, uint8_t * buffer, uint32_t bufLen)
{
	MTI200 * mti200 = (MTI200 *)param;
	if (!mti200) return;

	XbusParser_parseBuffer(mti200->_xbusParser, buffer, bufLen);
}

/*!
 * \brief Allocate message data buffer from the message data pool.
 */
void* MTI200::allocateMessageData(size_t bufSize)
{
	if (!bufSize) return 0;
	return (void *)pvPortMalloc(bufSize);
}

/*!
 * \brief Deallocate message data previously allocated from the message
 * data pool.
 */
void MTI200::deallocateMessageData(void const* buffer)
{
	if (!buffer) return;
	vPortFree((void *)buffer);
}

/*!
 * \brief XbusParser callback function to handle received messages.
 * \param message Pointer to the last received message.
 *
 * In this example received messages are copied into one of two message
 * queues for later handling by the main thread. Data messages are put
 * in one queue, while all other responses are placed in the second queue.
 * This is done so that data and other messages can be handled separately
 * by the application code.
 */
void MTI200::mtMessageHandler(void * param, struct XbusMessage const* message)
{ /* TODO: Add message parser */
	MTI200 * mti200 = (MTI200 *)param;
	if (!mti200) return;
	if (message->mid >= DREG_HEALTH && message->mid <= DREG_GYRO_BIAS_Z ) { // Message queue disabled - parse data immediately
		switch (message->mid) {
		case DREG_GYRO_PROC_X: /* Parse first address */
			mti200->parseMTData2MessageUM7(message);
			break;
		case DREG_QUAT_AB: /* Parse second address */
			mti200->parseMTData2MessageUM7(message);
			break;
		case DREG_EULER_PHI_THETA: /* Parse third address */
			mti200->parseMTData2MessageUM7(message);
			break;
		}
		// mti200->parseMTData2Message(message); // Parse Data objects
		deallocateMessageData(message->data);
	} else { // other type of message, put into response queue
		XbusMessage * msgPtr = (XbusMessage *) pvPortMalloc(sizeof(XbusMessage));
		if (msgPtr) {
			memcpy(msgPtr, message, sizeof(XbusMessage));
			if (xQueueSend(mti200->_responseQueue, (void * )&msgPtr,
					(TickType_t ) 0) != pdPASS) {
				deallocateMessageData(message->data); // failed putting message into queue - deallocate message data memory
			}
		} else {
			deallocateMessageData(message->data); // could not allocate memory for received Xbus message
		}
	}

#if 0 // USE MESSAGE MTI HANDLER
	if (message->mid == XMID_MtData2) { // Message queue disabled - parse data immediately
		mti200->parseMTData2Message(message);
		deallocateMessageData(message->data);
	} else { // other type of message, put into response queue
		XbusMessage * msgPtr = (XbusMessage *)pvPortMalloc(sizeof(XbusMessage));
		if (msgPtr) {
			memcpy(msgPtr, message, sizeof(XbusMessage));
			if (xQueueSend(mti200->_responseQueue, (void *)&msgPtr, (TickType_t) 0) != pdPASS ) {
				deallocateMessageData(message->data); // failed putting message into queue - deallocate message data memory
			}
		} else {
			deallocateMessageData(message->data); // could not allocate memory for received Xbus message
		}
	}
#endif

#if 0 // USE MESSAGE QUEUE BUFFER
	XbusMessage * msgPtr = (XbusMessage *)pvPortMalloc(sizeof(XbusMessage));
	if (msgPtr) {
		memcpy(msgPtr, message, sizeof(XbusMessage));
		if (message->mid == XMID_MtData2)
		{
			if (xQueueSend(mti200->_messageQueue, (void *)&msgPtr, (TickType_t) 0) != pdPASS ) {
				deallocateMessageData(message->data); // failed putting message into queue - deallocate message data memory
			}
		}
		else
		{
			if (xQueueSend(mti200->_responseQueue, (void *)&msgPtr, (TickType_t) 0) != pdPASS ) {
				deallocateMessageData(message->data); // failed putting message into queue - deallocate message data memory
			}
		}
	} else {
		deallocateMessageData(message->data); // could not allocate memory for received Xbus message
	}
#endif
}

/*!
 * \brief Send a message to the MT
 *
 * This function formats the message data and writes this to the MT serial
 * port. It does not wait for any response.
 */
void MTI200::sendMessage(XbusMessage const* m)
{
	uint8_t buf[64];
	size_t rawLength = XbusMessage_format(buf, m, XLLF_Uart);
	if (_uart) {
		_uart->Write(buf, rawLength);
	}
}
void MTI200::sendMessageUM7(XbusMessage const* m)
{ /* Added format for UM7 */
	uint8_t buf[64];
	size_t rawLength = XbusMessage_format(buf, m, XLLF_UM7);
	if (_uart) {
		_uart->Write(buf, rawLength);
	}
}

/*!
 * \brief Send a message to the MT and wait for a response.
 * \returns Response message from the MT, or NULL is no response received
 * within 500ms.
 *
 * Blocking behaviour is implemented by waiting for a response to be written
 * to the response queue by the XbusParser.
 */
XbusMessage const * MTI200::doTransaction(XbusMessage const* m)
{
	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	sendMessage(m);

	// Wait for the transmission to finish
	XbusMessage * msgPtr = NULL;
	xQueueReceive( _responseQueue, &msgPtr, ( TickType_t ) 500 ); // wait for response

	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back

	return msgPtr;
}
XbusMessage const * MTI200::doTransactionUM7(XbusMessage const* m)
{
	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	sendMessageUM7(m);

	// Wait for the transmission to finish
	XbusMessage * msgPtr = NULL;
	xQueueReceive( _responseQueue, &msgPtr, ( TickType_t ) 500 ); // wait for response

	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back

	return msgPtr;
}

/*!
 * \brief RAII object to manage message memory deallocation.
 *
 * Will automatically free the memory used by an XbusMessage when going out
 * of scope.
 */
class XbusMessageMemoryManager
{
	public:
		XbusMessageMemoryManager(XbusMessage const* message)
			: m_message(message)
		{
		}

		~XbusMessageMemoryManager()
		{
			if (m_message)
			{
				if (m_message->data)
					vPortFree((void *)m_message->data);
				vPortFree((void *)m_message);
			}
		}

	private:
		XbusMessage const* m_message;
};

/*!
 * \brief Dump information from a message to the PC serial port.
 */
void MTI200::dumpResponse(XbusMessage const* response)
{
	switch (response->mid)
	{
		case XMID_GotoConfigAck:
			Debug::print("MTI200 went to config mode.\r\n");
			break;

		case XMID_GotoMeasurementAck:
			Debug::print("MTI200 went to measurement mode.\r\n");
			break;

		case XMID_ResetAck:
			Debug::print("MTI200 has been reset.\r\n");
			break;

		case XMID_Error:
			Debug::print("MTI200 error!\r\n");
			break;

		default:
			Debug::printf("MTI200 response MID=%X, length=%d\r\n", response->mid, response->length);
			break;
	}
}

/*!
 * \brief Send a command to the MT and wait for a response.
 * \param cmdId The XsMessageId of the command to send.
 *
 * Commands are simple messages without and payload data.
 */
bool MTI200::sendCommand(XsMessageId cmdId)
{
	XbusMessage m = {cmdId};
	XbusMessage const * response = doTransaction(&m);
	XbusMessageMemoryManager janitor(response);

	if (response)
	{
		dumpResponse(response);
		return true;
	}
	else
	{
		Debug::print("Timeout waiting for response.\r\n");
		return false;
	}
}

bool MTI200::sendCommandUM7(XsMessageId cmdId)
{
	/* TODO: Change address here */
	XbusMessage m = {cmdId};
	XbusMessage const * response = doTransaction(&m);
	XbusMessageMemoryManager janitor(response);

	if (response)
	{
		dumpResponse(response);
		return true;
	}
	else
	{
		Debug::print("Timeout waiting for response.\r\n");
		return false;
	}
}

/*!
 * \brief Read the device ID of the motion tracker.
 */
uint32_t MTI200::readDeviceId(void)
{
	XbusMessage reqDid = {XMID_ReqDid};
	XbusMessage const * didRsp = doTransaction(&reqDid);
	XbusMessageMemoryManager janitor(didRsp);
	uint32_t deviceId = 0;
	if (didRsp)
	{
		if (didRsp->mid == XMID_DeviceId)
		{
			deviceId = *(uint32_t*)didRsp->data;
		}
	}
	return deviceId;
}

bool MTI200::resetEKFUM7(void)
{
	XbusMessage getFw = {RESET_EKF};
	XbusMessage const * didRsp = doTransactionUM7(&getFw);
	XbusMessageMemoryManager janitor(didRsp);
	bool reply = false;
	if (didRsp)
	{
		if (didRsp->mid == RESET_EKF)
		{
			reply = true;
		}
	}
	return reply;
}
uint32_t MTI200::getFirmwareRevisionUM7(void)
{
	XbusMessage getFw = {GET_FW_REVISION};
	XbusMessage const * didRsp = doTransactionUM7(&getFw);
	XbusMessageMemoryManager janitor(didRsp);
	uint32_t deviceId = 0;
	if (didRsp)
	{
		if (didRsp->mid == GET_FW_REVISION)
		{
			deviceId = *(uint32_t*)didRsp->data;
		}
	}
	return deviceId;
}

bool MTI200::setOutputRateUM7(uint8_t rate)
{
  uint8_t elements = 4;
	uint8_t read_write = 1;
	uint8_t is_batch = 0;
  uint8_t batch_length = 0;
  uint8_t conf[] = {0x0, 0x0, 0x0, rate};
	XbusMessage outputConfMsg = { CREG_COM_RATES4, elements, (void*) conf,
			read_write, is_batch, batch_length };
	XbusMessage const* outputConfRsp = doTransactionUM7(&outputConfMsg);
	XbusMessageMemoryManager janitor(outputConfRsp);

	bool first_reply = false;

	if (outputConfRsp) {
		if (outputConfRsp->mid == CREG_COM_RATES4) {
			Debug::print("Output configuration set to:\r\n");
			/*
			OutputConfiguration* conf = (OutputConfiguration*) outputConfRsp->data;
			for (int i = 0; i < outputConfRsp->length; ++i) {
				Debug::printf("\t%s: %d Hz\r\n",
						XbusMessage_dataDescription(conf->dtype), conf->freq);
				++conf;
			}
			*/
			first_reply = true;
		} else {
			dumpResponse(outputConfRsp);
		}
	} else {
		Debug::print("Failed to set output configuration.\r\n");
	}
	osDelay(10);
	conf[0] = rate;
	conf[1] = rate;
	conf[2] = 0x0;
	conf[3] = 0x0;
	XbusMessage outputConfMsg2 = { CREG_COM_RATES5, elements, (void*) conf,
				read_write, is_batch, batch_length };
	XbusMessage const* outputConfRsp2 = doTransactionUM7(&outputConfMsg2);
	XbusMessageMemoryManager janitor2(outputConfRsp2);
	if (outputConfRsp2) {
		if (outputConfRsp2->mid == CREG_COM_RATES5) {
			Debug::print("Output configuration set to:\r\n");
			/*
			OutputConfiguration* conf2 = (OutputConfiguration*) outputConfRsp2->data;
			for (int i = 0; i < outputConfRsp2->length; ++i) {
				Debug::printf("\t%s: %d Hz\r\n",
						XbusMessage_dataDescription(conf2->dtype), conf2->freq);
				++conf2;
			}
			*/
			return first_reply & true;
		} else {
			dumpResponse(outputConfRsp2);
		}
	} else {
		Debug::print("Failed to set output configuration.\r\n");
	}
	return false;
}

/*!
 * \brief Sets MT output configuration.
 * \param conf Pointer to an array of OutputConfiguration elements.
 * \param elements The number of elements in the configuration array.
 *
 * The response from the device indicates the actual values that will
 * be used by the motion tracker. These may differ from the requested
 * parameters as the motion tracker validates the requested parameters
 * before applying them.
 */
bool MTI200::setOutputConfiguration(OutputConfiguration const* conf, uint8_t elements)
{
	XbusMessage outputConfMsg = {XMID_SetOutputConfig, elements, (void*)conf};
	XbusMessage const* outputConfRsp = doTransaction(&outputConfMsg);
	XbusMessageMemoryManager janitor(outputConfRsp);
	if (outputConfRsp)
	{
		if (outputConfRsp->mid == XMID_OutputConfig)
		{
			Debug::print("Output configuration set to:\r\n");
			OutputConfiguration* conf = (OutputConfiguration*)outputConfRsp->data;
			for (int i = 0; i < outputConfRsp->length; ++i)
			{
				Debug::printf("\t%s: %d Hz\r\n", XbusMessage_dataDescription(conf->dtype), conf->freq);
				++conf;
			}
			return true;
		}
		else
		{
			dumpResponse(outputConfRsp);
		}
	}
	else
	{
		Debug::print("Failed to set output configuration.\r\n");
	}
	return false;
}

/*!
 * \brief Sets the motion tracker output configuration based on the function
 * of the attached device.
 *
 * The output configuration depends on the type of MTi-1 device connected.
 * An MTI-1 (IMU) device does not have an onboard orientation filter so
 * cannot output quaternion data, only inertial and magnetic measurement
 * data.
 * MTi-2 and MTi-3 devices have an onboard filter so can send quaternions.
 */
bool MTI200::configureMotionTracker(uint32_t SampleRate)
{
	uint32_t deviceId = readDeviceId();

	if (deviceId)
	{
		Debug::printf("Found device with ID: %08X.\r\n", deviceId);

		DeviceFunction function = XsDeviceId_getFunction(deviceId);
		Debug::printf("Device is an MTi-%d: %s.\r\n", function, XsDeviceId_functionDescription(function));

		if (function == DF_IMU) {
			Debug::print("Detected MTI device do not support estimate output!\n");
			return 0;
		}

		if ((968 * SampleRate) > _uart->BaudRate) Debug::print("Note: MTI sample rate might be too high to ensure consistent sampling without data loss\n");
		OutputConfiguration conf[] = {
			{XDI_PacketCounter, 65535}, // 65535 == fastest rate possible
			{XDI_SampleTimeFine, 65535},
			{XDI_Quaternion, SampleRate},
			//{XDI_DeltaQ, SampleRate}, // no need to include the DeltaQ, since this is just generated from the RateOfTurn and corresponds to the movement over a period of 1/(400 Hz)
			{XDI_Acceleration, SampleRate},
			{XDI_RateOfTurn, SampleRate},
			{XDI_MagneticField, std::min<uint32_t>(SampleRate, 100)},
			{XDI_StatusWord, 65535}
		};

		return setOutputConfiguration(conf,
				sizeof(conf) / sizeof(OutputConfiguration));
	}

	return false;
}
bool MTI200::configureMotionTrackerUM7(uint8_t SampleRate)
{
	// uint32_t firmwarerevision = getFirmwareRevisionUM7();

	if ((968 * SampleRate) > _uart->BaudRate)
		Debug::print(
				"Note: UM7 sample rate might be too high to ensure consistent sampling without data loss\n");
	uint8_t conf = SampleRate;

	return setOutputRateUM7(conf);

	return false;
}

/*!
 * \brief Wait for a wakeup message from the MTi.
 * \param timeout Time to wait to receive the wakeup message.
 * \return true if wakeup received within timeout, else false.
 *
 * The MTi sends an XMID_Wakeup message once it has completed its bootup
 * procedure. If this is acknowledged by an XMID_WakeupAck message then the MTi
 * will stay in configuration mode. Otherwise it will automatically enter
 * measurement mode with the stored output configuration.
 */
bool MTI200::waitForWakeup(uint32_t timeout)
{
	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	XbusMessage * msgPtr = NULL;
	xQueueReceive( _responseQueue, &msgPtr, ( TickType_t ) timeout ); // wait for wakeup

	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back

	if (msgPtr)
	{
		XbusMessageMemoryManager janitor(msgPtr);
		return msgPtr->mid == XMID_Wakeup;
	}
	return false;
}

/*!
 * \brief Send wakeup acknowledge message to MTi.
 *
 * Sending a wakeup acknowledge will cause the device to stay in configuration
 * mode instead of automatically transitioning to measurement mode with the
 * stored output configuration.
 */
void MTI200::sendWakeupAck(void)
{
	XbusMessage ack = {XMID_WakeupAck};
	sendMessage(&ack);
	Debug::print("MTI200 ready for operation.\r\n");
}


/*!
 * \brief Output the contents of a data message to the PC serial port.
 */
void MTI200::printMessageData(XbusMessage const* message)
{
	if (!message)
		return;

	Debug::print("MTData2:");
	uint16_t counter;
	if (XbusMessage_getDataItem(&counter, XDI_PacketCounter, message))
	{
		Debug::printf(" Packet counter: %5d", counter);
	}
	uint32_t sampleTimeFine;
	float time;
	if (XbusMessage_getDataItem(&sampleTimeFine, XDI_SampleTimeFine, message))
	{
		time = (float)sampleTimeFine / 10000.0f;
		Debug::printf(" Time: %.2f", time);
	}
	float ori[4];
	if (XbusMessage_getDataItem(ori, XDI_Quaternion, message))
	{
		Debug::printf(" Orientation: (% .3f, % .3f, % .3f, % .3f)", ori[0], ori[1],
				ori[2], ori[3]);
	}
	float acc[3];
	if (XbusMessage_getDataItem(acc, XDI_Acceleration, message))
	{
		Debug::printf(" Acceleration: (% .3f, % .3f, % .3f)", acc[0], acc[1], acc[2]);
	}
	float gyr[3];
	if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message))
	{
		Debug::printf(" Rate Of Turn: (% .3f, % .3f, % .3f)", gyr[0], gyr[1], gyr[2]);
	}
	float mag[3];
	if (XbusMessage_getDataItem(mag, XDI_MagneticField, message))
	{
		Debug::printf(" Magnetic Field: (% .3f, % .3f, % .3f)", mag[0], mag[1], mag[2]);
	}
	uint32_t status;
	if (XbusMessage_getDataItem(&status, XDI_StatusWord, message))
	{
		Debug::printf(" Status:%X", status);
	}
	Debug::print("\r\n");
}

XbusMessage MTI200::GetMessage(uint32_t timeout)
{
	XbusMessage msg;
	msg.data = 0;
	msg.length = 0;
	msg.mid = XMID_Undefined;

	XbusMessage * msgPtr = NULL;
	xQueueReceive( _messageQueue, &msgPtr, ( TickType_t ) timeout ); // check USB connection every 100 ms
	if (msgPtr) {
		memcpy(&msg, msgPtr, sizeof(XbusMessage));
		vPortFree((void *)msgPtr);
	}
	return msg;
}

void MTI200::parseMTData2Message(XbusMessage const* message)
{
	LastMeasurement_t tmp;

	if (!message)  return;

	xSemaphoreTake( _dataSemaphore, ( TickType_t ) portMAX_DELAY ); // take data semaphore to update data

	if (XbusMessage_getDataItem(&tmp.PackageCounter, XDI_PacketCounter, message)) {
		LastMeasurement.PackageCounter = tmp.PackageCounter;
	}

	uint32_t sampleTimeFine = 0;
	if (XbusMessage_getDataItem(&sampleTimeFine, XDI_SampleTimeFine, message)) {
		float time = (float)sampleTimeFine / 10000.0f;
		if (sampleTimeFine > 0)
			LastMeasurement.dt = time - LastMeasurement.Time;
		LastMeasurement.Time = time;
	}

	/* Note that measurements and estimates has to be rotated 180 degrees due to the mount/orientation of the Xsens IMU */
	if (XbusMessage_getDataItem(tmp.Quaternion, XDI_Quaternion, message)) {
		memcpy(LastMeasurement.Quaternion, tmp.Quaternion, sizeof(tmp.Quaternion));
		LastMeasurement.Quaternion[1] = -LastMeasurement.Quaternion[1];
		LastMeasurement.Quaternion[2] = -LastMeasurement.Quaternion[2];
	}

	if (XbusMessage_getDataItem(tmp.DeltaQ, XDI_DeltaQ, message)) {
		memcpy(LastMeasurement.DeltaQ, tmp.DeltaQ, sizeof(tmp.DeltaQ));
		LastMeasurement.DeltaQ[1] = -LastMeasurement.DeltaQ[1];
		LastMeasurement.DeltaQ[2] = -LastMeasurement.DeltaQ[2];
	}

	if (XbusMessage_getDataItem(tmp.Accelerometer, XDI_Acceleration, message)) {
		memcpy(LastMeasurement.Accelerometer, tmp.Accelerometer, sizeof(tmp.Accelerometer));
		LastMeasurement.Accelerometer[0] = -LastMeasurement.Accelerometer[0];
		LastMeasurement.Accelerometer[1] = -LastMeasurement.Accelerometer[1];
	}

	if (XbusMessage_getDataItem(tmp.Gyroscope, XDI_RateOfTurn, message)) {
		memcpy(LastMeasurement.Gyroscope, tmp.Gyroscope, sizeof(tmp.Gyroscope));
		LastMeasurement.Gyroscope[0] = -LastMeasurement.Gyroscope[0];
		LastMeasurement.Gyroscope[1] = -LastMeasurement.Gyroscope[1];
	}

	if (XbusMessage_getDataItem(tmp.Magnetometer, XDI_MagneticField, message)) {
		memcpy(LastMeasurement.Magnetometer, tmp.Magnetometer, sizeof(tmp.Magnetometer));
		LastMeasurement.Magnetometer[0] = -LastMeasurement.Magnetometer[0];
		LastMeasurement.Magnetometer[1] = -LastMeasurement.Magnetometer[1];
	}

	if (XbusMessage_getDataItem(&tmp.Status, XDI_StatusWord, message)) {
		LastMeasurement.Status = tmp.Status;
	}

	/* Successfully read all data - now overwrite LastMeasurement */
	/*xSemaphoreTake( _dataSemaphore, ( TickType_t ) portMAX_DELAY ); // take data semaphore to update data
	memcpy(&LastMeasurement, &tmp, sizeof(LastMeasurement_t));*/

	xSemaphoreGive( _dataSemaphore ); // give semaphore back
}

void MTI200::parseMTData2MessageUM7(XbusMessage const* message)
{
	LastMeasurement_t tmp;
	union {
		uint8_t byte_number[4];
		float float_number;
	} encodefloat;
	if (!message)  return;

	xSemaphoreTake( _dataSemaphore, ( TickType_t ) portMAX_DELAY ); // take data semaphore to update data
  bool copy_accmaggyro = false;
  bool copy_quat = false;
  bool copy_euler = false;

	switch (message->mid) {
	  case DREG_GYRO_PROC_X:
	  	if(message->is_batch){
	  		message->length;
			/* Get pointer to data */
	  	uint8_t const* dptr = (uint8_t const*) message->data;
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = (uint8_t) *dptr++;
			tmp.Gyroscope[0] = encodefloat.float_number;

			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Gyroscope[1] = encodefloat.float_number;

			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Gyroscope[2] = encodefloat.float_number;

			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;;
			/*
			GRYO_PROC_TIME = encodefloat.float_number;
			*/
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Accelerometer[0] = encodefloat.float_number;
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Accelerometer[1] = encodefloat.float_number;
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Accelerometer[2] = encodefloat.float_number;
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Time = encodefloat.float_number;
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Magnetometer[0] = encodefloat.float_number;
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Magnetometer[1] = encodefloat.float_number;
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;
			tmp.Magnetometer[2] = encodefloat.float_number;

			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = *dptr++;;
			/*
			MAG_PROC_TIME = encodefloat.float_number;
      */
			uint32_t sampleTimeFine = 0;

			float time = tmp.Time;
			if (sampleTimeFine > 0)
				LastMeasurement.dt = time - LastMeasurement.Time;
			LastMeasurement.Time = time;
	  	}
	  	copy_accmaggyro = true;
	  	break;
	  case DREG_QUAT_AB:
	  	if(message->is_batch){

			message->length;
			/* Get pointer to data */
			uint8_t const* dptr = (uint8_t const*) message->data;
			int16_t QUAT_A = 0;
			int16_t QUAT_B = 0;
			int16_t QUAT_C = 0;
			int16_t QUAT_D = 0;

			QUAT_A =  (uint16_t)(*dptr++ << 8);
			QUAT_A |=  *dptr++;
			QUAT_B =  (*dptr++) << 8;
			QUAT_B |=  *dptr++;
			QUAT_C =  (*dptr++) << 8;
			QUAT_C |=  *dptr++;
			QUAT_D =  (*dptr++) << 8;
			QUAT_D |=  *dptr++;
			for (int i = 0; i < 4; i++) {
				encodefloat.byte_number[3 - i] = (uint8_t) *dptr++;
			}
			tmp.Quaternion[0] = ((float) QUAT_A) / 29789.09091f;
			tmp.Quaternion[1] = ((float) QUAT_B) / 29789.09091f;
			tmp.Quaternion[2] = ((float) QUAT_C) / 29789.09091f;
			tmp.Quaternion[3] = ((float) QUAT_D) / 29789.09091f;

		}

		copy_quat = true;
		break;
	  case DREG_EULER_PHI_THETA:
		if (message->is_batch) {
			uint8_t const* dptr = (uint8_t const*) message->data;
			int16_t phi_roll, theta_pitch, psi_yaw, roll_rate, pitch_rate, yaw_rate;
			phi_roll = (uint8_t) *dptr++ << 8;
			phi_roll |= (uint8_t) *dptr++;
			theta_pitch = (uint8_t) *dptr++ << 8;
			theta_pitch |= (uint8_t) *dptr++;
			psi_yaw = (uint8_t) *dptr++ << 8;
			psi_yaw |= (uint8_t) *dptr++;

			roll_rate = (uint8_t) *dptr++ << 8;
			roll_rate |= (uint8_t) *dptr++;
			pitch_rate = (uint8_t) *dptr++ << 8;
			pitch_rate |= (uint8_t) *dptr++;
			yaw_rate = (uint8_t) *dptr++ << 8;
			yaw_rate |= (uint8_t) *dptr++;
      // TODO add delta q
			for (int i = 0; i < 4; i++)
				encodefloat.byte_number[3 - i] = (uint8_t) *dptr++;
			// EULER_TIME = encodefloat.float_number;
	  	}
		copy_euler = true;
	  	break;
	  default:
	  	break;
	}
	/*
	if (XbusMessage_getDataItem(&tmp.PackageCounter, XDI_PacketCounter, message)) {
		LastMeasurement.PackageCounter = tmp.PackageCounter;
	}

	uint32_t sampleTimeFine = 0;
	if (XbusMessage_getDataItem(&sampleTimeFine, XDI_SampleTimeFine, message)) {
		float time = (float)sampleTimeFine / 10000.0f;
		if (sampleTimeFine > 0)
			LastMeasurement.dt = time - LastMeasurement.Time;
		LastMeasurement.Time = time;
	}
  */
	if (copy_accmaggyro) {
		memcpy(LastMeasurement.Accelerometer, tmp.Accelerometer,
				sizeof(tmp.Accelerometer));
		LastMeasurement.Accelerometer[0] = -LastMeasurement.Accelerometer[0];
		LastMeasurement.Accelerometer[1] = -LastMeasurement.Accelerometer[1];
		memcpy(LastMeasurement.Gyroscope, tmp.Gyroscope, sizeof(tmp.Gyroscope));
		LastMeasurement.Gyroscope[0] = -LastMeasurement.Gyroscope[0];
		LastMeasurement.Gyroscope[1] = -LastMeasurement.Gyroscope[1];
		memcpy(LastMeasurement.Magnetometer, tmp.Magnetometer,
				sizeof(tmp.Magnetometer));
		LastMeasurement.Magnetometer[0] = -LastMeasurement.Magnetometer[0];
		LastMeasurement.Magnetometer[1] = -LastMeasurement.Magnetometer[1];
	}
	if (copy_quat) {
		memcpy(LastMeasurement.Quaternion, tmp.Quaternion, sizeof(tmp.Quaternion));
		LastMeasurement.Quaternion[1] = -LastMeasurement.Quaternion[1];
		LastMeasurement.Quaternion[2] = -LastMeasurement.Quaternion[2];
	}
	/* Note that measurements and estimates has to be rotated 180 degrees due to the mount/orientation of the Xsens IMU */
	/*

	if (XbusMessage_getDataItem(tmp.Quaternion, XDI_Quaternion, message)) {
		memcpy(LastMeasurement.Quaternion, tmp.Quaternion, sizeof(tmp.Quaternion));
		LastMeasurement.Quaternion[1] = -LastMeasurement.Quaternion[1];
		LastMeasurement.Quaternion[2] = -LastMeasurement.Quaternion[2];
	}

	if (XbusMessage_getDataItem(tmp.DeltaQ, XDI_DeltaQ, message)) {
		memcpy(LastMeasurement.DeltaQ, tmp.DeltaQ, sizeof(tmp.DeltaQ));
		LastMeasurement.DeltaQ[1] = -LastMeasurement.DeltaQ[1];
		LastMeasurement.DeltaQ[2] = -LastMeasurement.DeltaQ[2];
	}

	if (XbusMessage_getDataItem(tmp.Accelerometer, XDI_Acceleration, message)) {
		memcpy(LastMeasurement.Accelerometer, tmp.Accelerometer, sizeof(tmp.Accelerometer));
		LastMeasurement.Accelerometer[0] = -LastMeasurement.Accelerometer[0];
		LastMeasurement.Accelerometer[1] = -LastMeasurement.Accelerometer[1];
	}

	if (XbusMessage_getDataItem(tmp.Gyroscope, XDI_RateOfTurn, message)) {
		memcpy(LastMeasurement.Gyroscope, tmp.Gyroscope, sizeof(tmp.Gyroscope));
		LastMeasurement.Gyroscope[0] = -LastMeasurement.Gyroscope[0];
		LastMeasurement.Gyroscope[1] = -LastMeasurement.Gyroscope[1];
	}

	if (XbusMessage_getDataItem(tmp.Magnetometer, XDI_MagneticField, message)) {
		memcpy(LastMeasurement.Magnetometer, tmp.Magnetometer, sizeof(tmp.Magnetometer));
		LastMeasurement.Magnetometer[0] = -LastMeasurement.Magnetometer[0];
		LastMeasurement.Magnetometer[1] = -LastMeasurement.Magnetometer[1];
	}

	if (XbusMessage_getDataItem(&tmp.Status, XDI_StatusWord, message)) {
		LastMeasurement.Status = tmp.Status;
	}
  */

	/* Successfully read all data - now overwrite LastMeasurement */
	/*xSemaphoreTake( _dataSemaphore, ( TickType_t ) portMAX_DELAY ); // take data semaphore to update data
	memcpy(&LastMeasurement, &tmp, sizeof(LastMeasurement_t));*/

	xSemaphoreGive( _dataSemaphore ); // give semaphore back
}

MTI200::LastMeasurement_t MTI200::GetLastMeasurement()
{
	MTI200::LastMeasurement_t tmp;
	memset(&tmp, 0, sizeof(LastMeasurement_t));
	tmp.Quaternion[0] = 1.0; // as a safety measure if the semaphore can not be taken

	xSemaphoreTake( _dataSemaphore, ( TickType_t ) 3 ); // take data semaphore to read data
	tmp = LastMeasurement; // copy data
	xSemaphoreGive( _dataSemaphore ); // give semaphore back

	return tmp;
}

void MTI200::GetEstimates(Estimates_t& estimates)
{
	xSemaphoreTake( _dataSemaphore, ( TickType_t ) 3 ); // take data semaphore to read data

	memcpy(estimates.q, LastMeasurement.Quaternion, sizeof(LastMeasurement.Quaternion));
	//memcpy(estimates.dq, LastMeasurement.DeltaQ, sizeof(LastMeasurement.Quaternion));
	Quaternion_GetDQ_FromBody(estimates.q, LastMeasurement.Gyroscope, estimates.dq); // convert angular velocity into q_dot

	xSemaphoreGive( _dataSemaphore ); // give semaphore back
}
