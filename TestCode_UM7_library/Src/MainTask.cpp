/* Copyright (C) 2020-2020 Juan de Dios Flores Mendez. All rights reserved.
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
 * Web      :  https://github.com/jdios89
 * e-mail   :  juan.dios.flores@gmail.com
 * ------------------------------------------
 */

#include "MainTask.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
// #include "MemoryManagement.h"
#include "Timer.h"
//#include "PD4Cxx08.h"
//#include "CAN_CMD.h"
/* Include Periphiral drivers */
#include "EEPROM.h"
#include "I2C.h"
#include "IO.h"
#include "Timer.h"
#include "UART.h"
#include "USBCDC.h"

/* Include Device drivers */
#include "IMU.h"
#include "LSPC.hpp"
#include "MPU9250.h"
#include "MTI200.h"
#include "Parameters.h"

/* Miscelaneous includes */
#include <stdlib.h>
#include <vector>

/* Include Module libraries */
#include "Debug.h"

int32_t encoder1 = 0;
int32_t encoder2 = 0;
int32_t encoder3 = 0;

void testTask() {
	float desd = 0.0f;
//	FDCAN * fdcantest22 = new FDCAN(); // port object
}
void MainTask(void * pvParameters) {
	/* Use this task to:
	 * - Create objects for each module
	 *     (OBS! It is very important that objects created with "new"
	 *      only happens within a thread due to the usage of the FreeRTOS managed heap)
	 * - Link any modules together if necessary
	 * - Create message exchange queues and/or semaphore
	 * - (Create) and start threads related to modules
	 *
	 * Basically anything related to starting the system should happen in this thread and NOT in the main() function !!!
	 */

	/* Initialize global parameters */
	Parameters& params = *(new Parameters);
	/* Initialize EEPROM */
	EEPROM * eeprom = new EEPROM;
	eeprom->EnableSection(eeprom->sections.imu_calibration,
			sizeof(IMU::calibration_t)); // enable IMU calibration section in EEPROM
	eeprom->EnableSection(eeprom->sections.parameters,
			params.getParameterSizeBytes());
  /* Commented because it goes to hard fault handler */
	eeprom->Initialize();
	params.AttachEEPROM(eeprom); // attach EEPROM to load and store parameters into EEPROM

	/* Initialize microseconds timer */
	Timer * microsTimer = new Timer(Timer::TIMER6, 1000000); // create a 1 MHz counting timer used for micros() timing

	/* Initialize communication */
	USBCDC * usb = new USBCDC(USBCDC_TRANSMITTER_PRIORITY);
	LSPC * lspcUSB = new LSPC(usb, LSPC_RECEIVER_PRIORITY,
			LSPC_TRANSMITTER_PRIORITY); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
	Debug * dbg = new Debug(lspcUSB); // pair debug module with configured LSPC module to enable "Debug::print" functionality
	params.AttachLSPC(lspcUSB); // attach USB object to allow modification of parameters over USB

	/* Debug boot info */
	Debug::print("Booting...\n");

	/* Prepare Xsens IMU always, since it is used for logging and comparison purposes */
	/* UART _uart(UART::PORT_UART3, 460800, 500); */
	UART * uart = new UART(UART::PORT_UART3, 460800, 500);
	MTI200 * mti200 = new MTI200(uart);
  while(1) {	mti200->getFirmwareRevisionUM7();
              mti200->ConfigureUM7(255);
              osDelay(1) ; }
	if (params.estimator.ConfigureXsensIMUatBoot) {
		if (!mti200->Configure(2 * params.estimator.SampleRate)) { // configuration failed, so do not use/pass on to balance controller
			delete (mti200);
			mti200 = 0;
		}
	}

	/* Initialize and configure IMU */
	IMU * imu = 0;
	if (params.estimator.UseXsensIMU) {
		if (!mti200)
			ERROR("MTI200 selected but not available!");
		imu = mti200; // use Xsens MTI200 in Balance controller
		imu->AttachEEPROM(eeprom);
	}
	if (!imu->isCalibrated()) {
		imu->SetCalibration(params.sensor.default_accelerometer_bias,
				params.sensor.default_accelerometer_scale,
				params.sensor.default_gyroscope_bias,
				params.sensor.default_calibration_matrix);
	}

	/* Test code of real time task running */
	while (1) {
		HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
//		osDelay(2);
		HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
//		osDelay(500);
		HAL_GPIO_TogglePin(GPIOE, LD2_Pin);
		osDelay(500);
	}


}

//void Reboot_Callback(void * param, const std::vector<uint8_t>& payload)
//{
//	// ToDo: Need to check for magic key
//	NVIC_SystemReset();
//}
//
//void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload)
//{
//	// ToDo: Need to check for magic key
//	USBD_Stop(&USBCDC::hUsbDeviceFS);
//	USBD_DeInit(&USBCDC::hUsbDeviceFS);
//	Enter_DFU_Bootloader();
//}
