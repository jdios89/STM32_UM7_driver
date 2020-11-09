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
//#include "MemoryManagement.h"
#include "Timer.h"
//#include "PD4Cxx08.h"
//#include "CAN_CMD.h"


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
	/* Initialize microseconds timer */
	Timer * microsTimer = new Timer(Timer::TIMER6, 1000000); // create a 1 MHz counting timer used for micros() timing
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
