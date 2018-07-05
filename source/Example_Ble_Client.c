/*
 * Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for Licensee�s application development.
 * Fitness and suitability of the example code for any use within Licensee�s applications need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
 * Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee.
 * For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     (1) Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *     (2) Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *
 *     (3)The name of the author may not be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 *  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*----------------------------------------------------------------------------*/
/**
 * @ingroup APPS_LIST
 *
 * @defgroup XDK_APPLICATION_TEMPLATE XDK Application Template
 * @{
 *
 * @brief XDK Application Template
 *
 * @details Empty XDK Application Template without any functionality. Should be used as a template to start new projects.
 *
 * @file
 **/
/* module includes ********************************************************** */

/* system header files */
#include <stdio.h>
/* additional interface header files */
#include "FreeRTOS.h"
#include "timers.h"

/* own header files */
#include <stdio.h>
#include "BCDS_Basics.h"

/* additional interface header files */
#include "BCDS_Assert.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "BCDS_CmdProcessor.h"
//#include "BleAlpwDataExchange_Server.h"
#include "BleAlpwDataExchange_Client.h"
#include "XdkUsbResetUtility.h"

/* ble */
#include "BleEngine.h"
#include "BleGap.h"

/* own header files */
#include "Example_Ble_Client.h"

/* constant definitions ***************************************************** */

/* local variables ********************************************************** */
CmdProcessor_T *AppCmdProcessorHandle;

static volatile uint8_t isInterruptHandled = ENABLE_FLAG;
static uint8_t recievedData[MAX_DEVICE_LENGTH];

static BleHandler bleHandler;
static BD_ADDR hc05Ble_address;
static U8 txData[10];

static BleAlpwDataExchangeClient bleAlpwDataExchangeClient;
static U16 connHandle = 0;

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/**
 * @brief API called by xtimerPendFunctionCallFromISR function, which is registered during the USB ISR
 *
 * @param [in]callBackParam1 data buffer
 *
 * @param [in]callBackParam2 length
 */
static void interruptHandling(void *callBackParam1, uint32_t callBackParam2) {

	/* re-enable  the usb interrupt flag*/
	isInterruptHandled = ENABLE_FLAG;
	UNUSED_PARAMETER(callBackParam1);
	UNUSED_PARAMETER(callBackParam2);
}

/**
 * @brief USB recieve call back function
 *
 * @param[in] usbRcvBuffer recieved data
 *
 * @param[in] count length of the data received
 */
void callbackIsr(uint8_t *usbRcvBuffer, uint16_t count) {
	if (ENABLE_FLAG == isInterruptHandled) {
		isInterruptHandled = DISABLE_FLAG;

		/* add to timer queue*/
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		if (xTimerPendFunctionCallFromISR(interruptHandling, NULL, UINT8_C(0),
				&xHigherPriorityTaskWoken) == pdPASS) {
			memcpy(recievedData, usbRcvBuffer, count);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		} else {
			assert(false);
		}

	}
}

/*
 *
 */
void CORESTACK_BleCallback(BleEvent event, BleStatus status, void *param) {

	printf("CORESTACK_BleCallback event:%d, status: %d  \n\r", event, status);

	switch (event) {
	case BLEEVENT_INITIALIZATION_RSP:

		break;
	case BLEEVENT_PAIRING_COMPLETE:
		printf("pairing completed %d  \n\r", status);
		break;
	default:
		break;
	}

	UNUSED_PARAMETER(param);
}

/**
 *
 */

void BleGap_CallBack(BleGapEvent event, BleStatus status, void *parms) {

	BleStatus returnValue = BLESTATUS_FAILED;	//BLESSTATUS_SUCCESS

	switch (event) {
	case BLEGAP_EVENT_CONNECTED:

		returnValue = BLEALPWDATAEXCHANGE_CLIENT_LinkUpAlpwDataExchangeProfile(
				&bleAlpwDataExchangeClient, connHandle,
				BLEALPWDATAEXCHANGE_CLIENT_LINKUP_COMPLETE, false);

		if (returnValue == BLESTATUS_SUCCESS) {

			//TODO
			BLESMP_InitiatePairing(connHandle, 1);

		}

		break;
	case BLEGAP_EVENT_DISCONNECTED:
		printf("Link has been dropped, disconnected \n\r");
		break;
	default:
		break;
	}

	UNUSED_PARAMETER(status);

}
/**
 *
 */

void BleAlpwDataExchangeClient_CallBack(BleAlpwDataExchangeClientEvent event,
		BleStatus status, void* parms) {

	switch (event) {
	case BLEALPWDATAEXCHANGECLIENT_EVENT_UNLINKED:
		printf("Link has been dropped, unlinked.. \n\r");
		break;
	default:
		break;
	}

}

/* global functions ********************************************************* */

//-----------------------------------------
/** the function initializes the BMA and its process
 *
 * @brief The function initializes BMA(accelerometer)creates a auto reloaded
 * timer task which gets and transmit the accel raw data via BLE
 */
Retcode_T init(void) {

	printf("Ble Central .. Initializing...\n\r");

	//(1)- -- Initialization
	BleStatus returnValue = BLESTATUS_FAILED;	//BLESSTATUS_SUCCESS

	returnValue = BLEMGMT_Init();

	if (returnValue == BLESTATUS_FAILED) {
		return RETCODE_FAILURE;
	}
	printf("BLEMGMT_Init OK \n\r");

	bleHandler.callback = CORESTACK_BleCallback;

	returnValue = BLEMGMT_RegisterHandler(&bleHandler);

	if (returnValue != BLESTATUS_SUCCESS) {
		return RETCODE_FAILURE;
	}

	printf("BLEMGMT_RegisterHandler OK \n\r");

	//--  register device
	returnValue = BLEGAP_RegisterDevice(BLEGAPROLE_CENTRAL, BleGap_CallBack);

	if (returnValue != BLESTATUS_SUCCESS) {
		return RETCODE_FAILURE;
	}
	printf("BLEGAP_RegisterDevice OK \n\r");

	//-- init client
	returnValue = BLEALPWDATAEXCHANGE_CLIENT_Init(
			BleAlpwDataExchangeClient_CallBack);
	if (returnValue != BLESTATUS_SUCCESS) {
		return RETCODE_FAILURE;
	}

	printf("BLEALPWDATAEXCHANGE_CLIENT_Init OK \n\r");

	//(2)- -- Link-up
	hc05Ble_address.addr[0] = 0xC4;
	hc05Ble_address.addr[1] = 0x86;
	hc05Ble_address.addr[2] = 0xE9;
	hc05Ble_address.addr[3] = 0xF6;
	hc05Ble_address.addr[4] = 0x2C;
	hc05Ble_address.addr[5] = 0xFB;

	returnValue = BLEGAP_Connect(&hc05Ble_address, BLEADDRESS_PUBLIC);// returns pending

	if (returnValue == BLESTATUS_FAILED) {
		return RETCODE_FAILURE;
	}
	printf("BLEGAP_Connect OK \n\r");

	vTaskDelay(10000);

	//--- Operation:send some data
	int i = 0;
	for (i = 0; i < 10; i++) {
		txData[i] = i + 10;
	}

	printf("Sending some data \n\r");

	returnValue = BLEALPWDATAEXCHANGE_CLIENT_SendData(
			&bleAlpwDataExchangeClient, txData, 10,
			BLEALPWDATAEXCHANGE_CLIENT_DATA_RELIABLE);

	printf("BLEGAP_RegisterDevice returned %d \n\r", returnValue);

	if (returnValue == BLESTATUS_FAILED) {
		return RETCODE_FAILURE;
	}

	return RETCODE_OK;

}

/**
 * the function Deinitializes the timer task
 *
 *
 */
void deinit(void) {
	/*Suspend the BLE task*/
}

/**
 * @brief This is a template function where the user can write his custom application.
 *
 */
void appInitSystem(void * CmdProcessorHandle, uint32_t param2) {

	printf(" ... app Init System ... \n\r");

	if (CmdProcessorHandle == NULL) {
		printf("Command processor handle is null \n\r");
		assert(false);
	}
	BCDS_UNUSED(param2);

	AppCmdProcessorHandle = (CmdProcessor_T *) (CmdProcessorHandle);

	vTaskDelay(6000);	//just some time to allow printf show on console

	Retcode_T retVal = RETCODE_OK;
	retVal = init();

	if (RETCODE_OK == retVal) {
		printf(
				"Example_Ble_Master App Initialization completed successfully \r\n ");
	} else {
		printf("Example_Ble_Master App Initialization failed \r\n ");
	}
}
/**@} */
/** ************************************************************************* */
