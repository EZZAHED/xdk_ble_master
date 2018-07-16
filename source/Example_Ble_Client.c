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

#include "XDKAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_ACCEL_OVER_BLE

/* system header files */
#include <stdio.h>

/* own header files */
#include "Example_Ble_Client.h"

/* additional interface header files */
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
#include "task.h"

#include "XdkSensorHandle.h"
#include "XdkUsbResetUtility.h"

#include "BCDS_Basics.h"
#include "BCDS_Assert.h"
#include "BCDS_CmdProcessor.h"
#include "BCDS_BlePeripheral.h"
#include "BCDS_Retcode.h"
#include "BCDS_BidirectionalService.h"
#include "BCDS_Ble.h"

#include "BleEngine.h"
#include "BleGap.h"
#include "BleAlpwDataExchange_Client.h"

/* constant definitions ***************************************************** */

/* local variables ********************************************************** */
CmdProcessor_T *AppCmdProcessorHandle;
static volatile uint8_t isInterruptHandled = ENABLE_FLAG;
static uint8_t recievedData[MAX_DEVICE_LENGTH];

static BleHandler bleHandler;
static BD_ADDR hc05Ble; // hc05 ARDUINO module BLE Address
static U8 txData[10];

static BleAlpwDataExchangeClient bleAlpwDataExchangeClient;
static U16 connHandle = 0;

/* global variables ********************************************************* */

static xTimerHandle bleTransmitTimerHandle; /**< variable to store timer handle*/
CmdProcessor_T *AppCmdProcessorHandle;
static uint8_t bleTransmitStatus = NUMBER_ZERO; /**< Validate the repeated start flag */
static xTimerHandle bleTransmitTimerHandle; /**< variable to store timer handle*/
static SemaphoreHandle_t BleStartSyncSemphr = NULL;
static SemaphoreHandle_t BleWakeUpSyncSemphr = NULL;
static SemaphoreHandle_t SendCompleteSync = NULL;

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

//---------------------------------- Callbacks--------------------------------

/**
 * @brief Callback function called on BLE send completion
 *
 * @param [in]  sendStatus : event to be send by BLE during communication.
 *
 */
static void BleAccelDataSentCallback(Retcode_T sendStatus) {
	if (RETCODE_OK != sendStatus) {
		printf(
				"Error in transmitting the Accel Data over BLE. ERROR Code %ui  : \r\n",
				(unsigned int) sendStatus);
	}
	if (xSemaphoreGive(SendCompleteSync) != pdTRUE) {
		/*We would not expect this call to fail because we must have obtained the semaphore to get here.*/
		Retcode_RaiseError(
				RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
	}
}

/** The function to send start or stop message to Bidirectional DataExchange service
 *  @param [in]  param1 : Unused, Reserved for future use
 *  @param [in]  param2 : Differentiates start and stop command
 */
static void BleStartEndMsgSend(void * param1, uint32_t param2) {
	BCDS_UNUSED(param1);
	Retcode_T bleRetval = RETCODE_OK;

	if (param2 == BLE_TRIGGER_START_CMD) {
		bleRetval = BidirectionalService_SendData(
				((uint8_t*) "X      Y      Z"),
				((uint8_t) sizeof("X      Y      Z") - 1));
	}
	if (param2 == BLE_TRIGGER_END_CMD) {
		bleRetval = BidirectionalService_SendData(
				((uint8_t*) "Transfer Terminated!"),
				((uint8_t) sizeof("Transfer Terminated!") - 1));
	}
	if (RETCODE_OK == bleRetval) {
		if (pdFALSE == xSemaphoreTake(SendCompleteSync, BLE_SEND_TIMEOUT)) {
			bleRetval = RETCODE(RETCODE_SEVERITY_ERROR,
					SEMAPHORE_TIME_OUT_ERROR);
		}
	}
	if (RETCODE_OK != bleRetval) {
		printf(
				"Not able to send response to start or stop command  on BLE  : \r\n");
	}
}

/**
 * @brief Callback function called on data reception over BLE
 *
 * @param [in]  rxBuffer : Buffer in which received data to be stored.
 *
 * @param [in]  rxDataLength : Length of received data.
 */

static void BleDataReceivedCallBack(uint8_t *rxBuffer, uint8_t rxDataLength) {
	Retcode_T retVal = RETCODE_OK;
	uint8_t bleReceiveBuff[BLE_RECEIVE_BUFFER_SIZE];
	if (rxDataLength >= BLE_RECEIVE_BUFFER_SIZE) {
		printf("Data length received is invalid \n");
	} else {
		memset(bleReceiveBuff, 0, sizeof(bleReceiveBuff));
		memcpy(bleReceiveBuff, rxBuffer, rxDataLength);
		/* make sure that the received string is null-terminated */
		bleReceiveBuff[rxDataLength] = '\0';

		/* validate received data */
		if ((NUMBER_ZERO == strcmp((const char *) bleReceiveBuff, "start"))
				&& (NUMBER_ZERO == bleTransmitStatus)) {
			retVal = CmdProcessor_Enqueue(AppCmdProcessorHandle,
					BleStartEndMsgSend, NULL, UINT32_C(1));
			if (RETCODE_OK != retVal) {
				printf(
						"Failed to Enqueue BleStartEndMsgSend to Application Command Processor \r\n");
			}
			/* start accelerometer data transmission timer */
			if (pdTRUE
					!= xTimerStart(bleTransmitTimerHandle,
							(ONESECONDDELAY/portTICK_RATE_MS))) {
				/* Assertion Reason : Failed to start software timer. Check command queue size of software timer service*/
				assert(false);
			} else {
				bleTransmitStatus = NUMBER_ONE;
			}
		} else if ((NUMBER_ZERO == strcmp((const char *) bleReceiveBuff, "end"))
				&& (NUMBER_ONE == bleTransmitStatus)) {

			/* stop accelerometer data transmission timer */
			if (pdTRUE != xTimerStop(bleTransmitTimerHandle, NUMBER_ZERO)) {
				/* Assertion Reason: Failed to start software timer. Check command queue size of software timer service. */
				assert(false);
			} else {
				bleTransmitStatus = NUMBER_ZERO;
				retVal = CmdProcessor_Enqueue(AppCmdProcessorHandle,
						BleStartEndMsgSend, NULL, UINT32_C(0));
				if (RETCODE_OK != retVal) {
					printf(
							"Failed to Enqueue BleStartEndMsgSend to Application Command Processor \r\n");
				}
			}

		}
	}
}

/**
 *  The function to register the bidirectional service
 */

static Retcode_T BiDirectionalServiceRegistryCallback(void) {
	Retcode_T retval = RETCODE_OK;

	retval = BidirectionalService_Init(BleDataReceivedCallBack,
			BleAccelDataSentCallback);
	if (RETCODE_OK == retval) {
		retval = BidirectionalService_Register();
	}

	return (retval);
}

/**
 * @brief Callback function called on BLE event
 *
 * @param [in]  event : event to be send by BLE during communication.
 *
 * @param [in]  data : void pointer pointing to a data type based on event.
 *                     currently reserved for future use
 *
 * event                                    |   data type                   |
 * -----------------------------------------|-------------------------------|
 * BLE_PERIPHERAL_STARTED                   |   Retcode_T                   |
 * BLE_PERIPHERAL_SERVICES_REGISTERED       |   unused                      |
 * BLE_PERIPHERAL_SLEEP_SUCCEEDED           |   Retcode_T                   |
 * BLE_PERIPHERAL_WAKEUP_SUCCEEDED          |   Retcode_T                   |
 * BLE_PERIPHERAL_CONNECTED                 |   Ble_RemoteDeviceAddress_T   |
 * BLE_PERIPHERAL_DISCONNECTED              |   Ble_RemoteDeviceAddress_T   |
 * BLE_PERIPHERAL_ERROR                     |   Retcode_T                   |
 */
static void BleEventCallBack(BlePeripheral_Event_T event, void * data) {

	BlePeripheral_Event_T Event = event;
	BCDS_UNUSED(data);
	switch (Event) {
	case BLE_PERIPHERAL_SERVICES_REGISTERED:
		break;
	case BLE_PERIPHERAL_STARTED:
		printf("BLE powered ON successfully \r\n");
		if (xSemaphoreGive(BleStartSyncSemphr) != pdTRUE) {
			/* We would not expect this call to fail because we must have obtained the semaphore to get here.*/
			Retcode_RaiseError(
					RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
		}
		break;

	case BLE_PERIPHERAL_SLEEP_SUCCEEDED:
		printf("BLE successfully entered into sleep mode \r\n");
		break;

	case BLE_PERIPHERAL_WAKEUP_SUCCEEDED:
		printf("Device Wake up succceded  : \r\n");
		if (xSemaphoreGive(BleWakeUpSyncSemphr) != pdTRUE) {
			/*We would not expect this call to fail because we must have obtained the semaphore to get here.*/
			Retcode_RaiseError(
					RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
		}
		break;

	case BLE_PERIPHERAL_CONNECTED:
		printf("Device connected  : \r\n");
		break;

	case BLE_PERIPHERAL_DISCONNECTED:
		printf("Device Disconnected   : \r\n");
		break;
	case BLE_PERIPHERAL_ERROR:
		printf("BLE Error Event  : \r\n");
		break;

	default:
		/* assertion reason : invalid status of Bluetooth Device */
		assert(false);
		break;
	}
}

/** The function to get and transfer the accel data using BLE alpwise DataExchange service
 *
 * @brief        Gets the raw data from BMA280 Accel and transfer through the alphwise DataExchange service on BLE
 *
 * @param[in]   *param1: a generic pointer to any context data structure which will be passed to the function when it is invoked by the command processor.
 *
 * @param[in]    param2: a generic 32 bit value  which will be passed to the function when it is invoked by the command processor.
 */
static void BleAccelDataTransmit(void * param1, uint32_t param2) {
	BCDS_UNUSED(param1);
	BCDS_UNUSED(param2);

	printf("BleAccelDataTransmit ... CALLED\n\r");

}

/**
 * @brief        This is a application timer callback function used to enqueue BleAccelDataTransmit function
 *               to the command processor.
 *
 * @param[in]    pvParameters unused parameter.
 */
static void EnqueueAccelDatatoBLE(void *pvParameters) {
	BCDS_UNUSED(pvParameters);

	Retcode_T retVal = CmdProcessor_Enqueue(AppCmdProcessorHandle,
			BleAccelDataTransmit, NULL, UINT32_C(0));
	if (RETCODE_OK != retVal) {
		printf(
				"Failed to Enqueue BleAccelDataTransmit to Application Command Processor \r\n");
	}
}

/*
 *
 */
void CORESTACK_BleCallback(BleEvent event, BleStatus status, void *param) {

	printf("CORESTACK_BleCallback event: 0x%02X, status: %d  \n\r", event, status);

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

/*
 *
 */

/* global functions ********************************************************* */

//-----------------------------------------
/** the function initializes the BMA and its process
 *
 * @brief The function initializes BMA(accelerometer)creates a auto reloaded
 * timer task which gets and transmit the accel raw data via BLE
 */
Retcode_T init(void) {

	printf("................ Ble Central -- Init()  ............. \n\r");

	//(0)- -- Init low level functions
	Retcode_T retval = RETCODE_OK;

	static_assert((portTICK_RATE_MS != 0), "Tick rate MS is zero");

	BleStartSyncSemphr = xSemaphoreCreateBinary();
	if (NULL == BleStartSyncSemphr) {
		return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
	}
	BleWakeUpSyncSemphr = xSemaphoreCreateBinary();
	if (NULL == BleWakeUpSyncSemphr) {
		return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
	}
	SendCompleteSync = xSemaphoreCreateBinary();
	if (NULL == SendCompleteSync) {
		return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
	}

	/*initialize accel sensor*/
	retval = Accelerometer_init(xdkAccelerometers_BMA280_Handle);

	bleTransmitTimerHandle = xTimerCreate(
			(char * const ) "bleTransmitTimerHandle",
			pdMS_TO_TICKS(BLE_TX_FREQ), TIMER_AUTORELOAD_ON, NULL,
			EnqueueAccelDatatoBLE);
	if (NULL != bleTransmitTimerHandle) {
		retval = BlePeripheral_Initialize(BleEventCallBack,
				BiDirectionalServiceRegistryCallback);
		if (RETCODE_OK == retval) {
			printf("BlePeripheral_Initialize ... OK \n\r");

			retval = BlePeripheral_SetDeviceName(
					(uint8_t*) XDK_BLE_DEVICE_NAME);
		}
		/* Powering on BLE module*/
		if (RETCODE_OK == retval) {

			retval = BlePeripheral_Start();
		}
		if (RETCODE_OK == retval) {
			printf("BlePeripheral_Start ... OK \n\r");

			if (pdTRUE
					!= xSemaphoreTake(BleStartSyncSemphr,
							BLE_START_SYNC_TIMEOUT)) {
				printf(
						"Failed to Start BLE before timeout, Ble Initialization failed \n");
				retval = RETCODE(RETCODE_SEVERITY_ERROR,
						SEMAPHORE_TIME_OUT_ERROR);

			}
		}
		if (RETCODE_OK == retval) {
			retval = BlePeripheral_Wakeup();
		}
		/* Wake up BLE module*/
		if (RETCODE_OK == retval) {
			printf("BlePeripheral_Wakeup ... OK \n\r");

			if (pdTRUE
					!= xSemaphoreTake(BleWakeUpSyncSemphr,
							BLE_WAKEUP_SYNC_TIMEOUT)) {
				printf(
						"Failed to Wake up BLE before timeout, Ble Initialization failed \n");
				retval = RETCODE(RETCODE_SEVERITY_ERROR,
						SEMAPHORE_TIME_OUT_ERROR);
			}
		}
		if (RETCODE_OK == retval) {
			printf("Ble Initialization succeded \n");
		} else {
			printf("Ble Initialization Failed \r\n");
		}

		//(1)- -- Initialization
		BleStatus returnValue = BLESTATUS_FAILED;	//BLESSTATUS_SUCCESS

		returnValue = BLEMGMT_Init();

		if (returnValue == BLESTATUS_FAILED) {
			return RETCODE_FAILURE;
		}
		printf("BLEMGMT_Init ... OK \n\r");

		bleHandler.callback = CORESTACK_BleCallback;

		returnValue = BLEMGMT_RegisterHandler(&bleHandler);

		if (returnValue != BLESTATUS_SUCCESS) {
			return RETCODE_FAILURE;
		}

		printf("BLEMGMT_RegisterHandler ... OK \n\r");

		//--  register device
		returnValue = BLEGAP_RegisterDevice(BLEGAPROLE_CENTRAL,
				BleGap_CallBack);

		if (returnValue != BLESTATUS_SUCCESS) {
			return RETCODE_FAILURE;
		}
		printf("BLEGAP_RegisterDevice ... OK \n\r");

		//-- init client
		returnValue = BLEALPWDATAEXCHANGE_CLIENT_Init(
				BleAlpwDataExchangeClient_CallBack);
		if (returnValue != BLESTATUS_SUCCESS) {
			return RETCODE_FAILURE;
		}

		printf("BLEALPWDATAEXCHANGE_CLIENT_Init ... OK \n\r");

		//(2)- -- Link-up
		hc05Ble.addr[0] = 0x98  ; //0xC4;
		hc05Ble.addr[1] = 0xD3  ; //0x86;
		hc05Ble.addr[2] = 0x34  ; //0xE9;
		hc05Ble.addr[3] = 0x91  ; //0xF6;
		hc05Ble.addr[4] = 0x2D  ; //0x2C;
		hc05Ble.addr[5] = 0x0E  ; //0xFB;

		returnValue = BLEGAP_Connect(&hc05Ble, BLEADDRESS_PUBLIC); // returns pending

		if (returnValue == BLESTATUS_FAILED) {
			return RETCODE_FAILURE;
		}
		printf("BLEGAP_Connect ... OK \n\r");


		//-- wait for some time
		printf("Waiting for 10s ... OK \n\r");
		vTaskDelay(10000);

		//--- Operation:send some data
		int i = 0;
		for (i = 0; i < 10; i++) {
			txData[i] = i + 10;
		}
		BLEGATT_SUPPORT_WRITE_CHARACTERISTIC_WITHOUT_RESPONSE;
		printf("Sending some data \n\r");

		returnValue = BLEALPWDATAEXCHANGE_CLIENT_SendData(
				&bleAlpwDataExchangeClient, txData, 10,
				BLEALPWDATAEXCHANGE_CLIENT_DATA_NOT_RELIABLE);

		printf("BLEALPWDATAEXCHANGE_CLIENT_SendData returned %d \n\r",
				returnValue);

		if (returnValue == BLESTATUS_FAILED) {
			return RETCODE_FAILURE;
		}
		printf("BLEALPWDATAEXCHANGE_CLIENT_SendData ... OK \n\r");

		return RETCODE_OK;

	}
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
