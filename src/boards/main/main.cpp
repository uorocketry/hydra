#include <atmel_start.h>
// sbgCommonLib headers
#include <sbgCommon.h>
#include <version/sbgVersion.h>

// sbgECom headers
#include <sbgEComLib.h>

#define TASK_LED_STACK_SIZE (128*3 / sizeof(portSTACK_TYPE))
#define TASK_SBG_STACK_SIZE (128*50 / sizeof(portSTACK_TYPE))
// #define TASK_USART_STACK_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_SBG_STACK_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY + 2)

static TaskHandle_t      xLEDTask;
static TaskHandle_t      xSBGTask;
// static TaskHandle_t      xUSARTTask;

/**
 * LED task
 *
 * \param[in] p The void pointer for OS task Standard model.
 *
 */
void led_task(void *p)
{
	for(;;) {
		gpio_toggle_pin_level(LED);
		os_sleep(500);
	}
	vTaskDelete(xLEDTask);
}

/**
 * Example task of using COMPUTER to echo using the IO abstraction.
 */
// void COMPUTER_example_task(void *p)
// {
// 	struct io_descriptor *io;
// 	uint8_t data[] = "Hello World\r\n";

// 	(void)p;

// 	usart_os_get_io(&COMPUTER, &io);

// 	for(;;) {
// 		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)&data, sizeof(data));		
// 		os_sleep(1000);
// 	}
// 	vTaskDelete(xUSARTTask);
// }


/*!
 *	Callback definition called each time a new log is received.
 * 
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	msgClass								Class of the message we have received
 *	\param[in]	msg										Message ID of the log received.
 *	\param[in]	pLogData								Contains the received log data as an union.
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
	ASSERT(pLogData);

	char data[1024];

	SBG_UNUSED_PARAMETER(pHandle);
	SBG_UNUSED_PARAMETER(pUserArg);

	if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0)
	{
		//
		// Handle separately each received data according to the log ID
		//
		switch (msg)
		{
		case SBG_ECOM_LOG_EKF_EULER:
			//
			// Simply display euler angles in real time
			//
			
			sprintf(data, "Euler Angles: %3.1f\t%3.1f\t%3.1f\tStd Dev:%3.1f\t%3.1f\t%3.1f   \r",
				sbgRadToDegf(pLogData->ekfEulerData.euler[0]),			sbgRadToDegf(pLogData->ekfEulerData.euler[1]),			sbgRadToDegf(pLogData->ekfEulerData.euler[2]),
				sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[0]),	sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[1]),	sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[2]));
			COMPUTER.io.write(&COMPUTER.io, (uint8_t *)data, sizeof(data));
			break;
		default:
			break;
		}
	}

	return SBG_NO_ERROR;
}
/*!
 * Get and print product info.
 *
 * \param[in]	pECom					SbgECom instance.
 * \return								SBG_NO_ERROR if successful.
 */
static SbgErrorCode getAndPrintProductInfo(SbgEComHandle *pECom)
{
	SbgErrorCode					errorCode;
	SbgEComDeviceInfo				deviceInfo;
	int n;

	ASSERT(pECom);
	//
	// Get device inforamtions
	//
	errorCode = sbgEComCmdGetInfo(pECom, &deviceInfo);

	//
	// Display device information if no error
	//
	if (errorCode == SBG_NO_ERROR)
	{
		char	calibVersionStr[32];
		char	hwRevisionStr[32];
		char	fmwVersionStr[32];		
		char serNum[100];
		char prodCode[57];
		char hardRev[64];
		char firmVer[57];
		char calVer[57];

		sbgVersionToStringEncoded(deviceInfo.calibationRev, calibVersionStr, sizeof(calibVersionStr));
		sbgVersionToStringEncoded(deviceInfo.hardwareRev, hwRevisionStr, sizeof(hwRevisionStr));
		sbgVersionToStringEncoded(deviceInfo.firmwareRev, fmwVersionStr, sizeof(fmwVersionStr));
		SBG_LOG_WARNING(errorCode, "Before sprintf\n");

		n = sprintf(serNum, "Serial Number: (%0.9"PRIu32")\n", deviceInfo.serialNumber);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)serNum, n);
		n = sprintf(prodCode, "Product Code: %s\n", deviceInfo.productCode);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)prodCode, n);
		n = sprintf(hardRev, "Hardware Revision: %s\n",	hwRevisionStr);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)hardRev, n);
		n = sprintf(firmVer, "Firmware Version: %s\n", fmwVersionStr);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)firmVer, n);
		n = sprintf(calVer, "Calib. Version: %s\n", calibVersionStr);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)calVer, n);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)"\n", 1);
	}
	else
	{
		SBG_LOG_WARNING(errorCode, "Unable to retrieve device information");
	}

	return errorCode;
}
static SbgErrorCode sbgTestProcess(SbgInterface *pInterface) {
	SbgErrorCode errorCode = SBG_NO_ERROR;
	SbgEComHandle comHandle; 
	ASSERT(pInterface);

	errorCode = sbgEComInit(&comHandle, pInterface);
	if (errorCode == SBG_NO_ERROR) 
	{
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)"Welcome to the SBG test.\n", 26);
		getAndPrintProductInfo(&comHandle);
		errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_8);
		if (errorCode != SBG_NO_ERROR) 
		{
			SBG_LOG_WARNING(errorCode, "Unable to configure SBG_ECOM_LOG_IMU_DATA log");			
		}
		errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_8);

		if (errorCode != SBG_NO_ERROR)
		{
			SBG_LOG_WARNING(errorCode, "Unable to configure SBG_ECOM_LOG_EKF_EULER log");
		}

		sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)"Euler Angles display with estimated standard deviation - degrees\n", 66);
		while (1) 
		{
			errorCode = sbgEComHandle(&comHandle);
			if (errorCode == SBG_NOT_READY) 
			{
				sbgSleep(1);
			}
			else 
			{
				SBG_LOG_ERROR(errorCode, "Unable to process incoming sbgECom logs");
			}
		}
		sbgEComClose(&comHandle);
	}
	else 
	{
		SBG_LOG_ERROR(errorCode, "Unable to initialize the sbgECom library");
	}
	return errorCode;
}

/**
 * SBG task
 */
void SBG_task(void *p)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgInterface		sbgInterface;
	int					exitCode;
	struct io_descriptor *io;
	struct io_descriptor *ioComputer;
	struct calendar_date date;
	struct calendar_time time;
	char timeArray[10];

	(void)p;

	calendar_enable(&CALENDAR_0);

	date.year  = 2022;
	date.month = 11;
	date.day   = 11;

	time.hour = 20;
	time.min  = 45;
	time.sec  = 30;

	calendar_set_date(&CALENDAR_0, &date);
	calendar_set_time(&CALENDAR_0, &time);

	usart_os_get_io(&COMPUTER, &ioComputer);
	int32_t result = usart_os_get_io(&SBG, &io);
	struct calendar_date_time now;
	calendar_get_date_time(&CALENDAR_0, &now); // fill date_time struct with current time
	sprintf(timeArray, "%ld", get_UNIX_time(&CALENDAR_0, &now));
	COMPUTER.io.write(&COMPUTER.io, (uint8_t *)timeArray, 10);
	errorCode = sbgInterfaceSerialCreate(&sbgInterface, (char *)"SBG", 460800);
	if (errorCode == SBG_NO_ERROR)
	{
		errorCode = sbgTestProcess(&sbgInterface);
		if (errorCode == SBG_NO_ERROR) 
		{
			exitCode = EXIT_SUCCESS;
		}
		else 
		{
			exitCode = EXIT_FAILURE;
		}
		sbgInterfaceDestroy(&sbgInterface);
	}
	else 
	{
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)"Failed to create serial", 20);
		exitCode = EXIT_FAILURE;
	}
	// if (result == 0) { // Successfully assigned io to hardware.
	// 	// Create the connection to the SBG.
	// 	// errorCode = sbgInterfaceSerialCreate(&sbgInterface, (char *)"SBG", 460800);
	// 	if (errorCode == SBG_NO_ERROR) { // Successfully created serial connection.
	// 		errorCode = sbgTestProcess(&sbgInterface);
	// 		if (errorCode == SBG_NO_ERROR) 
	// 		{
	// 			exitCode = EXIT_SUCCESS;
	// 		}
	// 		else {
	// 			exitCode = EXIT_FAILURE;
	// 		}
	// 		sbgInterfaceDestroy(&sbgInterface);
	// 	}
	// 	else 
	// 	{
	// 		SBG_LOG_ERROR(errorCode, "unable to open serial interface");
	// 		exitCode = EXIT_FAILURE;
	// 	}
	// }
	// else {
	// 	COMPUTER.io.write(&COMPUTER.io, (uint8_t *)"Failed to assign io", 20);
	// 	exitCode = EXIT_FAILURE;
	// }
	vTaskDelete(xSBGTask); 
}

int main()
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	// xTaskCreate(COMPUTER_example_task, "USART", TASK_LED_STACK_SIZE, NULL, TASK_SBG_STACK_PRIORITY, &xUSARTTask);
	xTaskCreate(SBG_task, "SBG", TASK_SBG_STACK_SIZE, NULL, TASK_SBG_STACK_PRIORITY, &xSBGTask);
	xTaskCreate(led_task, "LED", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, &xLEDTask);
	vTaskStartScheduler();
	while(1) {
		// Something has gone wrong if we end here.
	}
}
