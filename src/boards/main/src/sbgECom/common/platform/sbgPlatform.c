// Standard headers
#include <stdarg.h>
#include <time.h>

// sbgCommonLib headers
#include <sbgCommon.h>


//----------------------------------------------------------------------//
//- Specific timing methods to reimplement for your platform           -//
//----------------------------------------------------------------------//

/*!
 *	Returns the current time in ms.
 *	\return				The current time in ms.
 */
SBG_COMMON_LIB_API uint32_t sbgGetTime(void)
{
	// struct timespec now;
	// clock_gettime(CLOCK_REALTIME, &now);
	//
	// Return the current time in ms
	//
	// return now.tv_sec * 1000 + now.tv_nsec / 1000000;
	return xTaskGetTickCount();
}

/*!
 *	Sleep for the specified number of ms.
 *	\param[in]	ms		Number of millisecondes to wait.
 */
SBG_COMMON_LIB_API void sbgSleep(uint32_t ms)
{
	// struct timespec			 req;
	// struct timespec			 rem;
	// int						 ret;

	// req.tv_sec = ms / 1000;
	// req.tv_nsec = (ms % 1000) * 1000000L;

	// for (;;)
	// {
	// 	ret = nanosleep(&req, &rem);

	// 	if ((ret == 0) || (errno != EINTR))
	// 	{
	// 		break;
	// 	}
	// 	else
	// 	{
	// 		req = rem;
	// 	}
	// }
	os_sleep(ms); // only valid if tick rate is 1000Hz, otherwise a conversion is needed.
}

//----------------------------------------------------------------------//
//- Specific logging methods to reimplement for your platform          -//
//----------------------------------------------------------------------//

/*!
 *	The method is called when one of the SBG_LOG_ERROR, SBG_LOG_WARNING, SBG_LOG_INFO or SBG_LOG_VERBOSE is called.
 *	It logs an error message with debug information and support a variable list of arguments
 *	\param[in]	pFileName					File name where the error occurred.
 *	\param[in]	pFunctionName				Function name where the error occurred.
 *	\param[in]	line						Line number where the error occurred.
 *	\param[in]	logType						Define if we have an error, a warning, an info or a verbose log.
 *	\param[in]	errorCode					The error code associated with the message.
 *	\param[in]	pFormat						The error message that will be used with the variable list of arguments.
 */
SBG_COMMON_LIB_API void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...)
{
	int n;
	char errorMsg[SBG_CONFIG_LOG_MAX_SIZE];
	va_list		args;

	ASSERT(pFunctionName);
	ASSERT(pFormat);

	SBG_UNUSED_PARAMETER(pFileName);
	SBG_UNUSED_PARAMETER(pCategory);
	
	// Initialize the list of variable arguments on the latest function argument
	//
	va_start(args, pFormat);

	//
	// Generate the error message string
	//
	vsprintf(errorMsg, pFormat, args); 

	//
	// Close the list of variable arguments
	//
	va_end(args);

	//
	// Log the correct message according to the log type
	//
	switch (logType)
	{
	case SBG_DEBUG_LOG_TYPE_ERROR:
		n = sprintf(errorMsg, "*ERR * %s(%"PRIu32"): %s - %s\r\n", pFunctionName, line, sbgErrorCodeToString(errorCode), errorMsg);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)errorMsg, n);
		break;
	case SBG_DEBUG_LOG_TYPE_WARNING:
		n = sprintf(errorMsg, "*WARN* %s(%"PRIu32"): %s - %s\r\n", pFunctionName, line, sbgErrorCodeToString(errorCode), errorMsg); 
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)errorMsg, n);
		break;
	case SBG_DEBUG_LOG_TYPE_INFO:
		n = sprintf(errorMsg, "*INFO* %s(%"PRIu32"): %s\r\n", pFunctionName, line, errorMsg); 
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)errorMsg, n);
		break;
	case SBG_DEBUG_LOG_TYPE_DEBUG:
		n = sprintf(errorMsg, "*DBG * %s(%"PRIu32"): %s\r\n", pFunctionName, line, errorMsg); 
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)errorMsg, n);
		break;
	default:
		n = sprintf(errorMsg, "*UKNW* %s(%"PRIu32"): %s\r\n", pFunctionName, line, errorMsg);
		COMPUTER.io.write(&COMPUTER.io, (uint8_t *)errorMsg, n);
		break;
	}
}
