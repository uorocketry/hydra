// sbgCommonLib headers
#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Include specific header for WIN32 and UNIX platforms               -//
//----------------------------------------------------------------------//


//----------------------------------------------------------------------//
//- Global singleton for the log callback							   -//
//----------------------------------------------------------------------//

/*!
 * Unique singleton used to log error messages.
 */
SbgCommonLibOnLogFunc	gLogCallback = NULL;

//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

SBG_COMMON_LIB_API uint32_t sbgGetTime(void)
{
	return 200;
}

SBG_COMMON_LIB_API void sbgSleep(uint32_t ms)
{

}

SBG_COMMON_LIB_API void sbgCommonLibSetLogCallback(SbgCommonLibOnLogFunc logCallback)
{
	//
	// TODO: should we implement lock / sync mechanisms ?
	//
	gLogCallback = logCallback;
}

SBG_COMMON_LIB_API void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...)
{

}
