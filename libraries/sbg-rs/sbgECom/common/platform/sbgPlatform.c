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


SBG_COMMON_LIB_API void sbgCommonLibSetLogCallback(SbgCommonLibOnLogFunc logCallback)
{
	//
	// TODO: should we implement lock / sync mechanisms ?
	//
	gLogCallback = logCallback;
}

