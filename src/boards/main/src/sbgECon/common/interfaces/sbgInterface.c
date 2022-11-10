/* sbgCommonLib headers */
#include <sbgCommon.h>

/* Local headers */
#include "sbgInterface.h"

//----------------------------------------------------------------------//
//- Private definitions                                                -//
//----------------------------------------------------------------------//

static const char *gInterfaceType[] = 
{
	[SBG_IF_TYPE_UNKNOW]		= "unknown",
	[SBG_IF_TYPE_SERIAL]		= "serial",
	[SBG_IF_TYPE_ETH_UDP]		= "eth UDP",
	[SBG_IF_TYPE_ETH_TCP_IP]	= "eth TCP",
	[SBG_IF_TYPE_FILE]			= "file"
};

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

void sbgInterfaceZeroInit(SbgInterface *pInterface)
{
	ASSERT(pInterface);

	//
	// Make sure the whole struct is zero init
	//
	memset(pInterface, 0x00, sizeof(*pInterface));
}

SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceDestroy(SbgInterface *pInterface)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;

	ASSERT(pInterface);

	if (pInterface->pDestroyFunc)
	{
		errorCode = pInterface->pDestroyFunc(pInterface);
	}
	
	return errorCode;
}

const char *sbgInterfaceTypeGetAsString(const SbgInterface *pInterface)
{
	ASSERT(pInterface);

	if (pInterface->type < SBG_ARRAY_SIZE(gInterfaceType))
	{
		return gInterfaceType[pInterface->type];
	}
	else
	{
		SBG_LOG_ERROR(SBG_INVALID_PARAMETER, "Unknown interface type: %" PRIu32, pInterface->type);
		return gInterfaceType[SBG_IF_TYPE_UNKNOW];
	}
}

void sbgInterfaceNameSet(SbgInterface *pInterface, const char *pName)
{
	size_t	nameLen;

	ASSERT(pInterface);
	ASSERT(pName);

	//
	// Only keep the end of the name that can fit in the interface name buffer
	//
	nameLen = strlen(pName);

	if (nameLen < SBG_ARRAY_SIZE(pInterface->name))
	{
		strcpy(pInterface->name, pName);
	}
	else
	{
		strcpy(pInterface->name, pName+(nameLen-(SBG_ARRAY_SIZE(pInterface->name)-1)));
	}
}
