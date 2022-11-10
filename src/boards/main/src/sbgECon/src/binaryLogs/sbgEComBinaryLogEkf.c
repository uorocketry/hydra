﻿#include "sbgEComBinaryLogEkf.h"

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComBinaryLogParseEkfEulerData(SbgStreamBuffer *pInputStream, SbgLogEkfEulerData *pOutputData)
{
	ASSERT(pInputStream);
	ASSERT(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp		= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->euler[0]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->euler[1]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->euler[2]		= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->eulerStdDev[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->eulerStdDev[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->eulerStdDev[2]	= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->status			= sbgStreamBufferReadUint32LE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

SbgErrorCode sbgEComBinaryLogWriteEkfEulerData(SbgStreamBuffer *pOutputStream, const SbgLogEkfEulerData *pInputData)
{
	ASSERT(pOutputStream);
	ASSERT(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->euler[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->euler[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->euler[2]);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->eulerStdDev[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->eulerStdDev[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->eulerStdDev[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->status);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}

SbgErrorCode sbgEComBinaryLogParseEkfQuatData(SbgStreamBuffer *pInputStream, SbgLogEkfQuatData *pOutputData)
{
	ASSERT(pInputStream);
	ASSERT(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp		= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->quaternion[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->quaternion[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->quaternion[2]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->quaternion[3]	= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->eulerStdDev[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->eulerStdDev[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->eulerStdDev[2]	= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->status			= sbgStreamBufferReadUint32LE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

SbgErrorCode sbgEComBinaryLogWriteEkfQuatData(SbgStreamBuffer *pOutputStream, const SbgLogEkfQuatData *pInputData)
{
	ASSERT(pOutputStream);
	ASSERT(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->quaternion[0]); 
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->quaternion[1]); 
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->quaternion[2]); 
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->quaternion[3]); 

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->eulerStdDev[0]); 
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->eulerStdDev[1]); 
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->eulerStdDev[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->status);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}

SbgErrorCode sbgEComBinaryLogParseEkfNavData(SbgStreamBuffer *pInputStream, SbgLogEkfNavData *pOutputData)
{
	ASSERT(pInputStream);
	ASSERT(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->velocity[0]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->velocity[1]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->velocity[2]		= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->velocityStdDev[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->velocityStdDev[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->velocityStdDev[2]	= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->position[0]		= sbgStreamBufferReadDoubleLE(pInputStream);
	pOutputData->position[1]		= sbgStreamBufferReadDoubleLE(pInputStream);
	pOutputData->position[2]		= sbgStreamBufferReadDoubleLE(pInputStream);

	pOutputData->undulation			= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->positionStdDev[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->positionStdDev[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->positionStdDev[2]	= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->status				= sbgStreamBufferReadUint32LE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

SbgErrorCode sbgEComBinaryLogWriteEkfNavData(SbgStreamBuffer *pOutputStream, const SbgLogEkfNavData *pInputData)
{
	ASSERT(pOutputStream);
	ASSERT(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocity[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocity[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocity[2]);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocityStdDev[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocityStdDev[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocityStdDev[2]);

	sbgStreamBufferWriteDoubleLE(pOutputStream, pInputData->position[0]);
	sbgStreamBufferWriteDoubleLE(pOutputStream, pInputData->position[1]);
	sbgStreamBufferWriteDoubleLE(pOutputStream, pInputData->position[2]);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->undulation);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->positionStdDev[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->positionStdDev[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->positionStdDev[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->status);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
