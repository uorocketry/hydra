// Standard headers
#include <stdio.h>
#include <string.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <errno.h>
// #include <termios.h>
// #include <sys/ioctl.h>

// sbgCommonLib headers
#include <sbgCommon.h>
#include <interfaces/sbgInterfaceSerial.h>

//----------------------------------------------------------------------//
//- Definitions                                                        -//
//----------------------------------------------------------------------//
#define SBG_IF_SERIAL_TX_BUFFER_SIZE			(16)					/*!< Define the transmission buffer size for the serial port. */
#define SBG_IF_SERIAL_RX_BUFFER_SIZE			(16)					/*!< Define the reception buffer size for the serial port. */

//----------------------------------------------------------------------//
//- Private methods declarations                                       -//
//----------------------------------------------------------------------//

/*!
 *	Destroy an interface initialized using sbgInterfaceSerialCreate.
 * 
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										SBG_NO_ERROR if the interface has been closed and released.
 */
static SbgErrorCode sbgInterfaceSerialDestroy(SbgInterface *pInterface)
{
	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_SERIAL);
		
	//
	// Test that we have a valid interface
	//
	if (pInterface)
	{

		//
		// Close the port com
		//
		usart_os_disable(SBG);

		// SBG_FREE(pSerialHandle);
		pInterface->handle = NULL;

		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 * Make an interface flush pending input and/or output data.
 *
 * If flags include SBG_IF_FLUSH_INPUT, all pending input data is discarded.
 * If flags include SBG_IF_FLUSH_OUTPUT, the function blocks until all output data has been written out.
 *
 * \param[in]	pInterface								Interface instance.
 * \param[in]	flags									Combination of the SBG_IF_FLUSH_INPUT and SBG_IF_FLUSH_OUTPUT flags.
 * \return												SBG_NO_ERROR if successful.
 */
static SbgErrorCode sbgInterfaceSerialFlush(SbgInterface *pInterface, uint32_t flags)
{
	SbgErrorCode					 errorCode;
	int								 fd;
	int								 result = 0;

	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_SERIAL);

	fd = *((int *)pInterface->handle);

	if ((result == 0) && (flags & SBG_IF_FLUSH_INPUT))
	{
		result = tcflush(fd, TCIFLUSH);

		if (result != 0)
		{
			SBG_LOG_ERROR(SBG_READ_ERROR, "unable to flush input, error:%s", strerror(errno));
		}
	}

	if ((result == 0) && (flags & SBG_IF_FLUSH_OUTPUT))
	{
		result = tcdrain(fd);

		if (result != 0)
		{
			SBG_LOG_ERROR(SBG_WRITE_ERROR, "unable to flush output, error:%s", strerror(errno));
		}
	}

	if (result == 0)
	{
		errorCode = SBG_NO_ERROR;
	}
	else
	{
		errorCode = SBG_ERROR;
	}
	
	return errorCode;
}

/*!
 * Change the serial interface baud rate immediately.
 * 
 * \param[in]	handle				Valid handle on an initialized interface.
 * \param[in]	baudRate			The new baudrate to apply in bps.
 * \return							SBG_NO_ERROR if everything is OK
 */
static SbgErrorCode sbgInterfaceSerialChangeBaudrate(SbgInterface *pInterface, uint32_t baudRate)
{
	uint32_t		response;

	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_SERIAL);
	//
	// Retrieve current options
	//
	if (pInterface)
	{	
		response = usart_os_set_baud_rate(SBG, baudRate); // may need to handle this differently 
		if (response != 0)
		{
			uint8_t errorMsg = "sbgInterfaceSerialChangeBaudrate: Unable to set speed.\n";
			SBG.io.write(&SBG.io, (uint8_t *)errorMsg, sizeof(errorMsg));
			return SBG_ERROR;
		}
	}
	else
	{
		uint8_t errorMsg = "sbgInterfaceSerialChangeBaudrate: Invalid.\n";
		SBG.io.write(&SBG.io, (uint8_t *)errorMsg, sizeof(errorMsg));
		return SBG_ERROR;
	}
	return SBG_NO_ERROR;
}

//----------------------------------------------------------------------//
//- Internal interfaces write/read implementations                     -//
//----------------------------------------------------------------------//

/*!
 * Try to write some data to an interface.
 * 
 * \param[in]	pInterface								Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
static SbgErrorCode sbgInterfaceSerialWrite(SbgInterface *pInterface, const void *pBuffer, size_t bytesToWrite)
{
	size_t          numBytesLeftToWrite = bytesToWrite;
	uint8_t        *pCurrentBuffer = (uint8_t*)pBuffer;
	ssize_t         numBytesWritten;
	int	            hSerialHandle;

	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_SERIAL);
	ASSERT(pBuffer);
	
	//
	// Get the internal serial handle
	//
	hSerialHandle = *((int*)pInterface->handle);

	//
	// Write the whole buffer
	//
	while (numBytesLeftToWrite > 0)
	{
		//
		// Write these bytes to the serial interface
		//
		numBytesWritten = write(hSerialHandle, pCurrentBuffer, numBytesLeftToWrite);
			
		//
		// Test the there is no error
		//	
		if (numBytesWritten == -1)
		{
			if (errno == EAGAIN)
			{
				sbgSleep(1);
			}
			else
			{
				//
				// An error has occurred during the write
				//
				fprintf(stderr, "sbgDeviceWrite: Unable to write to our device: %s\n", strerror(errno));
				return SBG_WRITE_ERROR;
			}
		}
		else
		{
			//
			// Update the buffer pointer and the number of bytes to write
			//
			numBytesLeftToWrite -= (size_t)numBytesWritten;
			pCurrentBuffer += (size_t)numBytesWritten;
		}
	}

	return SBG_NO_ERROR;
}

/*!
 * Try to read some data from an interface.
 * 
 * \param[in]	pInterface								Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
static SbgErrorCode sbgInterfaceSerialRead(SbgInterface *pInterface, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
	SbgErrorCode    errorCode;
	ssize_t			numBytesRead;

	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_SERIAL);
	ASSERT(pBuffer);
	ASSERT(pReadBytes);
		
	//
	// Read our buffer
	//
	numBytesRead = SBG.io.read(SBG.io, (uint8_t *)pBuffer, bytesToRead);
		
	//
	// Check if the read operation was successful
	//
	if (numBytesRead >= 0)
	{
		errorCode = SBG_NO_ERROR;
	}
	else
	{
		errorCode = SBG_READ_ERROR;
		numBytesRead = 0;
	}
		
	//
	// If we can, returns the number of read bytes
	//
	*pReadBytes = (size_t)numBytesRead;
		
	return errorCode;
}

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgInterfaceSerialCreate(SbgInterface *pInterface, const char *deviceName, uint32_t baudRate)
{
	ASSERT(pInterface);
	ASSERT(deviceName);

	//
	// Always call the underlying zero init method to make sure we can correctly handle SbgInterface evolutions
	//
	sbgInterfaceZeroInit(pInterface);

	int32_t serial = usart_os_enable(SBG);
			
	//
	// Test that the port has been initialized
	//
	if (serial == 0)
	{
		//
		// The serial port is ready so create a new serial interface
		//
		pInterface->handle = (void*)pSerialHandle;
		pInterface->type = SBG_IF_TYPE_SERIAL;

		//
		// Define the interface name
		//
		sbgInterfaceNameSet(pInterface, deviceName);

		//
		// Define all overloaded members
		//
		pInterface->pDestroyFunc	= sbgInterfaceSerialDestroy;
		pInterface->pReadFunc		= sbgInterfaceSerialRead;
		pInterface->pWriteFunc		= sbgInterfaceSerialWrite;
		pInterface->pFlushFunc		= sbgInterfaceSerialFlush;
		pInterface->pSetSpeedFunc	= sbgInterfaceSerialChangeBaudrate;

		//
		// Purge the communication
		//
		return sbgInterfaceSerialFlush(pInterface, SBG_IF_FLUSH_ALL);
	}
	else // log the error
	{
		char errorMsg[sizeof(deviceName) + 56];
		int n = sprintf(errorMsg, "sbgInterfaceSerialCreate: Unable to open the com port: %s\n", deviceName);
		SBG.io.write(&SBG.io, (uint8_t *)errorMsg, n);
	}
			
	//
	//	Release the allocated serial handle
	//
	SBG_FREE(pSerialHandle);

	return SBG_ERROR;
}
