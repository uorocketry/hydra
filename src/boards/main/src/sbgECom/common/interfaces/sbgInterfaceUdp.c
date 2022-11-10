// sbgCommonLib headers
#include <sbgCommon.h>
#include <interfaces/sbgInterfaceUdp.h>
#include <network/sbgNetwork.h>

// Standard headers
#ifdef WIN32
#include <winsock2.h>
#include <WS2tcpip.h>
#include <stdint.h>

#define SOCKLEN				int
#else // WIN32
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/types.h>
#include <sys/socket.h>

#define SOCKADDR_IN			struct sockaddr_in
#define SOCKADDR			struct sockaddr
#define SOCKET				int
#define SOCKLEN				socklen_t
#define INVALID_SOCKET		(~((SOCKET)0))
#define SOCKET_ERROR		(-1)
#define NO_ERROR			(0)
#define SD_BOTH				(2)

#define closesocket			close
#endif // WIN32

//----------------------------------------------------------------------//
//- Private definitions                                                -//
//----------------------------------------------------------------------//

#define SBG_INTERFACE_UDP_PACKET_MAX_SIZE		(1400)

/*!
 * Structure that stores all internal data used by the UDP interface.
 */
typedef struct _SbgInterfaceUdp
{
	SOCKET 			udpSocket;					/*!< The socket used to send and / or receive some UDP data. */
	sbgIpAddress	remoteAddr;					/*!< IP address to send data to. */
	uint32_t		remotePort;					/*!< Ethernet port to send data to. */
	uint32_t		localPort;					/*!< Ethernet port on which the interface is listening. */
} SbgInterfaceUdp;

//----------------------------------------------------------------------//
//- Private functions                                                  -//
//----------------------------------------------------------------------//

/*!
 * Returns the UDP interface instance.
 *
 * \param[in]	pInterface								Interface instance.
 * \return												The UDP interface instance.
 */
static SbgInterfaceUdp *sbgInterfaceUdpGet(SbgInterface *pInterface)
{
	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_ETH_UDP);
	ASSERT(pInterface->handle);

	return (SbgInterfaceUdp*)pInterface->handle;
}

/*!
 * Initialize the socket API.
 *
 * \return										SBG_NO_ERROR if the socket API has been correctly initialized.
 */
static SbgErrorCode sbgInterfaceUdpInitSockets(void)
{
#ifdef WIN32
	WSADATA wsaData;

	if (WSAStartup(MAKEWORD(2, 2), &wsaData) == NO_ERROR)
	{
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_ERROR;
	}
#else
	return SBG_NO_ERROR;
#endif
}

/*!
 * Uninitialize the socket API.
 *
 * \return										SBG_NO_ERROR if the socket API has been uninitialized.
 */
static SbgErrorCode sbgInterfaceUpdateCloseSockets(void)
{
#ifdef WIN32
	if (WSACleanup() == NO_ERROR)
	{
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_ERROR;
	}
#else
	return SBG_NO_ERROR;
#endif
}

/*!
 * Define if a socket should block or not on receive and send calls.
 *
 * \param[in]	pInterfaceUdp					The UDP interface to change socket mode.
 * \param[in]	blocking						Set to true for a blocking socket or false for a non blocking socket.
 * \return										SBG_NO_ERROR if the blocking status has been changed.
 */
static SbgErrorCode sbgInterfaceUdpSetSocketBlocking(SbgInterfaceUdp *pInterfaceUdp, bool blocking)
{
	ASSERT(pInterfaceUdp);

#ifdef WIN32
	u_long blockingMode;

	blockingMode = (blocking ? 0 : 1);

	if (ioctlsocket(pInterfaceUdp->udpSocket, FIONBIO, &blockingMode) == NO_ERROR)
	{
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_ERROR;
	}
#else // WIN32
	int32_t flags;

	flags = fcntl(pInterfaceUdp->udpSocket, F_GETFL, 0);

	if (flags >= 0)
	{
		flags = (blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK));

		if (fcntl(pInterfaceUdp->udpSocket, F_SETFL, flags) == 0)
		{
			return SBG_NO_ERROR;
		}
	}

	return SBG_ERROR;
#endif // WIN32
}

/*!
 * Destroy an interface initialized using sbgInterfaceUdpCreate.
 *
 * \param[in]	pInterface						Pointer on a valid UDP interface created using sbgInterfaceUdpCreate.
 * \return										SBG_NO_ERROR if the interface has been closed and released.
 */
static SbgErrorCode sbgInterfaceUdpDestroy(SbgInterface *pInterface)
{
	SbgInterfaceUdp		*pUdpHandle;

	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_ETH_UDP);

	pUdpHandle = sbgInterfaceUdpGet(pInterface);

	//
	// Close the socket
	//	
	shutdown(pUdpHandle->udpSocket, SD_BOTH);
	closesocket(pUdpHandle->udpSocket);

	//
	// free the allocated sbgInterfaceUdp instance
	//
	free(pUdpHandle);

	sbgInterfaceZeroInit(pInterface);

	return sbgInterfaceUpdateCloseSockets();
}

/*!
 * Try to write some data to an interface.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
static SbgErrorCode sbgInterfaceUdpWrite(SbgInterface *pInterface, const void *pBuffer, size_t bytesToWrite)
{
	SbgErrorCode			 errorCode;
	SbgInterfaceUdp			*pUdpHandle;
	SOCKADDR_IN				 outAddr;
	
	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_ETH_UDP);
	ASSERT(pBuffer);

	pUdpHandle = sbgInterfaceUdpGet(pInterface);
		
	outAddr.sin_family = AF_INET;
	outAddr.sin_addr.s_addr = pUdpHandle->remoteAddr;
	outAddr.sin_port = htons((uint16_t)pUdpHandle->remotePort);

	while (bytesToWrite != 0)
	{
		int					 partialWriteSize;
		int					 nrBytesSent;

		partialWriteSize = (int)bytesToWrite;

		if (partialWriteSize > SBG_INTERFACE_UDP_PACKET_MAX_SIZE)
		{
			partialWriteSize = SBG_INTERFACE_UDP_PACKET_MAX_SIZE;
		}

		nrBytesSent = sendto(pUdpHandle->udpSocket, pBuffer, partialWriteSize, 0, (SOCKADDR *)&outAddr, sizeof(outAddr));

		if (nrBytesSent != partialWriteSize)
		{
			break;
		}

		bytesToWrite -= (size_t)partialWriteSize;
		pBuffer = (const uint8_t *)pBuffer + partialWriteSize;
	}

	if (bytesToWrite == 0)
	{
		errorCode = SBG_NO_ERROR;
	}
	else
	{
		errorCode = SBG_WRITE_ERROR;
	}

	return errorCode;
}

/*!
 * Try to read some data from an interface.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32_t used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
static SbgErrorCode sbgInterfaceUdpRead(SbgInterface *pInterface, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
	SbgErrorCode			 errorCode;
	SbgInterfaceUdp			*pUdpHandle;
	SOCKADDR_IN				 remoteAddr;
	SOCKLEN					 remoteAddrLen;
	int						 ret;

	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_ETH_UDP);
	ASSERT(pBuffer);
	ASSERT(pReadBytes);

	pUdpHandle = sbgInterfaceUdpGet(pInterface);

	remoteAddrLen = sizeof(remoteAddr);
	ret = recvfrom(pUdpHandle->udpSocket, pBuffer, (int)bytesToRead, 0, (SOCKADDR *)&remoteAddr, &remoteAddrLen);

	if (ret != -1)
	{
		if ((pUdpHandle->remoteAddr == 0) &&
			(pUdpHandle->remotePort == 0))
		{
			pUdpHandle->remoteAddr = remoteAddr.sin_addr.s_addr;
			pUdpHandle->remotePort = ntohs(remoteAddr.sin_port);
		}
		else if ((pUdpHandle->remoteAddr != remoteAddr.sin_addr.s_addr) ||
			(pUdpHandle->remotePort != ntohs(remoteAddr.sin_port)))
		{
			char			 remoteAddrString[16];

			sbgNetworkIpToString(remoteAddr.sin_addr.s_addr, remoteAddrString, sizeof(remoteAddrString));
			SBG_LOG_WARNING(SBG_READ_ERROR, "received data from invalid remote host (%s:%u)", remoteAddrString, ntohs(remoteAddr.sin_port));
			ret = 0;
		}

		errorCode = SBG_NO_ERROR;
	}
#ifdef WIN32
	else if (WSAGetLastError() == WSAEWOULDBLOCK)
#else // WIN32
	else if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
#endif // WIN32
	{
		errorCode = SBG_NO_ERROR;
		ret = 0;
	}
	else
	{
		errorCode = SBG_READ_ERROR;
		SBG_LOG_ERROR(errorCode, "unable to receive data");
	}

	if (errorCode == SBG_NO_ERROR)
	{
		*pReadBytes = (size_t)ret;
	}

	return errorCode;
}


//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceUdpCreate(SbgInterface *pInterface, sbgIpAddress remoteAddr, uint32_t remotePort, uint32_t localPort)
{
	SbgErrorCode					 errorCode;
	
	ASSERT(pInterface);

	//
	// Always call the underlying zero init method to make sure we can correctly handle SbgInterface evolutions
	//
	sbgInterfaceZeroInit(pInterface);

	errorCode = sbgInterfaceUdpInitSockets();

	if (errorCode == SBG_NO_ERROR)
	{
		SbgInterfaceUdp				*pNewUdpHandle;

		pNewUdpHandle = malloc(sizeof(*pNewUdpHandle));

		if (pNewUdpHandle)
		{
			pNewUdpHandle->remoteAddr	= remoteAddr;
			pNewUdpHandle->remotePort	= remotePort;
			pNewUdpHandle->localPort	= localPort;

			pNewUdpHandle->udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

			if (pNewUdpHandle->udpSocket != INVALID_SOCKET)
			{
				errorCode = sbgInterfaceUdpSetSocketBlocking(pNewUdpHandle, false);

				if (errorCode == SBG_NO_ERROR)
				{
					SOCKADDR_IN	 bindAddress;
					int			 socketError;

					bindAddress.sin_family			= AF_INET;
					bindAddress.sin_addr.s_addr		= INADDR_ANY;
					bindAddress.sin_port			= htons((uint16_t)localPort);

					socketError = bind(pNewUdpHandle->udpSocket, (SOCKADDR *)&bindAddress, sizeof(bindAddress));

					if (socketError != SOCKET_ERROR)
					{
						char	interfaceName[48];
						char	ipStr[16];

						//
						// The serial port is ready so create a new serial interface
						//
						pInterface->handle			= pNewUdpHandle;
						pInterface->type			= SBG_IF_TYPE_ETH_UDP;

						//
						// Define the interface name
						//
						sbgNetworkIpToString(remoteAddr, ipStr, sizeof(ipStr));
						sprintf(interfaceName, "UDP: %s out: %u in: %u", ipStr, remotePort, localPort);
						sbgInterfaceNameSet(pInterface, interfaceName);
							
						//
						// Define all overloaded members
						//
						pInterface->pDestroyFunc	= sbgInterfaceUdpDestroy;
						pInterface->pReadFunc		= sbgInterfaceUdpRead;
						pInterface->pWriteFunc		= sbgInterfaceUdpWrite;

						return SBG_NO_ERROR;
					}
					else
					{
						errorCode = SBG_ERROR;
						SBG_LOG_ERROR(errorCode, "unable to bind socket");
					}
				}
				else
				{
					errorCode = SBG_ERROR;
					SBG_LOG_ERROR(errorCode, "unable to set non-blocking mode");
				}

				shutdown(pNewUdpHandle->udpSocket, SD_BOTH);
				closesocket(pNewUdpHandle->udpSocket);
			}
			else
			{
				errorCode = SBG_ERROR;
				SBG_LOG_ERROR(errorCode, "unable to create socket");
			}

			SBG_FREE(pNewUdpHandle);
		}
		else
		{
			errorCode = SBG_MALLOC_FAILED;
			SBG_LOG_ERROR(errorCode, "unable to allocate handle");
		}

		sbgInterfaceUpdateCloseSockets();
	}

	return errorCode;
}



SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceUdpAllowBroadcast(SbgInterface *pInterface, bool allowBroadcast)
{
	SbgErrorCode			 errorCode;
	SbgInterfaceUdp			*pUdpHandle;
	int						 socketError;
	int						 optValue;

	ASSERT(pInterface);
	ASSERT(pInterface->type == SBG_IF_TYPE_ETH_UDP);

	pUdpHandle = sbgInterfaceUdpGet(pInterface);

	//
	// Change socket options to allow broadcast
	//
	optValue = allowBroadcast;
	socketError = setsockopt(pUdpHandle->udpSocket, SOL_SOCKET, SO_BROADCAST, (const char *)&optValue, sizeof(optValue));

	if (socketError == NO_ERROR)
	{
		errorCode = SBG_NO_ERROR;
	}
	else
	{
		errorCode = SBG_ERROR;
		SBG_LOG_ERROR(errorCode, "unable to set socket options");
	}

	return errorCode;
}
