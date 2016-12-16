// UdpSocket.cpp: implementation of the CUdpSocket class.
//
//////////////////////////////////////////////////////////////////////
#include "UdpSocket.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
DWORD WINAPI RecvMsgThreadProc(LPVOID lpRunObj)
{
	CUdpSocket* pUdpSocket = (CUdpSocket*)lpRunObj;
	return pUdpSocket->RecvData();
}

DWORD CUdpSocket::RecvData()
{
	fd_set	fdread;
	int		ret;
	int		iSockSize;
	TCHAR   szRecvBuffer[UDP_RECV_BUF + 1];
	RecvCallback fn = (RecvCallback)m_pfn;

	while(WAIT_TIMEOUT == WaitForSingleObject(m_hExitRecvThread, 0))
	{
		timeval to;
		to.tv_sec = 1;
		to.tv_usec = 500;

		FD_ZERO(&fdread);
		FD_SET(m_sSocket, &fdread);
		if((ret = select(0, &fdread, NULL, NULL, &to)) == SOCKET_ERROR)  
		{
			continue;
		}

 		if(ret > 0)
		{
			if(FD_ISSET(m_sSocket, &fdread))
			{
				sockaddr_in from;
				iSockSize = sizeof(from);
				ret = recvfrom(m_sSocket, szRecvBuffer, UDP_RECV_BUF, 0, (struct sockaddr *)&from, &iSockSize);		

//PRINTF("formip=%s, fromport=%d,%d ret=%d", inet_ntoa(from.sin_addr), from.sin_port, ntohs(from.sin_port), ret);
				if(ret == SOCKET_ERROR)
				{
					int err =  WSAGetLastError();
					//如果远程主机关闭连，继续进行服务
					if(err == 10054)
					{
						
					}
					continue;
				}
				else
				{
					if(ret > 0)
					{
						szRecvBuffer[ret] = '\0';
						(*fn)(szRecvBuffer, ret, inet_ntoa(from.sin_addr), ntohs(from.sin_port));
					}
				}
			}
		}
	}

	// set exit completed event
	SetEvent(m_hExitRecvThreadCompleted);

	return 1;
}

CUdpSocket::CUdpSocket()
{
	m_sSocket = INVALID_SOCKET;
	m_hThread = NULL;
	m_hExitRecvThread = NULL;
	m_hExitRecvThreadCompleted = NULL;

	WORD wVersionRequested = MAKEWORD(2, 2);
	WSADATA wsaData;
	if(WSAStartup(wVersionRequested, &wsaData) != 0)
	{
		return;
	}
}

CUdpSocket::~CUdpSocket()
{
	// close
	CloseSocket();

	// clean
	//WSACleanup();
}

// bind
BOOL CUdpSocket::BindSocket(UINT uRecvPort, void *callback, int iRecvBuf, BOOL bUseRecvThreadFlag)
{
	// 如果sokcet还未被关闭
	if(m_sSocket != INVALID_SOCKET)
		return FALSE;

	// 创建SOCKET
	m_sSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if(m_sSocket == INVALID_SOCKET)
		return FALSE;
	
	// 使socket发送的数据具有广播特性
	BOOL bBroadcast = 1;
	setsockopt(m_sSocket, SOL_SOCKET, SO_BROADCAST, (const char*)&bBroadcast, sizeof(BOOL));
	if (iRecvBuf == 0)
		iRecvBuf = 1024*4*1024; //设置为32K	
	// 设置socket的接收缓冲
	if(iRecvBuf > 0)
		setsockopt(m_sSocket, SOL_SOCKET, SO_RCVBUF, (const char*)&iRecvBuf, sizeof(int));

	setsockopt( m_sSocket, SOL_SOCKET, SO_SNDBUF, ( const char* )&iRecvBuf, sizeof( int ) );

	sockaddr_in l_sockaddr;
	l_sockaddr.sin_family = AF_INET;
	l_sockaddr.sin_port = htons(uRecvPort);
	l_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	if(bind(m_sSocket, (struct sockaddr *)&l_sockaddr, sizeof(sockaddr_in)) == SOCKET_ERROR)
	{
		closesocket(m_sSocket);
		m_sSocket = INVALID_SOCKET;
		return FALSE;
	}

	if(!bUseRecvThreadFlag)
		return TRUE;

	// 创建消息接收线程
	DWORD dwThreadId;
	m_hThread = CreateThread(NULL, 0, RecvMsgThreadProc, (LPVOID)this, FALSE, &dwThreadId);
	if (!m_hThread)
		return FALSE;

	m_hExitRecvThread = CreateEvent(NULL, FALSE, FALSE, NULL);
	m_hExitRecvThreadCompleted = CreateEvent(NULL, FALSE, FALSE, NULL);

	// 设置回调
	m_pfn = callback;
	return TRUE;
}

// send data
int CUdpSocket::SendData(const char* pszIP, const UINT uPort, const char* pszBuf, int iBufLen)
{
	int		l_iRet = -1;

	if(m_sSocket != INVALID_SOCKET)
	{
		fd_set	fdwrite;
		FD_ZERO(&fdwrite);
		FD_SET(m_sSocket, &fdwrite);
		if((l_iRet = select(0, NULL, &fdwrite, NULL, NULL)) == SOCKET_ERROR)  
		{
			int err = WSAGetLastError();
			return l_iRet;
		}

		if(l_iRet > 0)
		{
			// 判断网络是否可写
			if(FD_ISSET(m_sSocket, &fdwrite))
			{
				struct sockaddr_in l_sockaddr;
				memset(&l_sockaddr, 0, sizeof(l_sockaddr) );
				l_sockaddr.sin_family = PF_INET;
				l_sockaddr.sin_port = htons(uPort);
				l_sockaddr.sin_addr.s_addr = inet_addr(pszIP);

				l_iRet = sendto(m_sSocket, pszBuf, iBufLen, 0, (struct sockaddr *)(&l_sockaddr), sizeof(l_sockaddr));
			}
		}
	}
	
	return l_iRet;
}

// send broadcast data
int CUdpSocket::SendBroadcastData(const UINT uPort, const char* pszBuf, int iBufLen)
{
	BOOL bBroadcast = 1;
	int l_iRet = -1;

	if(m_sSocket != INVALID_SOCKET)
	{
		fd_set fdwrite;
		FD_ZERO(&fdwrite);
		FD_SET(m_sSocket,&fdwrite);
		if((l_iRet = select(0, NULL, &fdwrite, NULL, NULL)) == SOCKET_ERROR)  
		{
			int err = WSAGetLastError();
			return l_iRet;
		}
		
		if(l_iRet > 0)
		{
			// 判断网络是否可写
			if(FD_ISSET(m_sSocket, &fdwrite))
			{
				struct sockaddr_in l_sockaddr;
				memset(&l_sockaddr, 0, sizeof(l_sockaddr) );
				l_sockaddr.sin_family = AF_INET;
				l_sockaddr.sin_addr.s_addr = INADDR_BROADCAST;
				l_sockaddr.sin_port = htons(uPort);

				l_iRet = sendto(m_sSocket, pszBuf, iBufLen, 0, (struct sockaddr *)&l_sockaddr, sizeof(l_sockaddr));
			}
		}
	}
	
	return l_iRet;
}

// close
BOOL CUdpSocket::CloseSocket()
{
	if(m_sSocket == INVALID_SOCKET)
		return FALSE;
	
	// 退出线程
	if(m_hThread)
	{
		SetEvent(m_hExitRecvThread);
		if(WAIT_TIMEOUT == WaitForSingleObject(m_hExitRecvThreadCompleted, 3000))
		{
			if (m_hThread)
			{
				::TerminateThread(m_hThread, -1);
			}
		}
		CloseHandle(m_hThread);	m_hThread = NULL;
	}

	// 关闭句柄
	CloseHandle(m_hExitRecvThread);	m_hExitRecvThread = NULL;
	CloseHandle(m_hExitRecvThreadCompleted); m_hExitRecvThreadCompleted = NULL;

	// 关闭SOCKET
	closesocket(m_sSocket);
	m_sSocket = INVALID_SOCKET;

	return TRUE;
}



