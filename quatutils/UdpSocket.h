/********************************************************************
	created:	2007/06/16
	created:	16:6:2007   15:20
	filename: 	f:\project\Fue\Source\4uLib\4uClass\UdpSocket.h
	file path:	f:\project\Fue\Source\4uLib\4uClass
	file base:	UdpSocket
	file ext:	h
	author:		Tod
	
	purpose:	世优UDP公用通讯模块
	Copyright (c) Beijing 4u Age Technologies Inc. , 2007
	All rights reserved.
*********************************************************************/

// UdpSocket.h: interface for the CUdpSocket class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_UDPSOCKET_H__0E929E79_B2F3_4B36_B04A_F9F5949B1700__INCLUDED_)
#define AFX_UDPSOCKET_H__0E929E79_B2F3_4B36_B04A_F9F5949B1700__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include "quatutils_global.h"
#include "winsock2.h"
#pragma comment(lib, "Ws2_32.lib")

#define	UDP_RECV_BUF		2048

// 接收数据的回调函数(数据指针,长度,源IP,端口)
typedef void (*RecvCallback)(char*, DWORD, char*, WORD);

class QUATUTILS_EXPORT CUdpSocket  
{
public:
	CUdpSocket();
	virtual ~CUdpSocket();

public:
	BOOL BindSocket(UINT uRecvPort, void *callback, int iRecvBuf = 0, BOOL bUseRecvThreadFlag = TRUE);
	int SendData(const char* pszIP, const UINT uPort, const char* pszBuf, int iBufLen);
	int SendBroadcastData(const UINT uPort, const char* pszBuf, int iBufLen);
	DWORD RecvData();
	BOOL CloseSocket();
private:
	SOCKET		m_sSocket;

	HANDLE		m_hThread;
	HANDLE		m_hExitRecvThread;
	HANDLE		m_hExitRecvThreadCompleted;

	void		*m_pfn;
	BOOL		m_bVistaFlag;
};

#endif // !defined(AFX_UDPSOCKET_H__0E929E79_B2F3_4B36_B04A_F9F5949B1700__INCLUDED_)
