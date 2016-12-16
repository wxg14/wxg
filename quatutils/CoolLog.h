/********************************************************************
	created:	2007/06/16
	created:	16:6:2007   15:19
	filename: 	f:\project\Fue\Source\4uLib\4uClass\CoolLog.h
	file path:	f:\project\Fue\Source\4uLib\4uClass
	file base:	CoolLog
	file ext:	h
	author:		Tod
	
	purpose:	���Ź���������־�����
	Copyright (c) Beijing 4u Age Technologies Inc. , 2007
	All rights reserved.
*********************************************************************/

/*
 *  �Զ�����log�ļ�����Ĭ��λ��Ϊexe�ļ�λ���µ�log�ļ���
 *  Ĭ�ϵ�log�ļ���Ϊexe�ļ���.log
 *  ���Ҫ�����ļ�����ֻҪ��SETLOGFILEPATH("�ļ�ȫ·��")����
 *  ������־��PRINTF("����%d", 1)������
*/

#ifndef ______COOLLOG_H______
#define ______COOLLOG_H______

#include <stdlib.h>
#include <Windows.h>


#define USE_STL
//#undef  USE_STL

//#define USE_MFC
#undef  USE_MFC

#ifdef USE_STL
#include <string>
using namespace std;
#endif

/*
 *	Macro definitions
 */
#define IFDEBUG( doit )	doit

#define PRINTF			debug.printf
#define SETLOGFILEPATH  debug.SetLogFileName
#define SHOW_CONSOLE	debug.ShowConsole();
#define HIDE_CONSOLE	debug.HideConsole();
#define SHOW_LASTERROR	debug.ShowLastError();
#define CLOSE_LOG		debug.CloseLog();

/*-----------------------------------------------------------------------
 *  Class        : CDebugPrintf
 *  Prototype    : class CDebugPrintf
 *  Description  : 
 *  Parent class : 
 *  History      : 
 */
class CDebugPrintf
{
public:
	CDebugPrintf();
	virtual ~CDebugPrintf();

	void printf( const char *fmt, ... );

#ifdef USE_STL
	void printf( string& str ) { printf( str.c_str() ); }
	void printf( string* str ) { printf( str->c_str() ); }
#endif

#ifdef USE_MFC
	void printf( CString& str ) { printf( str.GetBuffer(1024) ); }
	void printf( CString* str ) { printf( str->GetBuffer(1024) ); }
#endif

	void CloseLog();

	void ShowConsole(); 
	void HideConsole(); 
	void ShowLastError();
	bool SetLogFileName(const char *szFilePath);

private:
	bool Init();
	void CheckIsShowConsole();					// check is show console
	HANDLE			 m_hFile;					// log-file handle
	HANDLE			 m_hDebugMutex;				// allow to check that log-file is open by other process 
	CRITICAL_SECTION m_hLock;					// lock for thread-safe access

	DWORD			 m_dwStartTime;				// application start time 
	bool			 m_bUseConsole;				// use or not log-console with log-file

	HANDLE			 m_hConWrite;				// console handle
	TCHAR			 m_szLogFileName[_MAX_PATH];// log file path
	TCHAR			 m_szShowConsolePath[_MAX_PATH];// show console path

	char			 m_szBuffer[1024];
};

extern CDebugPrintf debug;

#endif


