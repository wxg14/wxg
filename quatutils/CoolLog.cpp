//#include"stdafx.h"
//#include <windows.h> 
#include <io.h>
#include "CoolLog.h"
#include <TCHAR.H>


const char DIVIDER[10] = { 13, 10, '-', '-', '-', '-', '-', '-', 13, 10 };
const char CrLf[3]	   = { 13, 10, 0 };

const char MUTEX_NAME[]	  = "LogByTod";	
CDebugPrintf debug;

/*-----------------------------------------------------------------------
 *  Function    : 
 *  Prototype   : CDebugPrintf::CDebugPrintf ()
 *  Description : 
 *  Parameters  :                   
 *  Return      : 
 *  History     : 
 */
CDebugPrintf::CDebugPrintf ()
{
	// 1.得到默认的日志文件名, 以及是否输出命令行窗口的路径
	// 1-1得到exe全路径
	TCHAR szExeFilePath[_MAX_PATH], szTemp[_MAX_PATH];
	GetModuleFileName(NULL, szExeFilePath, _MAX_PATH);

	// 1-2.得到exe的不带.exe和路径的纯名称, 如:c:\test\tod.exe 得到tod
	_tcscpy(szTemp, _tcsrchr(szExeFilePath, '\\') + 1);// 得到带.exe的EXE文件全名: tod.exe
	*(_tcsrchr(szTemp, '.')) = '\0'; //EXE的文件名,不包括扩展名: tod
	
	// 1-3.得到默认的日志文件路径
	_tcscpy(m_szLogFileName, szExeFilePath);
	*(_tcsrchr(m_szLogFileName, '\\') + 1) = '\0';
	_tcscat(m_szLogFileName, "Log\\");
	_tcscat(m_szLogFileName, szTemp);
	_tcscat(m_szLogFileName, ".log");
	
	// 1-4.得到默认的是否输出命令行的路径
	_tcscpy(m_szShowConsolePath, szExeFilePath);
	*(_tcsrchr(m_szShowConsolePath, '\\') + 1) = '\0';
	_tcscat(m_szShowConsolePath, szTemp);
	_tcscat(m_szShowConsolePath, "_sc");
	

	InitializeCriticalSection( &m_hLock );
	
	// 释放控制台
	m_hConWrite = NULL;
	FreeConsole();

	// 初始化日志
	CheckIsShowConsole();
	Init();
}

/*-----------------------------------------------------------------------
 *  Function    : 
 *  Prototype   : CDebugPrintf::CDebugPrintf ()
 *  Description : 
 *  Parameters  :                   
 *  Return      : 
 *  History     : 
 */
CDebugPrintf::~CDebugPrintf ()
{
	FreeConsole();
	CloseLog();

	DeleteCriticalSection( &m_hLock );
}

/*-----------------------------------------------------------------------
 *  Function    : 
 *  Prototype   : CDebugPrintf::Init()
 *  Description : 
 *  Parameters  :                   
 *  Return      : 
 *  History     : 
 */
bool CDebugPrintf::Init()
{
	EnterCriticalSection( &m_hLock );

	DWORD x;
	
	m_hDebugMutex = OpenMutex( MUTEX_ALL_ACCESS, FALSE, m_szLogFileName);  
	
	// We are the first application who wants to print to log file
	if ( m_hDebugMutex == 0 )
	{
		m_hDebugMutex = CreateMutex( 0, FALSE, m_szLogFileName );

		// Save old log file (up to 2)
		char newFile1[255];
		char newFile2[255];
		
		_tcscpy( newFile1, m_szLogFileName );
		_tcscat( newFile1, "~" );

		_tcscpy( newFile2, newFile1 );
		_tcscat( newFile2, "~" );

		CopyFile( newFile1, newFile2, FALSE );
		CopyFile( m_szLogFileName, newFile1, FALSE );

		// delete prev log file
		::SetFileAttributes(m_szLogFileName, FILE_ATTRIBUTE_NORMAL);
		::DeleteFile(m_szLogFileName);

		// create file
		HANDLE hFile = CreateFile( m_szLogFileName, GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE, 0, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, 0 );

		// DWORD dwTime = GetTickCount();
		// WriteFile( hFile, &dwTime, sizeof(dwTime),  &x, 0 );	
		// WriteFile( hFile, DIVIDER, sizeof(DIVIDER), &x, 0 );	
		CloseHandle( hFile );
	}

	m_hFile = CreateFile( m_szLogFileName, GENERIC_READ|GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0 );
	ReadFile( m_hFile, &m_dwStartTime, sizeof(m_dwStartTime), &x, 0 );	
	
	// 如果需要显示控制台，则显示
	if (m_bUseConsole)
		ShowConsole();
	else
		HideConsole();

	LeaveCriticalSection( &m_hLock );
	return true;
}

/*-----------------------------------------------------------------------
 *  Function    : 
 *  Prototype   : void CDebugPrintf::CloseLog ()
 *  Description : Closes log file and releases handles. 
 *  Parameters  :                   
 *  Return      : void
 *  History     : 
 */
void CDebugPrintf::CloseLog ()
{
	EnterCriticalSection( &m_hLock );

	CloseHandle( m_hFile );

	ReleaseMutex( m_hDebugMutex );
	CloseHandle( m_hDebugMutex );

	LeaveCriticalSection( &m_hLock );
}

/*-----------------------------------------------------------------------
 *  Function    : 
 *  Prototype   : void CDebugPrintf::printf ( const char *fmt, ... )
 *  Description : Prints formatted string to log-file
 *  Parameters  : const char *fmt - format string
                  ...             - list of parameters
 *  Return      : void
 *  History     : 
 */
void CDebugPrintf::printf ( const char *fmt, ... )
{
	EnterCriticalSection( &m_hLock );

	// Format string
	va_list arglist;
	va_start( arglist, fmt );
	//wvsprintf( m_szBuffer, fmt, arglist );//不支持float
	_vsnprintf(m_szBuffer, 1024, fmt, arglist); //wxg
	va_end(arglist);

	// Format time stamp
	DWORD x;
	char  buf[256];
	SYSTEMTIME time;
	GetLocalTime(&time);
	wsprintf( buf, "%02d/%02d/%02d %02d:%02d:%02d.%02d  ", 
					time.wYear, time.wMonth, time.wDay, 
					time.wHour, time.wMinute, time.wSecond,time.wMilliseconds);

	//DWORD dwDiffTime = GetTickCount()-m_dwStartTime;
	//wsprintf( buf, "%d.%03d ",
	//				dwDiffTime/1000,
	//				dwDiffTime%1000);

	// Write time stamp and formatted string to log-file
	SetFilePointer( m_hFile, 0, 0, FILE_END );
	WriteFile( m_hFile,	buf,    lstrlen(buf),    &x, 0 );	
	WriteFile( m_hFile,	m_szBuffer, lstrlen(m_szBuffer), &x, 0 );	
	WriteFile( m_hFile,	CrLf,   lstrlen(CrLf),   &x, 0 );	

	// 判断ShowConsolePath的文件夹是否存在, 并决定是否显示控制台
	CheckIsShowConsole();
	if (m_bUseConsole)
		ShowConsole();
	else
		HideConsole();

	if ( m_bUseConsole )
	{
		WriteFile( m_hConWrite,	buf,    lstrlen(buf),    &x, 0 );	
		WriteFile( m_hConWrite,	m_szBuffer, lstrlen(m_szBuffer), &x, 0 );	
		WriteFile( m_hConWrite,	CrLf,   lstrlen(CrLf),   &x, 0 );	
	}

	LeaveCriticalSection( &m_hLock );
}

/*-----------------------------------------------------------------------
*  Function    : CheckIsShowConsole
*  Prototype   : void CDebugPrintf::CheckIsShowConsole() 
*  Description : check path for is show console window
*  Parameters  :                   
*  Return      : void 
*  History     : 
*/
void CDebugPrintf::CheckIsShowConsole()
{
	//文件夹不存在，不显示控制台窗口
	if (_access(m_szShowConsolePath, 0) == -1)
	{
		m_bUseConsole = false;
	}
	else
	{
		m_bUseConsole = true;
	}
}

/*-----------------------------------------------------------------------
 *  Function    : ShowConsole
 *  Prototype   : void CDebugPrintf::ShowConsole() 
 *  Description : Shows log-console
 *  Parameters  :                   
 *  Return      : void 
 *  History     : 
 */
void CDebugPrintf::ShowConsole() 
{
	AllocConsole();
	
	m_hConWrite = GetStdHandle(STD_OUTPUT_HANDLE);

	// 设置控制台的标题和属性
	TCHAR l_szTemp[_MAX_PATH];
	_tcscpy(l_szTemp, m_szLogFileName);
	SetConsoleTitle(l_szTemp);
	SetConsoleTextAttribute(m_hConWrite,FOREGROUND_RED|BACKGROUND_INTENSITY);
}

/*-----------------------------------------------------------------------
 *  Function    : HideConsole
 *  Prototype   : void CDebugPrintf::HideConsole() 
 *  Description : Hides log-console
 *  Parameters  :                   
 *  Return      : void 
 *  History     : 
 */
void CDebugPrintf::HideConsole() 
{
	FreeConsole();
}

/*-----------------------------------------------------------------------
 *  Function    : ShowLastError
 *  Prototype   : void CDebugPrintf::ShowLastError() 
 *  Description : 
 *  Parameters  :                   
 *  Return      : void 
 *  History     : 
 */
void CDebugPrintf::ShowLastError() 
{
	char* lpMsgBuf;

	FormatMessage( FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,    
		NULL,
		GetLastError(),
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
		(LPTSTR) &lpMsgBuf,    
		0,    
		NULL );

	size_t len = _tcslen(lpMsgBuf)-1;

	if ( len>0 )
	{
		while ( len>=0 && (lpMsgBuf[len]==0x0d || lpMsgBuf[len]==0x0a) )
			len--;
		lpMsgBuf[len+1] = 0; 
		printf ( "GetLastError(): %s", (char*)lpMsgBuf );
		LocalFree( lpMsgBuf ); // Free the buffer.
	}
}

/*-----------------------------------------------------------------------
 *  Function    : SetLogFileName
 *  Prototype   : CDebugPrintf::SetLogFileName(char *szFilePath) 
 *  Description : 
 *  Parameters  : szFilePath - LogFile Path                  
 *  Return      : bool 
 *  History     : 
 */
bool CDebugPrintf::SetLogFileName(const char *szFilePath)
{
	if (_tcslen(szFilePath) < 4)
	{
		return false;
	}

	_tcscpy(m_szLogFileName, szFilePath);

	// reinit log
	CloseLog();
	Init();
	return true;
}
