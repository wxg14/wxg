///////////////////////////////////////////////////////////////////////////////////////
///多线程串口通信DLL头文件
///线程文件不再导出
///BUG修正中
///2008-5-8 V1.0
//////////////////////////////////////////////////////////////////////////////////////
#ifndef _SERIAL_H
#define _SERIAL_H

#ifdef ThreadSerialAPI
#else
#define ThreadSerialAPI _declspec(dllimport)
#endif

//接收函数
typedef void (* FOnReceiveData)(LPVOID,void*,DWORD);
typedef void (* FOnComBreak)(LPVOID,DWORD,COMSTAT stat);

// CComPort 命令目标
class CReadComThread;
class CSerialPort;
class ThreadSerialAPI CComPort;

class ThreadSerialAPI CComPort 
{
public:
	enum ReceiveMode
	{
		ManualReceiveByQuery,  //手动查询接收
		ManualReceiveByConst,  //定数接收
		AutoReceiveBySignal,   //信号自动接收
        AutoReceiveByBreak,	   //自动中断接收
	};
    //Enums
	enum FlowControl
	{
		NoFlowControl,
		CtsRtsFlowControl,
		CtsDtrFlowControl,
		DsrRtsFlowControl,
		DsrDtrFlowControl,
		XonXoffFlowControl
	};
	enum Parity
	{    
		EvenParity,
		MarkParity,
		NoParity,
		OddParity,
		SpaceParity
	};
	enum StopBits
	{
		OneStopBit,
		OnePointFiveStopBits,
		TwoStopBits
	};
	CComPort();
	virtual ~CComPort();

	void SetTimeOut(unsigned int nMsTimeOut=1);

	//1.打开,关闭串口函数
	BOOL Open(int nPort,ReceiveMode mode=AutoReceiveBySignal, DWORD dwBaud = 9600, Parity parity = NoParity, BYTE DataBits = 8, 
		  StopBits stopbits = OneStopBit, FlowControl fc = NoFlowControl);
	void Close();

	//2.设置接收函数,中断处理函数 
	void SetReceiveFunc(FOnReceiveData pfnOnReceiveData,LPVOID pSender);
	void SetBreakHandleFunc(FOnComBreak pfnOnComBreak);

	//3.获取自身参数
	int GetCurPortNum() { return this->m_CurPortNum;  }
	CSerialPort* GetSerialPort(); 
	HANDLE GetCloseHandle();
	ReceiveMode GetReceiveMode();

	//4.(线程类)通知接收处理函数     
	void ReceiveData(void* pBuf,DWORD InBufferCount);//线程调用的接收函数
	void ComBreak(DWORD dwMask);

	//6.输入,输出函数--返回实际个数
	DWORD GetInBufferCount();
	DWORD GetInput(void* pBuf,DWORD Count,DWORD dwMilliseconds=1000);
	DWORD Output(void* pBuf,DWORD Count);
	DWORD Output(unsigned char ch);
	bool IsOverlapped() { return m_IsOverlapped; }

protected:
	CSerialPort* m_pPort;                           //内含串口类
	CReadComThread* m_pReadThread;                  //读串口线程 

	LPVOID m_pSender;                               //保存的父窗体指针
	int m_CurPortNum;                               //当前端口号 
    FOnReceiveData m_pfnOnReceiveData;              //接收信号函数
	FOnComBreak    m_pfnOnComBreak;                 //串口事件处理函数
    ReceiveMode    m_RecvMode;                      //接收模式

	HANDLE         m_hWriteEvent;                    //写事件
	OVERLAPPED     m_WriteOverlapped;                //写重叠结构

	bool m_IsOverlapped;                            //是否重叠结构;
	unsigned int nTimeOut;							 //串口通信超时时间

private:
    HANDLE m_hCloseEvent; //E: A event handle to close thread  //Chinese:结束线程事件
};
#endif
/////////////////////////////////////////////////////////////////////////////////////////
//头文件到此结束
///////////////////////////////////////////////////////////////////////////////////////