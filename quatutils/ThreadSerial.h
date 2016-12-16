///////////////////////////////////////////////////////////////////////////////////////
///���̴߳���ͨ��DLLͷ�ļ�
///�߳��ļ����ٵ���
///BUG������
///2008-5-8 V1.0
//////////////////////////////////////////////////////////////////////////////////////
#ifndef _SERIAL_H
#define _SERIAL_H

#ifdef ThreadSerialAPI
#else
#define ThreadSerialAPI _declspec(dllimport)
#endif

//���պ���
typedef void (* FOnReceiveData)(LPVOID,void*,DWORD);
typedef void (* FOnComBreak)(LPVOID,DWORD,COMSTAT stat);

// CComPort ����Ŀ��
class CReadComThread;
class CSerialPort;
class ThreadSerialAPI CComPort;

class ThreadSerialAPI CComPort 
{
public:
	enum ReceiveMode
	{
		ManualReceiveByQuery,  //�ֶ���ѯ����
		ManualReceiveByConst,  //��������
		AutoReceiveBySignal,   //�ź��Զ�����
        AutoReceiveByBreak,	   //�Զ��жϽ���
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

	//1.��,�رմ��ں���
	BOOL Open(int nPort,ReceiveMode mode=AutoReceiveBySignal, DWORD dwBaud = 9600, Parity parity = NoParity, BYTE DataBits = 8, 
		  StopBits stopbits = OneStopBit, FlowControl fc = NoFlowControl);
	void Close();

	//2.���ý��պ���,�жϴ����� 
	void SetReceiveFunc(FOnReceiveData pfnOnReceiveData,LPVOID pSender);
	void SetBreakHandleFunc(FOnComBreak pfnOnComBreak);

	//3.��ȡ�������
	int GetCurPortNum() { return this->m_CurPortNum;  }
	CSerialPort* GetSerialPort(); 
	HANDLE GetCloseHandle();
	ReceiveMode GetReceiveMode();

	//4.(�߳���)֪ͨ���մ�����     
	void ReceiveData(void* pBuf,DWORD InBufferCount);//�̵߳��õĽ��պ���
	void ComBreak(DWORD dwMask);

	//6.����,�������--����ʵ�ʸ���
	DWORD GetInBufferCount();
	DWORD GetInput(void* pBuf,DWORD Count,DWORD dwMilliseconds=1000);
	DWORD Output(void* pBuf,DWORD Count);
	DWORD Output(unsigned char ch);
	bool IsOverlapped() { return m_IsOverlapped; }

protected:
	CSerialPort* m_pPort;                           //�ں�������
	CReadComThread* m_pReadThread;                  //�������߳� 

	LPVOID m_pSender;                               //����ĸ�����ָ��
	int m_CurPortNum;                               //��ǰ�˿ں� 
    FOnReceiveData m_pfnOnReceiveData;              //�����źź���
	FOnComBreak    m_pfnOnComBreak;                 //�����¼�������
    ReceiveMode    m_RecvMode;                      //����ģʽ

	HANDLE         m_hWriteEvent;                    //д�¼�
	OVERLAPPED     m_WriteOverlapped;                //д�ص��ṹ

	bool m_IsOverlapped;                            //�Ƿ��ص��ṹ;
	unsigned int nTimeOut;							 //����ͨ�ų�ʱʱ��

private:
    HANDLE m_hCloseEvent; //E: A event handle to close thread  //Chinese:�����߳��¼�
};
#endif
/////////////////////////////////////////////////////////////////////////////////////////
//ͷ�ļ����˽���
///////////////////////////////////////////////////////////////////////////////////////