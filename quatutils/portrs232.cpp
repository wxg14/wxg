#include "portrs232.h"
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include "CoolLog.h"



int arrBaudRate[22] = {    
50,                //POSIX ONLY
75,                //POSIX ONLY
110,
134,               //POSIX ONLY
150,               //POSIX ONLY
200,               //POSIX ONLY
300,
600,
1200,
1800,              //POSIX ONLY
2400,
4800,
9600,
14400,             //WINDOWS ONLY
19200,
38400,
56000,             //WINDOWS ONLY
57600,
76800,             //POSIX ONLY
115200,
128000,            //WINDOWS ONLY
256000             //WINDOWS ONLY
};

//串口接收事件中断
void OnReceiveData(LPVOID pSender,void* pBuf,DWORD nBufferCount)
{
	PortRs232* port = (PortRs232*)pSender;
	port->m_receiveFlag = true;
	if(nBufferCount > 0) 
	{
		//emit newData(m_serialPort->readAll());
		if (port->analyzer->newData((char*)pBuf,nBufferCount))
			port->m_valid = true;
		if (!port->m_valid)
			port->m_errorDataCount++;
	}
	else
		port->m_errorDataCount++;
	if ((!port->m_valid) && (port->m_errorDataCount > 10000))
	{
		//PRINTF("Open Port %s, Time out", port->portproperty.Portname.toAscii().data());
		port->analyzer->setReceiveFlag(true);
		port->m_isRunning = false;
	}
	//qApp->processEvents();
}

void OnComBreak(LPVOID pSender,DWORD dwMask,COMSTAT stat)
{
	//deal with the break of com here
	//开通或关闭串口通信时会引发此消息
	//::AfxMessageBox("Break!");
	PRINTF("OnComBreak!");
}

PortRs232::PortRs232(const PortProperty& portpty, bool useqnextserialport)
    : PortBase(portpty)
{
	m_useserialPort = useqnextserialport;
    m_isRunning = false;
    m_serialPort = 0;
 	m_comPort.SetReceiveFunc((FOnReceiveData)OnReceiveData,this); 
 	m_comPort.SetBreakHandleFunc(OnComBreak); 
}


void PortRs232::requestToStop(){
    m_isRunning = false;
}

PortRs232::~PortRs232(){
    //delete analyzer;

}

void PortRs232::portSetup(){
    m_serialPort->setBaudRate(portproperty.baudtype);
// 	FLOW_OFF,
// 		FLOW_HARDWARE,
// 		FLOW_XONXOFF
    m_serialPort->setFlowControl(FLOW_OFF);
    m_serialPort->setParity(PAR_NONE);
    m_serialPort->setDataBits(DATA_8);
    m_serialPort->setStopBits(STOP_1);
}


void PortRs232::run(){
    if(m_isRunning) return;
	if(!analyzer) return;
	if (m_useserialPort)
	{
		QString portname = "\\\\.\\" + portproperty.Portname;
		m_serialPort = new QextSerialPort(portname.toAscii(),QextSerialPort::EventDriven);
		portSetup();

		static int iIndex=100;
		iIndex++;
		
		canbedestroy = false;
		if(m_serialPort->open(QIODevice::ReadWrite)){
			//PRINTF("Open Port %s, succeed", portproperty.Portname.toAscii().data());
			portSetup();  //Note: on Windows settings must be called after m_serialPort->open, otherwise we get garbage when device is plugged for the fist time
			m_serialPort->setRts();//add by wxg
			PRINTF("Status = %08X",m_serialPort->lineStatus());
			//m_serialPort->setDtr();
			m_isRunning = true;
			m_serialPort->flush();
			m_errorDataCount = 0;
			m_valid = false;
			while(m_isRunning && m_serialPort && m_serialPort->isOpen()){
				int nbytesize = m_serialPort->bytesAvailable();
				if(nbytesize > 0) 
				{
				//emit newData(m_serialPort->readAll());
					if (analyzer->newData(m_serialPort->readAll(),iIndex))
						m_valid = true;
					if (!m_valid)
					   m_errorDataCount++;
				}
				else
					m_errorDataCount++;
				if ((!m_valid) && (m_errorDataCount > 100000))
				{
					//PRINTF("Read Port %s, Time out", portproperty.Portname.toAscii().data());
					analyzer->setReceiveFlag(true);
					break;
				}
				qApp->processEvents();
			}
			
		}else{
			analyzer->setReceiveFlag(true);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
			//PRINTF("Open Port %s, not succeed", portproperty.Portname.toAscii().data());
			emit message("Could not open port "+portproperty.Portname.toAscii()+"\nMake sure the port is available and not used by other applications.","critical");
		}
		m_isRunning = false;
		if(m_serialPort->isOpen()) m_serialPort->close();
		delete m_serialPort;
		emit stopped();
		canbedestroy = true;
	}
	else
	{
		m_isRunning=true;
		canbedestroy = false;
		m_receiveFlag = false;
		m_comPort.SetReceiveFunc((FOnReceiveData)OnReceiveData,this); 
		m_comPort.SetBreakHandleFunc(OnComBreak); 
		int portnum = portproperty.Portname.mid(3).toInt();
 		if(!(m_comPort.Open(portnum,CComPort::AutoReceiveBySignal,arrBaudRate[portproperty.baudtype],CComPort::NoParity,8,CComPort::OneStopBit)))
 		{
 			m_isRunning=false;
 			canbedestroy = true;
			analyzer->setReceiveFlag(true);
 			//::MessageBox(::GetForegroundWindow(), "串口打开失败", "提示信息", MB_ICONWARNING | MB_OK);
 		}
 		else
 		{
 			m_errorDataCount = 0;
 			m_valid = false;
			int ntimecount = 0;
 			while (m_isRunning)//wait
 			{
				if (!m_receiveFlag)
				{
					sleep(1);
					ntimecount++;
					if (ntimecount >=10)
					{
						analyzer->setReceiveFlag(true);
						break;
					}
				}
				qApp->processEvents();
 			}
 			m_comPort.Close();
 			emit stopped();
 			canbedestroy = true;
 		}

	}
}

bool PortRs232::isrunning()
{
	return m_isRunning;
}


void PortRs232::send(const QString & str){
    QByteArray data;
    QRegExp rx("^(char|short|long|hex)\\:(.*)$");
    if(rx.exactMatch(str)){
        QString format = rx.capturedTexts()[1];
        QStringList items = rx.capturedTexts()[2].split(QRegExp("\\s*,\\s*"));
        qDebug(format.toAscii());
        for(int i=0;i<items.length();i++){
            qDebug(("["+items[i]+"]").toAscii());
            if("char"==format){
                short num = items[i].toShort();
                data.append((char)num);
            }else if("short"==format){
                short num = items[i].toShort();
                data.append((char*)(&num),sizeof(num));
            }else if("long"==format){
                long num = items[i].toLong();
                data.append((char*)(&num),sizeof(num));
            }else if("hex"==format){
                data.append(QByteArray::fromHex(items[i].toAscii()));
            }
        };
    }else{
        data.append(str);
    }
    qDebug(QString(data).toAscii());
	
    if(m_isRunning) 
	{
		if (m_useserialPort)
		    m_serialPort->write(data.data(), data.length());
		else
			m_comPort.Output((void*)data.data(), data.length());
	}
}

void PortRs232::send(const char* data, int nsize)
{
	if(m_isRunning) 
	{
		if (m_useserialPort)
		    m_serialPort->write(data, nsize);
		else
			m_comPort.Output((void*)data, nsize);
	}
}
