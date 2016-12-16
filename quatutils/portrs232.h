#ifndef PORTRS232_H
#define PORTRS232_H

#include "quatutils_global.h"
#include "portbase.h"
#include "qextserialport.h"
#include "ThreadSerial.h"

#define MAX_DATA_LINE_LEN   1024
class QUATUTILS_EXPORT PortRs232 : public PortBase
{
Q_OBJECT
public:
    explicit PortRs232(const PortProperty& portpty, bool useqnextserialport = true);
    ~PortRs232();
    virtual void run();
    virtual void send(const QString & str);
	virtual void send(const char* data, int nsize);
	virtual bool isrunning();
	friend void OnReceiveData(LPVOID pSender,void* pBuf,DWORD nBufferCount);
private:
	CComPort m_comPort;
	bool m_useserialPort;
    QextSerialPort* m_serialPort;
    bool m_isRunning;
	bool m_receiveFlag;

	bool m_valid;
	int m_errorDataCount;

    void portSetup();
    void processDataLine();

signals:
    //void newData(const QByteArray&);
    void packetSeparator();
    void stopped();
    void message(const QString& text,const QString& type);

public slots:
    void requestToStop();
};

#endif // PORTRS232_H
