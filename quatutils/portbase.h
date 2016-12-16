#ifndef PORTBASE_H
#define PORTBASE_H

#include <QObject>
#include <QThread>
#include "quatutils_global.h"
#include "qextserialbase.h"
//#include "OpenPortDialog.h"
#include "StreamAnalyzer.h"

class QUATUTILS_EXPORT PortBase : public QThread
{
Q_OBJECT
public:
    explicit PortBase(const PortProperty& portpty);
    virtual ~PortBase();
    //DecoderBase* decoder;

    virtual void run() = 0;
    virtual void send(const QString & str) = 0;
	virtual void send(const char* data, int nsize) = 0;
    StreamAnalyzer* analyzer;
	bool canbedestroy;

protected:
    //Configuration* config;
	PortProperty portproperty;
    QByteArray dataBytes;
    QList<double> dataValues;


signals:
   // void stopped() = 0;
   // void newData(const QByteArray&);
   // void message(const QString& text,const QString& type);

public slots:
    //virtual void start() = 0;
    virtual void requestToStop() = 0;


};

PortBase* createPort(const PortProperty& portpty);

#endif // PORTBASE_H
