#ifndef ANALYZER_H
#define ANALYZER_H
#include <QObject>
class Analyzer: public QObject 
{
	Q_OBJECT
public:
	Analyzer() {};
	~Analyzer() {};
	virtual void setReceiveFlag(bool avalue) = 0;
};

#endif