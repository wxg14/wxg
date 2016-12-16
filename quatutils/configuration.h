#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include <QList>
#include <QIODevice>
#include <QCoreApplication>
#include <QByteArray>
#include <QHash>
#include <QPair>
#include <QtGlobal>
#include <QPainter>
#include <QSettings>
#include <math.h>
#include "quatutils_global.h"
#include "qextserialport.h"


#define MIN(A,B)  (((A)<(B)) ? (A) : (B) )
#define MAX(A,B)  (((A)>(B)) ? (A) : (B) )
#define PUT_IN_RANGE(V,VMIN,VMAX) MAX(VMIN,MIN(VMAX,V))
#define MAP_TO_RANGE(V,VMIN0,VMAX0,VMIN1,VMAX1) ( (VMIN1) +  ( (V) - (VMIN0) ) * ( (VMAX1) - (VMIN1) ) / ( (VMAX0) - (VMIN0) ) )

class QUATUTILS_EXPORT Configuration : public QObject
{
Q_OBJECT
public:
    explicit Configuration(QObject *parent = 0);
    void parse(const QString& str);
    const QString get(const QString& sectionName,const QString& propertyName,const QString& defaultValue="");
	void set(const QString& sectionName, const QString& propertyName, const QString& defaultValue="");
    QStringList fields;
    void openport(const QString& portname);
	void flush(const QString& filename);
	void load(const QString& filename);
private:
    QHash<QString, QHash<QString,QString > > sections;




signals:

public slots:

};

#endif // CONFIGURATION_H
