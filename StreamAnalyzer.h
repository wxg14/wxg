/*******************************************************************
*  Copyright(c) 2000-2013 Instech beijing company
*  All rights reserved.
*
*  文件名称: StreamAnalyzer
*  简要描述: 流数据解析类
*
*  创建日期: 2014.4.6
*  作者: wanglh
*  说明:
*
*  修改日期:
*  作者:
*  说明:
******************************************************************/
#ifndef STREAMANALYZER_H
#define STREAMANALYZER_H
#include "quatutils_global.h"
#include <QObject>
#include<Windows.h>
#define MAXCHIPCOUNT 60
#define MAXPACKETSIZE 48//wxg 150可能溢出
#define CAHRSPROTOCOL 0
#define CLPMSPROTOCOL 1

class QUATUTILS_EXPORT StreamAnalyzer: public QObject 
{
	Q_OBJECT
public:
	StreamAnalyzer();
	~StreamAnalyzer();
	virtual bool newData(const QByteArray& newBytes,int iIndex=0);
	virtual bool newData(const char* pdata, int nsize,int iIndex=0);
	virtual void setReceiveFlag(bool avalue);
	bool isMPU6050Port();
	void chipidInfront(bool bvalue);
	void setProtocol(byte protocol);
	void setLogFlag(bool avalue){m_bDetailLog = avalue;}
	int getFrameCount();
	int getValidateFrameCount();

private:
	bool m_bDetailLog;
	int m_count[BODY_PART_COUNT];
	bool m_bA5flag[BODY_PART_COUNT];
	bool m_b5Aflag[BODY_PART_COUNT];
	bool m_b3Aflag[BODY_PART_COUNT];
	unsigned short m_size[BODY_PART_COUNT];
	int m_intval;
	int m_TotalReceivedBytes;
	byte m_protocol;
	byte m_cachebuffer[MAXCHIPCOUNT][MAXPACKETSIZE];
	byte m_packetBytes[BODY_PART_COUNT][1024];//100的长度够了，静态分配内存，免得每次动态分配消耗的时间 校准数据比较大
	bool volatile m_receiveflag;
	bool m_legalflag;
	bool m_chipidInfront;//chipid放在包前第一个字节

	bool m_bReadID;//添加
	char m_chipID;
	int m_frameCount;
	int m_validateFrameCount;

	bool checkOut(byte* packetBytes);
	bool analyzeDataWithLPMSprotocol(const char* pdata, int nsize);
	bool analyzeDataWithAHRSprotocol(const char* pdata, int nsize,int iIndex = 0);
signals:
	void message(const QString& text,const QString& type);
	void newPacket(char chipID, char* packetBytes);
	//void UpdateComOffset(int iIndex,int cx,int cy,int cz,int radius);
	void UpdateComOffset(int iIndex,float* fParam,int iLen);
	void UpdateAccParam( int iIndex,float* fParam,int iLen );
	void UpdateGyroParam( int iIndex,int cx,int cy,int cz );
};

#endif