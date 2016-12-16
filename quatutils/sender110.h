#ifndef _SENDER_H_
#define _SENDER_H_
#include "quatutils_global.h"
#include "isender.h"
#include <vector>
#include <QString>
#include "osg/Quat"
#include "UdpSocket.h"
#include "configuration.h"
#include "IModel.h"
//#include "RenderThread.h"
//#include "locker.h"
//#include "Ogre/OgreVector3.h"
#include <QObject> 
#include<QDomDocument>  
#include<QDomElement>  
#include<QFile> 


using namespace std;


struct Bonetransform
{
	int id;
	QString name;
	float px,py,pz;
	float rx,ry,rz;
	float angle;
};

class QUATUTILS_EXPORT Sender: public ISender
{
//	Q_OBJECT
public:
	Sender();
	~Sender();
	static bool ExitMask;
	static bool CaliMask;
	static bool SaveMask;
	static bool SetDisRateMask;
	static bool AddIPAndPortMask;
	static double m_disrate;
	static unsigned int SegmentMask[BODY_PART_COUNT];
	static map<QString, int> m_sendtoIPs;
	bool m_calihavedone;
	void bind();
	bool sendData(int portID, float yaw, float pitch, float roll, bool zero);
	bool sendData(UINT portnum, char* buffer, int len);
	bool sendQuat(int portID, int segmentlabel, const osg::Quat& quat, bool cali = false, short moveflag = MAXSHORT, bool weightnessless = false);
	bool sendToAll(char* buffer, int len);
	bool sendState(int state);
	void load(Configuration* config);
	void swapTarPort();
//public slots:
	void sendFrame(bool cali);
	void calcOtherSegmentQuat();
	void setWeightNess(bool weightnessless);
	void hipsPosChange(double posx, double posy, double posz);
	void setRhandAngle(double aangle);
	void setModel(IModel* model);
//	void PrintLog(const char* pMsg);
private:
	void loadskeleton();
	void sendToMaya();
	bool jumpable();
	void ceckPiracy(Configuration* config);

	static const int BONENUM = BODY_NODE_COUNT+FINGER_NODE_COUNT*2  ;
	Bonetransform m_bones[BONENUM];//就记录了xml中的初始信息，没有更新操作
	osg::Vec3f m_bonepos[BONENUM];
	int m_parentbone[BONENUM];
	int m_iNodeCount[BODY_PART_COUNT];
	
	osg::Vec3f m_inirightfpos, m_inileftfpos, m_inihipspos;
	void iniParentrelationship();
	void inifootandhipspos();
	void calcPosition(bool cali);
	osg::Quat smooth_stay(osg::Quat new_q, int index);	// add by mvp ## 2015-7-25
	int genDatagram(char* pdata,osg::Quat* pQuat,osg::Vec3f* pVec,int iLen);
	int m_sampleCounter;
	char m_datagrambuffer[(BODY_NODE_COUNT+FINGER_NODE_COUNT*2+1)*16];
	//Locker m_lock;
	UINT m_tarport;
	UINT m_severport;
	UINT m_tarport1;
	bool m_jumpable;
	UINT m_AvatarID;
	short m_minmoveflag,	m_inilmflag, m_inirmflag;
	// add by mvp ## 2015-7-10
	//bool m_left_stay,m_right_stay;
	int m_left_stay,m_right_stay; // change the flag to int type
	short m_movelessseg, m_premovelessseg;
	float m_uplegLength, m_legLength, m_uplegTiltAngle, m_legTiltAngle;
	float m_fLrollParam,m_fLpitchParam,m_fRrollParam,m_fRpitchParam;	// add by mvp ## 2015-7-30
	//RenderThread m_SenderThread;
	UINT m_receiveport;
	unsigned int volatile m_segmentflag[BODY_PART_COUNT];
	osg::Quat m_segment[BODY_NODE_COUNT+1+FINGER_NODE_COUNT*2];

	// add the old Quat of the leg// add by mvp ## 2015-7-22
	//osg::Quat m_old_legs[6];	// 0/1/2 to the right of 15/16/17 and the 3/4/5 to the left of 19/20/21
	
	osg::Quat m_oldquat[12];	// smooth deal of the quat // add by mvp ## 2015-7-25
	int m_state[6];				// state of the smooth process .
    
	QString m_ModelName;	
	bool m_weightnessless;
	double m_hipPosIncx, m_hipPosIncy, m_hipPosIncz;
	vector<osg::Vec3f> m_jumpposqueue;

	//校准时右手的角度
	double m_rhangle;
	int m_iSendDataFormat;//0---quat,1---MVN
	float m_fLCorrectParam;//左脚滑纠偏系数
	float m_fRCorrectParam;//右脚滑纠偏系数

	CUdpSocket m_udpSocket;
	bool m_cansenddata;


	IModel* m_model;
//public slots:
//	void test();
};
#endif