#ifndef MPU6050CALCULATOR_H
#define MPU6050CALCULATOR_H
#include "quatutils_global.h"
#include <QObject>
#include "osg/Quat"
#include "isender.h"
//#include "sender.h"
//#include "PortWidget.h"
//#include "ViewerWidget.h"
#include "Calculator.h"
#include "quatutils.h"

//#define AVERAGECOUNT 5 

class QUATUTILS_EXPORT  MPU6050Calculator: public QObject 
{
	Q_OBJECT
public:
	MPU6050Calculator(ISender* sender);
	~MPU6050Calculator();
	void setID(int aID);
    void setChipID(int aChipID);
	void setSegmentLabel(int segmentlabel);
	void setAbscali(bool avalue);
	void setEulorToQuatType(char avalue);
	void setEulorSigned(bool ayaw, bool apitch, bool aroll);
	void setAngleInc(int avalue);
	void setHeadRefToHand(bool avalue);
	void setHeadRefAngle(int avalue);
	void setSmooth(bool avalue);

	//PortWidget* port;
	//ViewerWidget* viewer;
	double preyaw, prepitch, preroll;
	double yaw0, pitch0, roll0;
	void maskcali();
	void clearFrame();
	static void InitBaseYaw();
	void SetYawBase(bool bYawBase){m_bYawBaseNode = bYawBase;}
	int GetSegmentLabel(){return m_SegmentLabel;}
	double GetYaw0(){return yaw0;}
	void SetYawOffset(double offset);


protected:
	osg::Quat calcEulorToQuat(double yaw, double pitch, double roll);
	osg::Quat CalcQuatBetweenTwoEulor(double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2);
	osg::Quat CalcuQuat(double yaw, double pitch, double roll);
	osg::Quat CalcuQuat(osg::Quat quat);
	void SendEuler(double yaw, double pitch, double roll);
	void SendSmoothQuat(osg::Quat squat);
	void sendQuat(osg::Quat squat);
	void Outputdata(osg::Quat squat);
private:
	static double postyaw[BODY_PART_COUNT];
	static double postpitch[BODY_PART_COUNT];
	quatutils m_quater;
	bool m_moveless;
	ISender* m_sender;
    //Moving mover;
	int m_ID;
	int m_ChipID;
	int m_SegmentLabel;
	bool m_firstflag;
	int m_framecount, m_scount;
	int m_acount, m_sacount;
	double m_preincax, m_preincay, m_preincaz;
	double m_presyaw;
	bool m_cali;
//	int m_totalframe;
//	int m_FPS;
	short m_moveflag;
	char m_eulorToQuatType;
	bool m_yawnegative;
	bool m_pitchnegative;
	bool m_rollnegative;

	double m_yaw[3], m_pitch[3], m_roll[3];
	double m_syaw[3], m_spitch[3], m_sroll[3];
	osg::Quat m_quat[10], m_squat[10], m_prequat, quat0;
	double m_ax[3], m_ay[3], m_az[3];
	double m_sax[3], m_say[3], m_saz[3];

	bool m_abscali;
	int m_angleinc;
	bool m_headreftohand;
	int m_headrefangle;
	bool m_smooth;
    bool m_usequat;
	osg ::Quat m_pquat;

	int m_iBodyPart;//0身体 1左手指 2右手指
	bool m_bYawBaseNode;//航向参考节点
	static double m_rootRoll[2];
	double m_bYawOffset;

// 	float yawAv[AVERAGECOUNT+3],pitchAv[AVERAGECOUNT+3],rollAv[AVERAGECOUNT+3];
// 	int m_iCount;
signals:
	void outQuat(int aid, float x, float y, float z, float w);
	void outAngle(int aid, float yaw, float pitch, float roll);
	void outCom(int aid, float cx, float cy, float cz);
	void outPos(int aid, float cx, float cy, float cz);//wxg[2016-8-11]add
//	void test();
	void updatecom(int chipID,float xmax, float ymax, float zmax,float xmin, float ymin, float zmin);

public slots:
	void newAgcGrav(int chipID, double ax, double ay, double az, double gx, double gy, double gz, int cx, int cy, int cz, float gravx, float gravy, float gravz);
	void newAgcData(int chipID, double ax, double ay, double az, double gx, double gy, double gz, int cx, int cy, int cz);
	void newYawPitchRoll(int chipID, double yaw, double pitch, double roll, short moveflag, bool weightnessless = false);
	void newQuat(int chipID, float x, float y, float z, float w, short moveflag, bool weightnessless);
	void newPosition(int chipID, double posx, double posy, double posz);
	void bendData(float fir, float sec, float thd);
};

#endif