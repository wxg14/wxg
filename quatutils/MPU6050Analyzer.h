#ifndef MPU6050ANALYZER_H
#define MPU6050ANALYZER_H
#include "quatutils_global.h"
//#include <QWidget>
//#include "MPU6050Calculator.h"
//#include "sender.h"
#include "Analyzer.h"
//#include "configuration.h"
//#include "ViewerWidget.h"
#include<Windows.h>
//#include "CoolLog.h"
//#include "consts.h"
//#include "types.h"

#include <map>
#define COFFSETACCMODE 0xA0
#define COGINALATTMODE 0xA1
#define COGINALACGMODE 0xA2
#define CQUATMODE      0xA3
#define CGRAVMODE      0xA4
#define COFFSETCOMMODE 0xA6
#define CEULORMODE     0xA7
#define CCHIPIDMODE    0xA8
#define CGLOVEMODE     0xA9
#define CPOSITIONMODE  0xAA

#define ACCPARAMCOUNT 9

class QUATUTILS_EXPORT MPU6050Analyzer: public Analyzer 
{
	Q_OBJECT
public:
	MPU6050Analyzer();
	~MPU6050Analyzer();
	void maskAcc();
	void maskGyr();
	void setOffsetCom(int x, int y, int z);
	void setAccParam(float* fParam ,int iLen);
	void setGyroParam(int x, int y, int z);
	void setRadius(double radius){m_radius = radius;}
	double GetRadius(){return m_radius;}
	//void save(const QString& sectionname, Configuration* config);
	bool isMPU6050Port();
	void setReceiveFlag(bool avalue);
	void setChipidInFront(bool avalue);
	void setProtocol(int avalue);
	//void setConfig(Configuration* config);
	//bool isCalibration();
	//void setChipName(QString& chipname);
	// /QString getChipName();
	void setChipIndex(int nindex);
	//int getChipIndex();
	//int getChipID();
	void getOffsetCom(short* cx, short* cy, short* cz);
	void getOffsetAcc(short* ax, short* ay, short* az);
	void getAccParam(float* fParam,int iLen);
	void getOffsetGyr(short* gx, short* gy, short* gz);
	//void reloadChip();
	//void refreshFrame();
	void setFPS(byte fps);
	//MPU6050Calculator* getMPUCalclator(int chipid);
	void newPacket(byte chipID, byte* packetBytes);
	//QTabWidget* parentwid;
	//Sender* sender;
	bool GetComDirtyMark(){return m_bComDirtyMark;}
	bool GetAccDirtyMark(){return m_bAccDirtyMark;}
	bool GetGyrDirtyMark(){return m_bGyrDirtyMark;}
	void ClearDirtyMark(){m_bComDirtyMark=false;m_bAccDirtyMark=false;m_bGyrDirtyMark=false;}
	inline LONGLONG GetCycleCount() 
	{
		// 		__asm _emit 0x0F
		// 		__asm _emit 0x31
		LARGE_INTEGER lpFrequency;
		QueryPerformanceCounter(&lpFrequency); 
		return lpFrequency.QuadPart;
	} ;
	LONGLONG GetFrequency()
	{
		LARGE_INTEGER lpFrequency;
		::QueryPerformanceFrequency(&lpFrequency); //cpuÖ÷Æµ
		return lpFrequency.QuadPart;
	}
protected:
private:
    QString m_chipname;
	int m_chipIndex;
	int m_chipID;
	//Configuration* m_config;
	//byte m_packetBytes[40];
	int m_count;
	bool m_bA5flag;
	bool m_b5Aflag;
	byte m_size;
	int m_intval;
	short m_offsetcx, m_offsetcy, m_offsetcz;
	float m_fAccParam[ACCPARAMCOUNT];
	short m_offsetgx, m_offsetgy, m_offsetgz;
	short m_offsetax, m_offsetay, m_offsetaz;
	int m_preax, m_preay, m_preaz;
	int m_TotalReceivedBytes;
	bool m_firstflag;
	//bool m_calibration;
	byte m_framePS;
	byte m_AccRange;
	byte m_GyrRange;
	float m_gyrrate;
	byte m_FrameCount;
	int m_FPS;
	bool m_chipidinfront;
	int m_protocol;

	bool m_bReadID;//Ìí¼Ó
	//std::map<int, MPU6050Calculator*> m_calcmap;
	bool m_bComDirtyMark;
	bool m_bAccDirtyMark;
	bool m_bGyrDirtyMark;

	unsigned char m_GyrAndAccSettings;

	//int m_maxaccx, m_minaccx, m_maxaccy, m_minaccy, m_maxaccz, m_minaccz;
	double m_radius;
    int m_ax[3], m_ay[3], m_az[3], m_cx[3], m_cy[3], m_cz[3], m_gx[3], m_gy[3], m_gz[3];
	int m_framecount;
    bool m_maskacc;
	int m_maskgyr;
	bool volatile m_receiveflag;
	bool m_legalflag;

	int m_iSegmentLabel;
	int m_iBodyPartIndex;
	double m_sensitivity;
	short realValue(unsigned short avalue){
		if (avalue >32768)
			return (32768 - avalue);
		else 
			return avalue;
	};
	unsigned short mergeTwoByte(byte high, byte low);
	unsigned int mergeTwoShort(unsigned short high, unsigned short low);
	short genData(byte high, byte low);
	// /void loadProperties();
	void analyzeGyrAndAccSettings();
	void maskAccOffset(int ax, int ay, int az);
	void maskGyrOffset(int gx, int gy, int gz);

	void analyzePacketWithAHRSProtocol(byte chipID, byte* packetBytes);
	void analyzePacketWithLPMSProtocol(byte chipID, byte* packetBytes);

signals:
	void message(const QString& text,const QString& type);
	void newQuat(int chipID, float x, float y, float z, float w, short moveflag, bool weightnessless);
	void newYawPitchRoll(int chipID, double yaw, double pitch, double roll, short moveflag, bool weightnessless);
	void newAgcData(int chipID, double ax, double ay, double az, double gx, double gy, double gz, int cx, int cy, int cz);
	void newAgcGrav(int chipID, double ax, double ay, double az, double gx, double gy, double gz, int cx, int cy, int cz, float gravx, float gravy, float gravz);
	void bendData(float fir, float sec, float thd);
	void endAccMask();
	void frameCountPS(byte framecount);
	//void newMPUCalculator(MPU6050Calculator* calc, MPU6050Analyzer* aly);
	void newPosition(int chipID, double posx, double posy, double posz);	
};

#endif