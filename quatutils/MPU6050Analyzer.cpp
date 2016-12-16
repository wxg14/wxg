#include "MPU6050Analyzer.h"
#include "StreamAnalyzer.h"
#include <math.h>
#include <QString>
#include "CoolLog.h"

MPU6050Analyzer::MPU6050Analyzer()
{
	m_chipID = -1;
	m_count = 0;
	m_framecount = 0;
	m_bA5flag = false;
	m_b5Aflag = false;


	m_size = 0;
	m_intval = 0;
	m_offsetcx = 0;
	m_offsetcy = 0;
	m_offsetcz = 0;
	m_radius = 0;

	m_offsetgx = 0;
	m_offsetgy = 0;
	m_offsetgz = 0;

	m_TotalReceivedBytes = 0;

	m_maskacc = false;
	m_maskgyr = 0;
	m_sensitivity = 0.0;
	m_firstflag = true;
	m_receiveflag = false;
	m_legalflag = false;
	m_GyrAndAccSettings = 0;
	m_gyrrate = 0.0;

	m_FPS = 0;

	//m_config = NULL;
	m_bReadID = true;//添加
	m_sensitivity = 2048.0;
	m_gyrrate = 16.4;
	m_iSegmentLabel = -1;
	m_bComDirtyMark = false;
	m_bAccDirtyMark = false;
	m_bGyrDirtyMark = false;
}
MPU6050Analyzer::~MPU6050Analyzer()
{

}
//是否是6050端口
bool MPU6050Analyzer::isMPU6050Port()
{
	while (!m_receiveflag);
	return m_legalflag;
}

// void MPU6050Analyzer::setConfig(Configuration* config)
// {
//     m_config = config;
// }

void MPU6050Analyzer::setReceiveFlag(bool avalue)
{
    m_receiveflag = avalue;
}

unsigned short MPU6050Analyzer::mergeTwoByte(byte high, byte low)
{
	unsigned short ans = high;
	ans<<=8;
	ans |= low;
	return ans;
}

unsigned int MPU6050Analyzer::mergeTwoShort(unsigned short high, unsigned short low)
{
    unsigned int ans = high;
	ans<<=16;
	ans |= low;
	return ans;
}

short MPU6050Analyzer::genData(byte high, byte low)
{
	unsigned short ans = mergeTwoByte(high, low);
	return realValue(ans);
}

void MPU6050Analyzer::maskAcc()
{
    m_maskacc = true;
}

void MPU6050Analyzer::maskGyr()
{
	m_maskgyr = 100;
	m_offsetgx = 0;
	m_offsetgy = 0;
	m_offsetgz = 0;
}

void MPU6050Analyzer::setOffsetCom(int x, int y, int z)
{
//  m_offsetcx -= x;
// 	m_offsetcy -= y;
// 	m_offsetcz -= z;//modify by wxg 20160317
	m_offsetcx = x;
	m_offsetcy = y;
	m_offsetcz = z;
	m_bComDirtyMark=true;
}

void MPU6050Analyzer::setAccParam(float* fParam ,int iLen)
{
	//assert(iLen == ACCPARAMCOUNT);
	for(int i=0;i<ACCPARAMCOUNT;i++)
		m_fAccParam[i] = fParam[i];
	m_bAccDirtyMark = true;
}

void MPU6050Analyzer::setGyroParam(int x, int y, int z)
{
	m_offsetgx = x;
	m_offsetgy = y;
	m_offsetgz = z;
	m_bGyrDirtyMark=true;
}


// bool MPU6050Analyzer::isCalibration()
// {
// 	return m_calibration;
// }

// void MPU6050Analyzer::setChipName(QString& chipname)
// {
// 	m_chipname = chipname;
// }

// QString MPU6050Analyzer::getChipName()
// {
//    return m_chipname;
// }

// void MPU6050Analyzer::setChipIndex(int nindex)
// {
// 	if (m_chipIndex != nindex)
// 	{
//         m_chipIndex = nindex;
// 		//m_calibration = true;
// 		reloadChip();
// 	}
// }

// int MPU6050Analyzer::getChipIndex()
// {
//    return m_chipIndex;
// }

// int MPU6050Analyzer::getChipID()
// {
//    return m_chipID;
// }
void MPU6050Analyzer::getAccParam(float* fParam,int iLen)
{
	//assert(iLen == ACCPARAMCOUNT);
	for(int i=0;i<ACCPARAMCOUNT;i++)
		fParam[i] = m_fAccParam[i];
}

void MPU6050Analyzer::getOffsetCom(short* cx, short* cy, short* cz)
{
    *cx = m_offsetcx;
    *cy = m_offsetcy;
	*cz = m_offsetcz;
}

void MPU6050Analyzer::getOffsetAcc(short* ax, short* ay, short* az)
{
	*ax = m_offsetax;
	*ay = m_offsetay;
	*az = m_offsetaz;
}

void MPU6050Analyzer::getOffsetGyr(short* gx, short* gy, short* gz)
{
	*gx = m_offsetgx;
	*gy = m_offsetgy;
	*gz = m_offsetgz;
}
// void MPU6050Analyzer::refreshFrame()
// {
// 	map<int, MPU6050Calculator*>::iterator itr;
// 	for (itr = m_calcmap.begin(); itr!=m_calcmap.end(); itr++)
// 		itr->second->clearFrame();
// // 	emit frameCountPS(m_FrameCount);
// //	m_FrameCount = 0;
// }

void MPU6050Analyzer::setFPS(byte fps)
{
    m_FPS = fps;
}

void MPU6050Analyzer::setChipidInFront(bool avalue)
{
    m_chipidinfront = avalue;
}

void MPU6050Analyzer::setProtocol(int avalue)
{
    m_protocol = avalue;
}
// void MPU6050Analyzer::loadProperties()
// {
// 	if (m_config == NULL)
// 		return;
//     int nchipcount = m_config->get(CMAIN, CCHIPCOUNT, "0").toInt();
// 	//assert(nchipcount > 0);
// 	QString chipname;
// 	bool bfind = false;
// 	int i = 0;
// 	while(i<nchipcount)
// 	{
// 		chipname = QString("Chip%1").arg(i);
// 		m_chipIndex = i;
//         int nid = m_config->get(chipname, CCHIPID, "0").toInt();
// 		if (nid == m_chipID)
// 		{
// 			bfind = true;
// 			break;	
// 		}
// 		i++;
// 	}
// 	if (!bfind)
// 	{
// 		m_chipname = QString("Chip%1").arg(nchipcount);
// 		m_chipIndex = nchipcount;
// // 		m_maxaccx = -MAXINT;
// // 		m_minaccx = MAXINT;
// // 		m_maxaccy = -MAXINT;
// // 		m_minaccy = MAXINT;
// // 		m_maxaccz = -MAXINT;
// // 		m_minaccz = MAXINT;
// 		m_offsetcx = 0;
// 		m_offsetcy = 0;
// 		m_offsetcz = 0;
// 		m_offsetax = 0;
// 		m_offsetay = 0;
// 		m_offsetaz = 0;
// 		//m_calibration = false;
// 		m_FPS = 30;
// 	}
// 	else
// 	{
// 		m_chipname = chipname;
// 		//reloadChip();
// 	}
// }

void MPU6050Analyzer::analyzeGyrAndAccSettings()
{
//     m_AccRange = m_GyrAndAccSettings&3;
// 	if (m_AccRange == MPU6050_ACCEL_FS_2)
// 		m_sensitivity = 16384.0;
// 	else if (m_AccRange == MPU6050_ACCEL_FS_4)
// 		m_sensitivity = 8192.0;
// 	else if (m_AccRange == MPU6050_ACCEL_FS_8)
// 		m_sensitivity = 4096.0;
// 	else
		m_sensitivity = 2048.0;
// 	m_GyrRange = (m_GyrAndAccSettings>>2)&3;
// 	if (m_GyrRange == 0)
// 		m_gyrrate = 131.2;
// 	else if (m_GyrRange == 1)
// 		m_gyrrate = 65.6;
// 	else if (m_GyrRange == 2)
// 		m_gyrrate = 32.8;
// 	else 
		m_gyrrate = 16.4;
	//m_framePS = (m_GyrAndAccSettings>>4)&3;
// 	if (m_framePS == MPU6050_FramePS_FS_30)
// 		emit SetFramePS(28);
// 	else if (m_framePS == MPU6050_FramePS_FS_60)
// 		emit SetFramePS(55);
// 	else if (m_framePS == MPU6050_FramePS_FS_120)
// 		emit SetFramePS(107);
// 
// 	else
// 		emit SetFramePS(15);
}

int Gframecount[10] = {0};

void MPU6050Analyzer::maskAccOffset(int ax, int ay, int az)
{
	static int maxax= -MAXINT, maxay= -MAXINT, maxaz = -MAXINT, minax = MAXINT, minay = MAXINT, minaz = MAXINT;
	if (m_maskacc)
	{
		if ((abs(ax) < abs(az)) && (abs(ay) < abs(az)))
		{

			if (az < 0)
				minaz = az;
			if (az > 0)
				maxaz = az;
			m_offsetaz = (minaz + maxaz) / 2;
		}
		if ((abs(ax) < abs(ay)) && (abs(az) < abs(ay)))
		{
			if (ay < 0)
				minay = ay;
			if (ay > 0)
				maxay = ay;
			m_offsetay = (minay + maxay) / 2;
		}
		if ((abs(az) < abs(ax)) && (abs(ay) < abs(ax)))
		{
			if (ax < 0)
				minax = ax;
			if (ax > 0)
				maxax = ax;
			m_offsetax = (minax + maxax) / 2;
		}
		m_maskacc = false;
		m_bAccDirtyMark=true;
	}
}

void MPU6050Analyzer::maskGyrOffset(int gx, int gy, int gz)
{
	if (m_maskgyr)
	{
		m_maskgyr--;
		m_offsetgx += gx;
		m_offsetgy += gy;
		m_offsetgz += gz;
		if (m_maskgyr == 0)
		{
			m_offsetgx /=100.0;
			m_offsetgy /=100.0;
			m_offsetgz /=100.0;
			m_bGyrDirtyMark = true;
		}
	}
}

void MPU6050Analyzer::analyzePacketWithAHRSProtocol(byte chipID, byte* packetBytes)//主程序收到信号后进行数据分析 wxg3
{
	double yaw0, pitch0, roll0, yaw1, pitch1, roll1;

	int framesize = packetBytes[0];
	byte framefun = packetBytes[1];//b7 b6 b5 版本号；b4 加速度标记 b3 b2 b1 b0 数据类型
	byte btDataType = (framefun&0x0F);
	bool bAccFlag= (framefun&0x10)==0x10;
	byte btVer = framefun>>5;	// if the version >= 8 ,error come // marked by mvp ## 2015-8-6
	byte btChipID = packetBytes[2];//整个字节用来表示ID ，1 - 128为动捕服的，129 - 129+17为左手 128+19 - 129+35为右手
	// m_chipID = chipID;
	if (m_chipidinfront == 1)
	{
		int nID;
		short nmoveflag;
		float posx, posy, posz;
		static bool bBodyLeftHand=false;
		static bool bBodyRightHand=false;
		//nID = framefun & ((1<<5) - 1);
		if(btChipID <= 128)
		{
			nID = btChipID;//1~17
			if(!bBodyLeftHand && nID == 1)
				bBodyLeftHand = true;
			else if(!bBodyRightHand && nID == 9)
				bBodyRightHand = true;
			m_iBodyPartIndex=0;
		}
		else if(btChipID <= 146)
		{
			nID = btChipID - 129 + 24;//24~41
			if(btChipID -129 >= 15)//左手腕
			{
				if(bBodyLeftHand) return;
				nID = 1;
			}
			m_iBodyPartIndex = 1;
		}
		else if(btChipID <= 164)
		{
			nID = btChipID-147 + 42;//42~59
			if(btChipID - 147 >= 15)//右手腕
			{
				if(bBodyRightHand) return;
				nID = 9;
			}
			m_iBodyPartIndex = 2;
		}
		//if (((framefun >> 5)&1) == 1)
		if(btDataType == 1)//解算后的欧拉角
		{
			m_FrameCount++;
			short ny, np, nr;
			
			
			ny = mergeTwoByte(packetBytes[3], packetBytes[4]);
			//PRINTF("chipid:%d, yaw:%d", nID, ny);
			np = mergeTwoByte(packetBytes[5], packetBytes[6]);
			nr = mergeTwoByte(packetBytes[7], packetBytes[8]);
			if (framesize == 12)//wxg 应改为不同的数据类型btDataType
			{
				nmoveflag = mergeTwoByte(packetBytes[9], packetBytes[10]);
				//m_GyrAndAccSettings = packetBytes[8];
				//nmoveflag = MAXSHORT;
			}
			else if (framesize == 14)
			{
				nmoveflag = mergeTwoByte(packetBytes[9], packetBytes[10]);
				m_GyrAndAccSettings = packetBytes[11];
				nID = packetBytes[12];
			}
			else if (framesize == 18)
			{
				nmoveflag = mergeTwoByte(packetBytes[9], packetBytes[10]);
				posx = short(mergeTwoByte(packetBytes[11], packetBytes[12]))/10000.0;
				posy = short(mergeTwoByte(packetBytes[13], packetBytes[14]))/10000.0;
				posz = short(mergeTwoByte(packetBytes[15], packetBytes[16]))/10000.0;
				//posx = int(mergeTwoShort(mergeTwoByte(packetBytes[13], packetBytes[12]),mergeTwoByte(packetBytes[11], packetBytes[10])));
				//posy = int(mergeTwoShort(mergeTwoByte(packetBytes[17], packetBytes[16]),mergeTwoByte(packetBytes[15], packetBytes[14])));
				//posz = int(mergeTwoShort(mergeTwoByte(packetBytes[21], packetBytes[20]),mergeTwoByte(packetBytes[19], packetBytes[18])));
				emit newPosition(m_chipID, posx, posy, posz);

			}
			if (m_chipID == -1)
			{   
				m_chipID = nID;
			}
			bool weightnessless =false;
            if ((btChipID == 16)&&bAccFlag)//(framefun>>7)
				weightnessless = true;

			//wxg[2016-6-19]add
// 			static LONGLONG lLastTime=0;
// 			lLastTime = GetCycleCount();
			emit newYawPitchRoll(m_chipID, ny / 10.0, np / 10.0, nr / 10.0, nmoveflag, weightnessless);//得到欧拉角wxg2
// 			double fTimeSpace =(double)(GetCycleCount() - lLastTime) / GetFrequency();
// 			if(fTimeSpace > 0.1)
// 			{
// 				char szTemp[64]={0};
// 				sprintf(szTemp,"!!! time over(emit newYawPitchRoll):%f",fTimeSpace);
// 				PRINTF(szTemp);
// 			}
			//emit newYawPitchRoll(m_chipID, posx, posy, posz, nmoveflag);
			//PRINTF("yaw: %d, pitch:%d, roll: %d", ny, np, nr);
		}
		//else if (((framefun >> 5)&3) == 2)//四元数
else if(btDataType == 2)
		{
			m_chipID = nID;
			float x, y, z, w, yaw, pitch, roll, q[4];
			//w = short(mergeTwoByte(packetBytes[2], packetBytes[3]))/10000.0;
			//x = short(mergeTwoByte(packetBytes[4], packetBytes[5]))/10000.0;
			//y = short(mergeTwoByte(packetBytes[6], packetBytes[7]))/10000.0;
			//z = short(mergeTwoByte(packetBytes[8], packetBytes[9]))/10000.0;
			//if (framesize == 14)
			//{
   //             nmoveflag = mergeTwoByte(packetBytes[10], packetBytes[11]);
			//}
			//else if (framesize == 20)
			//{
			//	nmoveflag = mergeTwoByte(packetBytes[10], packetBytes[11]);
			//	posx = short(mergeTwoByte(packetBytes[12], packetBytes[13]))/10000.0;
			//	posy = short(mergeTwoByte(packetBytes[14], packetBytes[15]))/10000.0;
			//	posz = short(mergeTwoByte(packetBytes[16], packetBytes[17]))/10000.0;
			//	emit newPosition(m_chipID, posx, posy, posz);


			//}
			// changed by mvp ## 2015-8-6
			w = short(mergeTwoByte(packetBytes[3], packetBytes[4]))/10000.0;
			x = short(mergeTwoByte(packetBytes[5], packetBytes[6]))/10000.0;
			y = short(mergeTwoByte(packetBytes[7], packetBytes[8]))/10000.0;
			z = short(mergeTwoByte(packetBytes[9], packetBytes[10]))/10000.0;
			if (framesize == 14)
			{
                nmoveflag = mergeTwoByte(packetBytes[11], packetBytes[12]);
			}
			else if (framesize == 20)
			{
				nmoveflag = mergeTwoByte(packetBytes[11], packetBytes[12]);
				posx = short(mergeTwoByte(packetBytes[13], packetBytes[14]))/10000.0;
				posy = short(mergeTwoByte(packetBytes[15], packetBytes[16]))/10000.0;
				posz = short(mergeTwoByte(packetBytes[17], packetBytes[18]))/10000.0;
				emit newPosition(m_chipID, posx, posy, posz);
			}

			bool weightnessless =false;
			//if ((m_chipID == 16)&&(framefun>>7))
			if ((m_chipID == 16)&&bAccFlag)// edit by mvp ## 2015-8-7
				weightnessless = true;
			//yaw = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/pi; // yaw
			//pitch = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/pi; // pitch
			//roll = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/pi; // roll
			//			}

			emit newQuat(m_chipID, x, y, z, w, nmoveflag, weightnessless);
		}
		else//校准模式
		{

			int ax, ay, az, gx=0, gy=0, gz=0, cx, cy, cz;
			m_FrameCount++;
			ax = genData(packetBytes[3], packetBytes[4]);
			ay = genData(packetBytes[5], packetBytes[6]);
			az = genData(packetBytes[7], packetBytes[8]);

			maskAccOffset(ax, ay, az);

			gx = genData(packetBytes[9], packetBytes[10]);
			gy = genData(packetBytes[11], packetBytes[12]);
			gz = genData(packetBytes[13], packetBytes[14]);

			maskGyrOffset(gx, gy, gz);

			cx = genData(packetBytes[15], packetBytes[16]);
			cy = genData(packetBytes[17], packetBytes[18]);
			cz = genData(packetBytes[19], packetBytes[20]);
			//m_GyrAndAccSettings = packetBytes[14];
			if (m_chipID == -1)
			{   
				m_chipID = nID;
				analyzeGyrAndAccSettings();
				//loadProperties();

			}
			m_sensitivity = 2048.0; 
			//PRINTF("oginal data:chipID:%d, ax:%d, ay:%d, az:%d, cx:%d, cy:%d, cz:%d ", m_chipID, ax, ay, az, cx, cy, cz);
			if (m_sensitivity != 0.0)
				emit newAgcData(m_chipID, ax / 2048.0, ay / 2048.0, az / 2048.0, gx / 16.4, gy / 16.4, gz / 16.4, cx, cy, cz);//当前姿态模块的三轴数据

		}
	}
	else
	{		
		//while (framesize > 5)
		//{			
		if (CCHIPIDMODE == framefun/* && 6 == framesize*/)
		{
			m_FrameCount++;
			m_GyrAndAccSettings = packetBytes[2];
			int nID = packetBytes[3];
			if (m_chipID == -1)
			{   
				m_chipID = nID;
				analyzeGyrAndAccSettings();
				//loadProperties();
			}
			//framesize -=3;
			//packetBytes +=3;
		}
		else if (CEULORMODE == framefun/* && 10 == framesize*/)
		{
			m_FrameCount++;
			short ny, np, nr;
			short nmoveflag;
			int nID;
			ny = mergeTwoByte(packetBytes[2], packetBytes[3]);
			np = mergeTwoByte(packetBytes[4], packetBytes[5]);
			nr = mergeTwoByte(packetBytes[6], packetBytes[7]);
			if (framesize == 12)
			{
				m_GyrAndAccSettings = packetBytes[8];
				nID = packetBytes[9];
				nmoveflag = MAXSHORT;
			}
			else if (framesize == 14)
			{
				nmoveflag = mergeTwoByte(packetBytes[8], packetBytes[9]);
				m_GyrAndAccSettings = packetBytes[10];
				nID = packetBytes[11];
			}

			//m_chipID = nID;
			if (m_chipID == -1)
			{   
				m_chipID = chipID;
				analyzeGyrAndAccSettings();
				//loadProperties();
			}
			Gframecount[m_chipID]++;
			//if (m_chipID == 1)
			//PRINTF("chipID:%d, framecount:%d, yaw:%d, pitch:%d, roll:%d", m_chipID, Gframecount[m_chipID], ny, np, nr);
			//MPU6050Calculator* calc = getMPUCalclator(nID);
			//if (m_chipID == 1)
			bool weightnessless =false;
			if ((m_chipID == 16)&&(framefun>>7))
				weightnessless = true;
			emit newYawPitchRoll(m_chipID, ny / 10.0, np / 10.0, nr / 10.0, nmoveflag, weightnessless);
			//calc->newYawPitchRoll(ny / 10.0, np / 10.0, nr / 10.0);
			//framesize -=9;
			//packetBytes +=9;
		}
		else if (COFFSETACCMODE == framefun/* && 10 == framesize*/)
		{
			m_offsetax = mergeTwoByte(packetBytes[2], packetBytes[3]);
			m_offsetay = mergeTwoByte(packetBytes[4], packetBytes[5]);
			m_offsetaz = mergeTwoByte(packetBytes[6], packetBytes[7]);
			//PRINTF("accx:%d, accy:%d, accz:%d", m_offsetax, m_offsetay, m_offsetaz);
			//framesize -=9;
			//packetBytes +=9;
		}
		else if (COFFSETCOMMODE == framefun/* && 10 == framesize*/)
		{
			short hx, hy, hz;
			hx = mergeTwoByte(packetBytes[2], packetBytes[3]);

			hy = mergeTwoByte(packetBytes[4], packetBytes[5]);
			hz = mergeTwoByte(packetBytes[6], packetBytes[7]);
			//PRINTF("comx:%d, comy:%d, comz:%d", hx, hy, hz);
			//framesize -=9;
			//packetBytes +=9;
		}
		else if (COGINALACGMODE == framefun/* && 22 == framesize*/) { //原始的传感器ADC数据帧
			int ax, ay, az, gx, gy, gz, cx, cy, cz;
			m_FrameCount++;
			ax = genData(packetBytes[2], packetBytes[3]);
			ay = genData(packetBytes[4], packetBytes[5]);
			az = genData(packetBytes[6], packetBytes[7]);

			maskAccOffset(ax, ay, az);

			cx = genData(packetBytes[14], packetBytes[15]);
			cy = genData(packetBytes[16], packetBytes[17]);
			cz = genData(packetBytes[18], packetBytes[19]);

			gx = genData(packetBytes[8], packetBytes[9]);
			gy = genData(packetBytes[10], packetBytes[11]);
			gz = genData(packetBytes[12], packetBytes[13]);

			m_radius = sqrt(cx*cx + cy*cy + cz*cz + 0.0);
			m_sensitivity = 2048.0;
			m_gyrrate = 16.4;
			Gframecount[m_chipID]++;
			//if (m_chipID == 1)
			//PRINTF("framecountdddddddddddddddddddddddddddddddddddddddddddd:chipID:%d, framecount:%d", m_chipID, Gframecount[m_chipID]);
			//test
			//m_chipID = packetBytes[14];
			if (m_sensitivity != 0.0)
				emit newAgcData(m_chipID, ax / m_sensitivity, ay / m_sensitivity, az / m_sensitivity, gx / m_gyrrate, gy / m_gyrrate, gz / m_gyrrate, cx, cy, cz);
			m_firstflag = false;
			//framesize -=13;
			//packetBytes +=13;

		}
		else if (COGINALATTMODE == framefun/* && 18 == framesize*/){ //解算的姿态数据帧	
			m_FrameCount++;
			int yaw, pitch, roll, alt, tempr, press;
			int nccount;
			yaw = genData(packetBytes[2], packetBytes[3]);
			pitch = genData(packetBytes[4], packetBytes[5]);
			roll = genData(packetBytes[6], packetBytes[7]);
			alt = genData(packetBytes[8], packetBytes[9]);
			tempr = genData(packetBytes[10], packetBytes[11]);
			press = genData(packetBytes[12], packetBytes[13]);
			nccount = genData(packetBytes[14], packetBytes[15]);
			//framesize -=15;
			//packetBytes +=15;
			//m_GyrAndAccSettings = packetBytes[14];
			// 		nID = packetBytes[15];
			// 		emit newYawPitchRoll(yaw/10.0, pitch/10.0, roll/10.0);
			// 		if (m_chipID == -1)
			// 		{   
			// 			m_chipID = nID;
			// 			analyzeGyrAndAccSettings();
			// 			loadProperties();
			// 		}
		}
		else if (CQUATMODE == framefun/* && 20 == framesize*/)
		{
			float q0, q1, q2, q3;
			unsigned int nq0, nq1, nq2, nq3;
			m_FrameCount++;
			nq0 = mergeTwoShort(mergeTwoByte(packetBytes[4], packetBytes[5]), mergeTwoByte(packetBytes[2], packetBytes[3]));
			q0 = *((float*)(&nq0));
			nq1 = mergeTwoShort(mergeTwoByte(packetBytes[8], packetBytes[9]), mergeTwoByte(packetBytes[6], packetBytes[7]));
			q1 = *((float*)(&nq1));
			nq2 = mergeTwoShort(mergeTwoByte(packetBytes[12], packetBytes[13]), mergeTwoByte(packetBytes[10], packetBytes[11]));
			q2 = *((float*)(&nq2));
			nq3 = mergeTwoShort(mergeTwoByte(packetBytes[16], packetBytes[17]), mergeTwoByte(packetBytes[14], packetBytes[15]));
			q3 = *((float*)(&nq3));
			m_GyrAndAccSettings = packetBytes[18];
			int nID = packetBytes[19];
			if (m_chipID == -1)
			{   
				m_chipID = nID;
				analyzeGyrAndAccSettings();
				//loadProperties();
			};
			emit newQuat(m_chipID, q0, q1, q2, q3, 0, 0);
			// 			MPU6050Calculator* calc = getMPUCalclator(nID);
			// 			calc->newQuat(q0, q1, q2, q3);
			//framesize -=19;
			//packetBytes +=19;

		}
		else if (CGRAVMODE == framefun/* && 34 == framesize*/)
		{
			short ax, ay, az, gx, gy, gz, cx, cy, cz;
			m_FrameCount++;
			ax = mergeTwoByte(packetBytes[2], packetBytes[3]);
			ay = mergeTwoByte(packetBytes[4], packetBytes[5]);
			az = mergeTwoByte(packetBytes[6], packetBytes[7]);

			//maskAccOffset(ax, ay, az);

			cx = mergeTwoByte(packetBytes[14], packetBytes[15]);
			cy = mergeTwoByte(packetBytes[16], packetBytes[17]);
			cz = mergeTwoByte(packetBytes[18], packetBytes[19]);


			gx = mergeTwoByte(packetBytes[8], packetBytes[9]);
			gy = mergeTwoByte(packetBytes[10], packetBytes[11]);
			gz = mergeTwoByte(packetBytes[12], packetBytes[13]);

			float gravx, gravy, gravz;
			unsigned int nx, ny, nz;
			nx = mergeTwoShort(mergeTwoByte(packetBytes[22], packetBytes[23]), mergeTwoByte(packetBytes[20], packetBytes[21]));
			gravx = *((float*)(&nx));
			ny = mergeTwoShort(mergeTwoByte(packetBytes[26], packetBytes[27]), mergeTwoByte(packetBytes[24], packetBytes[25]));
			gravy = *((float*)(&ny));
			nz = mergeTwoShort(mergeTwoByte(packetBytes[30], packetBytes[31]), mergeTwoByte(packetBytes[28], packetBytes[29]));
			gravz = *((float*)(&nz));
			m_GyrAndAccSettings = packetBytes[32];
			//int nID = packetBytes[33];
			if (m_chipID == -1)
			{   
				m_chipID = chipID;
				analyzeGyrAndAccSettings();
				//loadProperties();
			}
			//Gframecount[m_chipID]++;
			//PRINTF("chipID:%d, framecountdddddddddddddddddddddddddddd:%d, index:%d", m_chipID, Gframecount[m_chipID], ax);

			//analyzeGyrAndAccSettings();
			//m_sensitivity = 2048.0;
			//m_gyrrate = 16.4;
			if (m_sensitivity != 0.0)
				emit newAgcGrav(m_chipID, ax / m_sensitivity, ay / m_sensitivity, az / m_sensitivity, gx / m_gyrrate, gy / m_gyrrate, gz / m_gyrrate, cx, cy, cz, gravx, gravy, gravz);
			//			MPU6050Calculator* calc = getMPUCalclator(nID);
			// 			if (m_sensitivity != 0)
			// 				calc->newAgcGrav(ax / m_sensitivity, ay / m_sensitivity, az / m_sensitivity, gx / m_gyrrate, gy / m_gyrrate, gz / m_gyrrate, cx, cy, cz, gravx, gravy, gravz);
			m_firstflag = false;
			//framesize -=33;
			//packetBytes +=33;
		}
		else if (CPOSITIONMODE == framefun)
		{
			float posx, posy, posz;
			unsigned int nposx, nposy, nposz;
			m_FrameCount++;
			nposx = mergeTwoShort(mergeTwoByte(packetBytes[4], packetBytes[5]), mergeTwoByte(packetBytes[2], packetBytes[3]));
			posx = *((float*)(&nposx));
			nposy = mergeTwoShort(mergeTwoByte(packetBytes[8], packetBytes[9]), mergeTwoByte(packetBytes[6], packetBytes[7]));
			posy = *((float*)(&nposy));
			nposz = mergeTwoShort(mergeTwoByte(packetBytes[12], packetBytes[13]), mergeTwoByte(packetBytes[10], packetBytes[11]));
			posz = *((float*)(&nposz));
			m_GyrAndAccSettings = packetBytes[14];
			int nID = packetBytes[15];
			if (m_chipID == -1)
			{   
				m_chipID = nID;
				analyzeGyrAndAccSettings();
				//loadProperties();
			};
			emit newPosition(m_chipID, posx, posy, posz);
			// 			MPU6050Calculator* calc = getMPUCalclator(nID);
			// 			calc->newQuat(q0, q1, q2, q3);
			//framesize -=15;
			//packetBytes +=15;
		}
		else if (CGLOVEMODE == framefun)
		{
			short fisnum, secnum, thdnum;
			m_FrameCount++;
			fisnum = mergeTwoByte(packetBytes[2], packetBytes[3]);
			secnum = mergeTwoByte(packetBytes[4], packetBytes[5]);
			thdnum = mergeTwoByte(packetBytes[6], packetBytes[7]);
			emit bendData(fisnum, secnum, thdnum);
			//framesize -=7;
			//packetBytes +=7;
		}
		else 
		{
			framesize = 0;
		}
		//framefun = packetBytes[1];
		//}		
	}
}

void MPU6050Analyzer::analyzePacketWithLPMSProtocol(byte chipID, byte* packetBytes)
{
	if (m_chipID == -1)
	{   
		m_chipID = chipID;
	}
	int nlen = (packetBytes[5]<<8) + packetBytes[4];
	float rax, ray, raz, rmx, rmy, rmz, rgx, rgy, rgz;
    unsigned int ntemp;
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[13], packetBytes[12]), mergeTwoByte(packetBytes[11], packetBytes[10]));
	rgx = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[17], packetBytes[16]), mergeTwoByte(packetBytes[15], packetBytes[14]));
	rgy = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[21], packetBytes[20]), mergeTwoByte(packetBytes[19], packetBytes[18]));
	rgz = *(float*)(&ntemp);

	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[25], packetBytes[24]), mergeTwoByte(packetBytes[23], packetBytes[22]));
	rax = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[29], packetBytes[28]), mergeTwoByte(packetBytes[27], packetBytes[26]));
	ray = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[33], packetBytes[32]), mergeTwoByte(packetBytes[31], packetBytes[30]));
	raz = *(float*)(&ntemp);

	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[37], packetBytes[36]), mergeTwoByte(packetBytes[35], packetBytes[34]));
	rmx = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[41], packetBytes[40]), mergeTwoByte(packetBytes[39], packetBytes[38]));
	rmy = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[45], packetBytes[44]), mergeTwoByte(packetBytes[43], packetBytes[42]));
	rmz = *(float*)(&ntemp);

	//emit newAgcData(m_chipID, rax, ray, raz, rgx, rgy, rgz, rmx, rmy, rmz);

    float q0, q1, q2, q3;
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[49], packetBytes[48]), mergeTwoByte(packetBytes[47], packetBytes[46]));
	q0 = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[53], packetBytes[52]), mergeTwoByte(packetBytes[51], packetBytes[50]));
	q1 = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[57], packetBytes[56]), mergeTwoByte(packetBytes[55], packetBytes[54]));
	q2 = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[61], packetBytes[60]), mergeTwoByte(packetBytes[59], packetBytes[58]));
	q3 = *(float*)(&ntemp);

	//emit newQuat(m_chipID, q0, q1, q2, q3);
	float yaw, pitch, roll;

	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[65], packetBytes[64]), mergeTwoByte(packetBytes[63], packetBytes[62]));
	roll = -*(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[69], packetBytes[68]), mergeTwoByte(packetBytes[67], packetBytes[66]));
	pitch = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[73], packetBytes[72]), mergeTwoByte(packetBytes[71], packetBytes[70]));
	yaw = *(float*)(&ntemp);
    emit newYawPitchRoll(m_chipID, yaw*180/3.1415926, pitch*180/3.1415926, roll*180/3.1415926, 0, false);

	float lax, lay, laz;//liner acc 
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[77], packetBytes[76]), mergeTwoByte(packetBytes[75], packetBytes[74]));
	lax = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[81], packetBytes[80]), mergeTwoByte(packetBytes[79], packetBytes[78]));
	lay = *(float*)(&ntemp);
	ntemp = mergeTwoShort(mergeTwoByte(packetBytes[85], packetBytes[84]), mergeTwoByte(packetBytes[83], packetBytes[82]));
	laz = *(float*)(&ntemp);
	//emit newYawPitchRoll(m_chipID, lax, lay, laz, 0);

}
void MPU6050Analyzer::newPacket(byte chipID, byte* packetBytes)//主程序收到信号后进行数据分析 wxg3
{
	m_legalflag = true;
	m_receiveflag = true;
	if (m_protocol == CAHRSPROTOCOL)
		analyzePacketWithAHRSProtocol(chipID, packetBytes);
	else if (m_protocol == CLPMSPROTOCOL)
		analyzePacketWithLPMSProtocol(chipID, packetBytes);
	else
	{
		//do nothings, there is no other protocol now;
	}
	
}

// MPU6050Calculator* MPU6050Analyzer::getMPUCalclator(int chipid)
// {
// 	map<int, MPU6050Calculator*>::iterator itr = m_calcmap.find(chipid);
//     if (itr == m_calcmap.end())
//     {
// // 		PortWidget* portwid = new PortWidget(parentwid->count(), parentwid);
// // 		portwid->setStyleSheet("QWidget { background-color:  rgb(48, 48, 48); border-width: 0px;}");
// // 		portwid->portname = m_chipname;
// 
// 		MPU6050Calculator* calc = new MPU6050Calculator(sender);
// 		if (m_config->get(CMAIN, CDrawPeople, "0").toUInt() == 1)
// 		{
// 			calc->setID(m_config->get(m_chipname, CBODYID, "-1").toInt()); 
// 			int nsegmentlabel = m_config->get(m_chipname, CSEGMENTLABEL, "-1").toInt();
// 			calc->setSegmentLabel(nsegmentlabel);
// 			if (nsegmentlabel != -1)
// 				sender->SegmentMask |= (1<<nsegmentlabel);
// 		}
// 		emit newMPUCalculator(calc, this);
// 		//calc->setChipID(aly->getChipID());
// // 		if (m_config->get(CMAIN, CSHOWPORTWINDOW, "0").toInt() == 1)
// // 			calc->port = portwid;
// // 		if (viewer)
// // 			calc->viewer = viewer;
// // 		QObject::connect(aly, SIGNAL(newAgcData(double, double, double, double, double, double, int, int, int)), 
// // 			calc, SLOT(newAgcData(double, double, double, double, double, double, int, int, int)));
// // 		QObject::connect(aly, SIGNAL(newYawPitchRoll(double, double, double)), 
// // 			calc, SLOT(newYawPitchRoll(double, double, double)));
// // 		QObject::connect(aly, SIGNAL(newQuat(float, float, float, float)), 
// // 			calc, SLOT(newQuat(float, float, float, float)));
// // 		QObject::connect(aly, SIGNAL(newAgcGrav(double, double, double, double, double, double, int, int, int, float, float, float)), 
// // 			calc, SLOT(newAgcGrav(double, double, double, double, double, double, int, int, int, float, float, float)));
// // 		QObject::connect(aly, SIGNAL(frameCountPS(byte)), 
// // 			calc, SLOT(frameCountPS(byte)));
// 		//portwid->viewer = m_viewerwid;
// 
// // 		if (m_config->get(CMAIN, CDrawPeople, "0").toUInt() == 0)
// // 			calc->setID(parentwid->count());
// // 		//portwid->analyzer = (MPU6050Analyzer*)(port->analyzer);
// // 		if (viewer)
// // 			viewer->addModle(parentwid->count());
// // 		parentwid->addTab(portwid, QString::fromLocal8Bit(portwid->portname.toAscii()));
// 		m_calcmap[chipid] = calc;
// 		return calc;
//     }
//     else
//     {
// 	    return itr->second;
//     }
// }
// void MPU6050Analyzer::reloadChip()
// {
// 	//m_calibration = true;
// 	m_radius = m_config->get(m_chipname,CRADIUS).toDouble();
// 	m_offsetcx = m_config->get(m_chipname, COFFSETCOMX).toInt();
// 	m_offsetcy = m_config->get(m_chipname, COFFSETCOMY).toInt();
// 	m_offsetcz = m_config->get(m_chipname, COFFSETCOMZ).toInt();
// 
// 	m_offsetax = m_config->get(m_chipname, COFFSETACCX).toShort();
// 	m_offsetay = m_config->get(m_chipname, COFFSETACCY).toShort();
// 	m_offsetaz = m_config->get(m_chipname, COFFSETACCZ).toShort();
// 	m_FPS = m_config->get(m_chipname, CFPS, "30").toUShort();
// 	int nfactor = 16384.0 / m_sensitivity;
// 	m_offsetax /= nfactor;
// 	m_offsetay /= nfactor;
// 	m_offsetaz /= nfactor;
// 	//m_offsetgx = m_config->get(m_chipname, COFFSETGYRX).toInt();
// 	//m_offsetgy = m_config->get(m_chipname, COFFSETGYRY).toInt();
// 	//m_offsetgz = m_config->get(m_chipname, COFFSETGYRZ).toInt();
// 
// // 	m_minaccx = m_config->get(m_chipname, minAccxStr(), "0").toInt();
// // 	m_maxaccx = m_config->get(m_chipname, MaxAccxStr(), "0").toInt();
// // 	m_minaccy = m_config->get(m_chipname, MinAccyStr(), "0").toInt();
// // 	m_maxaccy = m_config->get(m_chipname, MaxAccyStr(), "0").toInt();
// // 	m_minaccz = m_config->get(m_chipname, MinAcczStr(), "0").toInt();
// // 	m_maxaccz = m_config->get(m_chipname, MaxAcczStr(), "0").toInt();
// // 	int nfactor = 16384.0 / m_sensitivity;
// // 	m_minaccx /= nfactor;
// // 	m_maxaccx /= nfactor;
// // 	m_minaccy /= nfactor;
// // 	m_maxaccy /= nfactor;
// // 	m_minaccz /= nfactor;
// // 	m_maxaccz /= nfactor;
// }

// void MPU6050Analyzer::save(const QString& sectionname, Configuration* config)
// {
// 	config->set(m_chipname, CCHIPID, QString("%1").arg(m_chipID));
// 	config->set(m_chipname, CRADIUS, QString("%1").arg(m_radius));
//     config->set(m_chipname, COFFSETCOMX, QString("%1").arg(m_offsetcx));
// 	config->set(m_chipname, COFFSETCOMY, QString("%1").arg(m_offsetcy));
// 	config->set(m_chipname, COFFSETCOMZ, QString("%1").arg(m_offsetcz));
// 
// 	config->set(m_chipname, COFFSETGYRX, QString("%1").arg(m_offsetgx));
// 	config->set(m_chipname, COFFSETGYRY, QString("%1").arg(m_offsetgy));
// 	config->set(m_chipname, COFFSETGYRZ, QString("%1").arg(m_offsetgz));
// 
// 	//int nfactor = 16384.0 / m_sensitivity;
//     config->set(m_chipname, COFFSETACCX,  QString("%1").arg(m_offsetax));
//     config->set(m_chipname, COFFSETACCY,  QString("%1").arg(m_offsetay));
//     config->set(m_chipname, COFFSETACCZ,  QString("%1").arg(m_offsetaz));
// 	config->set(m_chipname, CFPS, QString("%1").arg(m_FPS));
// // 	config->set(m_chipname, minAccxStr(), QString("%1").arg(m_minaccx * nfactor));
// //  config->set(m_chipname, maxAccxStr(), QString("%1").arg(m_maxaccx * nfactor));
// // 	config->set(m_chipname, minAccyStr(), QString("%1").arg(m_minaccy * nfactor));
// // 	config->set(m_chipname, maxAccyStr(), QString("%1").arg(m_maxaccy * nfactor));
// // 	config->set(m_chipname, minAcczStr(), QString("%1").arg(m_minaccz * nfactor));
// // 	config->set(m_chipname, maxAcczStr(), QString("%1").arg(m_maxaccz * nfactor));
// }
