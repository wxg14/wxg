#include "sender.h"
//#include "types.h"
//#include "Ini.h"
//#include "common.h"
//#include "MPU6050TEST.h"
#include "quatangle.h"	// add by mvp ## 2015-7-22 四元数转换quatangle.h
#include "quatutils.h"
#include "common.h"
#include "CoolLog.h"
#include "qconsts.h"
//#include "FOgreWidget.h"
#include "osg/Point"
#include "mnv.h"
#include <string.h>
#include <ctime>
#include<Winsock2.h> 
#include <QMessageBox>
#include <QString.h>
#include <qsettings.h>
#include <qdatetime.h>
#include <tchar.h>
#include <assert.h>

float Myntohl(float val)
{
	union{
		UINT32 i;
		float f;
	}st;
	st.f = val;
	st.i = ntohl(st.i);
    return st.f;
}
void RecvDatafunc(char* pBuf, DWORD dwCount, char* pszIP, WORD wPort)
{
//	PRINTF("test");
	if (strcmp(pBuf,"Cali") == 0)
	{
		Sender::CaliMask = true;
		//PRINTF("Zero command");
		//SendMessage(GetCurrentThreadId(), RESETATTMSG, 0, 0);
		//PostMessage(Sender::Mainhandle, RESETATTMSG, 0, 0);
	}
// CEXIT 0x01
// CCALI 0x02
// CSAVE 0x03
// CSETDISRATE 0x04
// CADDIPANDPORT 0x05
	byte * pdata = (byte*)pBuf;
	if ((pdata[0] == 0xB4) && (pdata[1] ==0x4B))
	{
		int nlen = pdata[2];
		int ncommand = pdata[3];
		if (ncommand == 0x01)
		{
            Sender::ExitMask = true;
			//PRINTF("Receive Exit Command");
		}
		else if (ncommand == 0x02)
		{
            Sender::CaliMask = true;
			//PRINTF("Receive Calibration Command");
		}
		else if (ncommand == 0x03)
		{
			Sender::SaveMask = true;
			//PRINTF("Receive Save Command");
		}
		else if (ncommand == 0x04)
		{
			int nrate = (pdata[7]<<24) + (pdata[6]<<16) + (pdata[5]<<8) + pdata[4];
			Sender::m_disrate = *((float*)(&nrate));
			Sender::SetDisRateMask = true;
			//PRINTF("Receive SetDisRate Command");
		}
		else if (ncommand == 0x05)
		{
			int nport = (pdata[7]<<24) + (pdata[6]<<16) + (pdata[5]<<8) + pdata[4];
            char ip[128];
			QString newip;
			int iplen = pdata[2] - 7;
			for (int i=0; i<iplen; i++)
			{
				ip[i] = pdata[8+i];
			}
			ip[iplen] = 0;
			newip = ip;
			Sender::m_sendtoIPs[newip] = nport;
			Sender::AddIPAndPortMask = true;
			//PRINTF("Receive AddIPAndPort Command");
		}
	}
}
bool Sender::ExitMask = false;
bool Sender::CaliMask = false;
bool Sender::SaveMask = false;
bool Sender::SetDisRateMask = false;
bool Sender::AddIPAndPortMask = false;
unsigned int Sender::SegmentMask[BODY_PART_COUNT] = {0,0,0};
double Sender::m_disrate;
map<QString, int> Sender::m_sendtoIPs;
Sender::Sender()
{

	m_calihavedone = false;
	m_cansenddata = true;
	m_minmoveflag = MAXSHORT;
	m_premovelessseg = -1;
	m_movelessseg = -1;
	for(int i=0;i<BODY_PART_COUNT;i++)
		m_segmentflag[i]= 0;
	m_iNodeCount[0] = BODY_NODE_COUNT;
	m_iNodeCount[1] = FINGER_NODE_COUNT;
	m_iNodeCount[2] = FINGER_NODE_COUNT;
	m_sampleCounter = 0;
	m_inilmflag = 0;
	m_inirmflag = 0;
	// add by mvp ## 2015-7-8
	//m_left_stay = true;
	//m_right_stay = true;
	m_left_stay = 0;
	m_right_stay = 0;

	for (int i=0; i<BODY_NODE_COUNT+1+FINGER_NODE_COUNT*2; i++)
		m_segment[i].set(0.0, 0.0, 0.0, 1.0);
	//connect(&m_SenderThread, SIGNAL(notify()), this, SLOT(sendFrame()));
	//加载骨骼出事位置信息
	
	// add by mvp ## 2015-7-22
	// init the old legs quat.
	//for (int i=0; i<6; i++)
	//	m_old_legs[i].set(0.0, 0.0, 0.0, 1.0);

	for (int i=0; i<12; i++)	// init of the oldquat for smooth sort// add by mvp ## 2015-7-25
		m_oldquat[i].set(0.0, 0.0, 0.0, 1.0);
	for (int i =0;i<6;i++)
		m_state[i] = 0;
};

Sender::~Sender()
{
	m_udpSocket.CloseSocket(); 
	//fclose(Gfp);
}
void Sender::inifootandhipspos()
{
	for (int i=0; i<BONENUM; i++)
	{
		m_bonepos[i].set(0.0, 0.0, 0.0);
		int nsegid = i;
		while (nsegid != -1)
		{
			m_bonepos[i] +=  osg::Vec3f(m_bones[nsegid].px, m_bones[nsegid].py, m_bones[nsegid].pz);//计算所有骨骼相对根节点的位置
			nsegid = m_parentbone[nsegid];
		}
	}
	m_inirightfpos = m_bonepos[18];
	m_inileftfpos = m_bonepos[22];
	m_inihipspos = m_bonepos[0];	
}
void Sender::iniParentrelationship()
{
    m_parentbone[0] = -1;
	m_parentbone[1] = 0;
	m_parentbone[2] = 1;
	m_parentbone[3] = 2;
	m_parentbone[4] = 3;
	m_parentbone[5] = 4;
	m_parentbone[6] = 5;

	m_parentbone[7] = 4;
	m_parentbone[8] = 7;
	m_parentbone[9] = 8;
	m_parentbone[10] = 9;

	m_parentbone[11] = 4;
	m_parentbone[12] = 11;
	m_parentbone[13] = 12;
	m_parentbone[14] = 13;

	m_parentbone[15] = 0;
	m_parentbone[16] = 15;
	m_parentbone[17] = 16;
	m_parentbone[18] = 17;
	//m_parentbone[18] = 16;	// changed by mvp ## 2015-7-24

	m_parentbone[19] = 0;
	m_parentbone[20] = 19;
	m_parentbone[21] = 20;
	m_parentbone[22] = 21;
	//m_parentbone[22] = 20;	// changed by mvp ## 2015-7-24

}

void Sender::loadskeleton()
{ 
#ifndef NDEBUG
	QFile file(QString("E:\\motion\\motioncapture\\Win32\\Release\\media\\models\\%1.xml").arg(m_ModelName));   
#else
	TCHAR exepath[260];
	qcommon::GetExePath(exepath);
    QFile file(QString("%1").arg(exepath) + QString("\\media\\models\\%1.xml").arg(m_ModelName));
// 	QString temp1 = QString("%1").arg(exepath) + QString("\\media\\models\\skeleton.xml");
// 	PRINTF(temp1.toAscii());
#endif
    
	if(!file.open(QIODevice::ReadWrite))  
		return;  
	QDomDocument doc;  
	doc.setContent(&file);  
	QDomElement root = doc.firstChildElement("skeleton").firstChildElement("bones");
	QDomElement elt = root.firstChildElement("bone");
	for (; !elt.isNull(); elt = elt.nextSiblingElement("bone")) 
	{
		int nsegid = elt.attribute("segid").toInt();
		if (nsegid >= 0 && nsegid < BONENUM)
		{
		    m_bones[nsegid].id = elt.attribute("id").toInt();
			m_bones[nsegid].name = elt.attribute("name");
// 			if ((nsegid == 20)||(nsegid == 16))//大腿
// 			{
// 				m_bones[nsegid].px = 0;
// 				m_bones[nsegid].py = -m_uplegLength * cos(m_uplegTiltAngle * 3.1415926 / 180);
// 				m_bones[nsegid].pz = m_uplegLength * sin(m_uplegTiltAngle * 3.1415926 / 180);
// 			}
// 			else if ((nsegid == 21)||(nsegid == 17))//小腿
// 			{
// 				m_bones[nsegid].px = 0;
// 				m_bones[nsegid].py = -m_legLength * cos(m_legTiltAngle * 3.1415926 / 180);
// 				m_bones[nsegid].pz = m_legLength * sin(m_legTiltAngle * 3.1415926 / 180);
// 			}
// 			else//其他
// 			{

				QDomElement poselt = elt.firstChildElement("position");
				m_bones[nsegid].px = poselt.attribute("x").toFloat();
				m_bones[nsegid].py = poselt.attribute("y").toFloat();
				m_bones[nsegid].pz = poselt.attribute("z").toFloat();
			//}

			QDomElement rotelt = elt.firstChildElement("rotation");
			m_bones[nsegid].angle = rotelt.attribute("angle").toFloat();

			QDomElement axiselt = elt.firstChildElement("axis");
			m_bones[nsegid].rx = axiselt.attribute("x").toFloat();
			m_bones[nsegid].ry = axiselt.attribute("y").toFloat();
			m_bones[nsegid].rz = axiselt.attribute("z").toFloat(); 


		}

	}
	file.close();  
}
void Sender::bind()
{
	if(!m_udpSocket.BindSocket(m_receiveport , RecvDatafunc))
	{
		m_cansenddata = false;
		MessageBoxA(NULL,"绑定端口错误", "错误", MB_YESNO|MB_ICONQUESTION|MB_SYSTEMMODAL);
	}
	//m_SenderThread.startRender();
}

void Sender::calcOtherSegmentQuat()
{
	m_segment[17] = m_segment[18];
	m_segment[21] = m_segment[22];
	//Calc neck quat
	m_segment[5] = quatutils::slerp(m_segment[4], m_segment[6]);
	//assert((m_segment[5].x() != 0)&&(m_segment[5].y() != 0)&&(m_segment[5].z() != 0)&&(m_segment[5].w() != 0));
	//Calc L5 L3 T12
	m_segment[1] = quatutils::slerp(m_segment[0], m_segment[4], 0.1);
	m_segment[2] = quatutils::slerp(m_segment[0], m_segment[4], 0.3);
	m_segment[3] = quatutils::slerp(m_segment[2], m_segment[4], 0.5);
	//assert((m_segment[1].x() != 0)&&(m_segment[1].y() != 0)&&(m_segment[1].z() != 0)&&(m_segment[1].w() != 0));
	//assert((m_segment[2].x() != 0)&&(m_segment[2].y() != 0)&&(m_segment[2].z() != 0)&&(m_segment[2].w() != 0));
	//assert((m_segment[3].x() != 0)&&(m_segment[3].y() != 0)&&(m_segment[3].z() != 0)&&(m_segment[3].w() != 0));
	
}

int Sender::genDatagram(char* pdata,osg::Quat* pQuat,osg::Vec3f* pVec,int iLen)//按MVN格式打包数据
{
	MNV::MVN_HEADER header;
	int nsize = 0;
	header.IdString[0] = 'M';
	header.IdString[1] = 'I';
	header.IdString[2] = 'T';
	header.IdString[3] = 'P';
	header.IdString[4] = '0';
	header.IdString[5] = '2';

	header.SampleCounter = m_sampleCounter++;
	header.DatagramCounter = 0x80;
	header.ItemsNumber = iLen;//23

	time_t now_time; 
	now_time = time(NULL);
	header.TimeCode = now_time;

	header.AvatarID = m_AvatarID;
	nsize += sizeof(MNV::MVN_HEADER);
    memcpy((void*)pdata, (void*)(&header), nsize);
	pdata += sizeof(MNV::MVN_HEADER);

	for (int i=0; i<iLen; i++) 
	{
		MNV::PoseData02 posedata;
		posedata.segmentId = ntohl(UINT32(i+1));
		//if (i == 0)
		//{
		posedata.position.x = Myntohl(pVec[i].z()/m_disrate);
		posedata.position.y = Myntohl(pVec[i].x()/m_disrate);
		posedata.position.z = Myntohl(pVec[i].y()/m_disrate);
		//}
		//else
		//{
		//	posedata.position.x = ntohl(UINT32(0));
		//	posedata.position.y = ntohl(UINT32(0));
		//	posedata.position.z = ntohl(UINT32(0));
		//}
// 		posedata.quaternion.q1 = Myntohl(-m_segment[i].w());
// 		posedata.quaternion.q2 = Myntohl(m_segment[i].x());
// 		posedata.quaternion.q3 = Myntohl(m_segment[i].y());
// 		posedata.quaternion.q4 = Myntohl(-m_segment[i].z());
		posedata.quaternion.q1 = Myntohl(-pQuat[i].w());
		posedata.quaternion.q2 = Myntohl(-pQuat[i].z());
		posedata.quaternion.q3 = Myntohl(-pQuat[i].x());
		posedata.quaternion.q4 = Myntohl(-pQuat[i].y());
		memcpy((void*)pdata, (void*)(&posedata), sizeof(MNV::PoseData02));
		pdata += sizeof(MNV::PoseData02);
		nsize += sizeof(MNV::PoseData02);
	}

    return nsize;
}

void Sender::setWeightNess(bool weightnessless)
{
    m_weightnessless = weightnessless;
}

void Sender::hipsPosChange(double posx, double posy, double posz)
{
	//75 todo
	double tempx, tempy;
	tempx = posx;
	tempy = posy;
	//posx = tempx*cos(m_rhangle) - tempy*sin(m_rhangle);
	//posy = tempx*sin(m_rhangle) + tempy*cos(m_rhangle);
	//posy = tempy*cos(m_rhangle) - tempx*sin(m_rhangle);
	//posx = tempy*sin(m_rhangle) + tempx*cos(m_rhangle);
    m_hipPosIncx = posx*500;
	m_hipPosIncy = posz*500;
	m_hipPosIncz = posy*500;
}
void Sender::sendToMaya()
{
	float databuffer[4*24];
	for (int i=0; i< 23; i++)
	{
		databuffer[i*4] = m_segment[i].x();
		databuffer[i*4 + 1] = m_segment[i].y();
		databuffer[i*4 + 2] = -m_segment[i].z();
		databuffer[i*4 + 3] = -m_segment[i].w();
	}
	databuffer[23*4] = m_segment[23].x();
	databuffer[23*4 + 1] = m_segment[23].y();
	databuffer[23*4 + 2] = m_segment[23].z();
	sendToAll((char*)databuffer, sizeof(float)*4*24);

}

bool Sender::sendToAll(char* buffer, int len)
{
	map<QString, int>::iterator   it=m_sendtoIPs.begin();   
	for(;it!=m_sendtoIPs.end();++it)   
	{
		m_udpSocket.SendData(it->first.toAscii(), it->second, buffer, len);
	}
	return true;
}
void Sender::sendFrame(bool cali)
{
	float senddata[4*60];
	osg::Vec3f vec[60];
	osg::Quat quat[60];
	int iMVNIndex=0;
	if (!m_calihavedone)
		m_calihavedone = cali;
	//1684224,1701632 (1701633 + (1<<4) + (1<<22) + (1<<18) + (1<<7) + (1<<11) + (1<<6))
	int iBodyPartCount=0,iCompleteCount=0;
	for(int k=0;k<BODY_PART_COUNT;k++)
	{
// 		if (m_segmentflag[k] == SegmentMask[k] && SegmentMask[k] !=0)
// 		{
// 			if(k==0)
// 			{
// 				PRINTF("Sender::sendFrame::Begin calcPosition");
// 				calcOtherSegmentQuat();
// 				//计算位移
// 				calcPosition(cali);
// 				PRINTF("Sender::sendFrame::end calcPosition");
// 
// 			}
// 			if (m_calihavedone)
// 			{
// 				int sizeofdata = genDatagram(m_datagrambuffer,k);//按MVN格式打包数据
// 				sendToAll(m_datagrambuffer, sizeofdata);
// 			}
// 			int iBegin=0;
// 			for(int m=0;m<k;m++)
// 				iBegin+=(m_iNodeCount[m]+(m==0));
// 			for (int i=0; i< m_iNodeCount[k]+(k==0); i++)
// 			{
// 				senddata[i*4] = m_segment[i+iBegin].x();
// 				senddata[i*4 + 1] = m_segment[i+iBegin].y();
// 				senddata[i*4 + 2] = m_segment[i+iBegin].z();
// 				senddata[i*4 + 3] = m_segment[i+iBegin].w();
// 			}
// 			//发送到maya场景
// 			//sendToMaya();
// 
// 
// 			m_segmentflag[k] = 0;
// 	 		
// 			//if (m_calihavedone)
// 			   FOgreWidget::DriveBone((char*)(senddata), sizeof(float)*4*(m_iNodeCount[k]+(k==0)), cali, m_movelessseg,k);
// 			m_minmoveflag = MAXSHORT;
// 			m_movelessseg = -1;
// 
// 			//fwrite(senddata+8*4, sizeof(float), 12, Gfp);
// 			
// 		}
		if( SegmentMask[k] ==0) continue;//没有接入该设备
		iBodyPartCount++;
		if(m_segmentflag[k] == SegmentMask[k])
		{
			iCompleteCount++;
		}
	}
	if(iCompleteCount == iBodyPartCount && iCompleteCount>0)
	{
		//bool bBodyPart=false;
		int iDataLen=0;
		for(int k=0;k<BODY_PART_COUNT;k++)
		{
			if( SegmentMask[k] ==0) continue;//没有接入该设备
			if(m_segmentflag[k] == SegmentMask[k])
			{
				if(k==0)
				{
					//PRINTF("Sender::sendFrame::Begin calcPosition");
					calcOtherSegmentQuat();
					//计算位移
					calcPosition(cali);
					//PRINTF("Sender::sendFrame::end calcPosition");
				}
				int iBegin=0;
				for(int m=0;m<k;m++)
					iBegin+=(m_iNodeCount[m]+(m==0));
				int iIndex=0;
				for (int i=0; i< m_iNodeCount[k]+(k==0); i++)
				{
					if(k==0 && i==m_iNodeCount[k] || (k>0 && i>=15))//位移 \ 第六指 mvn不要
					{
						;//bBodyPart = true;
						if(k==0 && i==m_iNodeCount[k])
						{
							senddata[iIndex*4] = m_segment[i+iBegin].x();
							senddata[iIndex*4 + 1] = m_segment[i+iBegin].y();
							senddata[iIndex*4 + 2] = m_segment[i+iBegin].z();
							senddata[iIndex*4 + 3] = m_segment[i+iBegin].w();
							iIndex++;
						}
					}
					else
					{
						senddata[iIndex*4] = m_segment[i+iBegin].x();
						senddata[iIndex*4 + 1] = m_segment[i+iBegin].y();
						senddata[iIndex*4 + 2] = m_segment[i+iBegin].z();
						senddata[iIndex*4 + 3] = m_segment[i+iBegin].w();
						iIndex++;

						quat[iMVNIndex] = m_segment[i+iBegin];
						vec[iMVNIndex] = m_bonepos[i+iBegin];
						iMVNIndex++;
					}
				}
				
				memcpy(m_datagrambuffer+iDataLen,senddata,sizeof(float)*4*iIndex);
				iDataLen+=sizeof(float)*4*iIndex;
				if (m_model)
					m_model->DriveBone((char*)(senddata), sizeof(float)*4*iIndex, cali, m_movelessseg,k);

				m_segmentflag[k] = 0;
				
			}
		}
		if(m_calihavedone)
		{
			int sizeofdata = iDataLen;
			if(m_iSendDataFormat==1)
			{
				sizeofdata = genDatagram(m_datagrambuffer,quat,vec,iMVNIndex);//按MVN格式打包数据
			}
			sendToAll(m_datagrambuffer, sizeofdata);
		}

		m_minmoveflag = MAXSHORT;
		m_movelessseg = -1;
	}

}

bool Sender::sendState(int state)
{
	unsigned int ntemp = state;
	char data[10];
	data[0] = 0xB4;
	data[1] = 0x4B;
	data[2] = 7;
	data[3] = 0x06;
	data[4] = ntemp & 255;
	ntemp>>=8;
	data[5] = ntemp & 255;
	ntemp>>=8;
	data[6] = ntemp & 255;
	ntemp>>=8;
	data[7] = ntemp & 255;

	data[8] = 0xAA;
	return (m_udpSocket.SendData("127.0.0.1", m_severport, (char*)(data), 9) != -1);
}

void Sender::load(Configuration* config)
{
	ceckPiracy(config);
    m_tarport = config->get(CQMAIN, CQTARPORT).toUInt();
	m_tarport1 = config->get(CQMAIN, CQTARPORT1).toUInt();
	m_severport = config->get(CQMAIN, CQSEVERPORT).toUInt();
	m_receiveport = config->get(CQMAIN, CQRECEIVEPORT).toUInt();
	m_disrate = config->get(CQMAIN, CQDISRATE, "1").toInt();
	m_ModelName = config->get(CQMAIN, CQMODELNAME);
	m_AvatarID = config->get(CQMAIN, CQACTORID).toUInt();
	m_uplegLength = config->get(CQMAIN, CQULLENGTH).toFloat();
	m_legLength = config->get(CQMAIN, CQLLENGTH).toFloat();
	m_uplegTiltAngle = config->get(CQMAIN, CQULTANGLE).toFloat();
	m_legTiltAngle = config->get(CQMAIN, CQLLENGTH).toFloat();
	m_jumpable = (config->get(CQMAIN, CQJUMPABLE).toUInt() == 1);
	int nIPcount = config->get(CQMAIN, CQSENDIPCOUNT).toUInt();
	for (int i=0; i< nIPcount; i++)
	{
		m_sendtoIPs[config->get(CQMAIN, QString("IP%1").arg(i))] = config->get(CQMAIN, QString("Port%1").arg(i)).toInt();
	}

	loadskeleton();
	iniParentrelationship();
	inifootandhipspos();

	m_iSendDataFormat = config->get(CQMAIN,CDATAFORMAT,"0").toInt();

	m_fLCorrectParam = config->get(CQMAIN, CQLCORRECTPARAM,"0").toFloat();
	m_fRCorrectParam = config->get(CQMAIN, CQRCORRECTPARAM,"0").toFloat();
	
	m_fLrollParam	=	config->get(CQMAIN, CQL_ROLL_PARAM,"0").toFloat();
	m_fLpitchParam	=	config->get(CQMAIN, CQL_PITCH_PARAM,"0").toFloat();
	m_fRrollParam	=	config->get(CQMAIN, CQR_ROLL_PARAM,"0").toFloat();
	m_fRpitchParam	=	config->get(CQMAIN, CQR_PITCH_PARAM,"0").toFloat();
	
}

bool Sender::sendQuat(int portID, int segmentlabel, const osg::Quat& quat, bool cali, short moveflag, bool weightnessless)//准备驱动模型数据wxg6  moveflag为：加速度向量(扣除重力加速度后的)的模*1000
{
	//PRINTF("Sender::sendQuat::Begin sendQuat: %f, %f, %f, %f", quat.x(), quat.y(), quat.z(), quat.w());
	if (cali)//校准时 初始化两个脚掌的位置	// edit by mvp ## 2015-7-8
	{
	   if (segmentlabel == 18)
		   m_inirmflag = moveflag;
	   else if (segmentlabel == 22)
		   m_inilmflag = moveflag;
	}
	if (segmentlabel == 18)
	{
		moveflag -= m_inirmflag;
		if (moveflag < m_minmoveflag)//找出加速度最小（静止）的脚  m_minmoveflag值在所有模块采集一遍后复位
		{
			m_minmoveflag = moveflag;
		    m_movelessseg = segmentlabel;
		}	
		//PRINTF("right\t%d",moveflag);
		if (moveflag < 50)//the leg is stay...	// add by mvp ## 2015-7-20
		{
			m_right_stay++;
			if(m_right_stay > 50)
				m_right_stay = 50;
		}			
		else if(m_right_stay > 3)
			m_right_stay--;
		//m_right_stay = 50;	// set the right foot to stay // test by mvp ## 2015-7-25
	}
	else if (segmentlabel == 22)
	{
	    moveflag -= m_inilmflag;
		if (moveflag < m_minmoveflag)
		{
			m_minmoveflag = moveflag;
			m_movelessseg = segmentlabel;

		}	
		////PRINTF("left\t%d",moveflag);
		if (moveflag < 50)//the leg is stay...
		{
			m_left_stay++;
			if(m_left_stay > 50)
				m_left_stay = 50;
		}
		else if(m_left_stay > 3)
			m_left_stay--;
		//m_left_stay = 50;	// set the left foot to stay // test by mvp ## 2015-7-25
	}	
	
	//// add by mvp ## 2015-7-10		
	//if (cali)
	//{
	//   if (segmentlabel == 18)
	//   {
	//	   m_inirmflag = moveflag;
	//	   m_right_stay = true;
	//   }		   
	//   else if (segmentlabel == 22)
	//   {
	//	   m_inilmflag = moveflag;
	//	   m_left_stay = true;
	//   }	   
	//}
	//if (segmentlabel == 18)
	//{
	//	moveflag -= m_inirmflag;
	//	if (moveflag < MIN_MOVE)//the leg is stay...
	//	{
	//		m_right_stay = true;
	//	    m_movelessseg = segmentlabel;
	//	}
	//	else
	//		m_right_stay = false;
	//}
	//else if (segmentlabel == 22)
	//{
	//    moveflag -= m_inilmflag;
	//	if (moveflag < MIN_MOVE)
	//	{
	//		m_left_stay = true;
	//		m_movelessseg = segmentlabel;
	//	}
	//	else
	//		m_left_stay = false;
	//}

	if (!m_cansenddata) return false;
	/*
	if (segmentlabel >= 0 && segmentlabel < 24)												// edit by mvp ## 2015-7-8
	{    		
		assert(!((quat.x() == 0)&&(quat.y() == 0)&&(quat.z() == 0)&&(quat.w() == 0)));		// check the quat 
		m_segment[segmentlabel].set(quat.x(), quat.y(), quat.z(), quat.w());				// set the new quat data of sensor
		m_segmentflag[0] |= (1<<segmentlabel);												// add the sensor id
		sendFrame(cali);																	// send the frame when the whole sensor data arrived
	}*/
	if (segmentlabel >= 0 && segmentlabel < 24)		// changed by mvp ## 2015-7-8
	{
        //while (m_segmentflag & (1<<segmentlabel))
			sendFrame(cali);
			assert(!((quat.x() == 0)&&(quat.y() == 0)&&(quat.z() == 0)&&(quat.w() == 0)));
			//m_segment[segmentlabel].set(quat.x(), quat.y(), -quat.z(), -quat.w());//改变轴向???wxg1
			m_segment[segmentlabel].set(quat.x(), quat.y(), quat.z(), quat.w());
        //m_segment[segmentlabel] = quat;
		//m_lock.lock();
		m_segmentflag[0] |= (1<<segmentlabel);
		//m_lock.unLock();
	}
	else if(segmentlabel >= 24 && segmentlabel < 42 )
	{
		sendFrame(cali);
		assert(!((quat.x() == 0)&&(quat.y() == 0)&&(quat.z() == 0)&&(quat.w() == 0)));

		m_segment[segmentlabel].set(quat.x(), quat.y(), quat.z(), quat.w());
		segmentlabel-=24;
		m_segmentflag[1] |= (1<<segmentlabel);		
	}
	else if(segmentlabel >= 42 && segmentlabel < 60 )
	{
		sendFrame(cali);
		assert(!((quat.x() == 0)&&(quat.y() == 0)&&(quat.z() == 0)&&(quat.w() == 0)));

		m_segment[segmentlabel].set(quat.x(), quat.y(), quat.z(), quat.w());
		segmentlabel-=42;
		m_segmentflag[2] |= (1<<segmentlabel);		
	}

	else
	{
		sendToAll((char*)(quat._v), sizeof(double)*4);
	}

	//PRINTF("Sender::sendQuat::End sendQuat");
	return true;
}
// bool Sender::sendQuat(int portID, int segmentlabel, const osg::Quat& quat, bool cali, short moveflag, bool weightnessless)
// {
// 	//PRINTF("Sender::sendQuat::Begin sendQuat: %f, %f, %f, %f", quat.x(), quat.y(), quat.z(), quat.w());
// 	static int lcount,rcount = 0;
// 	if (cali)
// 	{
// 		if (segmentlabel == 18)
// 			m_inirmflag = moveflag;
// 		else if (segmentlabel == 22)
// 			m_inilmflag = moveflag;
// 	}
// 	if (segmentlabel == 18)
// 	{
// 		moveflag -= m_inirmflag;
// 		if (moveflag > 100)	// edit by mvp ## 2015-6-26
// 		{
// 			m_right_moveflag = true;		 
// 			rcount = 0;
// 		}
// 		else
// 		{
// 			if(rcount++ > 20)	// leave out the 
// 				m_right_moveflag = false;			
// 		}
// 		m_movelessseg = segmentlabel;
// 		m_minmoveflag = moveflag;
// 	}
// 	else if (segmentlabel == 22)
// 	{
// 		moveflag -= m_inilmflag;
// 		if (moveflag > 100)	// edit by mvp ## 2015-6-26
// 		{
// 			m_left_moveflag = true;		 
// 			lcount = 0;
// 		}
// 		else
// 		{
// 			if(lcount++ > 20)	// leave out the 
// 				m_left_moveflag = false;			
// 		}
// 		m_movelessseg = segmentlabel;
// 		m_minmoveflag = moveflag;
// 	}
// 	// 	if ((moveflag < m_minmoveflag) &&(segmentlabel == 18 || segmentlabel == 22))
// 	// 	{
// 	// 		m_minmoveflag = moveflag;
// 	// 		m_movelessseg = segmentlabel;
// 	// 	}
// 	if (!m_cansenddata) return false;
// 	if (segmentlabel != -1)
// 	{
// 		//while (m_segmentflag & (1<<segmentlabel))
// //		sendFrame(cali);
// 		//assert(!((quat.x() == 0)&&(quat.y() == 0)&&(quat.z() == 0)&&(quat.w() == 0)));
// 		m_segment[segmentlabel].set(quat.x(), quat.y(), quat.z(), quat.w());
// 		//m_segment[segmentlabel] = quat;
// 		//m_lock.lock();
// 		m_segmentflag[0] |= (1<<segmentlabel);
// 		//m_lock.unLock();
// 	}
// 	else
// 	{
// 		sendToAll((char*)(quat._v), sizeof(double)*4);
// 	}
// 
// 	//PRINTF("Sender::sendQuat::End sendQuat");
// 	return true;
// }

bool Sender::sendData(UINT portnum, char* buffer, int len)
{
	if (!m_cansenddata) return false;
	map<QString, int>::iterator   it=m_sendtoIPs.begin();   
	for(;it!=m_sendtoIPs.end();++it)   
	{
		m_udpSocket.SendData(it->first.toAscii(), portnum, buffer, len);
	}
	return true;
}

bool Sender::sendData(int portID, float yaw, float pitch, float roll, bool zero)
{
	if (!m_cansenddata) return false;
	byte *data;
	byte nlen = sizeof(float) * 3 + 1;
	data = new byte[nlen];

	memcpy(data, (void*)(&yaw), sizeof(float));
	memcpy(data+sizeof(float), (void*)(&pitch), sizeof(float));
	memcpy(data+sizeof(float)*2, (void*)(&roll), sizeof(float));

	if (zero)
		data[sizeof(float)*3] = 1;
	else
	    data[sizeof(float)*3] = 0;

	sendToAll((char*)data, nlen);
	delete[] data;
	return true;
}

void Sender::swapTarPort()
{
	UINT temp = m_tarport;
	m_tarport = m_tarport1;
	m_tarport1 = temp;
}

bool Sender::jumpable()
{
	//return false;
    if (!m_jumpable) return false;
 	if (!m_weightnessless)
    {
		//if ((m_bonepos[22].y() > m_inileftfpos.y() + 5) && (m_bonepos[18].y() > m_inileftfpos.y() + 5) && (m_left_stay == false) && (m_right_stay == false))	// changed by mvp ## 2015-7-10
        if ((m_bonepos[22].y() > m_inileftfpos.y() + 5) && (m_bonepos[18].y() > m_inileftfpos.y() + 5)&&(m_minmoveflag>50))//两脚高度离开地面5个单位以上，加		
		{	
			//PRINTF("___jumpable left=%f,initleft=%f,right=%f,initright=%f,m_minmoveflag=%d",(float)m_bonepos[22].y(), (float)m_inileftfpos.y() ,(float)m_bonepos[18].y() , (float)m_inileftfpos.y(),m_minmoveflag);
 		   return true;
		}
 	}
	return m_weightnessless;
// 	if (m_hipPosIncy > 0.23)
// 	{
//         if ((m_bonepos[17].y()>13)&&(m_bonepos[21].y()>13))
// 		    return true;
// 		else return false;
// 	}
// 	else  if (m_jumpposqueue.size() > 0)
// 	{
//         //if ((m_bonepos[18].y() > (m_inileftfpos.y() + 0.0001))&&(m_bonepos[22].y() > (m_inileftfpos.y() + 0.0001)))
// 			return true;
// 	}
// 	else
// 		return false;

}

double maxgtesty = 0;
// void Sender::calcPosition(bool cali)
// {
// 	static bool rfootonfloor = false, lfootonfloor=false;
// 	static double velocity = 0; 
// 	static osg::Vec3f rightfootpos, leftfootpos, hipspos;
// 
// 	if (cali)
// 	{
// 		rightfootpos.set(m_inirightfpos.x(), m_inirightfpos.y(), m_inirightfpos.z());
// 		leftfootpos.set(m_inileftfpos.x(), m_inileftfpos.y(), m_inileftfpos.z());
// 		hipspos.set(m_inihipspos.x(), m_inihipspos.y(), m_inihipspos.z());
// 		return;
// 	}
// 	if (!m_calihavedone)
// 		return;
// 
// 	osg::Vec3f rpos(0.0, 0.0, 0.0);
// 	int nseg = 18;
// 	while (nseg != 0)
// 	{
// 		rpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
// 		nseg = m_parentbone[nseg];
// 	}
// 
// 	osg::Vec3f lpos(0.0, 0.0, 0.0);
// 	nseg = 22;
// 	while (nseg != 0)
// 	{
// 		lpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
// 		nseg = m_parentbone[nseg];
// 
// 	}
// 
// 	osg::Vec3f hpos(0.0, 0.0, 0.0);
// 	float er = 0.001;
// 
// 	if (jumpable())		// 双脚离地
// 	{
// 
// 		hpos = m_bonepos[0];
// 		hpos._v[0] += m_hipPosIncx;
// 		hpos._v[1] += m_hipPosIncy;
// 		hpos._v[2] += m_hipPosIncz;
// 		for (int i=0; i<BONENUM; i++)
// 		{
// 			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
// 			nseg = i;
// 			while (nseg != 0)
// 			{
// 				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
// 				nseg = m_parentbone[nseg];
// 			}
// 		}
// 		leftfootpos  = m_bonepos[22];
// 		rightfootpos = m_bonepos[18];
// 	}
// 	else
// 	{
// 		//m_jumpposqueue.clear();
// 		if (m_movelessseg == 22)	
// 		{
// 			double ddis;
// 			if (!m_left_moveflag)		// add by mvp ## 2015-6-26
// 			{
// 				velocity = 0.0;
// 			}
// 			else
// 			{
// 				ddis = velocity / 120 + 9.8 /120/120/2;
// 				velocity += 9.8/120;
// 				if ((leftfootpos[1] - m_inileftfpos.y()) <ddis)
// 					ddis = leftfootpos[1] - m_inileftfpos.y(); 
// 
// 				leftfootpos[1] -= ddis;
// 			}
// 
// 			hpos = leftfootpos - lpos;
// 			rightfootpos = hpos + rpos;
// 		}
// 		else if (m_movelessseg == 18)
// 		{
// 			double ddis;
// 			if (m_right_moveflag)	// add by mvp ## 2015-6-26
// 			{
// 				velocity = 0.0;
// 			}
// 			else
// 			{
// 				ddis = velocity / 120 + 9.8 /120/120/2;
// 				velocity += 9.8/120;
// 				if ((rightfootpos[1] - m_inirightfpos.y()) <ddis)		// change m_inileftfpos  to  m_inirightfpos by mvp ## 2015-6-26
// 					ddis = rightfootpos[1] - m_inirightfpos.y();
// 
// 				rightfootpos[1] -= ddis;
// 			}
// 			hpos = rightfootpos - rpos;
// 			leftfootpos = hpos + lpos;
// 
// 		}
// 		else{
// 			hpos = hipspos;
// 		}
// 
// 		if (m_hipPosIncy > maxgtesty)
// 			maxgtesty = m_hipPosIncy;
// 		if (m_jumpable)
// 		{
// 			hpos._v[1] = m_bonepos[0]._v[1] + m_hipPosIncy;
// 		}
// 		if (m_bonepos[18].y() < m_bonepos[22].y())
// 		{
// 			hpos._v[1] -= m_bonepos[18].y() - m_inirightfpos.y();	// change m_inileftfpos  to  m_inirightfpos by mvp ## 2015-6-26
// 		}
// 		else
// 		{
// 			hpos._v[1] -= m_bonepos[22].y() - m_inileftfpos.y();
// 		}
// 		for (int i=0; i<BONENUM; i++)
// 		{
// 			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
// 			nseg = i;
// 			while (nseg != 0)
// 			{
// 				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
// 				nseg = m_parentbone[nseg];
// 			}
// 		}
// 	}
// 	m_segment[23].set(hpos.x() - hipspos.x(), hpos.y() - hipspos.y(), hpos.z() - hipspos.z(), 0);
// 	//m_segment[23].set(hpos.x(), hpos.y(), hpos.z(), 0);
// 	//hipspos = hpos;
// }

// edit by mvp ## 2015-7-15
// the two methord come to one 
/*void Sender::calcPosition(bool cali)
{
	static bool rfootonfloor = false, lfootonfloor=false;
	static double velocity = 0; 
	static osg::Vec3f rightfootpos, leftfootpos, hipspos;

	if (cali)
	{
		rightfootpos.set(m_inirightfpos.x(), m_inirightfpos.y(), m_inirightfpos.z());
		leftfootpos.set(m_inileftfpos.x(), m_inileftfpos.y(), m_inileftfpos.z());
		hipspos.set(m_inihipspos.x(), m_inihipspos.y(), m_inihipspos.z());
		return;
	}
	if (!m_calihavedone)
		return;

	osg::Vec3f rpos(0.0, 0.0, 0.0);//右脚掌相对根节点的位移
	int nseg = 18;//从右脚掌最末端开始 累计所有父节点的位移 
	while (nseg != 0)
	{
		rpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
		nseg = m_parentbone[nseg];
	}

	osg::Vec3f lpos(0.0, 0.0, 0.0);
	nseg = 22;//左脚掌
	while (nseg != 0)
	{
		lpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
		nseg = m_parentbone[nseg];
	}

	osg::Vec3f hpos(0.0, 0.0, 0.0);
	if (jumpable())//有跳跃
	{
		
		hpos = m_bonepos[0];
		hpos._v[0] += m_hipPosIncx;//原位置+移动值=根节点新位置
		hpos._v[1] += m_hipPosIncy;
		hpos._v[2] += m_hipPosIncz;
		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
		}
		leftfootpos  = m_bonepos[22];
		rightfootpos = m_bonepos[18];
	}
	else//静止或行走
	{
		//m_jumpposqueue.clear();
		if (m_movelessseg == 22)//左脚的加速度小 
		{
			double ddis;
			if (rfootonfloor)//右脚曾静止
			{
				velocity = 0.0;
				rfootonfloor = false;
				lfootonfloor = true;
			}
			else//右脚原先在动
			{
				ddis = velocity / 120 + 9.8 /120/120/2;//t=(1/120)秒 上一时刻到这一时刻的位移s=V0*t+0.5*g*t^2 
				velocity += 9.8/120;//新的初始速度
				if ((leftfootpos[1] - m_inileftfpos.y()) <ddis)//
					ddis = leftfootpos[1] - m_inileftfpos.y(); 

				leftfootpos[1] -= ddis;//
				lfootonfloor = true;
			}

			hpos = leftfootpos - lpos;//根节点的位置
			rightfootpos = hpos + rpos;
		}
		else if (m_movelessseg == 18)//右脚不动
		{
			double ddis;
			if (lfootonfloor)
			{
				velocity = 0.0;
				lfootonfloor = false;
				rfootonfloor = true;
			}
			else
			{
				ddis = velocity / 120 + 9.8 /120/120/2;
				velocity += 9.8/120;
				if ((rightfootpos[1] - m_inileftfpos.y()) <ddis)
					ddis = rightfootpos[1] - m_inileftfpos.y();
				rightfootpos[1] -= ddis;
				rfootonfloor = true;
			}
			hpos = rightfootpos - rpos;
			leftfootpos = hpos + lpos;
		}
		else{//
			hpos = hipspos;
		}

		if (m_hipPosIncy > maxgtesty)
			maxgtesty = m_hipPosIncy;//根节点最大上升高度 没有用到
		if (m_jumpable)
		{
			hpos._v[1] = m_bonepos[0]._v[1] + m_hipPosIncy;
	
		}
	 	if (m_bonepos[18].y() < m_bonepos[22].y())//wxg 20150630
	 	{
	 		hpos._v[1] -= m_bonepos[18].y() - m_inileftfpos.y();
      	}
	 	else
		{
	 		hpos._v[1] -= m_bonepos[22].y() - m_inileftfpos.y();
	 	}
		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());//更新根节点位置
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
		}
	}
	m_segment[23].set(hpos.x() - hipspos.x(), hpos.y() - hipspos.y(), hpos.z() - hipspos.z(), 0);//根节点偏移 maya才用，已注去
	//m_segment[23].set(hpos.x(), hpos.y(), hpos.z(), 0);
	//hipspos = hpos;
}*/
/*
void Sender::calcPosition(bool cali)
{
	static bool rfootonfloor = false, lfootonfloor=false;
	static double velocity = 0; 
	static osg::Vec3f rightfootpos, leftfootpos, hipspos;

	if (cali)
	{
		rightfootpos.set(m_inirightfpos.x(), m_inirightfpos.y(), m_inirightfpos.z());
		leftfootpos.set(m_inileftfpos.x(), m_inileftfpos.y(), m_inileftfpos.z());
		hipspos.set(m_inihipspos.x(), m_inihipspos.y(), m_inihipspos.z());
		return;
	}
	if (!m_calihavedone)
		return;

	osg::Vec3f rpos(0.0, 0.0, 0.0);//右脚掌相对根节点的位移
	int nseg = 18;//从右脚掌最末端开始 累计所有父节点的位移 
	while (nseg != 0)
	{
		rpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
		nseg = m_parentbone[nseg];
	}

	osg::Vec3f lpos(0.0, 0.0, 0.0);
	nseg = 22;//左脚掌
	while (nseg != 0)
	{
		lpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
		nseg = m_parentbone[nseg];
	}

	osg::Vec3f hpos(0.0, 0.0, 0.0);
	if (jumpable())//有跳跃
	{
		
		hpos = m_bonepos[0];
		hpos._v[0] += m_hipPosIncx;//原位置+移动值=根节点新位置
		hpos._v[1] += m_hipPosIncy;
		hpos._v[2] += m_hipPosIncz;
		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
		}
		leftfootpos  = m_bonepos[22];
		rightfootpos = m_bonepos[18];
	}
	else//静止或行走
	{
		//m_jumpposqueue.clear();
		if (m_movelessseg == 22)//左脚的加速度小 
		{
			double ddis;
			if (rfootonfloor)//右脚曾静止
			{
				velocity = 0.0;
				rfootonfloor = false;
				lfootonfloor = true;
			}
			else//右脚原先在动
			{
				ddis = velocity / 120 + 9.8 /120/120/2;//t=(1/120)秒 上一时刻到这一时刻的位移s=V0*t+0.5*g*t^2 
				velocity += 9.8/120;//新的初始速度
				if ((leftfootpos[1] - m_inileftfpos.y()) <ddis)//
					ddis = leftfootpos[1] - m_inileftfpos.y(); 

				leftfootpos[1] -= ddis;//
				lfootonfloor = true;
			}

			hpos = leftfootpos - lpos;//根节点的位置
			rightfootpos = hpos + rpos;
		}
		else if (m_movelessseg == 18)//右脚不动
		{
			double ddis;
			if (lfootonfloor)
			{
				velocity = 0.0;
				lfootonfloor = false;
				rfootonfloor = true;
			}
			else
			{
				ddis = velocity / 120 + 9.8 /120/120/2;
				velocity += 9.8/120;
				if ((rightfootpos[1] - m_inileftfpos.y()) <ddis)
					ddis = rightfootpos[1] - m_inileftfpos.y();
				rightfootpos[1] -= ddis;
				rfootonfloor = true;
			}
			hpos = rightfootpos - rpos;
			leftfootpos = hpos + lpos;
		}
		else{//
			hpos = hipspos;
		}

		if (m_hipPosIncy > maxgtesty)
			maxgtesty = m_hipPosIncy;//根节点最大上升高度 没有用到
		if (m_jumpable)
		{
			hpos._v[1] = m_bonepos[0]._v[1] + m_hipPosIncy;
	
		}
	 	if (m_bonepos[18].y() < m_bonepos[22].y())//wxg 20150630
	 	{
	 		hpos._v[1] -= m_bonepos[18].y() - m_inileftfpos.y();
      	}
	 	else
		{
	 		hpos._v[1] -= m_bonepos[22].y() - m_inileftfpos.y();
	 	}
		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());//更新根节点位置
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
		}
	}
	m_segment[23].set(hpos.x() - hipspos.x(), hpos.y() - hipspos.y(), hpos.z() - hipspos.z(), 0);//根节点偏移 maya才用，已注去
	//m_segment[23].set(hpos.x(), hpos.y(), hpos.z(), 0);
	//hipspos = hpos;
}
*/
// add by mvp ## 2015-7-25
// smooth of the quat of the legs 
osg::Quat Sender::smooth_stay(osg::Quat new_q, int index)
{
	//// add by mvp ## 2015-7-27
	//Quaternion<double> clac_angle2; 
	//osg::Quat tmp_q;
	//float pitch_new,yaw_new,roll_new;
	//yaw_new = 0.0;
	//pitch_new = 0.0;
	//roll_new = 0.0;			
	////calc the angle of the X Z Y Direction

	//clac_angle2.set(new_q.w(),new_q.x(),new_q.y(),new_q.z());
	//clac_angle2.Quat2Angle(pitch_new,yaw_new,roll_new,XZY);
	//r_new *= 0.5;
	//tmp_q = quatutils::EulorToQuat((double)yaw_new,(double)pitch_new,(double)roll_new, 1);

	switch(m_state[index])
	{
	case 0:
		m_oldquat[index] = new_q;
		m_state[index] = 1;
		break;
	case 1:
		m_oldquat[index] = quatutils::slerp(m_oldquat[index],new_q);
		m_oldquat[index + 6] = m_oldquat[index];
		m_state[index] = 2;
		break;
	case 2:
		m_oldquat[index] = quatutils::slerp(m_oldquat[index],new_q);
		m_oldquat[index + 6] = quatutils::slerp(m_oldquat[index +6],m_oldquat[index]);
		///m_state[index] = 3;
		break;
	}
	if(m_state[index] == 2)
		return m_oldquat[index+6];
	else
		return new_q;
}
 // whole_leg_pos_0.8、slerp_double、recalc the angle of yaw 
// slerp of smooth 
// add by mvp ## 2015-7-25
osg::Quat calcEulorToQuat(double yaw, double pitch, double roll)
{
	// 	osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (roll), osg::Z_AXIS,
	// 		osg::DegreesToRadians (pitch), osg::X_AXIS,
	// 		osg::DegreesToRadians (-yaw), osg::Y_AXIS);
	//  	return rlt;
// 	if (m_yawnegative)
// 		yaw = -yaw;
// 	if (m_pitchnegative)
// 		pitch = -pitch;
// 	if (m_rollnegative)
// 		roll = -roll;
	return quatutils::EulorToQuat(yaw, pitch, roll, 1);
}
void Sender::calcPosition(bool cali)
{
	static bool rfootonfloor = false, lfootonfloor=false;
	static double velocity = 0; 
	static osg::Vec3f rightfootpos, leftfootpos, hipspos;

	if (cali)
	{
		rightfootpos.set(m_inirightfpos.x(), m_inirightfpos.y(), m_inirightfpos.z());
		leftfootpos.set(m_inileftfpos.x(), m_inileftfpos.y(), m_inileftfpos.z());
		hipspos.set(m_inihipspos.x(), m_inihipspos.y(), m_inihipspos.z());
		return;
	}
	if (!m_calihavedone)
		return;

	osg::Vec3f rpos(0.0, 0.0, 0.0);//右脚掌相对根节点的位移
	int nseg = 18;//从右脚掌最末端开始 累计所有父节点的位移 
	while (nseg != 0)
	{
		rpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
		nseg = m_parentbone[nseg];
	}

	osg::Vec3f lpos(0.0, 0.0, 0.0);
	nseg = 22;//左脚掌
	while (nseg != 0)
	{
		lpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
		nseg = m_parentbone[nseg];
	}

	osg::Vec3f hpos(0.0, 0.0, 0.0);
	if (jumpable())//有跳跃
	{
		
		hpos = m_bonepos[0];
		hpos._v[0] += m_hipPosIncx;//原位置+移动值=根节点新位置
		hpos._v[1] += m_hipPosIncy;
		hpos._v[2] += m_hipPosIncz;
		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
		}
		leftfootpos  = m_bonepos[22];
		rightfootpos = m_bonepos[18];
	}
	else//静止或行走
	{
		//m_jumpposqueue.clear();
		if (m_movelessseg == 22)//左脚的加速度小 
		{
			double ddis;
			if (rfootonfloor)//右脚曾静止
			{
				velocity = 0.0;
				rfootonfloor = false;
				lfootonfloor = true;
			}
			else//右脚原先在动
			{
				ddis = velocity / 120 + 9.8 /120/120/2;//t=(1/120)秒 上一时刻到这一时刻的位移s=V0*t+0.5*g*t^2 
				velocity += 9.8/120;//新的初始速度
				if ((leftfootpos[1] - m_inileftfpos.y()) <ddis)//
					ddis = leftfootpos[1] - m_inileftfpos.y(); 

				leftfootpos[1] -= ddis;//
				lfootonfloor = true;
			}

			hpos = leftfootpos - lpos;//根节点的位置
			rightfootpos = hpos + rpos;
		}
		else if (m_movelessseg == 18)//右脚不动
		{
			double ddis;
			if (lfootonfloor)
			{
				velocity = 0.0;
				lfootonfloor = false;
				rfootonfloor = true;
			}
			else
			{
				ddis = velocity / 120 + 9.8 /120/120/2;
				velocity += 9.8/120;
				if ((rightfootpos[1] - m_inirightfpos.y()) <ddis)
					ddis = rightfootpos[1] - m_inirightfpos.y();
				rightfootpos[1] -= ddis;
				rfootonfloor = true;
			}
			hpos = rightfootpos - rpos;
			leftfootpos = hpos + lpos;
		}
		else{//
			hpos = hipspos;
		}

		if (m_hipPosIncy > maxgtesty)
			maxgtesty = m_hipPosIncy;//根节点最大上升高度 没有用到
		if (m_jumpable)
		{
			hpos._v[1] = m_bonepos[0]._v[1] + m_hipPosIncy;
//			m_hipPosIncy = 0;
	
		}
	 	if (m_bonepos[18].y() < m_bonepos[22].y())//wxg 20150630
	 	{
	 		hpos._v[1] -= m_bonepos[18].y() - m_inirightfpos.y();	// edit by mvp ## 2015-7-20
      	}
	 	else
		{
	 		hpos._v[1] -= m_bonepos[22].y() - m_inileftfpos.y();
	 	}
		// edit the Angle of the // add by mvp ## 2015-7-22
		//if(m_left_stay >= 50)
		//{

		//	for(int i = 19;i< 21;i++)
		//	{
		//		//osg::Quat tmp_q;
		//		//tmp_q = quatutils::slerp(m_segment[i],m_old_legs[i-16],0.4);	//80% of the old quat 
		//		//m_old_legs[i-16] = m_segment[i];
		//		//m_segment[i] = tmp_q;
		//		m_segment[i] = smooth_stay(m_segment[i],(i-16));
		//	}
		//}
		// //add by mvp ## 2015-7-25
		// //sort of smooth
		//for(int i = 19;i< 21;i++)
		//{
		//	if (m_left_stay >= 50)
		//		m_segment[i] = smooth_stay(m_segment[i],(i-16));
		//	else
		//		m_state[i-16] = 0;
		//}
		//for(int i = 15;i< 17;i++)
		//{
		//	if (m_right_stay >= 50)
		//		m_segment[i] = smooth_stay(m_segment[i],(i-15));
		//	else
		//		m_state[i-15] = 0;
		//}

		// edit by mvp ## 2015-7-31
		//if(m_right_stay >= 40)
		//{		
		//	osg::Quat tmp_q;
		//	Quaternion<double> clac_angle;
		//	float pitch_new,yaw_new,roll_new;
		//	float pitch_foot,yaw_foot_l,yaw_foot_r,roll_foot;
		//	float angle_l,angle_r;
		//	yaw_new = 0.0;
		//	pitch_new = 0.0;
		//	roll_new = 0.0;
		//	pitch_foot = 0.0;
		//	yaw_foot_l = 0.0;
		//	yaw_foot_r = 0.0;
		//	roll_foot = 0.0;
		//	angle_l = 0.0;
		//	angle_r = 0.0;
		//	//calc the angle of the X Z Y Direction

		//	clac_angle.set(m_segment[0].w(),m_segment[0].x(),m_segment[0].y(),m_segment[0].z());
		//	clac_angle.Quat2Angle(pitch_new,yaw_new,roll_new,XZY);
		//	PRINTF("hip:\t%f\tyaw:\t%f\troll:\t%f\t",pitch_new,yaw_new,roll_new);

		//	clac_angle.set(m_segment[18].w(),m_segment[18].x(),m_segment[18].y(),m_segment[18].z());
		//	clac_angle.Quat2Angle(pitch_foot,yaw_foot_r,roll_foot,XZY);

		//	clac_angle.set(m_segment[22].w(),m_segment[22].x(),m_segment[22].y(),m_segment[22].z());
		//	clac_angle.Quat2Angle(pitch_foot,yaw_foot_l,roll_foot,XZY);

		//	yaw_new = yaw_new - (yaw_foot_l + yaw_foot_r)/2.0;	// calc the angle of the change;

		//	PRINTF("leg_r::\t%f\tyaw_l:\t%f\tyaw_r:\t%f\troll:\t%f\t",pitch_foot,yaw_foot_l,yaw_foot_r,roll_foot);
		//	angle_l = yaw_new*m_fLCorrectParam;
		//	angle_r = yaw_new*m_fRCorrectParam;

		//	//edit by mvp ## 2015-7-30
		//	if(yaw_new >0.0)
		//	{
		//		// print the angle of the leg // debug by mvp ## 2015-7-30
		//		clac_angle.set(m_segment[15].w(),m_segment[15].x(),m_segment[15].y(),m_segment[15].z());
		//		clac_angle.Quat2Angle(pitch_new,yaw_new,roll_new,XZY);
		//		PRINTF("befor:\t%f\tyaw:\t%f\troll:\t%f\t",pitch_new,yaw_new,roll_new);
		//		PRINTF("edit angle\t%f\tpitch:\t%f\troll:\t%f\t",0.0,(m_fRpitchParam)*angle_r,(m_fRrollParam)*angle_r);
		//		
		//		tmp_q = calcEulorToQuat(0.0,m_fRpitchParam*angle_r,m_fRrollParam*angle_r);
		//		m_segment[15] = tmp_q * m_segment[15];
		//		// after calc
		//		clac_angle.set(m_segment[15].w(),m_segment[15].x(),m_segment[15].y(),m_segment[15].z());
		//		clac_angle.Quat2Angle(pitch_new,yaw_new,roll_new,XZY);
		//		PRINTF("After:\t%f\tyaw:\t%f\troll:\t%f\t",pitch_new,yaw_new,roll_new);
		//		

		//		tmp_q = calcEulorToQuat(0.0,0.0,(0.0)*angle_l);
		//		m_segment[19] = tmp_q * m_segment[19];
		//	}
		//	else
		//	{
		//		tmp_q = calcEulorToQuat(0.0,m_fLpitchParam*angle_l,m_fLrollParam*angle_l);
		//		m_segment[19] = tmp_q * m_segment[19];
		//		tmp_q = calcEulorToQuat(0.0,0.0,0.0);
		//		m_segment[15] = tmp_q * m_segment[15];
		//	}
		// }
		// rm the leg stay //debug by mvp ##2015-7-31
		osg::Quat tmp_q;
		Quaternion<double> clac_angle;
		float pitch_new,yaw_new,roll_new;
		float pitch_foot,yaw_foot_l,yaw_foot_r,roll_foot;
		float angle_l,angle_r;
		yaw_new = 0.0;
		pitch_new = 0.0;
		roll_new = 0.0;
		pitch_foot = 0.0;
		yaw_foot_l = 0.0;
		yaw_foot_r = 0.0;
		roll_foot = 0.0;
		angle_l = 0.0;
		angle_r = 0.0;
		//calc the angle of the X Z Y Direction

		clac_angle.set(m_segment[0].w(),m_segment[0].x(),m_segment[0].y(),m_segment[0].z());
		clac_angle.Quat2Angle(pitch_new,yaw_new,roll_new,XZY);
		PRINTF("hip:\t%f\tyaw:\t%f\troll:\t%f\t",pitch_new,yaw_new,roll_new);

		clac_angle.set(m_segment[18].w(),m_segment[18].x(),m_segment[18].y(),m_segment[18].z());
		clac_angle.Quat2Angle(pitch_foot,yaw_foot_r,roll_foot,XZY);

		clac_angle.set(m_segment[22].w(),m_segment[22].x(),m_segment[22].y(),m_segment[22].z());
		clac_angle.Quat2Angle(pitch_foot,yaw_foot_l,roll_foot,XZY);

		yaw_new = yaw_new - (yaw_foot_l + yaw_foot_r)/2.0;	// calc the angle of the change;

		PRINTF("leg_r::\t%f\tyaw_l:\t%f\tyaw_r:\t%f\troll:\t%f\t",pitch_foot,yaw_foot_l,yaw_foot_r,roll_foot);
		angle_l = yaw_new*m_fLCorrectParam;
		angle_r = yaw_new*m_fRCorrectParam;

		//edit by mvp ## 2015-7-30
		if(yaw_new >0.0)
		{
			// print the angle of the leg // debug by mvp ## 2015-7-30
			clac_angle.set(m_segment[15].w(),m_segment[15].x(),m_segment[15].y(),m_segment[15].z());
			clac_angle.Quat2Angle(pitch_new,yaw_new,roll_new,XZY);
			PRINTF("befor:\t%f\tyaw:\t%f\troll:\t%f\t",pitch_new,yaw_new,roll_new);
			PRINTF("edit angle\t%f\tpitch:\t%f\troll:\t%f\t",0.0,(m_fRpitchParam)*angle_r,(m_fRrollParam)*angle_r);
				
			tmp_q = calcEulorToQuat(0.0,m_fRpitchParam*angle_r,m_fRrollParam*angle_r);
			m_segment[15] = tmp_q * m_segment[15];
			// after calc
			clac_angle.set(m_segment[15].w(),m_segment[15].x(),m_segment[15].y(),m_segment[15].z());
			clac_angle.Quat2Angle(pitch_new,yaw_new,roll_new,XZY);
			PRINTF("After:\t%f\tyaw:\t%f\troll:\t%f\t",pitch_new,yaw_new,roll_new);
				

			tmp_q = calcEulorToQuat(0.0,0.0,0.0);
			m_segment[19] = tmp_q * m_segment[19];
		}
		else
		{
			tmp_q = calcEulorToQuat(0.0,m_fLpitchParam*angle_l,m_fLrollParam*angle_l);
			m_segment[19] = tmp_q * m_segment[19];
			tmp_q = calcEulorToQuat(0.0,0.0,0.0);
			m_segment[15] = tmp_q * m_segment[15];
		}
		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());//更新根节点位置
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
		}
	}
	m_segment[23].set(hpos.x() - hipspos.x(), hpos.y() - hipspos.y(), hpos.z() - hipspos.z(), 0);//根节点偏移 maya才用，已注去
	//m_segment[23].set(hpos.x(), hpos.y(), hpos.z(), 0);
	//hipspos = hpos;
}

// stay leg change // changed by mvp ## 2015-7-24
//void Sender::calcPosition(bool cali)
//{
//	static bool rfootonfloor = false, lfootonfloor=false;
//	static double velocity = 0; 
//	static osg::Vec3f rightfootpos, leftfootpos, hipspos;
//
//	if (cali)
//	{
//		rightfootpos.set(m_inirightfpos.x(), m_inirightfpos.y(), m_inirightfpos.z());
//		leftfootpos.set(m_inileftfpos.x(), m_inileftfpos.y(), m_inileftfpos.z());
//		hipspos.set(m_inihipspos.x(), m_inihipspos.y(), m_inihipspos.z());
//		return;
//	}
//	if (!m_calihavedone)
//		return;
//
//	osg::Vec3f rpos(0.0, 0.0, 0.0);//右脚掌相对根节点的位移
//	int nseg = 18;//从右脚掌最末端开始 累计所有父节点的位移 
//	while (nseg != 0)
//	{
//		rpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
//		nseg = m_parentbone[nseg];
//	}
//
//	osg::Vec3f lpos(0.0, 0.0, 0.0);
//	nseg = 22;//左脚掌
//	while (nseg != 0)
//	{
//		lpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
//		nseg = m_parentbone[nseg];
//	}
//
//	osg::Vec3f hpos(0.0, 0.0, 0.0);
//	if (jumpable())//有跳跃
//	{
//		
//		hpos = m_bonepos[0];
//		hpos._v[0] += m_hipPosIncx;//原位置+移动值=根节点新位置
//		hpos._v[1] += m_hipPosIncy;
//		hpos._v[2] += m_hipPosIncz;
//		for (int i=0; i<BONENUM; i++)
//		{
//			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
//			nseg = i;
//			while (nseg != 0)
//			{
//				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
//				nseg = m_parentbone[nseg];
//			}
//		}
//		leftfootpos  = m_bonepos[22];
//		rightfootpos = m_bonepos[18];
//	}
//	else//静止或行走
//	{
//		//m_jumpposqueue.clear();
//		if ((m_right_stay < 30) && (m_left_stay >= 30))//左脚的加速度小 
//		{
//			leftfootpos[1] = m_inileftfpos.y();
//			hpos = leftfootpos - lpos;//根节点的位置
//			rightfootpos = hpos + rpos;
//		}
//		else if ((m_right_stay >= 30) && (m_left_stay < 30))//右脚不动
//		{
//			rightfootpos[1] = m_inirightfpos.y();
//			hpos = rightfootpos - rpos;
//			leftfootpos = hpos + lpos;
//		}
//		else if((m_right_stay >= 30) && (m_left_stay >= 30))
//		{
//			leftfootpos[1] = m_inileftfpos.y();
//			rightfootpos[1] = m_inirightfpos.y();
//			hpos = (leftfootpos - lpos + rightfootpos - rpos)/2;
//		}
//		else if (m_right_stay > 20)
//		{
//			rightfootpos[1] = m_inirightfpos.y();
//			hpos = rightfootpos - rpos;
//			leftfootpos = hpos + lpos;
//		}
//		else if( m_left_stay > 20)
//		{
//			leftfootpos[1] = m_inileftfpos.y();
//			hpos = leftfootpos - lpos;
//			rightfootpos = hpos + rpos;
//		}
//		else
//		{
//			hpos = hipspos;
//		}
//
//		if (m_hipPosIncy > maxgtesty)
//			maxgtesty = m_hipPosIncy;//根节点最大上升高度 没有用到
//		if (m_jumpable)
//		{
//			hpos._v[1] = m_bonepos[0]._v[1] + m_hipPosIncy;
////			m_hipPosIncy = 0;
//	
//		}
//	 	if (m_bonepos[18].y() < m_bonepos[22].y())//wxg 20150630
//	 	{
//	 		hpos._v[1] -= m_bonepos[18].y() - m_inirightfpos.y();	// edit by mvp ## 2015-7-20
//      	}
//	 	else
//		{
//	 		hpos._v[1] -= m_bonepos[22].y() - m_inileftfpos.y();
//	 	}
//		for (int i=0; i<BONENUM; i++)
//		{
//			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());//更新根节点位置
//			nseg = i;
//			while (nseg != 0)
//			{
//				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
//				nseg = m_parentbone[nseg];
//			}
//		}
//	}
//	m_segment[23].set(hpos.x() - hipspos.x(), hpos.y() - hipspos.y(), hpos.z() - hipspos.z(), 0);//根节点偏移 maya才用，已注去
//	//m_segment[23].set(hpos.x(), hpos.y(), hpos.z(), 0);
//	//hipspos = hpos;
//}

// add by mvp ## 2015-7-10
// change the struct of left and right stay. 

/*
void Sender::calcPosition(bool cali)
{
	static bool rfootonfloor = false, lfootonfloor=false;
	static double velocity = 0; 
	static osg::Vec3f rightfootpos, leftfootpos, hipspos;

	if (cali)
	{
		rightfootpos.set(m_inirightfpos.x(), m_inirightfpos.y(), m_inirightfpos.z());
		leftfootpos.set(m_inileftfpos.x(), m_inileftfpos.y(), m_inileftfpos.z());
		hipspos.set(m_inihipspos.x(), m_inihipspos.y(), m_inihipspos.z());
		return;
	}
	if (!m_calihavedone)
		return;

	osg::Vec3f hpos(0.0, 0.0, 0.0);
	int nseg = 0;
	if (jumpable())//有跳跃
	{		
		hpos = m_bonepos[0];
		hpos._v[0] += m_hipPosIncx;//原位置+移动值=根节点新位置
		hpos._v[1] += m_hipPosIncy;
		hpos._v[2] += m_hipPosIncz;
		// init the Inc data //add by mvp ## 2015-7-8
		m_hipPosIncx = 0;
		m_hipPosIncy = 0;
		m_hipPosIncz = 0;
		// set the parent order
		m_parentbone[15] = 0;
		m_parentbone[16] = 15;
		m_parentbone[17] = 16;
		m_parentbone[18] = 17;

		m_parentbone[19] = 0;
		m_parentbone[20] = 19;
		m_parentbone[21] = 20;
		m_parentbone[22] = 21;

		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
		}
		leftfootpos  = m_bonepos[22];
		rightfootpos = m_bonepos[18];
		hipspos = hpos;
	}
	else//静止或行走
	{
		int tmp_flag = 0;
		if(m_left_stay)
			tmp_flag |= 0x01;
		if(m_right_stay)
			tmp_flag |= 0x02;
		switch(tmp_flag)
		{
		case 0:	// both for  move	// add by mvp ## 2015-7-9
			m_parentbone[15] = 0;
			m_parentbone[16] = 15;
			m_parentbone[17] = 16;
			m_parentbone[18] = 17;

			m_parentbone[19] = 0;
			m_parentbone[20] = 19;
			m_parentbone[21] = 20;
			m_parentbone[22] = 21;
			hpos = hipspos;
			for (int i=0; i<BONENUM; i++)
			{
				m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
				nseg = i;
				while (nseg != 0)
				{
					m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
					nseg = m_parentbone[nseg];
				}
			}
			leftfootpos  = m_bonepos[22];
			rightfootpos = m_bonepos[18];
			hipspos = m_bonepos[0];
			break;
		case 1:	// the left stay and the right move
			m_parentbone[22] = -1;
			m_parentbone[21] = 22;
			m_parentbone[20] = 21;
			m_parentbone[19] = 20;
			m_parentbone[0] = 19;		// edit by mvp ## 2015-7-14
			for(int i =19 ;i< 23;i++)
			{
				m_bonepos[i].set(leftfootpos.x(), leftfootpos.y(), leftfootpos.z());
				nseg = i;
				while (nseg != 22)
				{
					m_bonepos[i] -= m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
					nseg = m_parentbone[nseg];
				}
			}
			//PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",0,(float)m_bonepos[0]._v[0],(float)m_bonepos[0]._v[1],(float)m_bonepos[0]._v[2]);
			//PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",19,(float)m_bonepos[19]._v[0],(float)m_bonepos[19]._v[1],(float)m_bonepos[19]._v[2]);
			m_bonepos[0] = m_bonepos[19];
			hpos = m_bonepos[0];
			PRINTF("AAAAAAAAAAA\n");
			//hpos._v[1] += m_hipPosIncy;	// add the Incy of the Y distance
			m_hipPosIncy = 0;
			for (int i=0; i<BONENUM; i++)
			{
				if(i >= 19 && i<= 22)
					continue;
				m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
				nseg = i;
				while (nseg != 0)
				{
					m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
					nseg = m_parentbone[nseg];
				}
				PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",i,(float)m_bonepos[i]._v[0],(float)m_bonepos[i]._v[1],(float)m_bonepos[i]._v[2]);
			}
			rightfootpos = m_bonepos[18];
			hipspos = m_bonepos[0];
			break;
		case 2:	// the right stay and left move
			m_parentbone[18] = -1;
			m_parentbone[17] = 18;
			m_parentbone[16] = 17;
			m_parentbone[15] = 16;
			m_parentbone[0] = 15;
			for(int i = 15;i< 19;i++)
			{
				m_bonepos[i].set(rightfootpos.x(), leftfootpos.y(), leftfootpos.z());
				nseg = i;
				while (nseg != 18)
				{
					m_bonepos[i] -= m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
					nseg = m_parentbone[nseg];
				}
			}
			
			//PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",0,(float)m_bonepos[0]._v[0],(float)m_bonepos[0]._v[1],(float)m_bonepos[0]._v[2]);
			//PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",19,(float)m_bonepos[19]._v[0],(float)m_bonepos[19]._v[1],(float)m_bonepos[19]._v[2]);
			m_bonepos[0] = m_bonepos[15];
			hpos = m_bonepos[0];
			hpos._v[1] += m_hipPosIncy;	// add the Incy of the Y distance
			m_hipPosIncy = 0;
			for (int i=0; i<BONENUM; i++)
			{
				if(i>= 15 && i<= 18)
					continue;
				m_bonepos[i].set(hpos.x(),hpos.y(), hpos.z());
				nseg = i;
				while (nseg != 0)
				{
					m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
					nseg = m_parentbone[nseg];
				}
				PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",i,(float)m_bonepos[i]._v[0],(float)m_bonepos[i]._v[1],(float)m_bonepos[i]._v[2]);
			}
			leftfootpos = m_bonepos[22];
			m_bonepos[0]._v[1] += m_hipPosIncy;	// add the Incy of the Y distance
			m_hipPosIncy = 0;
			hipspos = m_bonepos[0];

			break;
		case 3: // stay for no move
			m_parentbone[22] = -1; // calc the two leg
			m_parentbone[21] = 22;
			m_parentbone[20] = 21;
			m_parentbone[19] = 20;
			
			m_parentbone[18] = -1;
			m_parentbone[17] = 18;
			m_parentbone[16] = 17;
			m_parentbone[15] = 16;
			// the bone pos of the two leg
			for (int i=15; i<22; i++)
			{
				nseg = i;
				if(i <19)
				{
					m_bonepos[i].set(rightfootpos.x(), rightfootpos.y(), rightfootpos.z());
					while (nseg != 18)
					{
						m_bonepos[i] -= m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
						//PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",i,(float)m_bonepos[i]._v[0],(float)m_bonepos[i]._v[1],(float)m_bonepos[i]._v[2]);
						nseg = m_parentbone[nseg];
					}
				}else
				{
					m_bonepos[i].set(leftfootpos.x(), leftfootpos.y(), leftfootpos.z());
					while (nseg != 22)
					{
						m_bonepos[i] -= m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
						//PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",i,(float)m_bonepos[i]._v[0],(float)m_bonepos[i]._v[1],(float)m_bonepos[i]._v[2]);
						nseg = m_parentbone[nseg];
					}
				}					
				

			}
			// set the pos of the new state
			m_bonepos[0]._v[0] = (m_bonepos[15]._v[0] + m_bonepos[19]._v[0])/2.0;
			//m_bonepos[0]._v[1] = (leftfootpos.y() - lpos.y() + rightfootpos.y() - rpos.y())/2.0;// edit by mvp ## 2015-7-13
			m_bonepos[0]._v[2] = (m_bonepos[15]._v[2] + m_bonepos[19]._v[2])/2.0;
			m_bonepos[0]._v[1] += m_hipPosIncy;	// add the Incy of the Y distance
			m_hipPosIncy = 0;
			hpos = m_bonepos[0];
			// the arm and the chest 
			for(int i = 0;i < BONENUM;i++)
			{
				if(i >= 15 && i<= 22)
					continue;
				m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
				nseg = i;
				while (nseg != 0)
				{
					m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
					nseg = m_parentbone[nseg];
				}
			}
			hipspos = m_bonepos[0];
			break;
		}

	}
	m_segment[23].set(hpos.x() - hipspos.x(), hpos.y() - hipspos.y(), hpos.z() - hipspos.z(), 0);//根节点偏移 maya才用，已注去
	m_parentbone[15] = 0;
	m_parentbone[16] = 15;
	m_parentbone[17] = 16;
	m_parentbone[18] = 17;

	m_parentbone[19] = 0;
	m_parentbone[20] = 19;
	m_parentbone[21] = 20;
	m_parentbone[22] = 21;
	//m_segment[23].set(hpos.x(), hpos.y(), hpos.z(), 0);
	//hipspos = hpos;
}
*/
// add the two methord // add by mvp ## 2015-7-16
/*
void Sender::calcPosition(bool cali)
{
	static bool rfootonfloor = false, lfootonfloor=false;
	static double velocity = 0; 
	static osg::Vec3f rightfootpos, leftfootpos, hipspos;

	if (cali)
	{
		rightfootpos.set(m_inirightfpos.x(), m_inirightfpos.y(), m_inirightfpos.z());
		leftfootpos.set(m_inileftfpos.x(), m_inileftfpos.y(), m_inileftfpos.z());
		hipspos.set(m_inihipspos.x(), m_inihipspos.y(), m_inihipspos.z());
		return;
	}
	if (!m_calihavedone)
		return;
	osg::Vec3f rpos(0.0, 0.0, 0.0);//右脚掌相对根节点的位移
	int nseg = 18;//从右脚掌最末端开始 累计所有父节点的位移 
	while (nseg != 0)
	{
		rpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
		nseg = m_parentbone[nseg];
	}

	osg::Vec3f lpos(0.0, 0.0, 0.0);
	nseg = 22;//左脚掌
	while (nseg != 0)
	{
		lpos += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
		nseg = m_parentbone[nseg];
	}
	osg::Vec3f hpos(0.0, 0.0, 0.0);
	if (jumpable())//有跳跃
	{		
		hpos = m_bonepos[0];
		hpos._v[0] += m_hipPosIncx;//原位置+移动值=根节点新位置
		hpos._v[1] += m_hipPosIncy;
		hpos._v[2] += m_hipPosIncz;
		// init the Inc data //add by mvp ## 2015-7-8
		m_hipPosIncx = 0;
		m_hipPosIncy = 0;
		m_hipPosIncz = 0;

		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
		}
		leftfootpos  = m_bonepos[22];
		rightfootpos = m_bonepos[18];
		hipspos = hpos;
	}
	else//静止或行走
	{
		int tmp_flag = 0;
		if(m_left_stay)
			tmp_flag |= 0x01;
		if(m_right_stay)
			tmp_flag |= 0x02;
		switch(tmp_flag)
		{
		case 0:	// both for  move	// add by mvp ## 2015-7-9
			hpos = hipspos;	// the last postion
			break;
		case 1:	// the left stay and the right move
			leftfootpos[1] = m_inileftfpos.y();
			hpos = leftfootpos - lpos;
			rightfootpos = hpos + rpos;
			break;
		case 2:	// the right stay and left move
			rightfootpos[1] = m_inileftfpos.y();
			hpos = rightfootpos - rpos;
			leftfootpos = hpos + lpos;
			break;
		case 3: // stay for no move
			hpos = hipspos;
			break;
		}
		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(hpos.x(), hpos.y(), hpos.z());
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);//更新位移信息
				nseg = m_parentbone[nseg];
			}
			//PRINTF("%d\tX:\t%f\tY:\t%f\tZ:\t%f\n",i,(float)m_bonepos[i]._v[0],(float)m_bonepos[i]._v[1],(float)m_bonepos[i]._v[2]);
		}
		rpos = m_bonepos[18];
		lpos = m_bonepos[22];
		hipspos = m_bonepos[0];
		switch(tmp_flag)
		{
		case 0:	// both for  move	// add by mvp ## 2015-7-9
			break;
		case 1:	// the left stay and the right move
			m_bonepos[22] = leftfootpos*0.9 + lpos* 0.1;
			//leftfootpos = m_bonepos[22];
			break;
		case 2:	// the right stay and left move
			m_bonepos[18] = rightfootpos*0.9 + rpos* 0.1;
			//rightfootpos = m_bonepos[18];
			break;
		case 3: // stay for no move
			m_bonepos[22] = leftfootpos*0.9 + lpos* 0.1;
			//leftfootpos = m_bonepos[22];
			m_bonepos[18] = rightfootpos*0.9 + rpos* 0.1;
			//rightfootpos = m_bonepos[18];
			break;
		}


	}
	m_segment[23].set(hpos.x() - hipspos.x(), hpos.y() - hipspos.y(), hpos.z() - hipspos.z(), 0);//根节点偏移 maya才用，已注去
	//m_segment[23].set(hpos.x(), hpos.y(), hpos.z(), 0);
	//hipspos = hpos;
}
*/
/*void Sender::calcPosition(bool cali)
{
	static osg::Vec3f hipspeed(0.0, 0.0, 0.0);
	int nseg;
	if (cali)
	{
		for (int i=0; i<BONENUM; i++)
		{
			m_bonepos[i].set(m_bonepos[0].x(), m_bonepos[0].y(), m_bonepos[0].z());
			nseg = i;
			while (nseg != 0)
			{
				m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
				nseg = m_parentbone[nseg];
			}
		}
		hipspeed.set(0.0, 0.0, 0.0);
		return;
	}
	if (!m_calihavedone)
		return;
	osg::Vec3f hippos;
	if (m_minmoveflag <= 150)
	{
         osg::Vec3f movelesssegposfromhips(0.0, 0.0, 0.0);
		 int nseg = m_movelessseg;
		 while (nseg != 0)
		 {
			 movelesssegposfromhips += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
			 nseg = m_parentbone[nseg];
		 }
		 if ((m_movelessseg == 18) || (m_movelessseg == 22))
		 {
			 static double velocity = 0;
			 if (m_premovelessseg != m_movelessseg)
			 {
				 velocity = 0;
			 }
			 else
			 {
				 double ddis;
				 ddis = velocity / 120 + 9.8 /120/120/2;
				 velocity += 9.8/120;
				 if ((m_bonepos[22].y() - m_inileftfpos.y()) <ddis)
					 ddis = m_bonepos[22].y()  - m_inileftfpos.y();

				 if ((m_bonepos[18].y() - m_inileftfpos.y()) < ddis)
					 ddis = m_bonepos[18].y() - m_inileftfpos.y();
				 m_bonepos[m_movelessseg][1] -= ddis;
			 }
		 }
		 hippos = m_bonepos[0];
		 m_bonepos[0] = m_bonepos[m_movelessseg] - movelesssegposfromhips;
		 for (int i=0; i<BONENUM; i++)
		 {
			 m_bonepos[i].set(m_bonepos[0].x(), m_bonepos[0].y(), m_bonepos[0].z());
			 nseg = i;
			 while (nseg != 0)
			 {
				 m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
				 nseg = m_parentbone[nseg];
			 }
		 }
		 hipspeed = m_bonepos[0] - hippos;
		 hipspeed /= 1.0/110.0; 
	}
	else
	{
         hippos = m_bonepos[0];
         //计算新的hips位移
		 osg::Vec3f hipaddpos;
		 hipaddpos = hipspeed * (1.0/110.0) + osg::Vec3f(0.0, -1.0, 0.0) * (0.5*9.8/110.0/110.0);
		 for (int i=0; i<BONENUM; i++)
		 {
			 float temp = m_inileftfpos.y() - m_bonepos[i].y();
			 if (temp > hipaddpos.y())
				 hipaddpos[1] = temp;
		 }
		 m_bonepos[0] = hippos + hipaddpos;
		 hipspeed += osg::Vec3f(0.0, -1.0, 0.0) * (9.8/110.0);

		 //end
		 for (int i=0; i<BONENUM; i++)
		 {
			 m_bonepos[i].set(m_bonepos[0].x(), m_bonepos[0].y(), m_bonepos[0].z());
			 nseg = i;
			 while (nseg != 0)
			 {
				 m_bonepos[i] += m_segment[m_parentbone[nseg]] * osg::Vec3f(m_bones[nseg].px, m_bones[nseg].py, m_bones[nseg].pz);
				 nseg = m_parentbone[nseg];
			 }
		 }

	}
	m_premovelessseg = m_movelessseg;
	m_segment[23].set(m_bonepos[0].x() - m_inihipspos.x(), m_bonepos[0].y() - m_inihipspos.y(), m_bonepos[0].z() - m_inihipspos.z(), 0);
}*/

void Sender::setRhandAngle(double aangle)
{
    m_rhangle = aangle * 3.1415926 / 180;
}

void Sender::setModel(IModel* model)
{
    m_model = model;
}

void Sender::ceckPiracy(Configuration* config)
{
	//wxg 先去掉限制
	return;

	QSettings* reg = new QSettings(CQREGISTERPATH,  
		QSettings::NativeFormat);
	int fyear, fmonth, fday, lyear, lmonth, lday;
	long long ldate = config->get(CQMAIN, CQLDATE, "0").toLong();
	bool isAvailable = true;
	if (reg->value(QString("%1").arg(ldate)).toString() == QString("%1").arg(ldate))
	{
		PRINTF("___regedit_currentuser_instech:%ld",ldate);
		isAvailable = false;
	}
	else
	{
		if (!qcommon::decode(config->get(CQMAIN, CQFDATE, "0").toLong(), fyear, fmonth, fday))//wxg 算法可能有问题
		{
			PRINTF("___fdata decode failed.");
			isAvailable = false;
		}
		else
		{
			if (!qcommon::decode(config->get(CQMAIN, CQLDATE, "0").toLong(), lyear, lmonth, lday))
			{
				PRINTF("___ldata decode failed.");
				isAvailable = false;
			}
			else
			{
				if (fyear>lyear)
				{
					PRINTF("___fyear>lyear.");
					isAvailable = false;
				}
				else if ((fyear == lyear)&&(fmonth > lmonth))
				{
					PRINTF("___(fyear == lyear)&&(fmonth > lmonth).");
					isAvailable = false;
				}
				else if ((fyear == lyear)&&(fmonth == lmonth)&&(fday > lday))
				{
					PRINTF("___fyear == lyear)&&(fmonth == lmonth)&&(fday > lday).");
					isAvailable = false;
				}
				else
				{
					//获得当前时间
					QDateTime nowTime = QDateTime::currentDateTime();
					int nyear, nmonth, nday;
					nowTime.date().getDate(&nyear, &nmonth, &nday);
					//与上次的时间比较
					if (nyear < fyear)
					{
						PRINTF("___nyear < fyear(cur vs last).");
						isAvailable = false;
					}
					else if ((nyear == fyear)&&(nmonth < fmonth))
					{
						PRINTF("___(nyear == fyear)&&(nmonth < fmonth)(cur vs last).");
						isAvailable = false;
					}
					else if ((nyear == fyear)&&(nmonth == fmonth)&&(nday < fday))
					{
						PRINTF("___(nyear == fyear)&&(nmonth == fmonth)&&(nday < fday)(cur vs last).");
						isAvailable = false;
					}
					else
					{
// 						if (nyear > lyear)//disable by wxg 跨年跨月会有问题
// 							isAvailable = false;
// 						else if ((nyear == lyear)&&(nmonth > lmonth))
// 							isAvailable = false;
// 						else if ((nyear == lyear)&&(nmonth == lmonth)&&(nday > lday))
// 							isAvailable = false;

						//保存当次日期
						config->set(CQMAIN, CQFDATE, QString("%1").arg(qcommon::encode(nyear, nmonth, nday)));
						TCHAR filename[MAX_PATH];
						if (qcommon::GetExePath(filename))
						{
							_tcscat(filename, _T(CQSETTINGPATH));
							QString fn = filename;
							config->flush(fn);
						}

					}

				}
			}

		}
	}
	if (!isAvailable)
	{
		reg->setValue(QString("%1").arg(ldate), QString("%1").arg(ldate));
		QMessageBox::warning(0, QObject::tr("提示"), QObject::tr("软件需要更新才能使用，请联系开发团队！"));
		delete reg;
		//退出程序
		exit(0);
	}
	delete reg;
}

//void Sender::PrintLog( const char* pMsg )
//{
//	if(m_model)
//		m_model->PrintLog(pMsg);
//}