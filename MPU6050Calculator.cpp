#include "MPU6050Calculator.h"
#include <assert.h>
#include "CoolLog.h"

double MPU6050Calculator::postyaw[BODY_PART_COUNT] = {361,361,361};
double MPU6050Calculator::postpitch[BODY_PART_COUNT] = {361,361,361};
double MPU6050Calculator::m_rootRoll[2] = {0};

double mindiffangle(double angle1, double angle2)
{

	double rlt;
	rlt = abs(angle1 - angle2); 
	if (rlt > 180)
		rlt = 360 - rlt;
	return rlt;
}

MPU6050Calculator::MPU6050Calculator(ISender* sender)
{
	m_sender = sender;
    //port = NULL;
	//viewer = NULL;
	preroll = -720;
	preyaw = -720;
	prepitch = -720;
	yaw0 = 0;
	pitch0 = 0;
	roll0 = 0;
	m_firstflag = true;
	m_framecount = 0;
	m_scount = 0;
	m_preincax = 0;
	m_preincay = 0;
	m_preincaz = 0;
	m_cali = false;
	m_acount = 0;
	m_sacount = 0;
	m_ID = -1;
	m_SegmentLabel = -1;
	m_usequat = false;
	//m_quater.setBaseYaw(165);
	m_bYawBaseNode = false;
	m_iBodyPart = 0;
	m_bYawOffset = 0.0;
//	m_iCount = 0;
}

MPU6050Calculator::~MPU6050Calculator()
{

}

void MPU6050Calculator::maskcali()
{
	m_cali = true;
	if (m_usequat)
	{
		yaw0 = atan2(2 * m_pquat.x() * m_pquat.y() + 2 * m_pquat.w() * m_pquat.z(), -2 * m_pquat.y()*m_pquat.y() - 2 * m_pquat.z() * m_pquat.z() + 1)* 180/pi; // yaw
		pitch0 = -asin(-2 * m_pquat.x() * m_pquat.z() + 2 * m_pquat.w() * m_pquat.y())* 180/pi; // pitch
		roll0 = atan2(2 * m_pquat.y() * m_pquat.z() + 2 * m_pquat.w() * m_pquat.x(), -2 * m_pquat.x() * m_pquat.x() - 2 * m_pquat.y() * m_pquat.y() + 1)* 180/pi; // roll
		if (m_bYawBaseNode)//m_SegmentLabel == 9
		{
			
			//int ndatasource = MPU6050TEST::m_config->get(CMAIN, CDATASOURCE, "1").toInt();
			//int nangleinc = MPU6050TEST::m_config->get(CMAIN, CCALIANGLEINC, "90").toInt();
			//m_sender->setRhandAngle(preyaw);
			//if (ndatasource == 1)
			//{
			postyaw[m_iBodyPart] = yaw0 + m_angleinc;//wxg 手指的yaw轴朝向手腕
			//if(m_iBodyPart==1)
			//	postyaw = preyaw-m_angleinc;
			if (postyaw[m_iBodyPart] > 180)
				postyaw[m_iBodyPart] -= 360;
			//}
			//else
			//{
			//	postyaw = preyaw - nangleinc;// preyaw + 90;
			//	if (postyaw < -180)
			//		postyaw += 360;
			//}
			PRINTF("maskcali: usequat : yaw = %f",postyaw[m_iBodyPart]);

		}
		quat0 = CalcQuatBetweenTwoEulor(yaw0, 0, 0, yaw0, pitch0, roll0);
		m_quater.setIniAngle(yaw0, pitch0, roll0);
		m_quater.setIniQuat(quat0);
	}
	else
	{
		if (m_bYawBaseNode)//右肘或中指 m_SegmentLabel == 9
		{
			//int ndatasource = MPU6050TEST::m_config->get(CMAIN, CDATASOURCE, "1").toInt();
			//int nangleinc = MPU6050TEST::m_config->get(CMAIN, CCALIANGLEINC, "90").toInt();
			//m_sender->setRhandAngle(preyaw);
			//if (ndatasource == 1)
			//{
			postyaw[m_iBodyPart] = preyaw + m_angleinc;//参考方向即校准时人的正前方 此时右手航向朝前，左手航向朝后
			postpitch[m_iBodyPart] = prepitch;//wxg20161202 姆指\小指的pitch不在同一个水平面上

			if(m_iBodyPart == 1)
			{
				postyaw[m_iBodyPart] += 180;//左手航向转到正前方
			}
			else if(m_iBodyPart == 0)
			{
				newPosition(-1,0,0,0);
				m_sender->hipsPosChange(-1, -1, -1);//wxg
			}

			//if(m_iBodyPart==1)//wxg 手指的yaw轴朝向手腕
			//	postyaw = preyaw-m_angleinc;
			// 			else if(m_iBodyPart == 0)//wxg Z轴方向没有标反，应该是‘点’
			// 				postyaw = preyaw - m_angleinc;
			if (postyaw[m_iBodyPart] > 180)
			{
				postyaw[m_iBodyPart] -= 360;
			}
// 			if(postyaw[0] == 361)
// 				postyaw[0] = postyaw[m_iBodyPart];

			//}
			//else
			//{
			//	postyaw = preyaw - nangleinc;// preyaw + 90;
			//	if (postyaw < -180)
			//		postyaw += 360;
			//}

			PRINTF("maskcali: m_bYawBaseNode.postyaw = %f,id=%d",postyaw[m_iBodyPart],m_ID);
		}
		//m_quater.setBaseYaw(postyaw);
		yaw0 = preyaw;
		pitch0 = 0;//prepitch;
		roll0 = 0;//preroll;
		quat0 = CalcQuatBetweenTwoEulor(yaw0, pitch0, roll0, preyaw, prepitch, preroll);
		pitch0 = prepitch;
		roll0 = preroll;
		m_quater.setIniAngle(yaw0, pitch0, roll0);
		m_quater.setIniQuat(quat0);//初始姿态对应的四元数
		if(m_ID == 1)
			PRINTF("yaw0 = %f,id=%d",yaw0,m_ID);
	}
	//add by mvp ## 2015-8-14
//	m_sender->setYaw0(yaw0);
}

osg::Quat MPU6050Calculator::calcEulorToQuat(double yaw, double pitch, double roll)
{
// 	osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (roll), osg::Z_AXIS,
// 		osg::DegreesToRadians (pitch), osg::X_AXIS,
// 		osg::DegreesToRadians (-yaw), osg::Y_AXIS);
//  	return rlt;
	if (m_yawnegative)
		yaw = -yaw;
	if (m_pitchnegative)
		pitch = -pitch;
	if (m_rollnegative)
		roll = -roll;
    return quatutils::EulorToQuat(yaw, pitch, roll, m_eulorToQuatType);//换成上位机坐标系
}

void MPU6050Calculator::clearFrame()
{
	//m_FPS = m_totalframe;
	//m_totalframe = 0;
}
osg::Quat MPU6050Calculator::CalcQuatBetweenTwoEulor(double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2)
{
	osg::Quat quat1, quat2;
	quat1 = calcEulorToQuat(yaw2, pitch2, roll2);
	quat2 = calcEulorToQuat(yaw1, pitch1, roll1);
	osg::Quat rlt = quat1 *quat2.inverse();
	return rlt;

}

osg::Quat MPU6050Calculator::CalcuQuat(double yaw, double pitch, double roll)//相对于初始校准 maskcali
{
	//bool babscali = MPU6050TEST::m_config->get(CMAIN, CABSCALI, "0").toInt() == 1;
//		osg::Quat quat1, quat2, invquat1, invquat2;
// 	quat1 = calcEulorToQuat(yaw, pitch, roll);
// 	if (m_abscali)
// 		quat2 = calcEulorToQuat(yaw0, 0.0, 0.0);
// 
// 	else
// 		quat2 = calcEulorToQuat(yaw0, pitch0, roll0);
// 
// 	invquat1 = quat1.inverse();
// 	invquat2 = quat2.inverse();
// 	osg::Quat rlt;
// 	if (m_abscali)
// 		rlt = quat0.inverse() * quat1;
// 	else
// 		rlt = quat1;
// 
// 
// 	rlt = rlt * invquat2;
// 
// 	double baseyaw;
// 	int iOffsetDegree=90;
// 	if((m_SegmentLabel >= 24 && m_SegmentLabel <=26) || (m_SegmentLabel >= 42 && m_SegmentLabel <=44))//大拇指
// 		iOffsetDegree = 45;
// 	else if((m_SegmentLabel >= 27 && m_SegmentLabel <=29) || (m_SegmentLabel >= 45 && m_SegmentLabel <=47))//食指
// 	{
// 		iOffsetDegree = 75;
// 	}
// 	else if((m_SegmentLabel >= 33 && m_SegmentLabel <=35) || (m_SegmentLabel >= 51 && m_SegmentLabel <=53))//无名指
// 	{
// 		iOffsetDegree = 105;
// 	}
// 	else if((m_SegmentLabel >= 36 && m_SegmentLabel <=38) || (m_SegmentLabel >= 54 && m_SegmentLabel <=56))//小指
// 	{
// 		iOffsetDegree = 120;
// 	}
// 	if(m_iBodyPart == 1)
// 		baseyaw = yaw0 - iOffsetDegree;//以校准时的正前方为参考方向，应该是左手-90 右手+90
// 	else if(m_iBodyPart ==2)
// 		baseyaw = yaw0 + iOffsetDegree;
// 	else if(m_iBodyPart == 0)
// 	{
// 		if ((m_SegmentLabel >=12)&&(m_SegmentLabel <= 14))
// 			baseyaw = yaw0 - 90;
// 		else if ((m_SegmentLabel >=8)&&(m_SegmentLabel <= 10))
// 			baseyaw = yaw0 + 90;
// 		else if (m_SegmentLabel == 18 || m_SegmentLabel == 22)
// 			baseyaw = yaw0;
// 		else if (m_SegmentLabel == 6)
// 		{
// 			if (m_headreftohand)
// 				baseyaw = postyaw;
// 			else
// 			{
// 				baseyaw = yaw0 + m_headrefangle;
// 			}
// 
// 		}
// 		else
// 			baseyaw = postyaw;
// 	}
// 	if (baseyaw != -720)
// 	{
// 		osg::Quat quat3 = CalcQuatBetweenTwoEulor(baseyaw, 0, 0, yaw0, 0, 0);
// 		rlt = quat3.inverse() * rlt;
// 		osg::Quat quat4 = calcEulorToQuat(baseyaw - yaw0, 0, 0);
// 		rlt = rlt * quat4.inverse();
// 	}


	//wxg test
//m_abscali=0;/////////////////////////////////////////

	osg::Quat quat1, quat2, invquat1, invquat2;
	quat1 = calcEulorToQuat(yaw, pitch, roll);

	if(m_bYawOffset!=0.0)
	{
		osg::Quat quat3 = CalcQuatBetweenTwoEulor(0, 0, 0, m_bYawOffset, 0, 0);
		quat1 = quat3*quat1;
	}

	if (m_abscali)
		quat2 = calcEulorToQuat(yaw0, 0.0, 0.0);

	else
	{
		quat2 = calcEulorToQuat(yaw0, pitch0, roll0);
	}

	invquat1 = quat1.inverse();
	invquat2 = quat2.inverse();
	osg::Quat rlt;
	if (m_abscali)
	{
// 		if(postyaw[1] != 361)
// 		{
// 			rlt = quat1*quat0.inverse();
// 			PRINTF("%f,%f,%f,%f",rlt.x(),rlt.y(),rlt.z(),rlt.w());
// 		}

		rlt = quat0.inverse() * quat1;
// if(postyaw[1] != 361)
// 		PRINTF("%f,%f,%f,%f",rlt.x(),rlt.y(),rlt.z(),rlt.w());
	}
	else
	{
		rlt = quat1;
	}

	rlt = rlt * invquat2;

	//wxg test
// 	if(!m_abscali)//wxg20161007
// 	{
// 		rlt = quat1*invquat2;
// 		//		
// 		double baseyaw = 0;
// 		if(m_iBodyPart ==1 || m_SegmentLabel==14)
// 			baseyaw = yaw0-90;
// 		else if(m_iBodyPart ==2 || m_SegmentLabel==10)
// 			baseyaw = yaw0+90;
// 
// 		osg::Quat quat3 = CalcQuatBetweenTwoEulor(baseyaw, 0, 0, yaw0, 0, 0);
// 		rlt = quat3.inverse() *rlt *quat3;
// 
// 
// 	}//////////////////////////

//	if(m_abscali)
	{
		double baseyaw = 0;
		double basepitch = 0;	// add by mvp ## 2015-9-7
		double baseroll = 0;
		if(m_iBodyPart>0)
		{	
	// 		int iUnitDegree=15;
	// 		int iOffsetDegree=90;
	// 		if((m_SegmentLabel >= 24 && m_SegmentLabel <=26) || (m_SegmentLabel >= 42 && m_SegmentLabel <=44))//大拇指
	// 			iOffsetDegree = 45;
	// 		else if((m_SegmentLabel >= 27 && m_SegmentLabel <=29) || (m_SegmentLabel >= 45 && m_SegmentLabel <=47))//食指
	// 		{
	// 			iOffsetDegree = 90-iUnitDegree*1;
	// 		}
	// 		else if((m_SegmentLabel >= 33 && m_SegmentLabel <=35) || (m_SegmentLabel >= 51 && m_SegmentLabel <=53))//无名指
	// 		{
	// 			iOffsetDegree = 90+iUnitDegree*1;
	// 		}
	// 		else if((m_SegmentLabel >= 36 && m_SegmentLabel <=38) || (m_SegmentLabel >= 54 && m_SegmentLabel <=56))//小指
	// 		{
	// 			iOffsetDegree = 90+iUnitDegree*2;
	// 		}
	// 		if(m_iBodyPart == 1)
	// 			baseyaw = yaw0 - iOffsetDegree;//postyaw为正前方 
	// 		else if(m_iBodyPart ==2)
	// 			baseyaw = yaw0 + iOffsetDegree; 


			baseyaw = postyaw[m_iBodyPart];
			basepitch = postpitch[m_iBodyPart];
		}
		else
		{
			if ((m_SegmentLabel >=12)&&(m_SegmentLabel <= 14))//左手 手的航向朝向指尖
				baseyaw = yaw0 - 90;
			else if ((m_SegmentLabel >=8)&&(m_SegmentLabel <= 10))
				baseyaw = yaw0 + 90 ;
			else if (m_SegmentLabel == 18 || m_SegmentLabel == 22)
			{
				baseyaw = yaw0;
			}
			else if (m_SegmentLabel == 6)
			{
				if (m_headreftohand)
					baseyaw = postyaw[m_iBodyPart];
				else
				{
					baseyaw = yaw0 + m_headrefangle;
				}

			}
			else
			{
				baseyaw = postyaw[m_iBodyPart];
			}

			//wxg20161007 test
			if(m_SegmentLabel == 14 && postyaw[1] != 361)
				baseyaw = postyaw[1];
			else if(m_SegmentLabel == 10 && postyaw[2] != 361)
				baseyaw = postyaw[2];
		}

	// 	if(m_iBodyPart>0)
	// 	{
	// 		baseyaw = postyaw;
	// 	}
	// 	else
	// 	{
	// 		if ((m_SegmentLabel >=12)&&(m_SegmentLabel <= 14))//左手 手的航向朝向指尖	// chip 1/3/2
	// 			baseyaw = yaw0 - 90;
	// 		else if ((m_SegmentLabel >=8)&&(m_SegmentLabel <= 10))	// chip 9/7/8
	// 			baseyaw = yaw0 + 90 ;
	// 		else if (m_SegmentLabel == 18 || m_SegmentLabel == 22)	// chip 13/10
	// 		{
	// 			baseyaw = yaw0;
	// 		}else if(m_SegmentLabel == 16)	// chip 11
	// 		{
	// 			baseyaw = yaw0 - 45;
	// //			basepitch = pitch0 + 5;	// add by mvp ## 2015-9-7
	// 		}else if(m_SegmentLabel == 15)	// chip 12
	// 		{
	// 			baseyaw = yaw0 + 90;
	// //			basepitch = pitch0 + 5;
	// 		}
	// 		else if(m_SegmentLabel == 20)	// chip 14
	// 		{
	// 			baseyaw = yaw0 + 45;
	// //			basepitch = pitch0 + 5;
	// 		}
	// 		else if(m_SegmentLabel == 19)	// chip 15
	// 		{
	// 			baseyaw = yaw0 - 90;
	// //			basepitch = pitch0 + 5;
	// 		}
	// 		else if(m_SegmentLabel == 4)	// chip 17
	// 		{
	// 			baseyaw = yaw0 + 180;
	// 			if(baseyaw > 180)
	// 				baseyaw -= 360;
	// 		}
	// 		else if(m_SegmentLabel == 11)	// chip 5
	// 		{
	// 			baseyaw = yaw0 - 90;
	// 		}
	// 		else if(m_SegmentLabel == 7)	// chip 6
	// 		{
	// 			baseyaw = yaw0 + 90;
	// 		}
	// 		//else if((m_SegmentLabel >=15)&&(m_SegmentLabel <= 22))	// change the leg to the yaw0 //edit by mvp ## 2015-9-1
	// 		//{
	// 		//	baseyaw = yaw0;
	// 		//}
	// 		else if (m_SegmentLabel == 6)	// chip 4
	// 		{
	// 			if (m_headreftohand)
	// 				baseyaw = postyaw;
	// 			else
	// 			{
	// 				baseyaw = yaw0 + m_headrefangle;
	// 			}
	// 
	// 		}
	// 		else if(m_SegmentLabel == 0)	// chip 16
	// 		{
	// 			baseyaw = yaw0;
	// 		}		
	// 		else
	// 			baseyaw = postyaw;	// edit by mvp ## 2015-9-1
	// 			//baseyaw = postyaw;
	// 	}		

			// QS:postyaw and yaw0 //add by mvp ## 2015-8-17
		//	baseyaw = postyaw;
		//}



	// 	{
	// 		//osg::Quat quat3 = CalcQuatBetweenTwoEulor(baseyaw, 0, 0, yaw0, 0, 0);
	// 		//rlt = quat3.inverse() * rlt;
	// 		//rlt = rlt * quat3;
	// 		osg::Quat quat3 = CalcQuatBetweenTwoEulor(yaw0, 0, 0, baseyaw, 0, 0);//先把轴向转到统一的方向,根据模块在人体的位置 计算相对航向偏转
	// 		rlt = quat3 * rlt;
	// 		//osg::Quat quat4 = calcEulorToQuat(yaw0-baseyaw, 0, 0);//rlt = rlt * quat4
	// 		rlt = rlt * quat3.inverse();
	// 	}//与下面的写法效果相同

	// add the basepitch0 and the baseroll0 // add by mvp ## 2015-9-8
	 //	if (baseyaw != -720)
		//{
		//	osg::Quat quat3 = CalcQuatBetweenTwoEulor(baseyaw, 0, 0, yaw0, 0, 0);
		//	rlt = quat3.inverse() * rlt;
		//	osg::Quat quat4 = calcEulorToQuat(baseyaw - yaw0, 0, 0);
		//	rlt = rlt * quat4.inverse();
		//}

		if (baseyaw != -720)
		{	
			osg::Quat quat3 = CalcQuatBetweenTwoEulor(baseyaw, 0, 0, yaw0, 0, 0);
			rlt = quat3.inverse() * rlt;//将旋转变化量整体旋转一个角度
			rlt = rlt * quat3;

// 			quat3 = CalcQuatBetweenTwoEulor(0, pitch0, 0, 0, basepitch, 0);
// 			rlt = quat3 * rlt * quat3.inverse() ;//将旋转变化量整体旋转一个角度
		}
		else
		{
			PRINTF("baseyaw==720!!!!!!!!!!m_segmentlable=%d",m_SegmentLabel);
		}
	}

	return rlt;

////////////////////////test1
// 	osg::Quat quat1, quat2, invquat1, invquat2;
// 	quat1 = calcEulorToQuat(yaw, pitch, roll);
// 	if (m_abscali)
// 		quat2 = calcEulorToQuat(yaw0, 0.0, 0.0);
// 	else
// 		quat2 = calcEulorToQuat(yaw0, pitch0, roll0);
// 
// 	invquat1 = quat1.inverse();
// 	invquat2 = quat2.inverse();
// 
// 	osg::Quat rlt;
// 
// 	rlt = quat1 * invquat2;//校准位置与当前位置的差值
// 
// 	if (m_abscali)
// 		//rlt = quat0.inverse() * quat1;
// 		rlt = rlt*quat0.inverse();
//  	else
//  		rlt = quat1;
// 
// 
// 	
// 
// 	double baseyaw;
// 	if(m_iBodyPart>0)
// 	{	
// 		// 		int iUnitDegree=15;
// 		// 		int iOffsetDegree=90;
// 		// 		if((m_SegmentLabel >= 24 && m_SegmentLabel <=26) || (m_SegmentLabel >= 42 && m_SegmentLabel <=44))//大拇指
// 		// 			iOffsetDegree = 45;
// 		// 		else if((m_SegmentLabel >= 27 && m_SegmentLabel <=29) || (m_SegmentLabel >= 45 && m_SegmentLabel <=47))//食指
// 		// 		{
// 		// 			iOffsetDegree = 90-iUnitDegree*1;
// 		// 		}
// 		// 		else if((m_SegmentLabel >= 33 && m_SegmentLabel <=35) || (m_SegmentLabel >= 51 && m_SegmentLabel <=53))//无名指
// 		// 		{
// 		// 			iOffsetDegree = 90+iUnitDegree*1;
// 		// 		}
// 		// 		else if((m_SegmentLabel >= 36 && m_SegmentLabel <=38) || (m_SegmentLabel >= 54 && m_SegmentLabel <=56))//小指
// 		// 		{
// 		// 			iOffsetDegree = 90+iUnitDegree*2;
// 		// 		}
// 		// 		if(m_iBodyPart == 1)
// 		// 			baseyaw = yaw0 - iOffsetDegree;//postyaw为正前方 
// 		// 		else if(m_iBodyPart ==2)
// 		// 			baseyaw = yaw0 + iOffsetDegree;
// 		baseyaw = postyaw;
// 	}
// 	else
// 	{
// 		if ((m_SegmentLabel >=12)&&(m_SegmentLabel <= 14))//左手 手的航向朝向指尖
// 			baseyaw = yaw0 - 90;
// 		else if ((m_SegmentLabel >=8)&&(m_SegmentLabel <= 10))
// 			baseyaw = yaw0 + 90 ;
// 		else if (m_SegmentLabel == 18 || m_SegmentLabel == 22)
// 		{
// 			baseyaw = yaw0;
// 		}
// 		else if (m_SegmentLabel == 6)
// 		{
// 			if (m_headreftohand)
// 				baseyaw = postyaw;
// 			else
// 			{
// 				baseyaw = yaw0 + m_headrefangle;
// 			}
// 
// 		}
// 		else
// 			baseyaw = postyaw;
// 	}
// // 	if (baseyaw != -720)//wxg这个判断是???
// // 	{
// // 		;
// // 	}
// // 	else
// // 	{
// // 		//PRINTF("baseyaw==720!!!!!!!!!!m_segmentlable=%d",m_SegmentLabel);
// // 	}
// // 	{
// // 		//osg::Quat quat3 = CalcQuatBetweenTwoEulor(baseyaw, 0, 0, yaw0, 0, 0);
// // 		//rlt = quat3.inverse() * rlt;
// // 		//rlt = rlt * quat3;
// // 		osg::Quat quat3 = CalcQuatBetweenTwoEulor(yaw0, 0, 0, baseyaw, 0, 0);
// // 		rlt = quat3 * rlt;
// // 		//osg::Quat quat4 = calcEulorToQuat(yaw0-baseyaw, 0, 0);//rlt = rlt * quat4
// // 		rlt = rlt * quat3.inverse();
// // 	}//与下面的写法效果相同
// 	if(baseyaw != -720)
// 	{
//  		osg::Quat quat3 = CalcQuatBetweenTwoEulor(yaw0, 0, 0, baseyaw, 0, 0);
//  		rlt =quat3*rlt;
//  		rlt = rlt*quat3.inverse();
// 	}
// 	else
// 	{
// 		PRINTF("baseyaw==720!!!!!!!!!!m_segmentlable=%d",m_SegmentLabel);
// 	}
//  	return rlt;
// 	//return m_quater.EulorToQuat_cali(yaw, pitch, roll, yaw0 + 90); 
}

osg::Quat MPU6050Calculator::CalcuQuat(osg::Quat quat)
{
	osg::Quat quat1, quat2, invquat1, invquat2;
	// debug by mvp ## 2015-8-7
	// osg::Quat quat_tmp;
	if (m_eulorToQuatType == 1)
	{
		quat1.set(-quat.x(), -quat.z(), -quat.y(), quat.w());
		//quat1.set(quat.x(), quat.z(), quat.y(), quat.w()); // add by mvp ## 2015-8-7
		//quat_tmp = calcEulorToQuat(0.0,0.0,45.0);	// pitch 旋转90
		//quat1 = quat_tmp * quat1;
	}
	else if (m_eulorToQuatType == 4)
	    quat1.set(-quat.y(), -quat.z(), quat.x(), quat.w());//quat1 = calcEulorToQuat(yaw, pitch, roll);
	else
	{
		//todo
        quat1.set(-quat.y(), -quat.z(), quat.x(), quat.w());
	}
	if (m_abscali)
		quat2 = calcEulorToQuat(yaw0, 0.0, 0.0);

	else
		quat2 = calcEulorToQuat(yaw0, pitch0, roll0);

	invquat1 = quat1.inverse();
	invquat2 = quat2.inverse();
	osg::Quat rlt;
	if (m_abscali)
		rlt = quat0.inverse() * quat1;
	else
		rlt = quat1;


	rlt = rlt * invquat2;

	double baseyaw;
	if ((m_SegmentLabel >=12)&&(m_SegmentLabel <= 14))
		baseyaw = yaw0 - 90;
	else if ((m_SegmentLabel >=8)&&(m_SegmentLabel <= 10))
		baseyaw = yaw0 + 90;
	else if (m_SegmentLabel == 18 || m_SegmentLabel == 22)
		baseyaw = yaw0;
	else if (m_SegmentLabel == 6)
	{
		if (m_headreftohand)
			baseyaw = postyaw[m_iBodyPart];
		else
		{
			baseyaw = yaw0 + m_headrefangle;
		}

	}
	else
		baseyaw = postyaw[m_iBodyPart];
	if (baseyaw != -720)
	{
		osg::Quat quat3 = CalcQuatBetweenTwoEulor(baseyaw, 0, 0, yaw0, 0, 0);
		rlt = quat3.inverse() * rlt;
		osg::Quat quat4 = calcEulorToQuat(baseyaw - yaw0, 0, 0);
		rlt = rlt * quat4.inverse();
	}
	return rlt;
}

void MPU6050Calculator::SendSmoothQuat(osg::Quat squat)
{
	if (m_smooth)
	{
		if (m_framecount == 0)
			m_quat[m_framecount++] = squat;
		else if (m_framecount == 1)
		{
			m_squat[m_scount++] = m_quat[0];
			m_quat[m_framecount++] = squat;
			//m_sender->sendQuat(m_ID, m_quat[0]);
		}
		else 
		{
			m_quat[m_framecount] = squat;
			m_quat[1] = quatutils::slerp(m_quat[0], m_quat[2]);
			for (int i=0; i<2; i++)
				m_quat[i] = m_quat[i+1];
			m_squat[m_scount++] = m_quat[0];
			//m_sender->sendQuat(m_ID, m_quat[0]);

		}
		if (m_scount == 2)
		{
			m_prequat = m_squat[0];
			sendQuat(m_squat[0]);
			//m_sender->sendQuat(m_ID, m_squat[0]);
		}
		else if (m_scount > 2)
		{
			m_squat[1] = quatutils::slerp(m_squat[0], m_squat[2]);
			for (int i=0; i<2; i++)
				m_squat[i] = m_squat[i+1];
			sendQuat(m_squat[0]);
			m_scount--;
		}
	}
	else 
		sendQuat(squat);
	m_cali = false;
}


void MPU6050Calculator::Outputdata(osg::Quat squat)
{
#ifndef TEST_TRAIL
	m_sender->sendQuat(m_ID, m_SegmentLabel, squat, m_cali, m_moveflag);//准备驱动模型数据wxg6
#endif
	//osg::Vec3d vdis;
	//if ((viewer != NULL)&&(!viewer->isHidden()))
	//	viewer->newQuat(m_ID, squat.x(), squat.y(), squat.z(), squat.w(), vdis);
	//test
	emit outQuat(m_ID, squat.x(), squat.y(), squat.z(), squat.w());//发送通知 驱动磁力计指示长方条wxg7
}
void MPU6050Calculator::sendQuat(osg::Quat squat)
{
// 	osg::Quat quat = squat * m_prequat.inverse();
// 	osg::Quat::value_type dangle;
// 	osg::Vec3f axis;
// 	quat.getRotate(dangle, axis);
// 	if (dangle > pi)
// 		dangle = 2 * pi - dangle;
// 	dangle = dangle * 180 / pi;
// 	quat = m_prequat;
// 	double granularity = MPU6050TEST::m_config->get(CMAIN, CGRANULARITY, 0).toDouble();  
// 	double tempangle = dangle;
// 	int ratio = int(dangle / granularity);
// 	while (tempangle > granularity)
// 	{
// 		quat = slerp(quat, squat, granularity / tempangle);
//         m_sender->sendQuat(m_ID, m_SegmentLabel, quat);
// 		tempangle-=granularity;
// 	}
    Outputdata(squat);
    m_prequat = m_squat[0];
}

void MPU6050Calculator::SendEuler(double yaw, double pitch, double roll)
{
	osg::Quat newq = CalcuQuat(yaw, pitch, roll);

//test wxg20160604	
// 	static int iCount=0;
// 	static int iBaseChip[2]={0};
// 	double yawOffset = atan2(2 * newq.x() * newq.y() + 2 * newq.w() * newq.z(), -2 * newq.y()*newq.y() - 2 * newq.z() * newq.z() + 1)* 180/pi; // yaw
// 	double pitchOffset = -asin(-2 * newq.x() * newq.z() + 2 * newq.w() * newq.y())* 180/pi; // pitch
// 	double rollOffset = atan2(2 * newq.y() * newq.z() + 2 * newq.w() * newq.x(), -2 * newq.x() * newq.x() - 2 * newq.y() * newq.y() + 1)* 180/pi; // roll
// 	double rollbak = rollOffset;
// 	if(m_iBodyPart > 0 && fabs(postyaw[m_iBodyPart]-361) > 0.001)
// 	{
// 		if(iBaseChip[0] == 0  && m_SegmentLabel < BODY_NODE_COUNT+1+FINGER_NODE_COUNT)
// 		{
// 			iBaseChip[0] = m_SegmentLabel;
// 			//m_rootRoll[0] = roll;
// 			//osg::Quat newq1= calcEulorToQuat(yawOffset, pitchOffset, rollOffset);
// 			//PRINTF("%f %f %f %f vs %f %f %f %f ",newq.x(),newq.y(),newq.z(),newq.w(),newq1.x(),newq1.y(),newq1.z(),newq1.w());
// 		}
// 		else if(iBaseChip[1] == 0 && m_SegmentLabel >= BODY_NODE_COUNT+1+FINGER_NODE_COUNT)
// 		{
// 			iBaseChip[1] = m_SegmentLabel;
// 			//m_rootRoll[1] = roll;
// 		}
// 
// 		if(m_SegmentLabel < BODY_NODE_COUNT+1+FINGER_NODE_COUNT)
// 		{
// 			if(iBaseChip[0] == m_SegmentLabel)
// 			{
// 				m_rootRoll[0] = rollOffset;
// 				iCount++;
// 			}
// 			else
// 				rollOffset = m_rootRoll[0];
// 		}
// 		else
// 		{
// 			if(iBaseChip[1] == m_SegmentLabel)
// 				m_rootRoll[1] = rollOffset;
// 			else
// 				rollOffset = m_rootRoll[1];
// 		}
// 		newq = quatutils::EulorToQuat(yawOffset, pitchOffset, rollOffset+(rollbak-rollOffset)/2,0);
// 		if(iCount%200 == 0)
// 			PRINTF("yaw0 = %f vs yawnow =%f ; pitch0 = %f vs pitchnow %f ; roll0 = %f vs rollnow = %f ; offset:yaw = %f, pitch = %f,  roll = %f (%f)    chip=%d ",yaw0,yaw,pitch0,pitch,roll0,roll,yawOffset,pitchOffset,rollOffset,rollbak,m_ID);
// 	}
// 	else if(m_iBodyPart ==0)
// 	{
// 		if(m_SegmentLabel == 14 && iBaseChip[0] != 0)
// 		{
// 			rollOffset  = m_rootRoll[0];
// 			newq = quatutils::EulorToQuat(yawOffset, pitchOffset, rollOffset+(rollbak-rollOffset)/2,0);
// 		}
// 		else if(m_SegmentLabel == 10 && iBaseChip[1] != 0)
// 		{
// 			rollOffset = m_rootRoll[1];
// 			newq = quatutils::EulorToQuat(yawOffset, pitchOffset, rollOffset+(rollbak-rollOffset)/2,0);
// 		}
// 		
// 	}
	//////////////////////////////////////////////////

	

	SendSmoothQuat(newq);

}

void MPU6050Calculator::setID(int aID)
{
     m_ID = aID;
}

void MPU6050Calculator::setChipID(int aChipID)
{
	m_ChipID = aChipID;
}

void MPU6050Calculator::setSegmentLabel(int segmentlabel)
{
	m_SegmentLabel = segmentlabel;
// 	//add by wxg
	if(segmentlabel == 9 || segmentlabel == BODY_NODE_COUNT+7 || segmentlabel == BODY_NODE_COUNT+FINGER_NODE_COUNT+7)//右手腕 中指
		m_bYawBaseNode = true;

	if(segmentlabel < 24)
		m_iBodyPart = 0;
	else if(segmentlabel <42)//
		m_iBodyPart =1;
	else if(segmentlabel < 60)
		m_iBodyPart = 2;
}


void MPU6050Calculator::setAbscali(bool avalue)
{
	m_abscali = avalue;
}

void MPU6050Calculator::setEulorToQuatType(char avalue)
{
    m_eulorToQuatType = avalue;
}

void MPU6050Calculator::setEulorSigned(bool ayaw, bool apitch, bool aroll)
{
    m_yawnegative = ayaw;
	m_pitchnegative = apitch;
	m_rollnegative = aroll;
}

void MPU6050Calculator::setAngleInc(int avalue)
{
    m_angleinc = avalue;
}

void MPU6050Calculator::setHeadRefToHand(bool avalue)
{
    m_headreftohand = avalue;
}

void MPU6050Calculator::setHeadRefAngle(int avalue)
{
    m_headrefangle = avalue;
}

void MPU6050Calculator::setSmooth(bool avalue)
{
	m_smooth = avalue;

}

void MPU6050Calculator::bendData(float fir, float sec, float thd)
{
	//m_totalframe++;
	float firangle, secangle, thdangle;
	firangle = (fir-900.0) * 75.0 / 50.0;
	secangle = (450-sec) * 85.0 / 400.0;
	thdangle = (660 - thd) * 50.0 / 536.0;
	float data[3];
	data[0] = -firangle;
	data[1] = -secangle;
	data[2] = -thdangle;
	m_sender->sendData(17789, (char*)data, 12);
// 	if (port && port->isCanShow())
// 	{
// 		port->beginDraw();
// 		port->drawText(50, 80, QString(tr("手套角度：   fir %1 sec %2 thd %3")).arg(firangle, -8, 'f', 1).arg(secangle, -6, 'f', 1).arg(thdangle, -7, 'f', 1)); 
// 		port->drawText(50, 100, QString(tr("FPS：      %1")).arg(m_FPS, -15));
// 		port->endDraw();
// 	}
	return;
}

void MPU6050Calculator::newYawPitchRoll(int chipID, double yaw, double pitch, double roll, short moveflag, bool weightnessless)//收到欧拉角wxg2
{
//	emit updatecom(chipID,0,0,0,0,0,0);//wxg test
	//m_totalframe++;
	m_moveflag = moveflag;
	if ((yaw == preyaw) && (pitch == prepitch) && (roll == preroll))
	{
        m_moveless = true;
	}
	else m_moveless = false;
	if (chipID == 16)
	    m_sender->setWeightNess(weightnessless);
#ifndef TEST_TRAIL
	//test by wxg////////////////////////////////
// 	if(m_iCount<AVERAGECOUNT+1)
// 	{
// 		yawAv[m_iCount] = yaw;
// 		pitchAv[m_iCount] = pitch;
// 		rollAv[m_iCount]=roll;
// 		m_iCount++;
// 	}
// 	else
// 	{
// 		m_iCount = AVERAGECOUNT+1;
// 		{
// 			for(int i=0;i<AVERAGECOUNT;i++)
// 			{
// 				yawAv[i] = yawAv[i+1];
// 				pitchAv[i] = pitchAv[i+1];
// 				rollAv[i] = rollAv[i+1];
// 			}
// 			yawAv[AVERAGECOUNT] = yaw;
// 			pitchAv[AVERAGECOUNT] = pitch;
// 			rollAv[AVERAGECOUNT]=roll;		
// 		}
// 		yawAv[m_iCount] = 0;
// 		pitchAv[m_iCount] = 0;
// 		rollAv[m_iCount] = 0;
// 		for(int i=1;i<AVERAGECOUNT+1;i++)
// 		{
// 			if(yawAv[i]<-90)
// 				yawAv[m_iCount]+=360;
// 			yawAv[m_iCount]+=yawAv[i];
// 
// 			pitchAv[m_iCount]+=pitchAv[i];
// 
// 			if(rollAv[i]<-90)
// 				rollAv[m_iCount]+=360;
// 			rollAv[m_iCount]+=rollAv[i];
// 		}
// 		m_iCount++;
// 		yawAv[m_iCount] = yawAv[AVERAGECOUNT+1]/AVERAGECOUNT;
// 		if(yawAv[m_iCount] > 180)
// 			yawAv[m_iCount] -= 360;
// 
// 		pitchAv[m_iCount] = pitchAv[AVERAGECOUNT+1]/AVERAGECOUNT;
// 
// 		rollAv[m_iCount] = rollAv[AVERAGECOUNT+1]/AVERAGECOUNT;
// 		if(rollAv[m_iCount] > 180)
// 			rollAv[m_iCount] -= 360;
// 		yaw = yawAv[m_iCount];
// 		pitch = pitchAv[m_iCount];
// 		roll = rollAv[m_iCount];
// 	}

	/////////////////////////////////////////
	SendEuler(yaw, pitch, roll);

#endif
	preyaw = yaw;
	prepitch = pitch;
	preroll = roll;
// 	if (port && port->isCanShow())
// 	{
// 		port->beginDraw();
// 		port->drawText(50, 80, QString(tr("解算角度：   yaw %1 pitch %2 roll %3")).arg(yaw, -8, 'f', 1).arg(pitch, -6, 'f', 1).arg(roll, -7, 'f', 1)); 
// 		port->drawText(50, 120, QString(tr("有效帧率：      %1%")).arg(GValidateFrameCount * 100.0 / GFrameCount));
// 		port->drawText(50, 100, QString(tr("FPS：      %1")).arg(m_FPS, -15));
// 		port->drawAngle(yaw, pitch, roll);
// 		port->endDraw();
// 	}

	return;
}

void MPU6050Calculator::newQuat(int chipID, float x, float y, float z, float w, short moveflag, bool weightnessless)//当前没有用到四元数模式
{
	//PRINTF("四元数模式！！！MPU6050Calculator::newQuat(int chipID, float x, float y, float z, float w, short moveflag, bool weightnessless)...");
	//m_totalframe++;
	//if (MPU6050TEST::m_config->get(CMAIN, CUSEQUAT, "0").toInt() == 0)
	//	return;
	m_moveflag = moveflag;
	m_usequat = true;
	if (chipID == 16)
		m_sender->setWeightNess(weightnessless);
#ifndef TEST_TRAIL
	osg::Quat quat;
	quat.set(x, y, z, w);
	osg::Quat newq = CalcuQuat(quat);
	SendSmoothQuat(newq);
#endif
	m_pquat.set(x, y, z, w);
	//PRINTF("MPU6050Calculator::newQuat.");
}

void MPU6050Calculator::newPosition(int chipID, double posx, double posy, double posz)
{
   //m_totalframe++;
// 	double dTemp = posy;
// 	posy = posz;
// 	posz = dTemp;
// 	posx = -posx;
	emit outPos(chipID,posx,posy,posz);

	////////////////////////////////wxg[2016-8-23]add 用左右脚的中点来算位移
	static double xl=0,yl=0,zl=0,xr=0,yr=0,zr=0;
	static double xl0=0,yl0=0,zl0=0,xr0=0,yr0=0,zr0=0;
	static BYTE btMask=0;
	if(chipID == -1)//校准 初始化
	{
		xl0=yl0=zl0=0;
		xr0=yr0=zr0=0;
		btMask =0;
		return;
	}
	if(chipID == 10)//left
	{
		 if(xl0 == 0 && yl0==0 && zl0==0)
		 {
			 xl = xl0 = posx;
			 yl = yl0 = posy;
			 zl = zl0 = posz;
		 }
		 xl = posx;
		 yl = posy;
		 zl = posz;
		 btMask |= 0x01;
		 if(btMask == 0x03)
		 {
			posx = (xl+xr)/2-(xl0+xr0)/2;
			posy = (yl+yr)/2-(yl0+yr0)/2;
			posz = (zl+zr)/2-(zl0+zr0)/2;

			xl0 = posx;
			yl0 = posy;
			zl0 = posz;
		 }
		 
	}
	else if(chipID == 13)//right
	{
		if(xr0 == 0 && yr0==0 && zr0==0)
		{
			xr = xr0 = posx;
			yr = yr0 = posy;
			zr = zr0 = posz;
		}
		xr = posx;
		yr = posy;
		zr = posz;
		btMask |= 0x02;
		if(btMask == 0x03)
		{
			posx = (xl+xr)/2-(xl0+xr0)/2;
			posy = (yl+yr)/2-(yl0+yr0)/2;
			posz = (zl+zr)/2-(zl0+zr0)/2;

			xr0 = posx;
			yr0 = posy;
			zr0 = posz;
		}
	}//////////////////////////////////////////////////////////////////////////////
	if(btMask == 0x03)
	{
	   m_sender->hipsPosChange(posx, posy, posz);
		//emit outPos(0,posx,posy,posz);
	}
//    emit outPos(chipID,posx,posy,posz);

//    if (port && port->isCanShow())
//    {
// 	   port->beginDraw(); 
// 	   port->drawText(450, 80, QString(tr("位移：      x %1 y %2 z %3")).arg(posx, -15).arg(posy, -15).arg(posz, -15));
// 	   port->drawAcc(posx, posy, posz);
// 	   port->endDraw();
//    }
   //if (viewer)
   //{

	   //if ((MPU6050TEST::m_config->get(CMAIN, CUSEQUAT, "0").toInt() == 0) && m_ID != -1)
	   //	viewer->newAngle(m_ID, yaw, pitch, roll, dis);
	   //viewer->newPosition(chipID, posx, posy, posz);

   //}
   return;
}

void MPU6050Calculator::newAgcGrav(int chipID, double ax, double ay, double az, double gx, double gy, double gz, int cx, int cy, int cz, float gravx, float gravy, float gravz)
{
	emit outCom(m_ID,ax,ay,az,gx,gy,gz, cx, cy, cz);
// 	double yaw, pitch, roll;
// 	m_totalframe++;
// 
// 	
// 	if ((abs(gx) >= 5.0) || (abs(gy) >= 5.0) || (abs(gz) >= 5.0) || m_firstflag)
// 	{
// 		yaw = mover.yaw;
// 		pitch = mover.pitch;
// 		roll = mover.roll;
// 		preyaw = yaw;
// 		prepitch = pitch;
// 		preroll = roll;
// 		m_firstflag = false;
// 	}
// 	else
// 	{
// 		yaw = preyaw;
// 		pitch = prepitch;
// 		roll = preroll;
// 	}
// 	emit outAngle(m_ID, yaw, pitch, roll);
	
	//osg::Vec3d dis;
	//if (viewer)
	//{

		//if ((MPU6050TEST::m_config->get(CMAIN, CUSEQUAT, "0").toInt() == 0) && m_ID != -1)
		//viewer->newAngle(m_ID, yaw, pitch, roll, dis);
		//viewer->newCom(cx, cy, cz);
		//viewer->newComxy(mover.ccx, mover.ccy, mover.ccz, mover.cxarroll, mover.cyarroll, mover.czarroll);
		
	//}
	//if (MPU6050TEST::m_config->get(CMAIN, CUSEQUAT, "0").toInt() == 0)
	//    SendEuler(yaw, pitch, roll);
// 	if (port && port->isCanShow())
// 	{
// 		port->beginDraw();
// 		port->drawText(50, 20, QString(tr("加速度：     x %1 y %2 z %3")).arg(ax, -10).arg(ay, -10).arg(az, -10));
// 		port->drawText(50, 40, QString(tr("陀螺仪：     x %1 y %2 z %3")).arg(gx, -10).arg(gy, -10).arg(gz, -10));
// 		port->drawText(50, 60, QString(tr("磁力计：     x %1 y %2 z %3")).arg(cx, -10).arg(cy, -10).arg(cz, -10));
// 		port->drawText(50, 80, QString(tr("解算角度：   yaw %1 pitch %2 roll %3")).arg(yaw, -8, 'f', 1).arg(pitch, -6, 'f', 1).arg(roll, -7, 'f', 1));
// 		port->drawText(50, 100, QString(tr("FPS：      %1")).arg(m_FPS, -15));
// 		//port->drawText(50, 100, QString(tr("相对位移：   x %1 y %2 z %3")).arg(dis.x(), -15, 'f', 1).arg(dis.y(), -15, 'f', 1).arg(dis.z(), -15, 'f', 1)); 
// 		osg::Vec3d pos = mover.getPosition();
// 		osg::Vec3d speed = mover.getSpeed();
// 		osg::Vec3d acc = mover.getAcc();
// 		osg::Vec3d gravacc = mover.getGravAcc();
// 
// 		port->drawText(450, 20, QString(tr("重力加速度：x %1 y %2 z %3")).arg(gravacc.x(), -15).arg(gravacc.y(), -15).arg(gravacc.z(), -15));
// 		port->drawText(450, 40, QString(tr("运动加速度：x %1 y %2 z %3")).arg(acc.x(), -15).arg(acc.y(), -15).arg(acc.z(), -15));
// 		port->drawText(450, 60, QString(tr("速度：      x %1 y %2 z %3")).arg(speed.x(), -15).arg(speed.y(), -15).arg(speed.z(), -15));
// 		port->drawText(450, 80, QString(tr("位移：      x %1 y %2 z %3")).arg(pos.x(), -15).arg(pos.y(), -15).arg(pos.z(), -15));
//         
// 		//port->drawAngle(yaw, pitch, roll);
//         port->drawOriginalAcc(ax, ay, az);     
// 		//port->drawOriginalCom(cx, cy, cz); 
// 		//port->drawOriginalGyr(gx, gy, gz);
// 		//port->drawGravityAcc(gravacc.x(), gravacc.y(), gravacc.z());
// 		//port->drawAcc(acc.x(), acc.y(), acc.z());
// 
// 		//port->drawAcc(pos.x(), pos.y(), pos.z());
// 		port->drawSpeed(speed.x(), speed.y(), speed.z());
// 		if (MPU6050TEST::m_config->get(CMAIN, CDRAWCOMCIRCLE, "0").toInt() == 1)
// 			port->drawCom(mover.ccx, mover.ccy);
// 		port->endDraw();
// 	}

}

void MPU6050Calculator::newAgcData(int chipID, double ax, double ay, double az, double gx, double gy, double gz, int cx, int cy, int cz)//收到校准数据wxg5
{
	emit outCom(m_ID,ax,ay,az,gx,gy,gz, cx, cy, cz);

	static float xmax=-255,ymax=-255,zmax=-255,xmin=255,ymin=255,zmin=255;
	//wxg test
	if(cx > xmax)
		xmax = cx;
	if(cx<xmin)
		xmin = cx;
	if(cy > ymax)
		ymax = cy;
	if(cy<ymin)
		ymin = cy;
	if(cz > zmax)
		zmax = cz;
	if(cz<zmin)
		zmin = cz;
   emit updatecom(chipID,xmax,ymax,zmax,xmin,ymin,zmin);

	//m_totalframe++;	
	//double yaw, pitch, roll;

	// 	if (MPU6050TEST::m_config->get(CMAIN, CACCMINUSGYR, 0).toInt() == 1)
	// 	{
	//   		double iax = gy*0.009;
	//   		double iay = gx*0.009 - gz*0.004;//gz*0.0045;//(gx-gz)*0.009;
	//   		double iaz = (gx + gy)*0.0006;
	//   		ax -= (iax - m_preincax) -abs(gz*0.0004);
	//   		ay += (iay - m_preincay);
	//   		az += abs(iaz);
	//   		m_preincax = iax;
	//   		m_preincay = iay;
	//   		//m_preincaz = iaz;
	// 	}
// 	mover.setAcc(ax, ay, az);
// 	mover.setCom(cx, cy, cz);
// 	mover.setGyr(gx, gy, gz);
// 	mover.calculate();
// 	if ((abs(gx) >= 5.0) || (abs(gy) >= 5.0) || (abs(gz) >= 5.0) || m_firstflag)
// 	{
// 		yaw = mover.yaw;
// 		pitch = mover.pitch;
// 		roll = mover.roll;
// 		preyaw = yaw;
// 		prepitch = pitch;
// 		preroll = roll;
// 		m_firstflag = false;
// 	}
// 	else
// 	{
// 		yaw = preyaw;
// 		pitch = prepitch;
// 		roll = preroll;
// 	}
// 	emit outAngle(m_ID, yaw, pitch, roll);
	
	//osg::Vec3d dis;
	//if (viewer)
	//{

		//if ((MPU6050TEST::m_config->get(CMAIN, CUSEQUAT, "0").toInt() == 0) && m_ID != -1)//yuan
		//viewer->newAngle(m_ID, yaw, pitch, roll, dis);
		//viewer->newCom(cx, cy, cz);
		//viewer->newComxy(mover.ccx, mover.ccy, mover.ccz, mover.cxarroll, mover.cyarroll, mover.czarroll);

	//}
//	SendEuler(yaw, pitch, roll);
// 	if (port && port->isCanShow())
// 	{
// 		port->beginDraw();
// 		port->drawText(50, 20, QString(tr("加速度：     x %1 y %2 z %3")).arg(ax, -10).arg(ay, -10).arg(az, -10));
// 		port->drawText(50, 40, QString(tr("陀螺仪：     x %1 y %2 z %3")).arg(gx, -10).arg(gy, -10).arg(gz, -10));
// 		port->drawText(50, 60, QString(tr("磁力计：     x %1 y %2 z %3")).arg(cx, -10).arg(cy, -10).arg(cz, -10));
// 		port->drawText(50, 80, QString(tr("解算角度：   yaw %1 pitch %2 roll %3")).arg(yaw, -8, 'f', 1).arg(pitch, -6, 'f', 1).arg(roll, -7, 'f', 1)); 
// 		/*port->drawText(50, 100, QString(tr("相对位移：   x %1 y %2 z %3")).arg(dis.x(), -15, 'f', 1).arg(dis.y(), -15, 'f', 1).arg(dis.z(), -15, 'f', 1)); */
// 		port->drawText(50, 100, QString(tr("FPS：      %1")).arg(m_FPS, -15));
// 		osg::Vec3d pos = mover.getPosition();
// 		osg::Vec3d speed = mover.getSpeed();
// 		osg::Vec3d acc = mover.getAcc();
// 		osg::Vec3d gravacc = mover.getGravAcc();
// 
// 		port->drawText(450, 20, QString(tr("重力加速度：x %1 y %2 z %3")).arg(gravacc.x(), -15).arg(gravacc.y(), -15).arg(gravacc.z(), -15));
// 		port->drawText(450, 40, QString(tr("运动加速度：x %1 y %2 z %3")).arg(acc.x(), -15).arg(acc.y(), -15).arg(acc.z(), -15));
// 		port->drawText(450, 60, QString(tr("速度：      x %1 y %2 z %3")).arg(speed.x(), -15).arg(speed.y(), -15).arg(speed.z(), -15));
// 		port->drawText(450, 80, QString(tr("位移：      x %1 y %2 z %3")).arg(pos.x(), -15).arg(pos.y(), -15).arg(pos.z(), -15));
// 
// 		//port->drawAngle(yaw, pitch, roll);
// 		port->drawOriginalAcc(ax, ay, az);
// 		// 		port->drawOriginalCom(cx, cy, cz);
// 
// 		// 		port->drawOriginalGyr(gx, gy, gz);
// 		port->drawGravityAcc(gravacc.x(), gravacc.y(), gravacc.z());
// 		// 		//port->drawAcc(acc.x, acc.y, acc.z);
// 		//port->drawAcc(mover.Rgyro.x, mover.Rgyro.y, mover.Rgyro.z);
// 		//port->drawSpeed(speed.x, speed.y, speed.z);
// 		//if (MPU6050TEST::m_config->get(CMAIN, CDRAWCOMCIRCLE, "0").toInt() == 1)//yuan
// 		port->drawCom(mover.ccx, mover.ccy);
// 		port->endDraw();
// 	}	
}

void MPU6050Calculator::InitBaseYaw()
{
	//postyaw = 361;
}

void MPU6050Calculator::SetYawOffset( double offset )
{
// 	m_bYawOffset = offset;
// 	yaw0 = yaw0+m_bYawOffset;
// 
// 	quat0 = CalcQuatBetweenTwoEulor(yaw0, 0, 0, yaw0, pitch0, roll0);
}