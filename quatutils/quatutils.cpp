#include "quatutils.h"
#include "Calculator.h"
#define INIAngle -720 

quatutils::quatutils()
{
    m_baseyaw = INIAngle;
}

quatutils::~quatutils()
{

}

void quatutils::setIniQuat(osg::Quat quat)
{
    m_quat0 = quat;
}

void quatutils::setabscali(bool babscali)
{
    m_abscali = babscali;
}

void quatutils::setIniAngle(double yaw, double pitch, double roll)
{
     m_yaw0 = yaw;
	 m_pitch0 = pitch;
	 m_roll0 = roll;
}
osg::Quat quatutils::EulorToQuat_XZY(double yaw, double pitch, double roll)
{
	//osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (-roll), osg::X_AXIS ,
	//	osg::DegreesToRadians (pitch), osg::Z_AXIS ,
	//	osg::DegreesToRadians (-yaw), osg::Y_AXIS);
	osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (roll), osg::X_AXIS ,
		osg::DegreesToRadians (pitch), osg::Z_AXIS ,
		osg::DegreesToRadians (yaw), osg::Y_AXIS);
	return rlt;
}
osg::Quat quatutils::EulorToQuat_XYZ(double yaw, double pitch, double roll)
{
	osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (roll), osg::X_AXIS ,
		osg::DegreesToRadians (pitch), osg::Y_AXIS ,
		osg::DegreesToRadians (yaw), osg::Z_AXIS);
	return rlt;
}
osg::Quat quatutils::EulorToQuat_YXZ(double yaw, double pitch, double roll)
{
	osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (roll), osg::Y_AXIS ,
		osg::DegreesToRadians (pitch), osg::X_AXIS ,
		osg::DegreesToRadians (yaw), osg::Z_AXIS);
	return rlt;
}
osg::Quat quatutils::EulorToQuat_YZX(double yaw, double pitch, double roll)
{
	osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (roll), osg::Y_AXIS ,
		osg::DegreesToRadians (pitch), osg::Z_AXIS ,
		osg::DegreesToRadians (yaw), osg::X_AXIS);
	return rlt;
}
osg::Quat quatutils::EulorToQuat_ZXY(double yaw, double pitch, double roll)
{
	osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (roll), osg::Z_AXIS ,
		osg::DegreesToRadians (pitch), osg::X_AXIS ,
		osg::DegreesToRadians (yaw), osg::Y_AXIS);
	return rlt;
}
osg::Quat quatutils::EulorToQuat_ZYX(double yaw, double pitch, double roll)
{
	osg ::Quat rlt = osg ::Quat( osg::DegreesToRadians (roll), osg::Z_AXIS ,
		osg::DegreesToRadians (pitch), osg::Y_AXIS ,
		osg::DegreesToRadians (yaw), osg::X_AXIS);
	return rlt;
}
osg::Quat quatutils::EulorToQuat(double yaw, double pitch, double roll, char calctype)
{
	if (calctype == 0)
		return EulorToQuat_XYZ(yaw, pitch, roll);
	else if (calctype == 1)
		return EulorToQuat_XZY(yaw, pitch, roll);
	else if (calctype == 2)
		return EulorToQuat_YXZ(yaw, pitch, roll);
	else if (calctype == 3)
		return EulorToQuat_YZX(yaw, pitch, roll);
	else if (calctype == 4)
		return EulorToQuat_ZXY(yaw, pitch, roll);
	else 
		return EulorToQuat_ZYX(yaw, pitch, roll);

}

//求四元数fquat到四元数tquat之间的插值
osg::Quat quatutils::slerp(osg::Quat& fquat, osg::Quat& tquat, double ratio)
{
	osg::Quat ftquat;
	osg::Quat temp;
	temp = fquat.inverse();
	ftquat = tquat * temp;
	osg::Quat::value_type dangle;
	osg::Vec3f axis;
	ftquat.getRotate(dangle, axis);

	osg::Quat squat;
	if (dangle > pi)
		squat.makeRotate((dangle - 2* pi)*ratio, axis);
	else
		squat.makeRotate(dangle*ratio, axis);
	//PRINTF(QString("angle:%1").arg(dangle).toStdString());

	return squat * fquat;
}

osg::Quat quatutils::CalcQuatBetweenTwoEulor(double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2, char calctype)
{
	osg::Quat quat1, quat2;
	quat1 = EulorToQuat(yaw2, pitch2, roll2, calctype);
	quat2 = EulorToQuat(yaw1, pitch1, roll1, calctype);
	osg::Quat rlt = quat1 *quat2.inverse();
	return rlt;

}

void quatutils::setBaseYaw(double yaw)
{
   m_baseyaw = yaw;
}

osg::Quat quatutils::EulorToQuat_cali(double yaw, double pitch, double roll, double baseyaw, char calctype)
{
	m_baseyaw = baseyaw;
	osg::Quat quat1, quat2, invquat1, invquat2;
	quat1 = EulorToQuat(yaw, pitch, roll, calctype);
	if (m_abscali)
		quat2 = EulorToQuat(m_yaw0, 0.0, 0.0, calctype);
	else
		quat2 = EulorToQuat(m_yaw0, m_pitch0, m_roll0, calctype);

	invquat1 = quat1.inverse();
	invquat2 = quat2.inverse();
	osg::Quat rlt;
	if (m_abscali)
		rlt = m_quat0.inverse() * quat1;
	else
		rlt = quat1;


	rlt = rlt * invquat2;
	if (m_baseyaw != INIAngle)
	{
		osg::Quat quat3 = CalcQuatBetweenTwoEulor(m_baseyaw, 0, 0, m_yaw0, 0, 0, calctype);
		rlt = quat3.inverse() * rlt;
		osg::Quat quat4 = EulorToQuat(m_baseyaw - m_yaw0, 0, 0, calctype);
		rlt = rlt * quat4.inverse();
	}
	return rlt;
}

osg::Quat quatutils::EulorToQuat(bool chickscene, bool rollzpitchxyawy, bool bcscene, bool camerascene, int seglabel, double yaw, double pitch, double roll)
{
	osg::Quat rlt;
	if (chickscene)
	{
		if (rollzpitchxyawy)
		{
			rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::Z_AXIS, 
				osg::DegreesToRadians(-pitch), osg::X_AXIS, 
				osg::DegreesToRadians(-yaw), osg::Y_AXIS);
		}
		else
		{
			//rlt = osg::Quat(osg::DegreesToRadians(roll), osg::X_AXIS, 
			//	osg::DegreesToRadians(-pitch), osg::Z_AXIS, 
			//	osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			rlt = osg::Quat(osg::DegreesToRadians(roll), osg::Y_AXIS, 
				osg::DegreesToRadians(pitch), osg::X_AXIS, 
				osg::DegreesToRadians(yaw), osg::Z_AXIS);
		}

	}
	else
	{
		if (bcscene)
		{
			if (seglabel == 8 || seglabel == 9)
			{
				rlt = osg::Quat(osg::DegreesToRadians(roll), osg::Z_AXIS, 
					osg::DegreesToRadians(pitch), osg::X_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 12 || seglabel == 13)
			{
				rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::Z_AXIS, 
					osg::DegreesToRadians(-pitch), osg::X_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 15 || seglabel == 16)
			{
				rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::X_AXIS, 
					osg::DegreesToRadians(pitch), osg::Z_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 19 || seglabel == 20)
			{
				rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::X_AXIS, 
					osg::DegreesToRadians(pitch), osg::Z_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 10)
			{
				rlt = osg::Quat(osg::DegreesToRadians(roll), osg::Z_AXIS, 
					osg::DegreesToRadians(pitch), osg::X_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 14)
			{
				rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::Z_AXIS, 
					osg::DegreesToRadians(-pitch), osg::X_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 0) //Pelvis
			{
				rlt = osg::Quat(osg::DegreesToRadians(roll), osg::X_AXIS, 
					osg::DegreesToRadians(-pitch), osg::Z_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 4) // T8
			{
				rlt = osg::Quat(osg::DegreesToRadians(roll), osg::X_AXIS, 
					osg::DegreesToRadians(-pitch), osg::Z_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 22) //left Toe
			{
				rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::X_AXIS, 
					osg::DegreesToRadians(pitch), osg::Z_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 18) //right Toe
			{
				rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::X_AXIS, 
					osg::DegreesToRadians(pitch), osg::Z_AXIS,                                                                                                                                   
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 7) //right Shoulder
			{
				rlt = osg::Quat(osg::DegreesToRadians(roll), osg::Z_AXIS, 
					osg::DegreesToRadians(pitch), osg::X_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 11) //left Shoulder
			{
				rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::Z_AXIS, 
					osg::DegreesToRadians(-pitch), osg::X_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else if (seglabel == 6) //head 
			{
				rlt = osg::Quat(osg::DegreesToRadians(-roll), osg::X_AXIS, 
					osg::DegreesToRadians(pitch), osg::Z_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}
			else
			{
				rlt = osg::Quat(osg::DegreesToRadians(roll), osg::Z_AXIS, 
					osg::DegreesToRadians(pitch), osg::X_AXIS, 
					osg::DegreesToRadians(-yaw), osg::Y_AXIS);
			}

		}
		else if (camerascene)
		{
			rlt = osg::Quat(osg::DegreesToRadians(roll), osg::Y_AXIS, 
				osg::DegreesToRadians(-pitch), osg::X_AXIS, 
				osg::DegreesToRadians(-yaw), osg::Z_AXIS);
		}
		else
		{
			rlt = osg::Quat(osg::DegreesToRadians(roll), osg::Y_AXIS, 
				osg::DegreesToRadians(pitch), osg::X_AXIS, 
				osg::DegreesToRadians(yaw), osg::Z_AXIS);
		}	
	}
	return rlt;
}
