#ifndef QUATUTILS_H
#define QUATUTILS_H

#include "quatutils_global.h"
#include "osg/Quat"

class QUATUTILS_EXPORT quatutils
{
public:
	quatutils();
	~quatutils();

	void setabscali(bool babscali);
	void setIniQuat(osg::Quat quat);
	void setIniAngle(double yaw, double pitch, double roll);
	void setBaseYaw(double yaw);
	static osg::Quat CalcQuatBetweenTwoEulor(double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2, char calctype);
	static osg::Quat EulorToQuat(bool chickscene, bool rollzpitchxyawy, bool bcscene, bool camerascene, int seglabel, double yaw, double pitch, double roll);
    static osg::Quat EulorToQuat(double yaw, double pitch, double roll, char calctype);
	static osg::Quat EulorToQuat_XZY(double yaw, double pitch, double roll);
	static osg::Quat EulorToQuat_XYZ(double yaw, double pitch, double roll);
	static osg::Quat EulorToQuat_YXZ(double yaw, double pitch, double roll);
	static osg::Quat EulorToQuat_YZX(double yaw, double pitch, double roll);
	static osg::Quat EulorToQuat_ZXY(double yaw, double pitch, double roll);
	static osg::Quat EulorToQuat_ZYX(double yaw, double pitch, double roll);

	static osg::Quat slerp(osg::Quat& fquat, osg::Quat& tquat, double ratio = 0.5);
	osg::Quat EulorToQuat_cali(double yaw, double pitch, double roll, double baseyaw, char calctype);

private:

	double m_yaw0, m_pitch0, m_roll0;
	double m_baseyaw;
	osg::Quat m_quat0;
	bool m_abscali;

};

#endif // QUATUTILS_H
