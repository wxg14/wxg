#include <windows.h>
#include "Calculator.h"
//#include "CoolLog.h"
#include<stdio.h>
#include<stdlib.h>
#include <QObject>
#include<math.h>

void makeRotate(double* quat, double angle, char axistype)
{
	const double epsilon = 0.0000001;
	double coshalfangle = cos( 0.5*angle );
	double sinhalfangle = sin( 0.5*angle );
    
	quat[0] = 0;
	quat[1] = 0;
	quat[2] = 0;
	if (axistype == 0)//x轴
	    quat[0] = sinhalfangle;
	else if (axistype == 1)//y轴
	    quat[1] = sinhalfangle;
	else
	    quat[2] = sinhalfangle;
	quat[3] = coshalfangle;
}

void MultiQuat(double* quat1, double* quat2)
{
	double quat[4];
	quat[0] = quat2[3]*quat1[0] + quat2[0]*quat1[3] + quat2[1]*quat1[2] - quat2[2]*quat1[1];
	quat[1] = quat2[3]*quat1[1] - quat2[0]*quat1[2] + quat2[1]*quat1[3] + quat2[2]*quat1[0];
	quat[2] = quat2[3]*quat1[2] + quat2[0]*quat1[1] - quat2[1]*quat1[0] + quat2[2]*quat1[3];
	quat[3] = quat2[3]*quat1[3] - quat2[0]*quat1[0] - quat2[1]*quat1[1] - quat2[2]*quat1[2];
	quat1[0] = quat[0];
	quat1[1] = quat[1];
	quat1[2] = quat[2];
	quat1[3] = quat[3];
}

void EulorToQuat(double* quat, double yaw, double pitch, double roll)
{
	double quat1[4], quat2[4], quat3[4];
	makeRotate(quat1, roll, 1);
	makeRotate(quat2, pitch, 0);
	makeRotate(quat3, yaw, 2);
	MultiQuat(quat1, quat2);
	MultiQuat(quat1, quat3);
	quat[0] = quat1[0];
	quat[1] = quat1[1];
	quat[2] = quat1[2];
	quat[3] = quat1[3];
}

void Vec3dXorVec3d(double*vec1, double* vec2, double* dest)
{
	dest[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1]; 
	dest[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
	dest[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];

}

void QuatMultiVec3d(double* quat, double* v, double* dest)
{
	// nVidia SDK implementation
	double uv[3], uuv[3]; 
	double qvec[3];
	qvec[0] = quat[0];
	qvec[1] = quat[1];
	qvec[2] = quat[2];
	Vec3dXorVec3d(qvec, v, uv);
	Vec3dXorVec3d(qvec, uv, uuv);
	double temp = 2.0*quat[3];
	uv[0] *= temp;
	uv[1] *= temp;
	uv[2] *= temp;
	uuv[0] *= 2.0;
	uuv[1] *= 2.0;
	uuv[2] *= 2.0; 
	dest[0] = v[0] + uv[0] + uuv[0];
	dest[1] = v[1] + uv[1] + uuv[1];
	dest[2] = v[2] + uv[2] + uuv[2];
}


//#include "osg\Matrix"
//欧拉角到矩阵的转换 (惯性到物体)
// matrix EulerToMatrix(double yaw, double pitch, double roll)
// {
//        matrix ret;
//        ret.m[0][0] = cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
// 
//        ret.m[0][1] = -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
//        ret.m[0][2] = sin(yaw)*cos(pitch);
//        
//        ret.m[1][0] = sin(roll)*cos(pitch);
//        ret.m[1][1] = cos(roll)*cos(pitch);
//        ret.m[1][2] = -sin(pitch);
//        
//        ret.m[2][0] = -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll);
//        ret.m[2][1] = sin(roll)*sin(yaw) + cos(yaw)*sin(pitch)*cos(roll);
//        ret.m[2][2] = cos(yaw)*cos(pitch); 
//        return ret;
// }
// 
// //欧拉角到矩阵的转换 (物体到惯性)
// matrix EulerToMatrix_R(double yaw, double pitch, double roll)
// {
//        matrix ret;
//        ret.m[0][0] = cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
// 
//        ret.m[0][1] = sin(roll)*cos(pitch);
//        ret.m[0][2] = -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll);
//        
//        ret.m[1][0] = -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
//        ret.m[1][1] = cos(roll)*cos(pitch);
//        ret.m[1][2] = sin(roll)*sin(yaw) + cos(yaw)*sin(pitch)*cos(roll);
//        
//        ret.m[2][0] = sin(yaw)*cos(pitch);
//        ret.m[2][1] = -sin(pitch);
//        ret.m[2][2] = cos(yaw)*cos(pitch); 
//        return ret;
// }
// 
// //从惯性到物体矩阵提取欧拉角
// void MatrixToEuler(const matrix& mat, double& yaw, double& pitch, double& roll)
// {
//      double sp = -mat.m[1][2];
//      if (sp<=-1.0f) 
//         pitch = -1.570796f;
//      else if (sp >= 1.0f)
//         pitch = 1.570796;
//      else 
//         pitch = asin(sp);
//      
//      //检查万象锁的情况，允许一些误差
//      if (sp > 0.99999f) {
//          roll = 0.0f;
//          yaw = atan2(-mat.m[2][0], mat.m[0][0]);
//      }
//      else {
//          yaw = atan2(mat.m[0][2], mat.m[2][2]);
//          roll = atan2(mat.m[1][0], mat.m[1][1]);
//      }
// }
// 
// //点乘于矩阵，点在左边 
// osg::Vec3d matrix::multipoint(const osg::Vec3d& point)
// {
//      osg::Vec3d ret;
//      ret[0] = point.x()*m[0][0] + point.y()*m[1][0] + point.z()*m[2][0];
//      ret[1] = point.x()*m[0][1] + point.y()*m[1][1] + point.z()*m[2][1]; 
//      ret[2] = point.x()*m[0][2] + point.y()*m[1][2] + point.z()*m[2][2];  
// 	 return ret;
// }
// 
// //矩阵相乘，参数矩阵在右边 
// matrix matrix::multimatrix(const matrix& mat)
// {
//      matrix ret;
//      for (int i=0; i<3; i++){
//          for (int j=0; j<3; j++){
//              ret.m[i][j] = 0; 
//              for (int k=0; k<3; k++)
//                  ret.m[i][j] += m[i][k]*mat.m[k][j]; 
//          } 
//      }
//      return ret;
// }
// 
// void EulerofEuler1ToEuler2(double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2, double& yaw, double& pitch, double& roll)
// {
//     matrix mat1 = EulerToMatrix_R(yaw1, pitch1, roll1);
//     matrix mat2 = EulerToMatrix(yaw2, pitch2, roll2);
//     matrix mat = mat1.multimatrix(mat2);
//     MatrixToEuler(mat, yaw, pitch, roll);
// }

Moving::Moving():VREF(3.3), VZeroG(1.65), Sensitivity(16384.0), g(9.8)
{
	m_HavCalcGrav = false;
    m_firstflag = true;
	m_preacc.set(0, 0, 0);
	m_IntervalTime = 1.0/FramePS;
}

void Moving::setAcc(double ax, double ay, double az)
{
	m_sumacc.set(ax, ay, az);
}
void Moving::setCom(int cx, int cy, int cz)
{
	m_com.set(cx, cy, cz);
}

void Moving::setGyr(double gx, double gy, double gz)
{
	m_gyr.set(gx*pi/180, gy*pi/180, gz*pi/180);
}
void Moving::calGravityAcc()
{
	if (m_HavCalcGrav)
		return;
    m_gravacc = m_acccal.calculate(m_sumacc, m_gyr);
	Rgyro = m_acccal.Rgyro;
}
void Moving::calAttAngle()
{

	/*
    计算航向角
    */
	float pangle = asin(m_gravacc.y()/m_gravacc.length());
	float rangle = -atan2(m_gravacc.x(), m_gravacc.z());
	pitch = pangle*180/pi;
	roll = rangle*180/pi;
	osg::Quat quat = osg::Quat(rangle, osg::Y_AXIS, 
		pangle, osg::X_AXIS, 
		0, osg::Z_AXIS);
	osg::Vec3d cc = quat * m_com;
//     Matrix yawmat(cos(rangle), yawmat.m[0][1] = 0, yawmat.m[0][2] = -sin(rangle), 0);
// 	yawmat.m[0][0] = cos(rangle), yawmat.m[0][1] = 0, yawmat.m[0][2] = -sin(rangle);
// 	yawmat.m[1][0] = 0,           yawmat.m[1][1] = 1, yawmat.m[1][2] = 0;
// 	yawmat.m[2][0] = sin(rangle), yawmat.m[2][1] = 0, yawmat.m[2][2] = cos(rangle);
// 
//     osg::Vec3d cc = m_com;
// 	cc = yawmat.multipoint(cc);
// 	cxarroll = cc.x();
// 	cyarroll = cc.y();
// 	czarroll = cc.z();
// 	yawmat.m[0][0] = 1, yawmat.m[0][1] = 0,            yawmat.m[0][2] = 0;
// 	yawmat.m[1][0] = 0, yawmat.m[1][1] = cos(pangle),  yawmat.m[1][2] = sin(pangle);
// 	yawmat.m[2][0] = 0, yawmat.m[2][1] = -sin(pangle), yawmat.m[2][2] = cos(pangle);
// 	cc = yawmat.multipoint(cc);
	yaw = atan2(cc.x(), cc.y())*180/pi;
	if (yaw < 0)
	   yaw = yaw + 360;
	//PRINTF(QString(QObject::tr("pitch:%1  roll:%2  yaw:%3")).arg(pitch).arg(roll).arg(yaw).toStdString());
	ccx = int(cc.x());
	ccy = int(cc.y());
	ccz = int(cc.z());//航向向量？
    /*
	*/
}
void Moving::calPosition()
{
	if (m_firstflag){
		m_firstflag = false;
		m_acc = osg::Vec3d(0, 0, 0);
		m_preacc = m_acc;
		return;
	}
	//m_acc.normalself();
	//由欧拉角求物体到惯性矩阵
    if (motionLess())
	{
		m_acc = osg::Vec3d(0, 0, 0);
		m_preacc = m_acc;
		m_v = osg::Vec3d(0, 0, 0);
		return;
	}
	if (m_sumacc == m_gravacc)
	{
		m_acc = osg::Vec3d(0, 0, 0);
	}
	else {
// 		matrix mat = EulerToMatrix_R(yaw*pi/180, -pitch*pi/180, roll*pi/180);
// 		m_acc.x = -m_sumacc.x;
// 		m_acc.y = m_sumacc.z;
// 		m_acc.z = m_sumacc.y;
// 		//转换加速度到物体到惯性坐标系下
// 		m_acc = mat.multipoint(m_acc);
// 		m_acc.x = -m_acc.x;
// 		double tempy = m_acc.y;
// 		m_acc.y = m_acc.z;
// 		m_acc.z = tempy;
		//double accy = m_acc.y;
		//计算x,z轴的加速度
		//m_acc.x = -ay/Sensitivity;
		//m_acc.y = az/Sensitivity;
		//m_acc.z = -ax/Sensitivity;
		//m_acc = mat.multipoint(m_acc);
		//m_acc.y = accy;
		osg::Vec3d vsumacc(m_sumacc.x(), m_sumacc.y(), m_sumacc.z());
		osg::Quat quat;
        EulorToQuat(quat._v, osg::DegreesToRadians(yaw), osg::DegreesToRadians(pitch), osg::DegreesToRadians(roll));
		osg::Vec3d vacc;
		//vacc = quat * vsumacc;
        QuatMultiVec3d(quat._v, vsumacc._v, vacc._v);

		/*osg::Quat quat = osg::Quat(osg::DegreesToRadians(roll), osg::Y_AXIS, 
			osg::DegreesToRadians(pitch), osg::X_AXIS, 
			osg::DegreesToRadians(yaw), osg::Z_AXIS); 
		osg::Vec3d vsumacc(m_sumacc.x(), m_sumacc.y(), m_sumacc.z());
		osg::Vec3d vacc = quat * vsumacc;*/
		m_acc.set(vacc.x(), vacc.y(), vacc.z());
		m_acc = m_acc + osg::Vec3d(0, 0,  -1.001);
	}
	//m_acc.y = int(m_acc.y*1000)/1000.0;
// 	if (abs(m_acc.y()) <= 0.009) 
// 		m_acc[1] = 0;
// 	if (abs(m_acc.z()) <= 0.009) 
// 		m_acc[2] = 0;
// 	if (abs(m_acc.x()) <= 0.009) 
// 		m_acc[0] = 0;
	//if (abs(m_acc.length()) < 0.005) m_acc = osg::Vec3d(0,0,0);
	osg::Vec3d acc = (m_acc + m_preacc)/2;
	
	m_position = m_position + (m_v * m_IntervalTime + acc * (g * (m_IntervalTime * m_IntervalTime) / 2.0)) * 1000;
	//更新速度
	m_v = m_v + acc * (g * m_IntervalTime);
	double temp = m_acc.length();
	/*if ((m_acc.x == m_preacc.y) && (m_acc.x == 0))
		m_v.x = 0;
	if ((m_acc.y == m_preacc.y) && (m_acc.y == 0)) 
		m_v.y = 0;
	if ((m_acc.z == m_preacc.y) && (m_acc.z == 0))
		m_v.z = 0;*/
	if (temp <= 0.01)
	{
		//m_v.x = m_v.x*0.3;
		//m_v.y = m_v.y*0.3;
		//m_v.z = m_v.z*0.3;
		//m_v = m_v*0.3;
		//if (abs(m_v.length()) <0.01)
		//	m_v = osg::Vec3d(0,0,0);
	}
	else {
		//m_v = m_v*0.8;
		//if (abs(m_v.length()) <0.01)
		//	m_v = osg::Vec3d(0,0,0);
	}
	
	m_preacc = m_acc;
	//m_acc = m_gravacc;
    return;
}
void Moving::calculate()//？
{
	//计算重力加速度
	calGravityAcc();
	//计算姿态角度
	calAttAngle();
	//计算位移
	calPosition();
	//m_acc = m_gravacc;

}
//gyr为弧度/秒
double maxgyro = -1000000, mingyro = 10000000;
osg::Vec3d gravacccalculator::calculate(osg::Vec3d acc, osg::Vec3d gyr)
{
	if (acc.length()>2)
		acc = m_preacc;
	else
		m_preacc = acc;
	wGyro = abs(acc.length() - 1) * 100;
//     if (wGyro < 10)
// 		wGyro = 10;
	//if (MPU6050TEST::m_config->get(CMAIN, "accminusgyr", 0).toInt() == 1)
	//{
// 	double gx = gyr.x * 180 / pi;
// 	double gy = gyr.y * 180 / pi;
// 	double gz = gyr.z * 180 / pi;
// 	double iax = gy*0.009;
// 	double iay = gx*0.009 - gz*0.0045;//(gx-gz)*0.009;
// 	double iaz = (gx + gy)*0.0006;
// 	acc.x -= (iax - m_preincax) -abs(gz*0.0004);
// 	acc.y += (iay - m_preincay);
// 	acc.z += abs(iaz);
// 	m_preincax = iax;
// 	m_preincay = iay;
//     m_preincaz = iaz;
	//}
	if (m_acc.x() == 0 && m_acc.y() == 0 && m_acc.z() == 0)//REst（0）= RAcc（0）
	{
		m_acc = acc;
		m_acc.normalize();
		m_gyr = gyr;
		return m_acc;
	}
	
	if (abs(m_acc.z()) <= 0.01)
	{
	   Rgyro = m_acc;
	}
	else
	{
		osg::Vec3d avggyr = (m_gyr + gyr) / 2;
		double axz = atan2(m_acc.x(), m_acc.z());
		axz = axz - avggyr.y() * T;
		double ayz = atan2(m_acc.y(), m_acc.z());
		ayz = ayz + avggyr.x() * T;
        int signRzGyro = ( cos(axz) >=0 ) ? 1 : -1;
		
		Rgyro[0] = sin(axz) / sqrt(1 + cos(axz) * cos(axz) * tan(ayz) * tan(ayz));
		Rgyro[1] = sin(ayz) / sqrt(1 + cos(ayz) * cos(ayz) * tan(axz) * tan(axz));
		//PRINTF(QString(QObject::tr("m_acc.x: %1 m_acc.z: %2 axz:%3 avggyr.y:%4  ayz:%5  avggyr.x:%6 x:%7")).arg(m_acc.x()).arg(m_acc.z()).arg(axz).arg(avggyr.y()).arg(ayz).arg(avggyr.x()).arg(Rgyro.x()).toStdString());
		//Rgyro.x =  1  / sqrt (1 + 1/(tan(axz)*tan(axz)) / (cos(ayz)*cos(ayz)));
		//Rgyro.y =  1  / sqrt (1 + 1/(tan(ayz)*tan(ayz)) / (cos(axz)*cos(axz)));   
        
		//if (m_acc.z <0 )
		//   Rgyro.z = - sqrt(1 - Rgyro.x * Rgyro.x - Rgyro.y * Rgyro.y);
		//else
		//   Rgyro.z = sqrt(1 - Rgyro.x * Rgyro.x - Rgyro.y * Rgyro.y);
		Rgyro[2] = signRzGyro * sqrt(1 - Rgyro.x() * Rgyro.x() - Rgyro.y() * Rgyro.y());
	}

	//Rest(n) = (Racc + Rgyro * wGyro ) / (1 + wGyro)
	//double lrgyro = Rgyro.length();
	//m_acc = (acc * abs(lrgyro - 1) + Rgyro * (lacc - 1)) / (abs(lrgyro - 1) + abs(lacc - 1));
	m_acc = (acc + Rgyro * wGyro) / (1 + wGyro);
	//m_acc = Rgyro;
	if (abs(m_acc.length() - 1) > abs(acc.length() - 1))
		//m_acc = (acc * abs(m_acc.length() - 1) + m_acc * abs(acc.length() - 1)) / (abs(m_acc.length() - 1) + abs(acc.length() - 1));
		m_acc = (acc + m_acc) / 2;
	//m_acc.normalself();
	m_gyr = gyr;
	return m_acc;
}
