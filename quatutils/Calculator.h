#ifndef _CALCULATOR_H_
#define _CALCULATOR_H_
#include "quatutils_global.h"
const float pi = 3.14159265358979323846;
#include<math.h>
#include "osg/Quat"
#include "osg/Point"
#define FramePS 107.0
// class osg::Vec3d{
// public:
//         double x, y, z;
// 		osg::Vec3d(){x = 0; y = 0; z = 0;};
// 		osg::Vec3d(double ax, double ay, double az){x = ax; y = ay; z = az;};
// 		osg::Vec3d &operator =(const osg::Vec3d& p){x = p.x; y = p.y; z = p.z; return *this;};
// 		bool operator ==(const osg::Vec3d& p){return ((x == p.x) && (y == p.y) && (z == p.z));};
// 		osg::Vec3d operator /(double d) const {return osg::Vec3d(x/d, y/d, z/d);};
// 		osg::Vec3d operator +(const osg::Vec3d& p) const {return osg::Vec3d(x + p.x, y + p.y, z + p.z);};
// 		osg::Vec3d operator -(const osg::Vec3d& p) const {return osg::Vec3d(x - p.x, y - p.y, z - p.z);};
// 		osg::Vec3d operator *(double m) const {return osg::Vec3d(x*m, y*m, z*m);};
// 		double operator *(const osg::Vec3d& p) const {
// 			return x*p.x + y*p.y + z*p.z;
// 		}
// 		double length(){return sqrt(x*x + y*y + z*z);};
// 		void normalself(){
// 			double temp = sqrt(x*x + y*y + z*z);
// 			if (temp > 0.0f)
// 			{
// 				x/=temp;
// 				y/=temp;
// 				z/=temp;
// 			}
// 		};
// 
// };
//typedef osg::Vec3d osg::Vec3d; 
//  
// typedef struct matrix {
//         double m[3][3];
//         osg::Vec3d multipoint(const osg::Vec3d& point);
//         matrix multimatrix(const matrix& mat); 
// }matrix;
class gravacccalculator {
public:
	gravacccalculator(){
		T = 1/FramePS;
	    m_acc.set(0, 0, 0);
	};
	~gravacccalculator(){};
	void SetT(double t)
	{
        T = t;
	}
	osg::Vec3d calculate(osg::Vec3d acc, osg::Vec3d gyr);
	osg::Vec3d Rgyro;
private:
	//double m_preincax, m_preincay, m_preincaz;
	osg::Vec3d m_acc;
	osg::Vec3d m_gyr;
	osg::Vec3d m_preacc;
	double wGyro;
	double T;
};
class QUATUTILS_EXPORT Moving 
{
public:
	Moving();
	~Moving(){};

	
	osg::Vec3d getAcc(){return m_acc;};
	osg::Vec3d getPosition(){return m_position;};
	osg::Vec3d getSpeed(){return m_v;};
	osg::Vec3d getGravAcc(){return m_gravacc;};
	void setAcc(double ax, double ay, double az);
	void setCom(int cx, int cy, int cz);
	void setGyr(double gx, double gy, double gz);
	void setIntervalTime(double intime)
	{
		m_IntervalTime = intime;
		m_acccal.SetT(intime);
	}
	bool motionLess(float extremum = 2.0)//旋转2度以内认为静止
	{
		return ((abs(m_gyr.x()/pi*180) <= extremum)&&(abs(m_gyr.y()/pi*180) <= extremum)&&(abs(m_gyr.z()/pi*180) <= extremum));
	}
	void SetGrav(float gravx, float gravy, float gravz)
	{
// 		if (m_firstflag)
// 		{
// 			m_gravacc.set(m_sumacc.x(), m_sumacc.y(), m_sumacc.z());
// 		}
// 		if (motionLess(10.0))
// 		{
// 			m_gravacc = (m_gravacc + m_sumacc) / 2;
// 		}
// 		else
// 		{
// 			osg::Vec3d gravv(gravx, gravy, gravz);
// 			m_gravacc = (m_gravacc + gravv) / 2;
// 		}
		m_HavCalcGrav = true;
		if ((!motionLess(3.0)) || m_firstflag)
            m_gravacc.set(gravx, gravy, gravz);
	}
	void setAtt(double yaw, double pitch, double roll){
		m_yaw = yaw; 
		m_pitch = pitch;
		m_roll = roll;
	}
	void calculate();
	float pitch;
	float roll, preroll;
	float yaw;
    bool m_HavCalcGrav;

	int ccx, ccy, ccz;
	int cxarroll, cyarroll, czarroll;
    osg::Vec3d Rgyro;
private:
	void calAttAngle();
	void calPosition();
	void calGravityAcc();
	const double VREF;//ADC的参考电压
	const double VZeroG;//零加速度电压
	const double Sensitivity;//0.4785V;//灵敏度
	const double g;//重力加速度

	gravacccalculator m_acccal;//重力加速度计算器
	bool m_firstflag;
	double m_IntervalTime;//间隔时间，单位：秒
	osg::Vec3d m_position, m_v;
	osg::Vec3d m_sumacc;//加速度总和
	osg::Vec3d m_acc;//减去重力加速度后的加速度
	osg::Vec3d m_preacc;//上一个acc
	osg::Vec3d m_gravacc;//重力加速度
	osg::Vec3d m_com;//磁罗盘数据
	osg::Vec3d m_gyr;//陀螺仪数据

	double m_yaw ,m_pitch, m_roll;

};
//欧拉角到矩阵的转换 (惯性到物体)
// matrix EulerToMatrix(double yaw, double pitch, double roll);
// //欧拉角到矩阵的转换 (物体到惯性)
// matrix EulerToMatrix_R(double yaw, double pitch, double roll);
// 
// //从惯性到物体矩阵提取欧拉角
// void MatrixToEuler(const matrix& mat, double& yaw, double& pitch, double& roll); 
// //计算欧拉角2现对于欧拉角1的欧拉角 
// void EulerofEuler1ToEuler2(double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2, double& yaw, double& pitch, double& roll);
#endif
