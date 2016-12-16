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
	bool motionLess(float extremum = 2.0)//��ת2��������Ϊ��ֹ
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
	const double VREF;//ADC�Ĳο���ѹ
	const double VZeroG;//����ٶȵ�ѹ
	const double Sensitivity;//0.4785V;//������
	const double g;//�������ٶ�

	gravacccalculator m_acccal;//�������ٶȼ�����
	bool m_firstflag;
	double m_IntervalTime;//���ʱ�䣬��λ����
	osg::Vec3d m_position, m_v;
	osg::Vec3d m_sumacc;//���ٶ��ܺ�
	osg::Vec3d m_acc;//��ȥ�������ٶȺ�ļ��ٶ�
	osg::Vec3d m_preacc;//��һ��acc
	osg::Vec3d m_gravacc;//�������ٶ�
	osg::Vec3d m_com;//����������
	osg::Vec3d m_gyr;//����������

	double m_yaw ,m_pitch, m_roll;

};
//ŷ���ǵ������ת�� (���Ե�����)
// matrix EulerToMatrix(double yaw, double pitch, double roll);
// //ŷ���ǵ������ת�� (���嵽����)
// matrix EulerToMatrix_R(double yaw, double pitch, double roll);
// 
// //�ӹ��Ե����������ȡŷ����
// void MatrixToEuler(const matrix& mat, double& yaw, double& pitch, double& roll); 
// //����ŷ����2�ֶ���ŷ����1��ŷ���� 
// void EulerofEuler1ToEuler2(double yaw1, double pitch1, double roll1, double yaw2, double pitch2, double roll2, double& yaw, double& pitch, double& roll);
#endif
