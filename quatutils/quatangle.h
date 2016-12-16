#pragma once
#include <assert.h>
#include <stdio.h>
#include <math.h>

enum AxieOrd
{
	ZYX=0,
	ZYZ,
	ZXY,
	ZXZ,
	YXZ,
	YXY,
	YZX,
	YZY,
	XYZ,
	XYX,
	XZY,
	XZX
};

template<class T>
struct Quaternion
{
  T w,x,y,z;	
  
  Quaternion(T w0=1, T x0=0, T y0=0, T z0=0)
  {
    set(w0, x0, y0, z0);
  }
  
  void set(T w0, T x0, T y0, T z0)
  {
    T t=1/sqrt(w0*w0+x0*x0+y0*y0+z0*z0);
   if(t>1e-6)
   {
		w=t*w0;
		x=t*x0;
		y=t*y0;
		z=t*z0;
   }
   else
   { 
	   w=1.0;
	   x=y=z=0.0;
   }
  }

  void euler(T &rotz, T &roty, T &rotx)
  {
      //ZYX 次序输出
    static const double ANGLE=180/3.1415926;
    T r12=1-2*(y*y+z*z);
    T r11=2*(x*y+z*w);
    T r21= -2*(x*z-y*w);
    T r31=2*(y*z+x*w);
    T r32=1-2*(x*x+y*y);
    
    rotz=ANGLE*atan2(r11, e12);
    roty=ANGLE*asin(r21);
    rotx=ANGLE*atan2(r31,r32);
    
    return;
  }

  void Angle2Quat(T r1,T r2,T r3,AxieOrd iaxistype)
  {
	  static const float rad=3.1415926/180.0;
	  r1 *= rad;
	  r2 *= rad;
	  r3 *= rad;

	  float cangr1 = cos(r1/2.0); 
	  float cangr2 =  cos(r2/2.0);
	  float cangr3 = cos(r3/2.0); 
	  float sangr1 =  sin(r1/2.0);
	  float sangr2 =  sin(r2/2.0);
	  float sangr3 =  sin(r3/2.0);

	  switch (iaxistype)
	  {
	  case 	ZYX:
		  {
			  w = cangr1*cangr2*cangr3 + sangr1*sangr2*sangr3;
			  x = cangr1*cangr2*sangr3 - sangr1*sangr2*cangr3;
			  y = cangr1*sangr2*cangr3 + sangr1*cangr2*sangr3;
			  z = sangr1*cangr2*cangr3 - cangr1*sangr2*sangr3;
	  			break;
		  }
		 case 	 ZYZ:
		  {
			  w = cangr1*cangr2*cangr3 - sangr1*cangr2*sangr3;
			  x = cangr1*sangr2*sangr3 - sangr1*sangr2*cangr3;
			  y = cangr1*sangr2*cangr3 + sangr1*sangr2*sangr3;
			  z = sangr1*cangr2*cangr3 + cangr1*cangr2*sangr3;
				break;
		  }
		 case 	ZXY:
		  {
			  w = cangr1*cangr2*cangr3 - sangr1*sangr2*sangr3;
			  x = cangr1*sangr2*cangr3 - sangr1*cangr2*sangr3;
			  y = cangr1*cangr2*sangr3 + sangr1*sangr2*cangr3;
			  z = cangr1*sangr2*sangr3 + sangr1*cangr2*cangr3;
			  break;
		  }
		 case 	ZXZ:
		  {
			  w = cangr1*cangr2*cangr3 - sangr1*cangr2*sangr3;
			  x = cangr1*sangr2*cangr3 + sangr1*sangr2*sangr3;
			  y = sangr1*sangr2*cangr3 - cangr1*sangr2*sangr3;
			  z = cangr1*cangr2*sangr3 + sangr1*cangr2*cangr3;
			  break;
		  }
			case YXZ:
			{
				w = cangr1*cangr2*cangr3 + sangr1*sangr2*sangr3;
				x = cangr1*sangr2*cangr3 + sangr1*cangr2*sangr3;
				y = sangr1*cangr2*cangr3 - cangr1*sangr2*sangr3;
				z = cangr1*cangr2*sangr3 - sangr1*sangr2*cangr3;
				 break;
			}
			case  YXY:
			{
				w = cangr1*cangr2*cangr3 - sangr1*cangr2*sangr3;
				x = cangr1*sangr2*cangr3 + sangr1*sangr2*sangr3;
				y = sangr1*cangr2*cangr3 + cangr1*cangr2*sangr3;
				z = cangr1*sangr2*sangr3 - sangr1*sangr2*cangr3;
				break;
			}
			case  	  YZX:
			{
				w = cangr1*cangr2*cangr3 - sangr1*sangr2*sangr3;
				x = cangr1*cangr2*sangr3 + sangr1*sangr2*cangr3;
				y = cangr1*sangr2*sangr3 + sangr1*cangr2*cangr3;
				z = cangr1*sangr2*cangr3 - sangr1*cangr2*sangr3;
				break;
			}
			case    YZY:
			{
				w = cangr1*cangr2*cangr3 - sangr1*cangr2*sangr3;
				x = sangr1*sangr2*cangr3 - cangr1*sangr2*sangr3;
				y = cangr1*cangr2*sangr3 + sangr1*cangr2*cangr3;
				z = cangr1*sangr2*cangr3 + sangr1*sangr2*sangr3;
				break;
			}
			case   XYZ:
			{
				w = cangr1*cangr2*cangr3 - sangr1*sangr2*sangr3;
				x = cangr1*sangr2*sangr3 + sangr1*cangr2*cangr3;
				y = cangr1*sangr2*cangr3 - sangr1*cangr2*sangr3;
				z = cangr1*cangr2*sangr3 + sangr1*sangr2*cangr3;
				break;
			}
			case   XYX:
			{
				w = cangr1*cangr2*cangr3 - sangr1*cangr2*sangr3;
				x = cangr1*cangr2*sangr3 + sangr1*cangr2*cangr3;
				y = cangr1*sangr2*cangr3 + sangr1*sangr2*sangr3;
				z = sangr1*sangr2*cangr3 - cangr1*sangr2*sangr3;
					break;
			}
			case  XZY:
			{
				w = cangr1*cangr2*cangr3 + sangr1*sangr2*sangr3;
				x = sangr1*cangr2*cangr3 - cangr1*sangr2*sangr3;
				y = cangr1*cangr2*sangr3 - sangr1*sangr2*cangr3;
				z = cangr1*sangr2*cangr3 + sangr1*cangr2*sangr3;
					break;
			}
			case  XZX:
			{
				w = cangr1*cangr2*cangr3 - sangr1*cangr2*sangr3;
				x = cangr1*cangr2*sangr3 + sangr1*cangr2*cangr3;
				y = cangr1*sangr2*sangr3 - sangr1*sangr2*cangr3;
				z = cangr1*sangr2*cangr3 + sangr1*sangr2*sangr3;
					break;
			}
			default:
				break;
	  }
  }

  void ThreeAxisRot(float r11, float r12,float r21,float r31, float r32, float &r1, float &r2, float &r3)
  {
	 static const  float ANGLE=180/3.1415926;
	 r1 = ANGLE*atan2( r11, r12 );
	 r2 = ANGLE*asin( r21 );
	 r3 = ANGLE*atan2( r31, r32 );
  }
  void TwoAxisRot(float r11, float r12,float r21,float r31, float r32, float &r1, float &r2, float &r3)
  {
	  static const  float ANGLE=180/3.1415926;
	  r1 = ANGLE*atan2( r11, r12 );
	  r2 = ANGLE*acos( r21 );
	  r3 = ANGLE*atan2( r31, r32 );
  }

  void Quat2Angle(float &xangle,float &yangle,float &zangle,AxieOrd iaxistype)
  {
	  float r1=0,r2=0,r3=0;
	  xangle = yangle = zangle = 0;
	  switch (iaxistype)
	  {
	  case ZYX:
		  {
			  ThreeAxisRot( 2.0*(x*y + w*z), w*w + x*x - y*y - z*z,-2*(x*z - w*y), 2*(y*z + w*x),w*w - x*x - y*y + z*z, r1,r2,r3);

			  zangle = r1;
			  yangle = r2;
			  xangle = r3;
			  break;
		  }
	  case ZYZ:
		  {
				 TwoAxisRot( 2*(y*z - w*x), 2*(x*z + w*y), w*w - x*x - y*y + z*z, 2*(y*z + w*x), -2*(x*z - w*y), r1 ,r2, r3);
				 zangle = r1;
				 yangle = r2;
				 xangle = r3;
				  break;
		  }
	  case ZXY:
		  {
				ThreeAxisRot( -2*(x*y - w*z),  w*w - x*x + y*y - z*z, 2*(y*z + w*x), -2*(x*z - w*y), w*w - x*x - y*y + z*z, r1 ,r2, r3);
				
				zangle = r1;
				xangle = r2;
				yangle = r3;
				 break;
		  }
	  case ZXZ:
		  {
				TwoAxisRot( 2*(x*z + w*y), -2*(y*z - w*x), w*w - x*x - y*y + z*z, 2*(x*z - w*y), 2*(y*z + w*x), r1 ,r2, r3);
				
				zangle = r1;
				xangle = r2;
				yangle = r3;						  
				break;
		  }
	  case YXZ:
		  {
				ThreeAxisRot( 2*(x*z + w*y),  w*w - x*x - y*y + z*z, -2*(y*z - w*x),  2*(x*y + w*z), w*w - x*x + y*y - z*z, r1 ,r2, r3);
				
				yangle = r1;
				xangle = r2;
				zangle = r3;									  
				break;
		  }
	  case YXY:
		  {
				TwoAxisRot( 2*(x*y - w*z), 2*(y*z + w*x), w*w - x*x + y*y - z*z, 2*(x*y + w*z), -2*(y*z - w*x), r1 ,r2, r3);

				yangle = r1;
				xangle = r2;
				zangle = r3;									  
				break;
		  }

	  case YZX:
		  {
				ThreeAxisRot( -2*(x*z - w*y),w*w + x*x - y*y - z*z, 2*(x*y + w*z), -2*(y*z - w*x), w*w - x*x + y*y - z*z, r1 ,r2, r3);

				yangle = r1;
				zangle = r2;
				xangle = r3;		
				break;
		  }
	  case YZY:
		  {
				TwoAxisRot( 2*(y*z + w*x), -2*(x*y - w*z),w*w - x*x + y*y - z*z, 2*(y*z - w*x), 2*(x*y + w*z), r1 ,r2, r3);
				
				yangle = r1;
				zangle = r2;
				xangle = r3;											  
				break;
		  }
	  case XYZ:
		  {
				ThreeAxisRot( -2*(y*z - w*x), w*w - x*x - y*y + z*z, 2*(x*z + w*y), -2*(x*y - w*z), w*w + x*x - y*y - z*z, r1 ,r2, r3);
				
				xangle = r1;
				yangle = r2;
				zangle = r3;		
				break;
		  }

	  case XYX:
		  {
				TwoAxisRot( 2*(x*y + w*z), -2*(x*z - w*y), w*w + x*x - y*y - z*z, 2*(x*y - w*z), 2*(x*z + w*y), r1 ,r2, r3);
				
				xangle = r1;
				yangle = r2;
				zangle = r3;		
				break;		
		  }

	  case XZY:
		  {
				ThreeAxisRot( 2*(y*z + w*x), w*w - x*x + y*y - z*z,-2*(x*y - w*z), 2*(x*z + w*y), w*w + x*x - y*y - z*z, r1 ,r2, r3);
				
				xangle = r1;
				zangle = r2;
				yangle = r3;		
				break;	
		  }
	  case XZX:
		  {
				TwoAxisRot( 2*(x*z - w*y), 2*(x*y + w*z),w*w + x*x - y*y - z*z, 2*(x*z + w*y), -2*(x*y - w*z), r1 ,r2, r3);
				
				xangle = r1;
				zangle = r2;
				yangle = r3;		
				break;		
		  }
	  default:
	  	break;
	  }
  }

  Quaternion<T> inv()
  {
    return Quaternion<T>(-w,x,y,z);
  }

};

template<class T>
Quaternion<T> operator*(const Quaternion<T> &q1, const Quaternion<T> &q2)
{
  
  /*
    q1=(w1,x1,y1,z1)
    q2=(w2,x2,y2,z2)
    q1 * q2 = 
    (w1*w2 - x1*x2 - y1*y2 - z1*z2) + 
    (w1*x2 + x1*w2 + y1*z2 - z1*y2) i + 
    (w1*y2 - x1*z2 + y1*w2 + z1*x2) j + 
    (w1*z2 + x1*y2 - y1*x2 + z1*w2) k 
  */
  
  T w1=q1.w, x1=q1.x, y1=q1.y, z1=q1.z;
  T w2=q2.w, x2=q2.x, y2=q2.y, z2=q2.z;
  
  T w,x,y,z;
  
  w=(w1*w2 - x1*x2 - y1*y2 - z1*z2);
  x=(w1*x2 + x1*w2 + y1*z2 - z1*y2);
  y=(w1*y2 - x1*z2 + y1*w2 + z1*x2);
  z=(w1*z2 + x1*y2 - y1*x2 + z1*w2);
  
  return Quaternion<T>(w, x, y, z);
}


template<class T>
T dot(const Quaternion<T> &q0, const Quaternion<T> &q1)
{
  return q0.w*q1.w+q0.x*q1.x+q0.y*q1.y+q0.z*q1.z;
}

template<class T>
Quaternion<T> slerp(const Quaternion<T> &q0, const Quaternion<T> &q1, T t)
{
  if(t<=0) return q0;
  if(t>=1) return q1;
  
  T cosVal=dot(q0,q1);
  
  Quaternion<T> q1_;
  if(cosVal<0)
    q1_.set(-q1.w, -q1.x, -q1.y, -q1.z), cosVal=-cosVal;
  else 
    q1_.set(q1);
  
  T a0, a1;
  if(cosVal>(T)0.99999)
    a0=1-t, a1=t;
  else 
    {
      T sinVal=sqrt(1-cosVal*cosVal);
      T ang=atan2(sinVal, cosVal);
      T oneOverSin=1/sinVal;
      a0=sin((1-t)*ang)*oneOverSin;
      a1=sin(t*ang)*oneOverSin;
    }
  
  T w, x, y ,z;
  
  w=a0*q0.w+a1*q1_.w;
  x=a0*q0.x+a1*q1_.x;
  y=a0*q0.y+a1*q1_.y;
  z=a0*q0.z+a1*q1_.z;
  
  return Quaternion<T>(w, x, y, z);
}
