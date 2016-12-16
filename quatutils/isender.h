#ifndef _ISENDER_H_
#define _ISENDER_H_
#include "osg/Quat"
class ISender
{
public:
	virtual bool sendData(unsigned int portnum, char* buffer, int len)=0;
	virtual bool sendQuat(int portID, int segmentlabel, const osg::Quat& quat, bool cali = false, short moveflag = 0x7fff, bool weightnessless = false)=0;
	virtual void setWeightNess(bool weightnessless)=0;
	virtual void hipsPosChange(double posx, double posy, double posz)=0;
};

#endif