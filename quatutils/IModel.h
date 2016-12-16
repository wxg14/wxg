#ifndef _IMODEL_H_
#define _IMODEL_H_
class IModel
{
public:
	//virtual void DriveBone(char* pBuf,DWORD dwCount, bool cali, short movelessseg = -1) = 0;
	virtual void DriveBone(char* pBuf,DWORD dwCount, bool cali, short movelessseg = -1,int iBodyPartFlag=0) = 0;
//	virtual void PrintLog(const char* pMsg)=0;
};

#endif