/*
    一些常用的函数
*/
#ifndef _COMMON_H_
#define _COMMON_H_
#include<windows.h>
//#include "CoolLog.h"
#include "quatutils_global.h"
#include "qextserialport.h"
#include<vector>
#include<osg\Vec3d>

using namespace std;
class QUATUTILS_EXPORT qcommon
{
public:
	//返回exe目录，需要调用的地方释放内存
	static TCHAR* GetExePath_C();
	//返回exe目录，需要调用的地方分配足够的内存
	static TCHAR* GetExePath(TCHAR* path);

	static bool OpenComSucceed(char* comname);
	static bool OpenComSucceed(PortProperty ppty);
	static long CenterAndRadiusAlgorithm(vector<osg::Vec3d> composlist,float &resultx,float &resulty,float &resultz,float &resultr);
	static long long encode(int year, int month, int day);
	static bool decode(long long code, int& year, int& month, int& day);
};

#endif
