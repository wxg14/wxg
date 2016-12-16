/*
    һЩ���õĺ���
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
	//����exeĿ¼����Ҫ���õĵط��ͷ��ڴ�
	static TCHAR* GetExePath_C();
	//����exeĿ¼����Ҫ���õĵط������㹻���ڴ�
	static TCHAR* GetExePath(TCHAR* path);

	static bool OpenComSucceed(char* comname);
	static bool OpenComSucceed(PortProperty ppty);
	static long CenterAndRadiusAlgorithm(vector<osg::Vec3d> composlist,float &resultx,float &resulty,float &resultz,float &resultr);
	static long long encode(int year, int month, int day);
	static bool decode(long long code, int& year, int& month, int& day);
};

#endif
