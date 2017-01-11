#ifndef QUATUTILS_GLOBAL_H
#define QUATUTILS_GLOBAL_H

#include <QtCore/qglobal.h>

#define FINGER_NODE_COUNT 18
#define BODY_NODE_COUNT 23
#define BODY_PART_COUNT 3 //0---身体 1---右手   2---左手

//#define MIN_MOVE	20	// add by mvp ## 2015-7-10

struct CalcSetting
{
	bool bShortMode;
	bool bFollowRoot;
	bool bRootRepair;
	bool bThumbRepair;
	float fParamYaw[BODY_NODE_COUNT+1+FINGER_NODE_COUNT*2];
	float fLeftHandYawOffset;//-180~180
	float fRightHandYawOffset;//-180~180
};

#ifdef QUATUTILS_LIB
# define QUATUTILS_EXPORT Q_DECL_EXPORT
#else
# define QUATUTILS_EXPORT Q_DECL_IMPORT
#endif

#endif // QUATUTILS_GLOBAL_H
