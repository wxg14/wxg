#ifndef _MNV_H_
#define _MNV_H_
#include <iostream>
#include <map>
#include <string>
using namespace std;
typedef unsigned char BYTE;
typedef unsigned __int32 UINT32;

#pragma pack(push,1)
namespace MNV {
	struct MVN_HEADER
	{
		char IdString[6];
		UINT32 SampleCounter;
		BYTE DatagramCounter;
		BYTE ItemsNumber;
		UINT32 TimeCode;
		BYTE AvatarID;
		BYTE Reserved[7];
	};


	struct Position
	{	
		float x;
		float y;
		float z;
	};

	struct Rotation
	{	
		float x;
		float y;
		float z;
	};

	struct Quaternion
	{	
		float q1;
		float q2;
		float q3;
		float q4;
	};

	struct PoseData01
	{
		UINT32 segmentId;
		Position position;
		Rotation rotation;
	};

	struct PoseData02
	{
		UINT32 segmentId;
		Position position;
		Quaternion quaternion;
	};

	struct PoseData03
	{
		UINT32 pointID;
		Position position;
	};


	struct PoseData04
	{
		UINT32 tagId;
		Position position;
	};

	struct ScaleInformation
	{
		UINT32 segmentId;
		Position position;
		//9x4 bytes matrix containing the rotation and scale of the segment relative to the default
		float m[3][3];
	};

	struct PropInformation
	{
		UINT32 segmentId;
		string propName;
	};
	struct MetaData
	{	
		std::map<std::string, std::string> meta;
	};
}
#pragma pack(pop)


#endif








