/*
    一些常用的函数
*/

#include "common.h"
#include<string.h>
#include<tchar.h>

#include <math.h>
#include<time.h>
#include "quatutils.h"

#include "CoolLog.h"

#include "calibration.h"
bool qcommon::m_bInited = false;
//CalcConfig* qcommon::m_pCC = NULL;

//返回exe目录，需要调用的地方释放内存
TCHAR * qcommon::GetExePath_C()
{
	TCHAR exeFullPath[MAX_PATH]; // MAX_PATH在WINDEF.h中定义了，等于260  
	memset(exeFullPath,0,MAX_PATH);    
	GetModuleFileName(NULL,exeFullPath,MAX_PATH);  
	TCHAR *p = _tcsrchr(exeFullPath, _T('\\'));  
	*p=0x00; 
	TCHAR* exepath = _tcsdup(exeFullPath);
	return exepath;
}
//返回exe目录，需要调用的地方分配足够的内存
TCHAR* qcommon::GetExePath(TCHAR* path)
{
	TCHAR exeFullPath[MAX_PATH]; // MAX_PATH在WINDEF.h中定义了，等于260  
	memset(exeFullPath,0,MAX_PATH);    
	GetModuleFileName(NULL,exeFullPath,MAX_PATH);  
	TCHAR *p = _tcsrchr(exeFullPath, _T('\\'));  
	*p=0x00; 
	_tcscpy(path, exeFullPath);
    return path;
}
bool qcommon::OpenComSucceed(char* comname)
{
	HANDLE hcom;
	hcom = CreateFile(comname,	// 串口名称或设备路径//\\\\.\\COM11
		GENERIC_READ | GENERIC_WRITE,	// 读写方式
		0,				// 共享方式：独占
		NULL,			// 默认的安全描述符
		OPEN_EXISTING,	// 创建方式
		0,				// 不需设置文件属性
		NULL);			// 不需参照模板文件

	
	bool reslt = (hcom != INVALID_HANDLE_VALUE); 
	if (!reslt)
	{
		int nlasterror = GetLastError();
		//PRINTF("************************************* Open File %s Failed, %d, %d", comname, nlasterror, reslt);
	}
	//else 
    //    PRINTF("************************************* Open File %s Succeed, %d, %d", comname, 0, reslt);
	CloseHandle(hcom);
	return reslt;
}

bool qcommon::OpenComSucceed(PortProperty ppty)
{
	QString portname = "\\\\.\\" + ppty.Portname;
	QextSerialPort* serialPort = new QextSerialPort(portname.toAscii(),QextSerialPort::EventDriven);
	serialPort->setBaudRate(ppty.baudtype);
	serialPort->setFlowControl(FLOW_OFF);
	serialPort->setParity(PAR_NONE);
	serialPort->setDataBits(DATA_8);
	serialPort->setStopBits(STOP_1);
	bool reslt = serialPort->open(QIODevice::ReadWrite); 
	if (!reslt)
	{
		int nlasterror = GetLastError();
		//PRINTF("************************************* Open File %s Failed, %d, %d", portname.toAscii().data(), nlasterror, reslt);
	}
	//else 
	//	PRINTF("************************************* Open File %s Succeed, %d, %d", portname.toAscii().data(), 0, reslt);
	if(serialPort->isOpen()) serialPort->close();
	delete serialPort;
	return reslt;
}

void qcommon::InitCalibration()
{
	m_bInited = calibrationInitialize();
}
bool qcommon::CalibrationAlgorithm(vector<osg::Vec3d> composlist,float fRefVal,float* pRetGroup)
{
	bool bRet = true;

	if(m_bInited)
	{
		int iSampleCount=composlist.size();
		mwArray h(3,3,mxDOUBLE_CLASS),b(3,1,mxDOUBLE_CLASS),cx(1,1,mxDOUBLE_CLASS);
		mwArray ax(3,iSampleCount,mxDOUBLE_CLASS),bx(1,1,mxINT16_CLASS);
		double ai[3000];
		short bi=iSampleCount;//采样个数
		double cd=fRefVal;//51;//北京场强
		//int iSensitivity=2048;
		for(int i=0;i<iSampleCount;i++)
		{
			ai[i*3] = composlist[i].x();
			ai[i*3+1] = composlist[i].y();
			ai[i*3+2] = composlist[i].z();
		}
		ax.SetData(ai,iSampleCount*3);
		bx.SetData(&bi,1);
		cx.SetData(&cd,1);

		try{
			Correct(2,h,b,ax,bx,cx);
		}
		catch(...)
		{
// 			int iIndex=0;
// 			pRetGroup[iIndex++] = 1;
// 			pRetGroup[iIndex++] = 0;
// 			pRetGroup[iIndex++] = 0;
// 			pRetGroup[iIndex++] = 1;
// 			pRetGroup[iIndex++] = 0;
// 			pRetGroup[iIndex++] = 1;
// 			pRetGroup[iIndex++] = 0;
// 			pRetGroup[iIndex++] = 0;
// 			pRetGroup[iIndex++] = 0;	
			//calibrationTerminate();
			return false;
		}
		double outh[9],outb[3];
		h.GetData(outh,9);
		b.GetData(outb,3);

		int iIndex=0;
		pRetGroup[iIndex++] = outh[0];
		pRetGroup[iIndex++] = outh[3];
		pRetGroup[iIndex++] = outh[6];
		pRetGroup[iIndex++] = outh[4];
		pRetGroup[iIndex++] = outh[7];
		pRetGroup[iIndex++] = outh[8];
		pRetGroup[iIndex++] = outb[0];
		pRetGroup[iIndex++] = outb[1];
		pRetGroup[iIndex++] = outb[2];

	}
	else
		bRet = false;

	return bRet;
}
void qcommon::TerminateCalibration()
{
	calibrationTerminate();
	m_bInited = false;
}

long qcommon::CenterAndRadiusAlgorithm(vector<osg::Vec3d> composlist,float &resultx,float &resulty,float &resultz,float &resultr)
{	
	bool bRet = true;
	double V,X[4],Y[4],Z[4],rx,ry,rz,L102,L202,L302;
	double pt[5][3];	
	double ratereceive=100;
	int icount=0,i=0,choosecount=10000,iLimit=100000;
	int irand,s[5];
	int trnd=0;
	int listsize=composlist.size();
	int ptcount=listsize;
	double Maxx=composlist[0].x(),Minx=composlist[0].x(),Maxy=composlist[0].y(),Miny=composlist[0].y(),Maxz=composlist[0].z(),Minz=composlist[0].z(),Maxr=0;
	int iMaxCount=1000;
	for (i=1;i<listsize;i++)
	{
		if (Maxx<composlist[i].x())
		{
			Maxx=composlist[i].x();
		}
		if (Minx>composlist[i].x())
		{
			Minx=composlist[i].x();
		}
		if (Maxy<composlist[i].y())
		{
			Maxy=composlist[i].y();
		}
		if (Miny>composlist[i].y())
		{
			Miny=composlist[i].y();
		}
		if (Maxz<composlist[i].z())
		{
			Maxz=composlist[i].z();
		}
		if (Minz>composlist[i].z())
		{
			Minz=composlist[i].z();
		}
	}
	Maxr=(Maxx-Minx)/2;
	if (Maxr<(Maxy-Miny)/2)
	{
		Maxr=(Maxy-Miny)/2;
	}
	if (Maxr<(Maxz-Minz)/2)
	{
		Maxr=(Maxz-Minz)/2;
	}
	int iCheckCount = 0;
	while(icount<choosecount)
	{
		srand((long)time(NULL)+trnd);		
		s[1]=rand()%ptcount;
		s[2]=rand()%ptcount;
		s[3]=rand()%ptcount;
		s[4]=rand()%ptcount;
		trnd=rand()%ptcount;
		if (s[1]==s[2] || s[1]==s[3] || s[1]==s[4] || s[2]==s[3] || s[2]==s[4] || s[3]==s[4])
		{
			iCheckCount++;
			if(iCheckCount-icount>iMaxCount && (icount>iMaxCount || (icount==0 && iCheckCount>iLimit)))
			{
				bRet = false;
				break;
			}
			continue;
		}

		for (i=1;i<=4;i++)
		{
			irand=s[i];
			pt[i][0]=composlist[irand].x();
			pt[i][1]=composlist[irand].y();
			pt[i][2]=composlist[irand].z();
		}
		for (i=1;i<=3;i++)
		{
			X[i]=pt[i+1][0]-pt[1][0];
			Y[i]=pt[i+1][1]-pt[1][1];
			Z[i]=pt[i+1][2]-pt[1][2];
		}

		V=(X[1]*Y[2]*Z[3]+X[3]*Y[1]*Z[2]+X[2]*Y[3]*Z[1]-X[3]*Y[2]*Z[1]-X[2]*Y[1]*Z[3]-X[1]*Y[3]*Z[2])/6;
		if (V==0)
		{
			iCheckCount++;
			if(iCheckCount-icount>iMaxCount && (icount>iMaxCount || (icount==0 && iCheckCount>iLimit)))
			{
				bRet = false;
				break;
			}
			continue;
		}

		L102=(pt[2][0]-pt[1][0])*(pt[2][0]-pt[1][0])+(pt[2][1]-pt[1][1])*(pt[2][1]-pt[1][1])+(pt[2][2]-pt[1][2])*(pt[2][2]-pt[1][2]);
		L202=(pt[3][0]-pt[1][0])*(pt[3][0]-pt[1][0])+(pt[3][1]-pt[1][1])*(pt[3][1]-pt[1][1])+(pt[3][2]-pt[1][2])*(pt[3][2]-pt[1][2]);
		L302=(pt[4][0]-pt[1][0])*(pt[4][0]-pt[1][0])+(pt[4][1]-pt[1][1])*(pt[4][1]-pt[1][1])+(pt[4][2]-pt[1][2])*(pt[4][2]-pt[1][2]);

		rx=pt[1][0]+((Y[2]*Z[3]-Y[3]*Z[2])*L102-(Y[1]*Z[3]-Y[3]*Z[1])*L202+(Y[1]*Z[2]-Y[2]*Z[1])*L302)/12/V;
		ry=pt[1][1]-((X[2]*Z[3]-X[3]*Z[2])*L102-(X[1]*Z[3]-X[3]*Z[1])*L202+(X[1]*Z[2]-X[2]*Z[1])*L302)/12/V;
		rz=pt[1][2]+((X[2]*Y[3]-X[3]*Y[2])*L102-(X[1]*Y[3]-X[3]*Y[1])*L202+(X[1]*Y[2]-X[2]*Y[1])*L302)/12/V;	
		if (rx>Maxx || rx<Minx || ry>Maxy || ry<Miny || rz>Maxz || rz<Minz)
		{
			iCheckCount++;
			if(iCheckCount-icount>iMaxCount && (icount>iMaxCount || (icount==0 && iCheckCount>iLimit)))
			{
				bRet = false;
				break;
			}
			continue;
		}

		double Trate=0,Trad=0,Ttmp=0;				
		double rd[512];
		for (i=0;i<listsize;i++)
		{
			rd[i]=sqrt((rx-composlist[i].x())*(rx-composlist[i].x())+(ry-composlist[i].y())*(ry-composlist[i].y())+(rz-composlist[i].z())*(rz-composlist[i].z()));
			Ttmp=Ttmp+rd[i];
		}
		Trad=Ttmp/listsize;
		if (Trad>Maxr)
		{
			iCheckCount++;
			if(iCheckCount-icount>iMaxCount && (icount>iMaxCount || (icount==0 && iCheckCount>iLimit)))
			{
				bRet = false;
				break;
			}
			continue;
		}

		for (i=0;i<listsize;i++)
		{
			Trate=Trate+fabs((rd[i]-Trad)/Trad);
		}
		Trate=Trate/listsize;	
		icount++;
		iCheckCount++;
		if (Trate>ratereceive)
		{	
			continue;
		}
		ratereceive=Trate;
		resultx=rx;
		resulty=ry;
		resultz=rz;
		resultr=Trad;			
	}	
/*	
	HANDLE h = CreateFile("C:\\spiritLazioLog\\xyz\\mydata.txt",FILE_ALL_ACCESS,FILE_SHARE_READ|FILE_SHARE_WRITE,NULL,OPEN_ALWAYS,FILE_ATTRIBUTE_NORMAL,NULL);
	if ( INVALID_HANDLE_VALUE != h )
	{
		DWORD num;
		SetFilePointer (h,0,NULL,FILE_END);
		//spiritLazioLog
		//sprintf_s(szInfo,1024,"%04d-%02d-%02d, %02d:%02d:%02d   %s\r\n",oT.wYear,oT.wMonth,oT.wDay,oT.wHour,oT.wMinute,oT.wSecond,pMsg); 
		char szInfo[1024];

		sprintf_s(szInfo,1024,"Trate=%f,x=%f,y=%f,z=%f,r=%f \r\n",ratereceive,resultx,resulty,resultz,resultr);
		
		WriteFile(h,szInfo,strlen(szInfo),&num,NULL);
		
		CloseHandle(h);
	}*/

	return icount;
}

long long qcommon::encode(int year, int month, int day)
{
	long long t;
	char sxv[8], lxv[8];
	t = (year*100+month)*100+day;
	for (int i=7; i>=0; i--)
	{
		sxv[i] = t%10;
		t/=10;
	}
	lxv[0] = sxv[0], lxv[1] = sxv[7], lxv[2] = sxv[1], lxv[3] = sxv[6];
	lxv[4] = sxv[2], lxv[5] = sxv[5], lxv[6] = sxv[3], lxv[7] = sxv[4];
	t = 0;
	for (int i=0; i<8; i++)
	{
		t = t*10 + lxv[i];
	}


	t*=7;

	int c = t;

	while (c>=10)
	{
		int temp = c;
		c = 0;
		while (temp)
		{
			c+=temp%10;
			temp/=10;
		}       
	}

	return t*10+c;
}

bool qcommon::decode(long long code, int& year, int& month, int& day)
{
	long long t;
	int cc;
	t = code/10;
	cc = code%10;

	int c = t;

	while (c>=10)
	{
		int temp = c;
		c = 0;
		while (temp)
		{
			c+=temp%10;
			temp/=10;
		}       
	} 
	if (c != cc) return false;

	t/=7;

	char sxv[8], lxv[8];
	for (int i=7; i>=0; i--)
	{
		lxv[i] = t%10;
		t/=10;
	}
	sxv[0] = lxv[0], sxv[7] = lxv[1], sxv[1] = lxv[2], sxv[6] = lxv[3];
	sxv[2] = lxv[4], sxv[5] = lxv[5], sxv[3] = lxv[6], sxv[4] = lxv[7];

	year = sxv[0]*1000+sxv[1]*100+sxv[2]*10+sxv[3];
	month = sxv[4]*10+sxv[5];
	day = sxv[6]*10+sxv[7];
	if ((year>=2015)&&(month>0)&&(month<13)&&(day>0)&&(day<32))
		return true;
	else
		return false;
}

/*
void CenterAndRadiusAlgorithm(vector<osg::Vec3d> composlist,float &resultx,float &resulty,float &resultz,float &resultr)
{
	// Tol1：第一级公差， Tol2：第二级公差， StepS：搜索步长， ValMax：搜索的最大范围
	double Tol1=0.01,Tol2=0.1,StepS=0.01,ValMax=1000;
	int totalcount=500,choosecount=100;

	int icount=4,i=0;
	int ratecount=0;
	double Point[5][3];
	double result[128][5];
	int irand,s[5];
	int trnd=0;

	long endtime,starttime=(long)time(NULL);

	while(ratecount<choosecount)
	{		
		srand((long)time(NULL)+trnd);		
		s[1]=rand()%totalcount;
		s[2]=rand()%totalcount;
		s[3]=rand()%totalcount;
		s[4]=rand()%totalcount;
		trnd=rand()%totalcount;

		for (i=1;i<=4;i++)
		{
			irand=s[i];
			Point[i][0]=composlist[irand].x();
			Point[i][1]=composlist[irand].y();
			Point[i][2]=composlist[irand].z();
		}
		int Wflag=0;

		double Vsqrt;

		double x,y,z,xx,yy,zz;
		x=(Point[1][0]+Point[2][0])/2;
		y=(Point[1][1]+Point[2][1])/2;
		z=(Point[1][2]+Point[2][2])/2;
		xx=x;
		yy=y;
		zz=z;

		//计算单位法向量
		long double Ta,Tb,Tc;
		Ta=Point[2][1]-Point[1][1];
		Tb=Point[1][0]-Point[2][0];
		Tc=0;
		Vsqrt=sqrt(Ta*Ta+Tb*Tb);
		if (Vsqrt==0)
		{
			Ta=1;
			Tb=0;
		}
		else
		{
			Ta=Ta/Vsqrt;
			Tb=Tb/Vsqrt;
		}	

		double check1,check2,check3,check4;

		//根据三点确定三角形，计算外心，误差小于一级公差
		int Sflag=0;
		while(Sflag==0)
		{
			check1=sqrt((x-Point[1][0])*(x-Point[1][0])+(y-Point[1][1])*(y-Point[1][1])+(z-Point[1][2])*(z-Point[1][2]));
			check2=sqrt((x-Point[2][0])*(x-Point[2][0])+(y-Point[2][1])*(y-Point[2][1])+(z-Point[2][2])*(z-Point[2][2]));
			check3=sqrt((x-Point[3][0])*(x-Point[3][0])+(y-Point[3][1])*(y-Point[3][1])+(z-Point[3][2])*(z-Point[3][2]));
			if (fabs(check3-check2)+fabs(check3-check1)<Tol1)
			{
				Sflag=1;
			} 
			else
			{
				x=x+Ta*StepS;
				y=y+Tb*StepS;
			}
			check1=sqrt((xx-Point[1][0])*(xx-Point[1][0])+(yy-Point[1][1])*(yy-Point[1][1])+(zz-Point[1][2])*(zz-Point[1][2]));
			check2=sqrt((xx-Point[2][0])*(xx-Point[2][0])+(yy-Point[2][1])*(yy-Point[2][1])+(zz-Point[2][2])*(zz-Point[2][2]));
			check3=sqrt((xx-Point[3][0])*(xx-Point[3][0])+(yy-Point[3][1])*(yy-Point[3][1])+(zz-Point[3][2])*(zz-Point[3][2]));
			if (fabs(check3-check2)+fabs(check3-check1)<Tol1)
			{
				Sflag=2;
			} 
			else
			{
				xx=xx-Ta*StepS;
				yy=yy-Tb*StepS;
			}
			if (fabs(x)>ValMax || fabs(y)>ValMax || fabs(z)>ValMax || fabs(xx)>ValMax || fabs(yy)>ValMax || fabs(zz)>ValMax)
			{
				Sflag=3;
			}
		}

		if (Sflag==1)
		{
			xx=x;
			yy=y;
			zz=z;
		}
		if (Sflag==2)
		{
			x=xx;
			y=yy;
			z=zz;
		}

		//判断是否超越搜索的最大范围
		if (Sflag!=3)
		{
			check1=(x-Point[1][0])*(x-Point[1][0])+(y-Point[1][1])*(y-Point[1][1])+(z-Point[1][2])*(z-Point[1][2]);
			check2=(x-Point[2][0])*(x-Point[2][0])+(y-Point[2][1])*(y-Point[2][1])+(z-Point[2][2])*(z-Point[2][2]);
			check3=(x-Point[3][0])*(x-Point[3][0])+(y-Point[3][1])*(y-Point[3][1])+(z-Point[3][2])*(z-Point[3][2]);	

			//计算单位法向量
			long double Da,Db,Dc;
			Da=(Point[1][1]-Point[2][1])*(Point[2][2]-Point[3][2])-(Point[2][1]-Point[3][1])*(Point[1][2]-Point[2][2]);
			Db=(Point[1][2]-Point[2][2])*(Point[2][0]-Point[3][0])-(Point[2][2]-Point[3][2])*(Point[1][0]-Point[2][0]);
			Dc=(Point[1][0]-Point[2][0])*(Point[2][1]-Point[3][1])-(Point[2][0]-Point[3][0])*(Point[1][1]-Point[2][1]);
			Vsqrt=sqrt(Da*Da+Db*Db+Dc*Dc);
			int Bflag=0;
			if (Vsqrt==0)
			{
				Bflag=1;
			}
			else
			{
				Da=Da/Vsqrt;
				Db=Db/Vsqrt;
				Dc=Dc/Vsqrt;
			}			

			//根据四点确定圆球，计算球心，误差小于二级公差
			int Vflag=0;	
			while(Vflag==0 && Bflag==0)
			{		
				check1=sqrt((x-Point[1][0])*(x-Point[1][0])+(y-Point[1][1])*(y-Point[1][1])+(z-Point[1][2])*(z-Point[1][2]));
				check2=sqrt((x-Point[2][0])*(x-Point[2][0])+(y-Point[2][1])*(y-Point[2][1])+(z-Point[2][2])*(z-Point[2][2]));
				check3=sqrt((x-Point[3][0])*(x-Point[3][0])+(y-Point[3][1])*(y-Point[3][1])+(z-Point[3][2])*(z-Point[3][2]));
				check4=sqrt((x-Point[4][0])*(x-Point[4][0])+(y-Point[4][1])*(y-Point[4][1])+(z-Point[4][2])*(z-Point[4][2]));
				if (fabs(check4-check3)+fabs(check4-check2)+fabs(check4-check1)<Tol2)
				{
					Vflag=1;
				} 
				else
				{
					x=x+Da*StepS;
					y=y+Db*StepS;
					z=z+Dc*StepS;
				}
				check1=sqrt((xx-Point[1][0])*(xx-Point[1][0])+(yy-Point[1][1])*(yy-Point[1][1])+(zz-Point[1][2])*(zz-Point[1][2]));
				check2=sqrt((xx-Point[2][0])*(xx-Point[2][0])+(yy-Point[2][1])*(yy-Point[2][1])+(zz-Point[2][2])*(zz-Point[2][2]));
				check3=sqrt((xx-Point[3][0])*(xx-Point[3][0])+(yy-Point[3][1])*(yy-Point[3][1])+(zz-Point[3][2])*(zz-Point[3][2]));
				check4=sqrt((xx-Point[4][0])*(xx-Point[4][0])+(yy-Point[4][1])*(yy-Point[4][1])+(zz-Point[4][2])*(zz-Point[4][2]));
				if (fabs(check4-check3)+fabs(check4-check2)+fabs(check4-check1)<Tol2)
				{
					Vflag=2;
				} 
				else
				{
					xx=xx-Da*StepS;
					yy=yy-Db*StepS;
					zz=zz-Dc*StepS;
				}	
				if (fabs(x)>ValMax || fabs(y)>ValMax || fabs(z)>ValMax || fabs(xx)>ValMax || fabs(yy)>ValMax || fabs(zz)>ValMax)
				{
					Vflag=3;
				}
			}

			if (Vflag==2)
			{
				x=xx;
				y=yy;
				z=zz;
			}

			//判断是否超越搜索的最大范围
			if (Vflag!=3)
			{
				result[ratecount][1]=x;
				result[ratecount][2]=y;
				result[ratecount][3]=z;
				double Trate=0,Trad=0,Ttmp=0;
				int listsize=composlist.size();
				//
				//listsize=totalcount;
				double rd[512];
				for (i=0;i<listsize;i++)
				{
					rd[i]=sqrt((x-composlist[i].x())*(x-composlist[i].x())+(y-composlist[i].y())*(y-composlist[i].y())+(z-composlist[i].z())*(z-composlist[i].z()));
					Ttmp=Ttmp+rd[i];
				}
				Trad=Ttmp/listsize;
				for (i=0;i<listsize;i++)
				{
					Trate=Trate+fabs((rd[i]-Trad)/Trad);
				}
				Trate=Trate/listsize;
				result[ratecount][0]=Trate;
				result[ratecount][4]=Trad;
				ratecount++;
			}
		}
		//
		endtime=(long)time(NULL);
		if (endtime-starttime>5)
		{
			//break;
		}
	}

	int key=0;
	for (icount=1;icount<ratecount;icount++)
	{
		if (result[icount][0]<result[key][0])
		{
			key=icount;
		}
	}

	resultx=result[key][1];
	resulty=result[key][2];
	resultz=result[key][3];
	resultr=result[key][4];

	LPCSTR pppp="C:\\spiritLazioLog\\data.txt";
	HANDLE h = CreateFile(pppp,FILE_ALL_ACCESS,FILE_SHARE_READ|FILE_SHARE_WRITE,NULL,OPEN_ALWAYS,FILE_ATTRIBUTE_NORMAL,NULL);
	if ( INVALID_HANDLE_VALUE != h )
	{
		DWORD num;
		SetFilePointer (h,0,NULL,FILE_END);
		//spiritLazioLog
		//sprintf_s(szInfo,1024,"%04d-%02d-%02d, %02d:%02d:%02d   %s\r\n",oT.wYear,oT.wMonth,oT.wDay,oT.wHour,oT.wMinute,oT.wSecond,pMsg); 
		char szInfo[1024];

		sprintf_s(szInfo,1024,"误差率=%f,x=%f,y=%f,z=%f,r=%f  \r\n",result[key][0],resultx,resulty,resultz,resultr);
		
		WriteFile(h,szInfo,strlen(szInfo),&num,NULL);
		
		CloseHandle(h);
	}	

	return;
}
//*/

//void qcommon::ModifyData(int iBodyPart,osg::Quat* _segment,bool bShortMode,bool bFollowRoot,bool bRootRepair,bool bThumbRepair,float* fParamYaw)
void qcommon::ModifyData(int iBodyPart,osg::Quat* _segment,const CalcSetting* pSetting)
{
	////夹心旋转 将整个机体坐标系做一个旋转(如：绕yaw=90的四元数旋转另一个四元数，则表示该四元数yaw不变，pitch=roll0、roll=-pitch0)
	double pi = 3.14159265358979;
	osg::Quat quat;
	int iHandID=10;
	double yawSum=0.0;
	double yaw_new=0 ,pitch_new=0,roll_new=0;
	static int iStep=0;
	iStep++;

	if(iBodyPart == 1)
	{			
		iHandID=14;
		if(fabs(pSetting->fParamYaw[iHandID]) > 0.0001)
		{
			quat = _segment[iHandID];
			double yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
			osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(fabs(yaw_new)*pSetting->fParamYaw[iHandID], 0, 0 ,0, 0, 0,4);

// 			if(iStep % 1000 == 0)
// 			{
// 				quat = _segment[14];
// 				yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 				pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 				roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 				PRINTF("***** 14 ****yaw = %f,pitch = %f,roll = %f",yaw_new,pitch_new,roll_new);
// 				PRINTF("----------------------------------paramYaw14 %f",fabs(yaw_new)*pSetting->fParamYaw[iHandID]);
// 			}
			//_segment[iHandID] = quat3.inverse() * _segment[iHandID]* quat3;
			_segment[iHandID] =  _segment[iHandID] * quat3;//quat3 * _segment[iHandID] 效果是一样的
		}
		for (int i=24; i<39; i++)
		{
			//wxgtest 通过配置来修正航向不准的问题，等比
			if(fabs(pSetting->fParamYaw[i]) > 0.0001)
			{
				quat = _segment[i];
				double yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
				osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(fabs(yaw_new)*pSetting->fParamYaw[i], 0, 0 ,0, 0, 0,4);
				//_segment[i] = quat3.inverse() * _segment[i]* quat3;
				_segment[i] = quat3 * _segment[i];
				_segment[i] = _segment[i] * quat3;
			}
		}
		//算出各指与手背航向的平均差值 中指+食指+无名
		//if(pSetting->bRootRepair)
		{
			for(int j=1;j<10;j++)
			{
				if(j%3!=0)
				{
					quat = _segment[iHandID]*_segment[26+j].inverse();
					double yaw_delta = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
					yawSum += yaw_delta;
				}
			}
		}
		float fThumbYaw=0.0;
		for (int i=24; i<39; i++)
		{
			if((i-24)%3 == 0 && i>26)//wxg20160901 
			{
				float fRatio=0.5,fCorrectRatio=0;

				for(int m=0;m<3;m++)
				{
					if(m==2)
					{
						//if(m_bFollowRoot  && i!=36)//小指初始是斜的，有问题
						if(pSetting->bFollowRoot)//小指初始是斜的，有问题
						{ 
							//各指与手背同pitch 同时用各指与手背航向的平均差值(yawSum/6.0)来修正航向
							quat = _segment[iHandID]*_segment[i+m-2].inverse();
							double pitch_delta = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
							osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yawSum/6.0, pitch_delta,0 ,4);
							_segment[i+m-2] = quat3 * _segment[i+m-2];
							_segment[i+m-1] = quat3 * _segment[i+m-1];

							
							//与指根或手背同航向
							quat = _segment[iHandID]*_segment[i+m-2].inverse();
							double yaw_delta = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	

							//pitch_delta = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
							pitch_delta = 0.0;

							quat = _segment[i+m-1]*_segment[i+m-2].inverse();
							yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 			
							//pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
							roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
							// 
							// 	
							fCorrectRatio = 1-fabs(roll_new)/90 ;
							if(fCorrectRatio<0) fCorrectRatio=0;
							yaw_delta = fCorrectRatio*(yaw_new/2) + (1-fCorrectRatio)*yaw_delta;//根据弯曲程度，设置根节点（两节点的中间值）和手背的权重，决定向谁靠拢

							if(i!=36)
								fThumbYaw += yaw_delta;//大姆指根据这个平均数做修正
							else
							{
								osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,fThumbYaw/3.0, 0,0 ,4);
								_segment[24] = quat3  * _segment[24];
								_segment[25] = quat3  * _segment[25];
								_segment[26] = quat3  * _segment[26];
							}

							//修改指根的pitch是为了握拳时各指尖向掌心靠拢
							if(i == 36)
								pitch_delta-=30;
							else if(i==33)
								pitch_delta-=15;
							else if(i==27)
								pitch_delta+=15;
							quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yaw_delta, pitch_delta,0 ,4);//航向取两个节点的平均值20161128 
							_segment[i+m-2] = quat3 * _segment[i+m-2];

							//第2节参考第1节
							quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,0, 0,roll_new ,4);
							_segment[i+m-1] = quat3 * _segment[i+m-2];
						}

						//估算第三指节
						if(pSetting->bShortMode)
						{
							quat = quatutils::slerp(_segment[i+m-2], _segment[i+m-1],0.5);
							quat  = quat*_segment[i+m-2].inverse();//
							roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
							fRatio = fabs(roll_new)/40;//40第2节与第一节的弯曲度
							if(i == 36)
							{
								if(fRatio>1)  fRatio=1;
							}
							else
							{
								if(fRatio>0.8) fRatio=0.8;
							}

							quat = quatutils::slerp(_segment[i+m-2], _segment[i+m-1],fRatio);		
							quat  = quat*_segment[i+m-2].inverse();

							_segment[i+m] = quat*_segment[i+m-1];
						}
					}
					else
					{
					}
				}
				//手背节点纠正wxg20161118
// 				if( i == 30 )
// 				{
// 					//test
// 					if(iStep % 1000 == 0)
// 					{
// 						quat = _segment[i];
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 						PRINTF("***** 30 ****yaw = %f,pitch = %f,roll = %f",yaw_new,pitch_new,roll_new);
// 						quat = _segment[14];
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 						PRINTF("***** 14 ****yaw = %f,pitch = %f,roll = %f",yaw_new,pitch_new,roll_new);
// 						quat = _segment[i]*_segment[14].inverse();
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 						PRINTF("***** 30/14 ****yaw = %f,pitch = %f,roll = %f",yaw_new,pitch_new,roll_new);
// 
// 						osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,90, 0, 0,4);
// 						quat = _segment[14];
// 						quat = quat3 * quat;
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 						PRINTF("***** 14(+90度) ****yaw = %f,pitch = %f,roll = %f",yaw_new,pitch_new,roll_new);
// 
// 						quat = _segment[14];
// 						quat =  quat * quat3;
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 						PRINTF("***** 14(+90度)*反 ****yaw = %f,pitch = %f,roll = %f",yaw_new,pitch_new,roll_new);
// 
// 						quat = _segment[14];
// 						quat = quat3.inverse() * quat * quat3;
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 						PRINTF("***** 14(+90度 夹心) ****yaw = %f,pitch = %f,roll = %f",yaw_new,pitch_new,roll_new);
// 
// 						quat = _segment[14];
// 						quat = quat3 * quat * quat3.inverse();
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 						PRINTF("***** 14(+90度 反夹心) ****yaw = %f,pitch = %f,roll = %f",yaw_new,pitch_new,roll_new);							
// 					}
// 				}
			}
			else if(i <= 26)
			{
				//wxgtest 通过配置来修正航向不准的问题，等比
				int m=0;

				//修正各指与手背航向的平均差值(yawSum/6.0)
				osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yawSum/6.0, 0,0 ,4);
				_segment[i] = quat3 * _segment[i];	
			}
		}

		//整体偏移与动捕一致
		if(fabs(pSetting->fLeftHandYawOffset)>0.0001 )
		{
			float fHandYawOffset = pSetting->fLeftHandYawOffset;//pSetting->fHandYawOffset
			osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,fHandYawOffset, 0,0 ,4);//test 90
			for (int i=24; i<39; i++)
			{
				_segment[i] = _segment[i] * quat3;	
			}
			_segment[14] = _segment[14] * quat3;
		}

	}

	if(iBodyPart == 2)
	{
// 		for (int i=42; i<57; i++)
// 		{
// 			//wxg[2016-9-3]add屏蔽未端节点
// 			if(pSetting->bShortMode)
// 			{
// 				if((i-42)%3 == 0 && i>44)//wxg20160901 test
// 				{
// 					float fRatio=0.5,fCorrectRatio=0;
// 					for(int m=0;m<3;m++)
// 					{
// 						if(m==2)
// 						{
// 							//if(m_bFollowRoot && i!=54)
// 							if(pSetting->bFollowRoot)
// 							{
// 								quat = _segment[48]*_segment[i+m-2].inverse();
// 								double pitch_delta = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 								{
// 									osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,0, pitch_delta,0 ,4);
// 									_segment[i+m-2] = quat3 * _segment[i+m-2];//*quat3.inverse();//各指与中指同pitch 
// 									_segment[i+m-1] = quat3 * _segment[i+m-1];//*quat3.inverse();
// 
// 								}
// 								//else
// 								{
// 									quat = _segment[48]*_segment[i+m-2].inverse();
// 									double yaw_delta = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
// 									pitch_delta = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 
// 									quat = _segment[i+m-1]*_segment[i+m-2].inverse();
// 									yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 			
// 									//pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 									roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 									// 
// 									// 	
// 									fCorrectRatio = 1-fabs(roll_new)/90 ;
// 									if(fCorrectRatio<0) fCorrectRatio=0;
// 									yaw_delta = fCorrectRatio*(yaw_new/2) + (1-fCorrectRatio)*yaw_delta;
// 									//osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(yaw_new*fCorrectRatio, pitch_new*fCorrectRatio, 0 ,0, 0, 0,4);
// 									//_segment[i+m-1] = quat3 * _segment[i+m-1];
// 									if(i == 54)
// 										pitch_delta+=30;
// 									else if(i==51)
// 										pitch_delta+=15;
// 									else if(i==45)
// 										pitch_delta-=15;
// 									//osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yaw_delta, pitch_delta,0 ,4);//航向取两个节点的平均值20161128
// 									osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(yaw_delta, pitch_delta, 0 ,0, 0,0 ,4);//航向取两个节点的平均值
// 									_segment[i+m-2] = quat3 * _segment[i+m-2];
// 									quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,0, 0,roll_new ,4);
// 									_segment[i+m-1] = quat3 * _segment[i+m-2];
// 								}
// 							}
// 
// 							quat = quatutils::slerp(_segment[i+m-2], _segment[i+m-1],fRatio);
// 							quat  = quat*_segment[i+m-2].inverse();//
// 							roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 							////_segment[i+m] = quat*_segment[i+m-1]*quat.inverse();
// 							fRatio = fabs(roll_new)/40;
// 							if(fRatio>0.8) fRatio=0.8;
// 							quat = quatutils::slerp(_segment[i+m-2], _segment[i+m-1],fRatio);
// 							quat  = quat*_segment[i+m-2].inverse();
// 
// 							_segment[i+m] = quat*_segment[i+m-1];
// 
// 						}
// 						else
// 						{
// 							//wxgtest 通过配置来修正航向不准的问题，等比
// 							//int m=0;
// 							if(fabs(pSetting->fParamYaw[i+m]) > 0.0001)
// 							{
// 								quat = _segment[i+m];
// 								yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
// 								osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(fabs(yaw_new)*pSetting->fParamYaw[i+m], 0, 0 ,0, 0, 0,4);
// 								_segment[i+m] = quat3.inverse() * _segment[i+m]* quat3;
// 							}
// 						}
// 					}
// 					//手背节点纠正wxg20161118
// 					if(i == 48 && pSetting->bRootRepair)
// 					{
// 						quat = _segment[i]*_segment[10].inverse();
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						//yaw_new += GetYawOffset(false);
// 						osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yaw_new, pitch_new, 0,4);
// 						_segment[10] = quat3 * _segment[10];
// 
// 						//大姆指也要同步
// 						_segment[42] = quat3*_segment[42];
// 						_segment[43] = quat3*_segment[43];
// 						_segment[44] = quat3*_segment[44];
// 					}
// 				}
// 				else if(i<=44)
// 				{
// 					//wxgtest 通过配置来修正航向不准的问题，等比
// 					int m=0;
// 					if(fabs(pSetting->fParamYaw[i+m]) > 0.0001)
// 					{
// 						quat = _segment[i+m];
// 						double yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
// 						osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(fabs(yaw_new)*pSetting->fParamYaw[i+m], 0, 0 ,0, 0, 0,4);
// 						_segment[i+m] = quat3.inverse() * _segment[i+m]* quat3;
// 					}
// 					if(i==44 && pSetting->bThumbRepair)
// 					{
// 						double yaw_new=0 ,pitch_new=0,roll_new=0;
// 
// 						quat = _segment[42]*_segment[10].inverse();
// 						yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 
// 						pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
// 						roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
// 						double fYawRatio=0,fRollRatio=0;
// 						if(roll_new+15 > 0 )
// 						{
// 							// 											if(pitch_new>0)
// 							// 												fRollRatio = roll_new/(15+pitch_new*3);
// 							// 											else
// 							fRollRatio = (roll_new+15)/30;
// 							if(fRollRatio > 1)
// 								fRollRatio = 1;
// 						}
// 						//osg::Quat quat4 = quatutils::CalcQuatBetweenTwoEulor( 0, 0, 0 ,45*fYawRatio-45*fRollRatio,0, 45*fRollRatio,4);
// 						osg::Quat quat4 = quatutils::CalcQuatBetweenTwoEulor( 0, 0, 0 ,-30*fRollRatio,10*fRollRatio, 20*fRollRatio,4);
// 						//_segment[i-1] = quat4 * quat3 * _segment[i-1];
// 						_segment[42] = quat4  * _segment[42];
// 						_segment[43] = quat4  * _segment[43];
// 						_segment[44] = quat4  * _segment[44];
// 						// 										if(m_bDetailLog && istep%50 == 0)
// 						// 											PRINTF("==============i = 42/10 yaw=%f,pitch = %f, roll=%f  ",yaw_new,pitch_new,roll_new);
// 
// 					}
// 
// 				}
// 			}////////////////////////////////////
// 		}					
		iHandID=10;
		if(fabs(pSetting->fParamYaw[iHandID]) > 0.0001)
		{
			quat = _segment[iHandID];
			double yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
			osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(fabs(yaw_new)*pSetting->fParamYaw[iHandID], 0, 0 ,0, 0, 0,4);

			//_segment[iHandID] = quat3.inverse() * _segment[iHandID]* quat3;
			_segment[iHandID] =  _segment[iHandID] * quat3;//quat3 * _segment[iHandID] 效果是一样的
		}
		for (int i=42; i<57; i++)
		{
			//wxgtest 通过配置来修正航向不准的问题，等比
			if(fabs(pSetting->fParamYaw[i]) > 0.0001)
			{
				quat = _segment[i];
				double yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
				osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(fabs(yaw_new)*pSetting->fParamYaw[i], 0, 0 ,0, 0, 0,4);
				//_segment[i] = quat3.inverse() * _segment[i]* quat3;
				_segment[i] = quat3 * _segment[i];
				_segment[i] = _segment[i] * quat3;
			}
		}
		//算出各指与手背航向的平均差值 中指+食指+无名
		//if(pSetting->bRootRepair)
		{
			for(int j=1;j<10;j++)
			{
				if(j%3!=0)
				{
					quat = _segment[iHandID]*_segment[44+j].inverse();
					double yaw_delta = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	
					yawSum += yaw_delta;
				}
			}
		}
		float fThumbYaw=0.0;
		for (int i=42; i<57; i++)
		{
			if((i-42)%3 == 0 && i>44)//wxg20160901 
			{
				float fRatio=0.5,fCorrectRatio=0;

				for(int m=0;m<3;m++)
				{
					if(m==2)
					{
						//if(m_bFollowRoot  && i!=36)//小指初始是斜的，有问题
						if(pSetting->bFollowRoot)//小指初始是斜的，有问题
						{ 
							//各指与手背同pitch 同时用各指与手背航向的平均差值(yawSum/6.0)来修正航向
							quat = _segment[iHandID]*_segment[i+m-2].inverse();
							double pitch_delta = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
							osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yawSum/6.0, pitch_delta,0 ,4);
							//osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(yawSum/6.0, pitch_delta, 0 ,0, 0,0 ,4);
							_segment[i+m-2] = quat3 * _segment[i+m-2];
							_segment[i+m-1] = quat3 * _segment[i+m-1];


							//与指根或手背同航向
							quat = _segment[iHandID]*_segment[i+m-2].inverse();
							double yaw_delta = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 	

							//pitch_delta = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
							pitch_delta = 0.0;

							quat = _segment[i+m-1]*_segment[i+m-2].inverse();
							yaw_new = atan2(2 * quat.z() * quat.x() + 2 * quat.w() * quat.y(), -2 * quat.y()*quat.y() - 2 * quat.x() * quat.x() + 1)* 180/pi; // yaw 			
							//pitch_new = asin(-2 * quat.y() * quat.z() + 2 * quat.w() * quat.x())* 180/pi; // pitch
							roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
							// 
							// 	
							fCorrectRatio = 1-fabs(roll_new)/90 ;
							if(fCorrectRatio<0) fCorrectRatio=0;
							yaw_delta = fCorrectRatio*(yaw_new/2) + (1-fCorrectRatio)*yaw_delta;//根据弯曲程度，设置根节点（两节点的中间值）和手背的权重，决定向谁靠拢

							if(i!=54)
								fThumbYaw += yaw_delta;//大姆指根据这个平均数做修正
							else
							{
								osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,fThumbYaw/3.0, 0,0 ,4);
								_segment[42] = quat3  * _segment[42];
								_segment[43] = quat3  * _segment[43];
								_segment[44] = quat3  * _segment[44];
							}

							//修改指根的pitch是为了握拳时各指尖向掌心靠拢
							if(i == 54)
								pitch_delta-=30;
							else if(i==51)
								pitch_delta-=15;
							else if(i==45)
								pitch_delta+=15;
							//quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yaw_delta, pitch_delta,0 ,4);//航向取两个节点的平均值20161128 
							quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yaw_delta, pitch_delta,0 ,4);//航向取两个节点的平均值
							_segment[i+m-2] = quat3 * _segment[i+m-2];

							//第2节参考第1节
							quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,0, 0,roll_new ,4);
							_segment[i+m-1] = quat3 * _segment[i+m-2];
						}

						//估算第三指节
						if(pSetting->bShortMode)
						{
							quat = quatutils::slerp(_segment[i+m-2], _segment[i+m-1],0.5);
							quat  = quat*_segment[i+m-2].inverse();//
							roll_new = atan2(2 * quat.x() * quat.y() + 2 * quat.w() * quat.z(), -2 * quat.x() * quat.x() - 2 * quat.z() * quat.z() + 1)* 180/pi; // roll 握拳
							fRatio = fabs(roll_new)/40;//40第2节与第一节的弯曲度
							if(i == 54)
							{
								if(fRatio>1)  fRatio=1;
							}
							else
							{
								if(fRatio>0.8) fRatio=0.8;
							}

							quat = quatutils::slerp(_segment[i+m-2], _segment[i+m-1],fRatio);		
							quat  = quat*_segment[i+m-2].inverse();

							_segment[i+m] = quat*_segment[i+m-1];
						}
					}
					else
					{
					}
				}
				//手背节点纠正wxg20161118
				if( i == 48 )
				{

				}
			}
			else if(i <= 44)
			{
				//wxgtest 通过配置来修正航向不准的问题，等比
				int m=0;

				//修正各指与手背航向的平均差值(yawSum/6.0)
				osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,yawSum/6.0, 0,0 ,4);
				_segment[i] = quat3 * _segment[i];	
			}
		}

		//整体偏移与动捕一致
		if(fabs(pSetting->fRightHandYawOffset)>0.0001 )
		{
			float fHandYawOffset = pSetting->fRightHandYawOffset;//pSetting->fHandYawOffset
			osg::Quat quat3 = quatutils::CalcQuatBetweenTwoEulor(0, 0, 0 ,fHandYawOffset, 0,0 ,4);//test 90
			for (int i=42; i<57; i++)
			{
				_segment[i] = _segment[i] * quat3;	
			}
			_segment[10] = _segment[10] * quat3;
		}
	}
}