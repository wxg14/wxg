//#include"stdafx.h"
#include "StreamAnalyzer.h"
#include "CoolLog.h"
//#include "consts.h"
//#include "GlobalData.h"
#include <QString>

//extern int GFrameCount;
//extern int GValidateFrameCount;
#define ACCPARAMCOUNT 9

StreamAnalyzer::StreamAnalyzer()
{
	for(int iIndex=0;iIndex<BODY_PART_COUNT;iIndex++)
	{
		m_count[iIndex] = 0;
		m_bA5flag[iIndex]  = false;
		m_b5Aflag[iIndex]  = false;
		m_b3Aflag[iIndex]  = false;


		m_size[iIndex]  = 0;
	}

	m_intval = 0;

	m_TotalReceivedBytes = 0;
	m_receiveflag = false;
	m_legalflag = false;
	m_chipidInfront = true;

	m_bReadID = true;
	//是否是LPMS_B姿态模块
	m_protocol = 0;
	m_bDetailLog = false;
}

StreamAnalyzer::~StreamAnalyzer()
{

}

void StreamAnalyzer::setReceiveFlag(bool avalue)
{
    m_receiveflag = avalue;
}

void StreamAnalyzer::chipidInfront(bool bvalue)
{
	m_chipidInfront = bvalue;
}

void StreamAnalyzer::setProtocol(byte protocol)
{
	m_protocol = protocol;
}

int StreamAnalyzer::getFrameCount()
{
    return m_frameCount;
}

int StreamAnalyzer::getValidateFrameCount()
{
	return m_validateFrameCount;
}

bool StreamAnalyzer::analyzeDataWithAHRSprotocol(const char* pdata, int nsize,int iIndex)//数据解析 a5 5a不一定在最前面，头尾相连循环存储
{
	m_TotalReceivedBytes += nsize;
	//PRINTF("StreamAnalyzer::newData received %d bytes", nsize);
	bool bvalid = false;
	for(int i=0;i<nsize;i++){

		// 		if (true == m_bReadID)
		// 		{
		// 			m_chipID = pdata[i];
		// 			m_bReadID = false;
		//  		}

		m_intval++;
		byte c = byte(pdata[i]);

		if ((!m_bA5flag[iIndex]) && (!m_b5Aflag[iIndex]))
		{
			if (c == 0xA5)
			{
				m_bA5flag[iIndex] = true;
			}
			else
				m_bA5flag[iIndex] = false;
		}
		else if (m_bA5flag[iIndex])
		{
			if (c == 0x5A)
				m_b5Aflag[iIndex] = true;
			else 
				m_b5Aflag[iIndex] = false;
			m_bA5flag[iIndex] = false;
		}
		else if (m_b5Aflag[iIndex])
		{
			if(c == 0xA5 && i+1<nsize && (unsigned char)pdata[i+1]==0x5A && (i+m_size[iIndex]-m_count[iIndex])<nsize && (unsigned char)pdata[i+m_size[iIndex]-m_count[iIndex]] != 0xA5)//add by wxg 两个udp包之间可能丢数据
			{
				if(m_bDetailLog)
				{
					char szData[1024]={0};
					char szTemp[4]={0};
					int iLen=m_count[iIndex];
					if(iLen>128) iLen = 128;
					for(int m=0;m<iLen;m++)
					{
						sprintf(szTemp,"%02X ",(byte)m_packetBytes[iIndex][m]);
						strcat(szData,szTemp);
					}
					sprintf(szTemp,"%02X ",(byte)pdata[i]);
					strcat(szData,szTemp);
					sprintf(szTemp,"%02X ",(byte)pdata[i+1]);
					strcat(szData,szTemp);
					PRINTF("!!!data lost(index=%d):%s .m_size=%d,cur=%d,%02X",iIndex,szData,m_size[iIndex],m_count[iIndex],(unsigned char)pdata[i+m_size[iIndex]-m_count[iIndex]]);
				}
				m_count[iIndex] = 0;
				m_b5Aflag[iIndex] = false;
				continue;
			}

			if (m_count[iIndex] == 0 )//&& c>=6 && c<=MAXPACKETSIZE
			{
				m_size[iIndex] = c;
				//if (m_b5Aflag)
				{ 
					m_packetBytes[iIndex][m_count[iIndex]++] = c;
				}
			}
			else if (m_count[iIndex] != 0 && m_count[iIndex] < m_size[iIndex])
			{//modify by wxg (m_count < m_size-1)更改协议
// 				if (m_count[iIndex] >= MAXPACKETSIZE)//disabled by wxg20160617 有了前面的A5 5A的实时判断就不会出问题
// 				{
// 					PRINTF("!!!m_count(%d) >= MAXPACKETSIZE",m_count[iIndex]);
// 					m_count[iIndex] = 0;
// 					m_b5Aflag[iIndex] = false;
// 					break;
// 				}

				m_packetBytes[iIndex][m_count[iIndex]++] = c;
				if(m_count[iIndex] == m_size[iIndex]  && m_size[iIndex] != 0xFF)
				{
					if (!checkOut(m_packetBytes[iIndex]))
						emit message(QString("check error!"), QString("critical"));
					else 
					{
						if(m_packetBytes[iIndex][0] == 0x83 && m_packetBytes[iIndex][1] == 0xC3)//wxg[2016-6-17]add 收到校准参数
						{
							char szData[512]={0};
							char szTemp[4]={0};
							for(int j=0;j<(unsigned char)m_packetBytes[iIndex][0];j++)
							{
								sprintf(szTemp,"%02X ",(unsigned char)m_packetBytes[iIndex][j]);
								strcat(szData,szTemp);
							}
							PRINTF("Mag Answer received(len=%d):%s",m_packetBytes[iIndex][0],szData);
							//if((unsigned char)pdata[i+3] == 0xC3)//有可能数据不全
							{
								int iCount=0;
								short cx,cy,cz,radius;
								int iRealLen=(unsigned char)m_packetBytes[iIndex][0];
								float fParam[4];
								for(int j=0;j<iRealLen-3;j+=8)
								{
									cx = ((unsigned char)m_packetBytes[iIndex][j+3] << 8) | (unsigned char)m_packetBytes[iIndex][j+2];
									cy = ((unsigned char)m_packetBytes[iIndex][j+5] << 8) | (unsigned char)m_packetBytes[iIndex][j+4];
									cz = ((unsigned char)m_packetBytes[iIndex][j+7] << 8) | (unsigned char)m_packetBytes[iIndex][j+6];
									radius = ((unsigned char)m_packetBytes[iIndex][j+9] << 8) | (unsigned char)m_packetBytes[iIndex][j+8];
									fParam[0] = (float)cx;
									fParam[1] = (float)cy;
									fParam[2] = (float)cz;
									fParam[3] = (float)radius;

									//emit UpdateComOffset(iCount,cx,cy,cz,radius);
									emit UpdateComOffset(iCount,fParam,4);
									PRINTF("%d : cx = %d, cy = %d, cz = %d,radius = %d\n",iCount++,cx,cy,cz,radius);
								} 

							}
						}
						else if(m_packetBytes[iIndex][1] == 0xC9)//新磁力计
						{
							PRINTF("收到新磁力计校准数据...");

							int iCount=0;
							float fPAcc[ACCPARAMCOUNT];
							char szMsg[256]={0};
							int iRealLen= m_packetBytes[iIndex][3]*256+m_packetBytes[iIndex][2]-1;//扣除校验
							for(int j=4;j<iRealLen;j+=36)
							{
								for(int k=0;k<ACCPARAMCOUNT;k++)
								{
									char* p=(char*)(fPAcc+k);
									p[0]=m_packetBytes[iIndex][j+4*k];
									p[1]=m_packetBytes[iIndex][j+1+4*k];
									p[2]=m_packetBytes[iIndex][j+2+4*k];
									p[3]=m_packetBytes[iIndex][j+3+4*k];
									//fPAcc[k] = (float)(((unsigned char)m_packetBytes[iIndex][j+3+4*k] << 24) | (unsigned char)m_packetBytes[iIndex][j+2+4*k] << 16 | (unsigned char)m_packetBytes[iIndex][j+1+4*k] << 8 | (unsigned char)m_packetBytes[iIndex][j+4*k]);
								}
								emit UpdateComOffset(iCount,fPAcc,ACCPARAMCOUNT);

								//PRINTF("%d : p1 = %f, p2 = %f, p3 = %f,p4 = %f,p5 = %f,p6 = %f,p7 = %f,p8 = %f,p9 = %f \n",iCount++,fPAcc[0],fPAcc[1],fPAcc[2],fPAcc[3],fPAcc[4],fPAcc[5],fPAcc[6],fPAcc[7],fPAcc[8]);
								sprintf(szMsg,"Mag %d : p1 = %f, p2 = %f, p3 = %f,p4 = %f,p5 = %f,p6 = %f,p7 = %f,p8 = %f,p9 = %f\r\n",iCount++,fPAcc[0],fPAcc[1],fPAcc[2],fPAcc[3],fPAcc[4],fPAcc[5],fPAcc[6],fPAcc[7],fPAcc[8]);
								PRINTF(szMsg);
							}	
						}
						else if(m_packetBytes[iIndex][1] == 0xC5)//收到加速度计 太长了
						{
// 							int iCount=0;
// 							float fPAcc[ACCPARAMCOUNT];
// 							int iRealLen= m_packetBytes[iIndex][0]; // m_packetBytes[iIndex][3]*256+m_packetBytes[iIndex][2]-1;//扣除校验
// // 							for(int j=4;j<iRealLen;j+=36)
// // 							{
// // 								for(int k=0;k<ACCPARAMCOUNT;k++)
// // 									fPAcc[k] = ((unsigned char)m_packetBytes[iIndex][j+3+4*k] << 24) | (unsigned char)m_packetBytes[iIndex][j+2+4*k] << 16 | (unsigned char)m_packetBytes[iIndex][j+1+4*k] << 8 || (unsigned char)m_packetBytes[iIndex][j+4*k];
// // 								emit UpdateAccParam(iCount,fPAcc,ACCPARAMCOUNT);
// // 								PRINTF("%d : p1 = %f, p2 = %f, p3 = %f,p4 = %f,p5 = %f,p6 = %f,p7 = %f,p8 = %f,p9 = %f \n",iCount++,fPAcc[0],fPAcc[1],fPAcc[2],fPAcc[3],fPAcc[4],fPAcc[5],fPAcc[6],fPAcc[7],fPAcc[8]);
// // 
// // 							}
// 							iCount = m_packetBytes[iIndex][2];
// 							for(int k=0;k<ACCPARAMCOUNT;k++)
// 								fPAcc[k] = ((unsigned char)m_packetBytes[iIndex][6+4*k] << 24) | (unsigned char)m_packetBytes[iIndex][5+4*k] << 16 | (unsigned char)m_packetBytes[iIndex][4+4*k] << 8 || (unsigned char)m_packetBytes[iIndex][3+4*k];
// 							emit UpdateAccParam(iCount,fPAcc,ACCPARAMCOUNT);
// 							PRINTF("%d : p1 = %f, p2 = %f, p3 = %f,p4 = %f,p5 = %f,p6 = %f,p7 = %f,p8 = %f,p9 = %f \n",iCount,fPAcc[0],fPAcc[1],fPAcc[2],fPAcc[3],fPAcc[4],fPAcc[5],fPAcc[6],fPAcc[7],fPAcc[8]);
							PRINTF("收到加速度校准数据...");

							int iCount=0;
							float fPAcc[ACCPARAMCOUNT];
							char szMsg[256]={0};
							int iRealLen= m_packetBytes[iIndex][3]*256+m_packetBytes[iIndex][2]-1;//扣除校验
							for(int j=4;j<iRealLen;j+=36)
							{
								for(int k=0;k<ACCPARAMCOUNT;k++)
								{
									char* p=(char*)(fPAcc+k);
									p[0]=m_packetBytes[iIndex][j+4*k];
									p[1]=m_packetBytes[iIndex][j+1+4*k];
									p[2]=m_packetBytes[iIndex][j+2+4*k];
									p[3]=m_packetBytes[iIndex][j+3+4*k];
									//fPAcc[k] = (float)(((unsigned char)m_packetBytes[iIndex][j+3+4*k] << 24) | (unsigned char)m_packetBytes[iIndex][j+2+4*k] << 16 | (unsigned char)m_packetBytes[iIndex][j+1+4*k] << 8 | (unsigned char)m_packetBytes[iIndex][j+4*k]);
								}
								emit UpdateAccParam(iCount,fPAcc,ACCPARAMCOUNT);

								//PRINTF("%d : p1 = %f, p2 = %f, p3 = %f,p4 = %f,p5 = %f,p6 = %f,p7 = %f,p8 = %f,p9 = %f \n",iCount++,fPAcc[0],fPAcc[1],fPAcc[2],fPAcc[3],fPAcc[4],fPAcc[5],fPAcc[6],fPAcc[7],fPAcc[8]);
								sprintf(szMsg,"Acc %d : p1 = %f, p2 = %f, p3 = %f,p4 = %f,p5 = %f,p6 = %f,p7 = %f,p8 = %f,p9 = %f\r\n",iCount++,fPAcc[0],fPAcc[1],fPAcc[2],fPAcc[3],fPAcc[4],fPAcc[5],fPAcc[6],fPAcc[7],fPAcc[8]);
								PRINTF(szMsg);
							}	
						}
						else if(m_packetBytes[iIndex][0] == 0x63 && m_packetBytes[iIndex][1] == 0xC6)//收到陀螺
						{
							char szData[512]={0};
							char szTemp[4]={0};
							for(int j=0;j<(unsigned char)m_packetBytes[iIndex][0];j++)
							{
								sprintf(szTemp,"%02X ",(unsigned char)m_packetBytes[iIndex][j]);
								strcat(szData,szTemp);
							}
							PRINTF("Gyro Answer received(len=%d):%s",m_packetBytes[iIndex][0],szData);
							//if((unsigned char)pdata[i+3] == 0xC3)//有可能数据不全
							{
								int iCount=0;
								short cx,cy,cz;
								int iRealLen=(unsigned char)m_packetBytes[iIndex][0];
								for(int j=0;j<iRealLen-3;j+=6)
								{
									cx = ((unsigned char)m_packetBytes[iIndex][j+3] << 8) | (unsigned char)m_packetBytes[iIndex][j+2];
									cy = ((unsigned char)m_packetBytes[iIndex][j+5] << 8) | (unsigned char)m_packetBytes[iIndex][j+4];
									cz = ((unsigned char)m_packetBytes[iIndex][j+7] << 8) | (unsigned char)m_packetBytes[iIndex][j+6];
									//radius = ((unsigned char)m_packetBytes[iIndex][j+9] << 8) | (unsigned char)m_packetBytes[iIndex][j+8];
									emit UpdateGyroParam(iCount,cx,cy,cz);
									PRINTF("%d : gx = %d, gy = %d, gz = %d\n",iCount++,cx,cy,cz);
								} 

							}							
						}
						else if((m_packetBytes[iIndex][1] & 0x0F) == 8)//器件报错
						{
							char szData[512]={0};
							char szTemp[4]={0};
							if(m_packetBytes[iIndex][2] == 1)//部分节点断线
							{
								emit message(QString("sensor error!"), QString("critical"));
								for(int a=0;a<m_packetBytes[iIndex][0]-4;a++)
								{
									byte btChipID = m_packetBytes[iIndex][a+3];
									byte nChipId;
									if(btChipID <= 17)
									{
										nChipId = btChipID;
									}
									else if(btChipID <= 146)
									{
										nChipId = btChipID - 129 + 24;//24~41
										if(btChipID -129 >= 15)//左手腕
										{
											nChipId = 1;
										}
									}
									else if(btChipID <= 164)
									{
										nChipId = btChipID-147 + 42;//42~59
										if(btChipID -147 >= 15)//右手腕
										{
											nChipId = 9;
										}
									}
									sprintf(szTemp,"%d ",nChipId);
									strcat(szData,szTemp);
								}
								PRINTF("部分节点断线:chipid = %s",szData);
							}
							else if(m_packetBytes[iIndex][2] == 2)//全部节点失联：根节点断线 或 数据线短路
							{
								emit message(QString("device error!"), QString("critical"));
								PRINTF("无法读取数据:根节点断线 或 数据线短路!");
							}
							else if(m_packetBytes[iIndex][2] == 3)//磁力计异常
							{
								emit message(QString("mag error!"), QString("critical"));
								for(int a=0;a<m_packetBytes[iIndex][0]-4;a++)
								{
									byte btChipID = m_packetBytes[iIndex][a+3];
									byte nChipId;
									if(btChipID <= 17)
									{
										nChipId = btChipID;
									}
									else if(btChipID <= 146)
									{
										nChipId = btChipID - 129 + 24;//24~41
										if(btChipID -129 >= 15)//左手腕
										{
											nChipId = 1;
										}
									}
									else if(btChipID <= 164)
									{
										nChipId = btChipID-147 + 42;//42~59
										if(btChipID -147 >= 15)//右手腕
										{
											nChipId = 9;
										}
									}
									sprintf(szTemp,"%d ",nChipId);
									strcat(szData,szTemp);
								}
								PRINTF("磁力计异常:chipid = %s",szData);
							}
						}
						else if((m_packetBytes[iIndex][1] & 0x0F) < 8)
						{
							byte nchipid;
							bool bIgnore=false;
							if (m_chipidInfront) 
							{
								//nchipid = m_packetBytes[1] & ((1<<5) - 1);//modify by wxg
								byte btChipID = m_packetBytes[iIndex][2];
								static bool bBodyLeftHand=false;
								static bool bBodyRightHand=false;
								if(btChipID <= 17)
								{
									nchipid = btChipID;//1~17
									if(bBodyLeftHand && nchipid == 1)
										bIgnore = true;
									else if(bBodyRightHand && nchipid == 9)
										bIgnore = true;
								}
								else if(btChipID <= 146)
								{
									nchipid = btChipID - 129 + 24;//24~41
									if(btChipID -129 >= 15)//左手腕
									{
										if(!bBodyLeftHand) bBodyLeftHand=true;
										nchipid = 1;
									}
								}
								else if(btChipID <= 164)
								{
									nchipid = btChipID-147 + 42;//42~59
									if(btChipID -147 >= 15)//右手腕
									{
										if(!bBodyRightHand) bBodyRightHand = true;
										nchipid = 9;
									}
								}
							}
							else
								nchipid= m_packetBytes[iIndex][m_size[iIndex] - 3]; //倒数第三个字节放ChipID
							//gchipframecount[nchipid]++;
							//ggframecount[nchipid]++;
							//PRINTF("chipID: %d, framecount: %d", nchipid, gchipframecount[nchipid]);
							//if (nchipid ==4)
							//{
							if(!bIgnore)
							{
								for (int j=0; j<m_count[iIndex]; j++)
									m_cachebuffer[nchipid][j] = m_packetBytes[iIndex][j];
								emit newPacket(char(nchipid), (char*)(m_cachebuffer[nchipid]));//新数据通知主程序wxg1
							}

							//}


							//emit newPacket(m_chipID, (char*)(m_cachebuffer[m_chipID]));
							//  					if (false == m_bReadID)
							//  					{				

							//  						m_bReadID = true;
							//  					}
							//emit message(QString("check error!"), QString("critical"));
							bvalid = true;
						}

					}
					m_intval = 0;
					m_count[iIndex] = 0;
					m_b5Aflag[iIndex] = false;
				}
				else if(m_count[iIndex] == m_size[iIndex] && m_size[iIndex] == 0xFF)//加速度标记数据
				{
					m_size[iIndex] = m_packetBytes[iIndex][3]*256+m_packetBytes[iIndex][2];
				}
			}
			else
			{
				if(m_bDetailLog)
				{
					PRINTF("!!!invalid data format(index=%d):len=%d,MAXPACKETSIZE=%d,curlen=%d",iIndex,c,MAXPACKETSIZE,m_count[iIndex]);
					char szData[512]={0};
					{
						char szTemp[4]={0};
						for(int m=i-2;m<min(i+126,nsize);m++)
						{
							sprintf(szTemp,"%02X ",(byte)pdata[m]);
							strcat(szData,szTemp);
						}
						PRINTF("!!!invalid data:%s .",szData);
					}
				}
				m_count[iIndex] = 0;
				m_b5Aflag[iIndex] = false;
			}
		}
	}
	if (m_TotalReceivedBytes > 200)
		m_receiveflag = true;
	return bvalid;
}
bool StreamAnalyzer::analyzeDataWithLPMSprotocol(const char* pdata, int nsize)
{
	m_TotalReceivedBytes += nsize;
	//PRINTF("StreamAnalyzer::newData received %d bytes", nsize);
	bool bvalid = false;

	//disabled by wxg
// 	unsigned short ncommand; 
// 	for(int i=0;i<nsize;i++){
// 		m_intval++;
// 		byte c = byte(pdata[i]);
// 		if (!m_b3Aflag)
// 		{
// 			if (c == 0x3A)
// 			{
// 				m_b3Aflag = true;
// 				m_count = 0;
// 			}
// 		}
// 		else if (m_b3Aflag)
// 		{
// 			if (m_count == 0)
// 			{
// 				m_chipID = c;
// 				m_packetBytes[m_count++] = c;
// 			}
// 			else if (m_count == 1){
// 				m_packetBytes[m_count++] = c;
// 			}
// 			else if (m_count == 2)
// 			{
// 				ncommand = c;
// 				m_packetBytes[m_count++] = c;
// 			}
// 			else if (m_count == 3)
// 			{
// 				ncommand = (c<<8) + ncommand;
// 				m_packetBytes[m_count++] = c;
// 			}
// 			else if (m_count == 4)
// 			{
// 				m_size = c;
// 				m_packetBytes[m_count++] = c;
// 			}
// 			else if (m_count == 5)
// 			{
//                 m_size = (c<<8) + m_size;
// 				m_packetBytes[m_count++] = c;
// 				if (m_size != 80)
// 				{
// 					m_count = 0;
// 					m_b3Aflag = false;
// 					continue;
// 				}
// 			}
// 			else if ((m_count - 6) < (m_size + 2))
// 			{
// 				if (m_count >= MAXPACKETSIZE)
// 				{
// 					m_count = 0;
// 					m_b3Aflag = false;
// 					continue;
// 				}
// 				m_packetBytes[m_count++] = c;
// 			}
// 			else{
// 				if (!checkOut(m_packetBytes))
// 					emit message(QString("check error!"), QString("critical"));
// 				else
// 				{
// 					if (m_chipID > -1 && m_chipID < MAXCHIPCOUNT)
// 					{
// 						for (int j=0; j<m_count; j++)
// 							m_cachebuffer[m_chipID][j] = m_packetBytes[j];
// 						emit newPacket(char(m_chipID), (char*)(m_cachebuffer[m_chipID]));
// 						bvalid = true;
// 					}
// 
// 					
// 				}
// 				m_intval = 0;
// 				m_count = 0;
// 				m_b3Aflag = false;
// 			}
// 		}
// 	}
// 	if (m_TotalReceivedBytes > 200)
// 		m_receiveflag = true;
	return bvalid;
}
//int ggframecount[40] = {0};
bool StreamAnalyzer::newData(const char* pdata, int nsize,int iIndex)//UDP接收回调触发wxg8
{
	if (m_protocol == CAHRSPROTOCOL)
	{
		static int iMax=0;
		if(iIndex<100 && iMax<iIndex)
			iMax = iIndex;
		else if(iIndex > 100)
			iIndex = iIndex-100+iMax;
		return analyzeDataWithAHRSprotocol(pdata, nsize,iIndex);
	}
	else if (m_protocol == CLPMSPROTOCOL)
	{
        return analyzeDataWithLPMSprotocol(pdata, nsize);
	}
	else
	{
		//assert(false);
		//do nothings;
	}
	return true;
}

bool StreamAnalyzer::newData(const QByteArray& newBytes,int iIndex)
{
	return newData(newBytes.data(), newBytes.length(),iIndex);
}


bool StreamAnalyzer::isMPU6050Port()
{
	while (!m_receiveflag);
	return m_legalflag;
}
//校验接收到的数据包是否合法
bool StreamAnalyzer::checkOut(byte* packetBytes)
{
	if (m_protocol == CAHRSPROTOCOL)
	{
		short framesize;//帧字节数，不包括开始的两个字节

		m_frameCount++;
		framesize = packetBytes[0];
		if(framesize ==0xFF)
			framesize = packetBytes[3]*256+packetBytes[2];
		unsigned int temp=0;
		//for (int i=0; i<framesize - 2; i++)
		for (int i=0; i<framesize - 1; i++)//modify by wxg 更改协议格式
		{
			temp+=byte(packetBytes[i]);
		}
		bool rslt = (temp%256 == byte(packetBytes[framesize - 1]));//wxg framesize-2
		if (rslt)
		{
			m_validateFrameCount++;
			m_legalflag = true;
			//PRINTF(CCHECKOUTSUCCEED);
		}
		else if(m_bDetailLog)
		{
			char szData[256]={0};
			//if(framesize*3<256)
			{
				char szTemp[4]={0};
				for(int i=0;i<framesize;i++)
				{
					sprintf(szTemp,"%02X ",packetBytes[i]);
					strcat(szData,szTemp);
				}
			}
			PRINTF("!!!check error:%s --- checksum=%d vs %d size=%d",szData,temp%256,byte(packetBytes[framesize - 1]),framesize);
		}
		return rslt;
	}
	else if (m_protocol == CLPMSPROTOCOL)
	{
		byte framesize;
		m_frameCount++;
		framesize = (packetBytes[5]<<8) + packetBytes[4] + 6;
		unsigned int temp=0;
		for (int i=0; i<framesize; i++)
		{
			temp+=byte(packetBytes[i]);
		}
		bool rslt = (temp%65536 == (unsigned short)((packetBytes[framesize+1] <<8) + packetBytes[framesize]));
		if (rslt)
		{
			m_validateFrameCount++;
			m_legalflag = true;
			//PRINTF(CCHECKOUTSUCCEED);
		}
		//else
		//	PRINTF(CCHECKOUTFAILED);
        return rslt;
	}
	else
	{
		return false;
	}

}