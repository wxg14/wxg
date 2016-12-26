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
	//�Ƿ���LPMS_B��̬ģ��
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

bool StreamAnalyzer::analyzeDataWithAHRSprotocol(const char* pdata, int nsize,int iIndex)//���ݽ��� a5 5a��һ������ǰ�棬ͷβ����ѭ���洢
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
			if(c == 0xA5 && i+1<nsize && (unsigned char)pdata[i+1]==0x5A && (i+m_size[iIndex]-m_count[iIndex])<nsize && (unsigned char)pdata[i+m_size[iIndex]-m_count[iIndex]] != 0xA5)//add by wxg ����udp��֮����ܶ�����
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
			{//modify by wxg (m_count < m_size-1)����Э��
// 				if (m_count[iIndex] >= MAXPACKETSIZE)//disabled by wxg20160617 ����ǰ���A5 5A��ʵʱ�жϾͲ��������
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
						if(m_packetBytes[iIndex][0] == 0x83 && m_packetBytes[iIndex][1] == 0xC3)//wxg[2016-6-17]add �յ�У׼����
						{
							char szData[512]={0};
							char szTemp[4]={0};
							for(int j=0;j<(unsigned char)m_packetBytes[iIndex][0];j++)
							{
								sprintf(szTemp,"%02X ",(unsigned char)m_packetBytes[iIndex][j]);
								strcat(szData,szTemp);
							}
							PRINTF("Mag Answer received(len=%d):%s",m_packetBytes[iIndex][0],szData);
							//if((unsigned char)pdata[i+3] == 0xC3)//�п������ݲ�ȫ
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
						else if(m_packetBytes[iIndex][1] == 0xC9)//�´�����
						{
							PRINTF("�յ��´�����У׼����...");

							int iCount=0;
							float fPAcc[ACCPARAMCOUNT];
							char szMsg[256]={0};
							int iRealLen= m_packetBytes[iIndex][3]*256+m_packetBytes[iIndex][2]-1;//�۳�У��
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
						else if(m_packetBytes[iIndex][1] == 0xC5)//�յ����ٶȼ� ̫����
						{
// 							int iCount=0;
// 							float fPAcc[ACCPARAMCOUNT];
// 							int iRealLen= m_packetBytes[iIndex][0]; // m_packetBytes[iIndex][3]*256+m_packetBytes[iIndex][2]-1;//�۳�У��
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
							PRINTF("�յ����ٶ�У׼����...");

							int iCount=0;
							float fPAcc[ACCPARAMCOUNT];
							char szMsg[256]={0};
							int iRealLen= m_packetBytes[iIndex][3]*256+m_packetBytes[iIndex][2]-1;//�۳�У��
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
						else if(m_packetBytes[iIndex][0] == 0x63 && m_packetBytes[iIndex][1] == 0xC6)//�յ�����
						{
							char szData[512]={0};
							char szTemp[4]={0};
							for(int j=0;j<(unsigned char)m_packetBytes[iIndex][0];j++)
							{
								sprintf(szTemp,"%02X ",(unsigned char)m_packetBytes[iIndex][j]);
								strcat(szData,szTemp);
							}
							PRINTF("Gyro Answer received(len=%d):%s",m_packetBytes[iIndex][0],szData);
							//if((unsigned char)pdata[i+3] == 0xC3)//�п������ݲ�ȫ
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
						else if((m_packetBytes[iIndex][1] & 0x0F) == 8)//��������
						{
							char szData[512]={0};
							char szTemp[4]={0};
							if(m_packetBytes[iIndex][2] == 1)//���ֽڵ����
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
										if(btChipID -129 >= 15)//������
										{
											nChipId = 1;
										}
									}
									else if(btChipID <= 164)
									{
										nChipId = btChipID-147 + 42;//42~59
										if(btChipID -147 >= 15)//������
										{
											nChipId = 9;
										}
									}
									sprintf(szTemp,"%d ",nChipId);
									strcat(szData,szTemp);
								}
								PRINTF("���ֽڵ����:chipid = %s",szData);
							}
							else if(m_packetBytes[iIndex][2] == 2)//ȫ���ڵ�ʧ�������ڵ���� �� �����߶�·
							{
								emit message(QString("device error!"), QString("critical"));
								PRINTF("�޷���ȡ����:���ڵ���� �� �����߶�·!");
							}
							else if(m_packetBytes[iIndex][2] == 3)//�������쳣
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
										if(btChipID -129 >= 15)//������
										{
											nChipId = 1;
										}
									}
									else if(btChipID <= 164)
									{
										nChipId = btChipID-147 + 42;//42~59
										if(btChipID -147 >= 15)//������
										{
											nChipId = 9;
										}
									}
									sprintf(szTemp,"%d ",nChipId);
									strcat(szData,szTemp);
								}
								PRINTF("�������쳣:chipid = %s",szData);
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
									if(btChipID -129 >= 15)//������
									{
										if(!bBodyLeftHand) bBodyLeftHand=true;
										nchipid = 1;
									}
								}
								else if(btChipID <= 164)
								{
									nchipid = btChipID-147 + 42;//42~59
									if(btChipID -147 >= 15)//������
									{
										if(!bBodyRightHand) bBodyRightHand = true;
										nchipid = 9;
									}
								}
							}
							else
								nchipid= m_packetBytes[iIndex][m_size[iIndex] - 3]; //�����������ֽڷ�ChipID
							//gchipframecount[nchipid]++;
							//ggframecount[nchipid]++;
							//PRINTF("chipID: %d, framecount: %d", nchipid, gchipframecount[nchipid]);
							//if (nchipid ==4)
							//{
							if(!bIgnore)
							{
								for (int j=0; j<m_count[iIndex]; j++)
									m_cachebuffer[nchipid][j] = m_packetBytes[iIndex][j];
								emit newPacket(char(nchipid), (char*)(m_cachebuffer[nchipid]));//������֪ͨ������wxg1
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
				else if(m_count[iIndex] == m_size[iIndex] && m_size[iIndex] == 0xFF)//���ٶȱ������
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
bool StreamAnalyzer::newData(const char* pdata, int nsize,int iIndex)//UDP���ջص�����wxg8
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
//У����յ������ݰ��Ƿ�Ϸ�
bool StreamAnalyzer::checkOut(byte* packetBytes)
{
	if (m_protocol == CAHRSPROTOCOL)
	{
		short framesize;//֡�ֽ�������������ʼ�������ֽ�

		m_frameCount++;
		framesize = packetBytes[0];
		if(framesize ==0xFF)
			framesize = packetBytes[3]*256+packetBytes[2];
		unsigned int temp=0;
		//for (int i=0; i<framesize - 2; i++)
		for (int i=0; i<framesize - 1; i++)//modify by wxg ����Э���ʽ
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