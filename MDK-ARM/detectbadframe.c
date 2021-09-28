#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>


#include "stm32f4xx_hal.h"
#include "pt.h"
#include "tasks.h"
#include "constants.h"


extern uint16_t gDataBufferTx[IMGSIZE];
extern uint16_t gDataBufferComplete[IMGSIZE];
extern uint16_t gDataBufferBadLineDeted[IMGSIZE];
extern uint8_t g_BadLine;
extern bool b_ReveiveData;
extern bool b_uvcSending;

uint8_t m_pFrameStraightLineMap[IMGSIZE - 3];
uint8_t m_pFrameHorizontalLineMap[IMGSIZE - 3];
uint8_t m_pFrameLineMapL[IMGSIZE - 3];
uint8_t m_pFrameLineMapR[IMGSIZE - 3];
uint16_t m_pFrameLast[IMGSIZE - 3];
uint8_t m_pFrameDiffMap[IMGSIZE - 3];
uint8_t lineMapWhite[80] = {0};
uint8_t lineMapBlack[80] = {0};

void DetectBadLine(uint16_t *frameBuffer,int type);
void DetectBadPixel(uint16_t *frameBuffer);

uint16_t getNeighborAvg(const int x, const int y, uint16_t *frameBuffer, uint8_t *pDiffMap);
bool getAvailableNeighbor(const int x, const int y, const int incX, const int incY, uint16_t *frameBuffer, uint8_t *pDiffMap,uint16_t *value);
bool g_bStartDetectBadLine = true;

PT_THREAD( DetectBadLine_task(struct pt *pt_ptr))
{
	PT_BEGIN(pt_ptr);
	
	static uint32_t detectBandLineTime = 0;
	
	detectBandLineTime = HAL_GetTick();
	
	while(1)
	{
		PT_WAIT_UNTIL(pt_ptr,((HAL_GetTick() - detectBandLineTime) > ONESECONDE) && b_ReveiveData == false && b_uvcSending == false && g_bStartDetectBadLine == true);
		
		memcpy(gDataBufferBadLineDeted,gDataBufferComplete,IMGSIZE * sizeof(uint16_t));
		

		/*DetectBadLine(&gDataBufferBadLineDeted[2],White);
		DetectBadPixel(&gDataBufferTx[2]);*/
		detectBandLineTime = HAL_GetTick();
	}
	
	PT_END(pt_ptr);
}


void DetectBadLine(uint16_t *frameBuffer,int type)
{
	
		switch(type){
			
			case White:
			{
				static uint8_t scanWhiteLineSection_whd,scanWhiteLineSection_wft;
				
				if(g_BadLine == WhiteLineAllSection)
				{
					scanWhiteLineSection_whd = 0;
					scanWhiteLineSection_wft = 80;
				}
		
			
				for(int i = scanWhiteLineSection_whd; i < scanWhiteLineSection_wft; ++i)
				{
		        if(lineMapWhite[i] != 0)
						{
							continue;
						}
						
						int iLine = i * 80;
						for(int j = 1; j < 80 - 1; ++j)
						{
								if (((frameBuffer[iLine + j - 1] - frameBuffer[iLine + j]) > 30))
										m_pFrameLineMapL[iLine + j] = (m_pFrameLineMapL[iLine + j] >= BLIND_ACC_COUNT ) ? BLIND_ACC_COUNT  : (m_pFrameLineMapL[iLine + j]+=1);
								else
										m_pFrameLineMapL[iLine + j] = (m_pFrameLineMapL[iLine + j] > 0) ? --m_pFrameLineMapL[iLine + j] : 0;

								if (((frameBuffer[iLine + j + 1] - frameBuffer[iLine + j]) > 30))
										m_pFrameLineMapR[iLine + j] = (m_pFrameLineMapR[iLine + j] >= BLIND_ACC_COUNT ) ? BLIND_ACC_COUNT  : (m_pFrameLineMapR[iLine + j]+=1);
								else
										m_pFrameLineMapR[iLine + j] = (m_pFrameLineMapR[iLine + j] > 0) ? --m_pFrameLineMapR[iLine + j] : 0;
						}
				}
		
				for(int j = 1; j < 80 - 1; ++j)
				{
						int iAccL = 0;
						int iAccR = 0;
						for(int i = scanWhiteLineSection_whd; i < scanWhiteLineSection_wft; ++i)
						{
								if (m_pFrameLineMapL[i * 80 + j] >= BLIND_ACC_COUNT)
										++iAccL;
								if (m_pFrameLineMapR[i * 80 + j] >= BLIND_ACC_COUNT)
										++iAccR;
						}
						if (iAccL > 80 * 0.4 && iAccR > 80 * 0.4)
						{
								//single bad line
								lineMapWhite[j] = SingleBadLine;
						}
						else if (iAccL > 80 * 0.4)
						{
								//maybe multi bad line: left side
								lineMapWhite[j] = MultiBadLineLeft;
						}
						else if (iAccR > 80 * 0.4)
						{
								//maybe multi bad line: right side
								lineMapWhite[j] = MultiBadLineRight;
						}
				}
				if(g_BadLine == DynamicDetectBadLine)
				{
					scanWhiteLineSection_whd += 10;
					scanWhiteLineSection_wft += 10;
					if(scanWhiteLineSection_wft > 80)
					{
						scanWhiteLineSection_whd = 0;
						scanWhiteLineSection_wft = 10;
					}
				}
				
				
				break;
			}
				
			case Black:
			{
				static uint8_t scanWhiteLineSection_bhd,scanWhiteLineSection_bft;
				if(g_BadLine == BlackLineAllSection)
				{
					scanWhiteLineSection_bhd = 0;
					scanWhiteLineSection_bft = 80;
				}
				
				for(int i = scanWhiteLineSection_bhd; i < scanWhiteLineSection_bft; ++i)
				{
					if(lineMapBlack[i] != 0)
					{
						continue;
					}
					
					int iLine = i * 80;
					for(int j = 1; j < 80 - 1; ++j)
					{
						if (((frameBuffer[iLine + j ] - frameBuffer[iLine + j - 1]) > 20))
								m_pFrameLineMapL[iLine + j] = (m_pFrameLineMapL[iLine + j] >= BLIND_ACC_COUNT ) ? BLIND_ACC_COUNT  : (m_pFrameLineMapL[iLine + j]+=1);
						else
								m_pFrameLineMapL[iLine + j] = (m_pFrameLineMapL[iLine + j] > 0) ? --m_pFrameLineMapL[iLine + j] : 0;

						if (((frameBuffer[iLine + j ] - frameBuffer[iLine + j +1]) > 20))
								m_pFrameLineMapR[iLine + j] = (m_pFrameLineMapR[iLine + j] >= BLIND_ACC_COUNT ) ? BLIND_ACC_COUNT  : (m_pFrameLineMapR[iLine + j]+=1);
						else
								m_pFrameLineMapR[iLine + j] = (m_pFrameLineMapR[iLine + j] > 0) ? --m_pFrameLineMapR[iLine + j] : 0;
					}
				}
		
				for(int j = 1; j < 80 - 1; ++j)
				{
					int iAccL = 0;
					int iAccR = 0;
					for(int i = scanWhiteLineSection_bhd; i < scanWhiteLineSection_bft; ++i)
					{
							if (m_pFrameLineMapL[i * 80 + j] >= BLIND_ACC_COUNT)
								++iAccL;
							if (m_pFrameLineMapR[i * 80 + j] >= BLIND_ACC_COUNT)
								++iAccR;
					}
					if (iAccL > 80 * 0.4 && iAccR > 80 * 0.4)
					{
							//single bad line
							lineMapBlack[j] = SingleBadLine;
					}
					else if (iAccL > 80 * 0.4)
					{
							//maybe multi bad line: left side
							lineMapBlack[j] = MultiBadLineLeft;
					}
					else if (iAccR > 80 * 0.4)
					{
							//maybe multi bad line: right side
							lineMapBlack[j] = MultiBadLineRight;
					}
				}
				if(g_BadLine == DynamicDetectBadLine)
				{
					scanWhiteLineSection_bhd += 10;
					scanWhiteLineSection_bft += 10;
					if(scanWhiteLineSection_bft > 80)
					{
						scanWhiteLineSection_bhd = 0;
						scanWhiteLineSection_bft = 10;
					}
				}
				break;
			}
	}
		
	/*for(int i = 1; i < 80 - 1 ; i++)                  Horizontal line detect 
	{
		int horizontalline = i;
		for(int j = 0; j < 80; j++)
		{
			if(((frameBuffer[(horizontalline - 1) + (80 * j)] - frameBuffer[horizontalline + (80 * j)]) > 30) && (frameBuffer[(horizontalline + 1) + (80 * j)] - frameBuffer[horizontalline + (80 * j)]) > 30 )
			{
				m_pFrameHorizontalLineMap[horizontalline + (j * 80)] = (m_pFrameHorizontalLineMap[horizontalline + (j * 80)] >= BLIND_ACC_COUNT * 1.5) ? BLIND_ACC_COUNT * 1.5 : ++m_pFrameHorizontalLineMap[horizontalline + (j * 80)];
			}
			else
				m_pFrameHorizontalLineMap[horizontalline + (j * 80)] = (m_pFrameHorizontalLineMap[horizontalline + (j * 80)] > 0) ? --m_pFrameHorizontalLineMap[horizontalline + (j *80)] : 0;
		}
	}
	
	for(int i = 0; i < 80 ; i++)
	{
		int iAcc = 0;
		for(int j = 1; j < 80 - 1; j++)
		{
			if(m_pFrameHorizontalLineMap[j*80 + i] >= BLIND_ACC_COUNT)
			{
				iAcc++;
			}
		}
		
		if(iAcc > 80 * 0.4)
		{
			for(int j = 1; j < 80 - 1 ; j++)
			{
				frameBuffer[j * 80 + i] = (frameBuffer[j * 80 + i -1] + frameBuffer[j * 80 +i + 1]) / 2;
			}	
		}
	}*/
}


void FixBadLine(uint8_t* lineMap,uint16_t* frameBuffer)
{
		for(int j = 1; j < 80 - 1; ++j)
    {
        if (lineMap[j] == 1 && lineMap[j + 1] == 0)
        {
            //good luck. single bad line
            for(int i = 0; i < 80; ++i)
            {
							frameBuffer[i * 80 + j] = (frameBuffer[i * 80 + j - 1] + frameBuffer[i * 80 + j + 1]) / 2;
            }
        }
        else if(lineMap[j] == 1 || lineMap[j] == 2)
        {
					if (j + 1 < (80 - 1) && (lineMap[j + 1] == 3 || lineMap[j + 1] == 1))
          {
						int m = 2;
            if (j - 2 < 0)
							m = 1;

            int n = 2;
            if (j + 2 >= 80)
							n = 1;

            for(int i = 0; i < 80; ++i)
            {
							frameBuffer[i * 80 + j] = (frameBuffer[i * 80 + j - m] + frameBuffer[i * 80 + j - 1] + frameBuffer[i * 80 + j + n]) / 3;
              frameBuffer[i * 80 + j + 1] = (frameBuffer[i * 80 + j - 1] + frameBuffer[i * 80 + j + 1] + frameBuffer[i * 80 + j + n]) / 3;
            }
          }
        }
			}
}




void DetectBadPixel(uint16_t *frameBuffer)
{
	static bool bFirst = true;
	
	if(bFirst)
	{
		for(int i = 0; i < IMGSIZE - 3; i++)
			m_pFrameLast[i] = frameBuffer[i];
		bFirst = false;
		return;
	}
	
	for(int i = 0; i < 80*80; i++)
	{
		if(frameBuffer[i] - m_pFrameLast[i] == 0)
			m_pFrameDiffMap[i] = (m_pFrameDiffMap[i] >= BLIND_ACC_COUNT * 1.5) ? BLIND_ACC_COUNT * 1.5 : ++m_pFrameDiffMap[i];
		
		else
		{
			if((i - 1) >= 0 && (i + 1) < 80*80)
			{
				if (((frameBuffer[i] - frameBuffer[i - 1]) > 300 && (frameBuffer[i] - frameBuffer[i + 1]) > 300) || ((frameBuffer[i] - frameBuffer[i - 1]) < -300 && (frameBuffer[i] - frameBuffer[i + 1]) < -300))
          m_pFrameDiffMap[i] = (m_pFrameDiffMap[i] >= BLIND_ACC_COUNT * 1.5) ? BLIND_ACC_COUNT * 1.5 : BLIND_ACC_COUNT;
				else
					m_pFrameDiffMap[i] = (m_pFrameDiffMap[i] > 0) ? --m_pFrameDiffMap[i] : 0;
			}
			else
				m_pFrameDiffMap[i] = (m_pFrameDiffMap[i] > 0) ? --m_pFrameDiffMap[i] : 0;
		}
	}
	
	memcpy(m_pFrameLast,frameBuffer,IMGSIZE - 3);
	
		
	/*for(int i = 0; i < 80*80; i++)
	{
		if(m_pFrameDiffMap[i] >= BLIND_ACC_COUNT)
		{
			int iX = i % 80;
      int iY = i / 80;
      frameBuffer[i] = getNeighborAvg(iX, iY, frameBuffer, m_pFrameDiffMap);
		}
	}*/
}

void FixBadPixel(uint16_t *frameBuffer)
{
	for(int i = 0; i < 80*80; i++)
	{
		if(m_pFrameDiffMap[i] >= BLIND_ACC_COUNT)
		{
			int iX = i % 80;
			int iY = i / 80;
			frameBuffer[i] = getNeighborAvg(iX, iY, frameBuffer, m_pFrameDiffMap);	
		}
	}
}



uint16_t getNeighborAvg(const int x, const int y, uint16_t *frameBuffer, uint8_t *pDiffMap)
{
	uint32_t iRst = 0;
	uint16_t iValue;
	int iCount = 0;
	
	for(int i = -1; i <= 1; i++)
	{
		for(int j = -1; j <= 1; j++)
		{
			if(i == 0 && j == 0)
				continue;
			
			iValue = 0;
			bool bRst = getAvailableNeighbor(x, y, j, i, frameBuffer, pDiffMap,&iValue);
			
			if(bRst)
			{
				iRst += iValue;
        iCount++;				
			}
		}
	}
	
	if(iCount > 0)
		iRst /= iCount;
	else
		iRst = frameBuffer[ y * 80 + x];
	
	return iRst;
}

bool getAvailableNeighbor(const int x, const int y, const int incX, const int incY, uint16_t *frameBuffer, uint8_t *pDiffMap,uint16_t *value)
{
	bool bRst = false;
	int iNextX = x + incX;
	int iNextY = y + incY;
	
	
	if(iNextX < 0 || iNextX >= 80 || iNextY < 0 || iNextY >= 80)
		return false;
	
	if(pDiffMap[iNextY * 80 + iNextX] >= BLIND_ACC_COUNT)
	{
		bRst = getAvailableNeighbor(iNextX, iNextY, incX, incY, frameBuffer, pDiffMap,value);
	}
	else
	{
		bRst = true;
		*value = frameBuffer[iNextY * 80 + iNextX];	
	}
	return bRst;
}




