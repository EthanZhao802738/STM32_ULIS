#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "pt.h"
#include "tasks.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"
#include "constants.h"


extern volatile uint8_t g_uvc_stream_status;
extern uint8_t gRGBBuf[FRAME_LINE_LENGTH * IMAGE_NUM_LINES * 3];
extern uint16_t gDataBufferComplete[IMGSIZE];

bool b_uvcSending;
void TemperatureToRGB(uint8_t *dst,uint8_t *pBuf);

PT_THREAD( uvc_task(struct pt *pt_ptr))
{
	PT_BEGIN(pt_ptr);
	static uint16_t count = 0, i, idx = 0;

	static uint8_t uvc_header[2] = { 2, 0 };
	static uint32_t uvc_xmit_row = 0, uvc_xmit_plane = 0, uvc_xmit_seg = 0;
	static uint8_t packet[VIDEO_PACKET_SIZE_MAX];
	static int image_num_segments;
	static uint32_t transferring_timer = 0;
	
	image_num_segments = 1;
	uvc_xmit_row = 0;
	uvc_xmit_plane = 0;
	
	while(1)
	{
		PT_WAIT_UNTIL(pt_ptr,((g_uvc_stream_status == 1 || g_uvc_stream_status == 2)));
		if(g_uvc_stream_status == 1)
		{
			uvc_header[0] = 2;
			uvc_header[1] = 0;
			UVC_Transmit_FS(uvc_header, 2);
			g_uvc_stream_status = 2;
			
		}
		else if(g_uvc_stream_status == 2)
		{
			b_uvcSending = true;
			TemperatureToRGB((uint8_t*)gRGBBuf,(uint8_t*)gDataBufferComplete);
			
			count = 0;
			packet[count++] = uvc_header[0];
			packet[count++] = uvc_header[1];
			
			while(uvc_xmit_row < IMAGE_NUM_LINES && count < VIDEO_PACKET_SIZE_ALT2)
			{
				uint8_t *rgbLine = &gRGBBuf[uvc_xmit_row * 80 *3];
				idx = 0;
				
				if(uvc_xmit_row == 80)
				{
					g_uvc_stream_status = 2;
				}
				
				for(int i = 0; i < FRAME_LINE_LENGTH; i++)
				{
					packet[count++] = rgbLine[idx++];
					packet[count++] = rgbLine[idx++];
					packet[count++] = rgbLine[idx++];
				}
				uvc_xmit_row++;
			}
			
			if(uvc_xmit_row == (IMAGE_NUM_LINES))					// End of the frame flag
			{
				if(++uvc_xmit_seg == image_num_segments)
				{
					packet[1] |= 0x2;
				}
			}					
			
			transferring_timer = HAL_GetTick();
			PT_WAIT_UNTIL(pt_ptr,(UVC_Transmit_FS(packet, count) != USBD_BUSY) || ((HAL_GetTick() - transferring_timer) > 20));
			
			
			if(packet[1] & 0x02)
			{
				uvc_header[1] ^= 1;
				uvc_xmit_seg = 0;
				b_uvcSending = false;
				PT_EXIT(pt_ptr);
			}
			else if(uvc_xmit_row == (IMAGE_NUM_LINES))
			{
				PT_EXIT(pt_ptr);
			}
		}
	}
	
	PT_END(pt_ptr);
}




void TemperatureToRGB(uint8_t *dst,uint8_t *pBuf)
{
	pBuf += 4;                 // ignore header
	
	for(int i = 0; i < 80*80; i++, dst += 3, pBuf += 2)
	{
		dst[0] = 128;
		dst[1] = pBuf[0];
		dst[2] = pBuf[1];
	}
	
	
	/* footer */ 							// 23 pixel footer + 5 version number
	
	for(int i = 0; i < 28; i++, dst += 3, pBuf += 2)
	{
		dst[0] = 128;
		dst[1] = pBuf[0];
		dst[2] = pBuf[1];	
	}
	
	
	
}
