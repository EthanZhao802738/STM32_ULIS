#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "pt.h"
#include "tasks.h"
#include "constants.h"


extern bool spi_tx_pi_Cplt;
extern SPI_HandleTypeDef hspi1;
extern uint16_t gDataBufferComplete[IMGSIZE];
extern uint16_t gDataBufferTxComplete[IMGSIZE];

PT_THREAD( SendToPi_task(struct pt *pt_ptr))
{
	PT_BEGIN(pt_ptr);
	
	while(1)
	{
		
		PT_WAIT_UNTIL(pt_ptr,spi_tx_pi_Cplt == true);
		gDataBufferComplete[ IMGSIZE - 6] = gDataBufferComplete[ IMGSIZE - 1];
		HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)gDataBufferComplete, (IMGSIZE - 5));
		spi_tx_pi_Cplt = false;
		
	}
	
	
	PT_END(pt_ptr);
}