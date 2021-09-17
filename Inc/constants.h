#ifndef CONSTANTS_H_
#define CONSTANTS_H_


//#define IMGSIZE							6423
//#define IMGSIZE							6426		//hamlet for header 0xFF00 0x00FF......footer 0x0FF0
#define IMGSIZE							(6431)		//hamlet for header 0xFF00 0x00FF......footer 0x0FF0 and 10 byte Version number
#define FRAME_LINE_LENGTH 	(80)
#define IMAGE_NUM_LINES 		(82)		//hamlet for usb uvc 80 + 2 (debug lines)
#define BUFFER_SIZE					(128)
#define ONESECONDE					1000

#define BLIND_ACC_COUNT 10

#define RTS_ENCRYPTION_MODE	0

#define GET_OFFSET_FROM_IMAGE() \
	(((gDataBufferTx[2] & 1) << 9) \
	| ((gDataBufferTx[3] & 1) << 8) \
	| ((gDataBufferTx[4] & 1) << 7) \
	| ((gDataBufferTx[5] & 1) << 6) \
	| ((gDataBufferTx[6] & 1) << 5) \
	| ((gDataBufferTx[7] & 1) << 4) \
	| ((gDataBufferTx[8] & 1) << 3) \
	| ((gDataBufferTx[9] & 1) << 2) \
	| ((gDataBufferTx[10] & 1) << 1) \
	| (gDataBufferTx[11] & 1))

#define SET_OFFSET_INTO_IMAGE(a) ({ \
    gDataBufferTx[2] = (gDataBufferTx[2] & 0xFFFE) | (((a) >> 9) & 1); \
    gDataBufferTx[3] = (gDataBufferTx[3] & 0xFFFE) | (((a) >> 8) & 1); \
    gDataBufferTx[4] = (gDataBufferTx[4] & 0xFFFE) | (((a) >> 7) & 1); \
    gDataBufferTx[5] = (gDataBufferTx[5] & 0xFFFE) | (((a) >> 6) & 1); \
    gDataBufferTx[6] = (gDataBufferTx[6] & 0xFFFE) | (((a) >> 5) & 1); \
    gDataBufferTx[7] = (gDataBufferTx[7] & 0xFFFE) | (((a) >> 4) & 1); \
    gDataBufferTx[8] = (gDataBufferTx[8] & 0xFFFE) | (((a) >> 3) & 1); \
    gDataBufferTx[9] = (gDataBufferTx[9] & 0xFFFE) | (((a) >> 2) & 1); \
    gDataBufferTx[10] = (gDataBufferTx[10] & 0xFFFE) | (((a) >> 1) & 1); \
    gDataBufferTx[11] = (gDataBufferTx[11] & 0xFFFE) | ((a) & 1); })

		
#define SingleBadLine 1
#define MultiBadLineLeft 2
#define MultiBadLineRight 3
#define OneSecond 35
#define WaitingTime 165   // 5 sec * 33(one frame per second)		
#define White 1
#define Black 2
#define WhiteLineAllSection 1
#define BlackLineAllSection 2
#define DynamicDetectBadLine 3
				
	
		
#endif
