#ifndef __USBD_UVC_LEPTON_XU_H
#define __USBD_UVC_LEPTON_XU_H

#include "usbd_uvc.h"

typedef enum tagVigilResult
{
   VIGIL_OK                            = 0,     /* Camera ok */
   VIGIL_COMM_OK                       = VIGIL_OK, /* Camera comm ok (same as LEP_OK) */

   VIGIL_ERROR                         = -1,    /* Camera general error */
   VIGIL_NOT_READY                     = -2,    /* Camera not ready error */
   VIGIL_RANGE_ERROR                   = -3,    /* Camera range error */
   VIGIL_CHECKSUM_ERROR                = -4,    /* Camera checksum error */
   VIGIL_BAD_ARG_POINTER_ERROR         = -5,    /* Camera Bad argument  error */
   VIGIL_DATA_SIZE_ERROR               = -6,    /* Camera byte count error */
   VIGIL_UNDEFINED_FUNCTION_ERROR      = -7,    /* Camera undefined function error */
   VIGIL_FUNCTION_NOT_SUPPORTED        = -8,    /* Camera function not yet supported error */
   VIGIL_DATA_OUT_OF_RANGE_ERROR       = -9,    /* Camera input DATA is out of valid range error */
   VIGIL_COMMAND_NOT_ALLOWED           = -11,   /* Camera unable to execute command due to current camera state */

   /* OTP access errors */
   VIGIL_OTP_WRITE_ERROR               = -15,   /*!< Camera OTP write error */
   VIGIL_OTP_READ_ERROR				    = -16,   /* double bit error detected (uncorrectible) */

   VIGIL_OTP_NOT_PROGRAMMED_ERROR      = -18,   /* Flag read as non-zero */

   /* I2C Errors */
   VIGIL_ERROR_I2C_BUS_NOT_READY       = -20,   /* I2C Bus Error - Bus Not Avaialble */
   VIGIL_ERROR_I2C_BUFFER_OVERFLOW     = -22,   /* I2C Bus Error - Buffer Overflow */
   VIGIL_ERROR_I2C_ARBITRATION_LOST    = -23,   /* I2C Bus Error - Bus Arbitration Lost */
   VIGIL_ERROR_I2C_BUS_ERROR           = -24,   /* I2C Bus Error - General Bus Error */
   VIGIL_ERROR_I2C_NACK_RECEIVED       = -25,   /* I2C Bus Error - NACK Received */
   VIGIL_ERROR_I2C_FAIL                = -26,   /* I2C Bus Error - General Failure */

   /* Processing Errors */
   VIGIL_DIV_ZERO_ERROR                = -80,   /* Attempted div by zero */

   /* Comm Errors */
   VIGIL_COMM_PORT_NOT_OPEN            = -101,  /* Comm port not open */
   VIGIL_COMM_INVALID_PORT_ERROR       = -102,  /* Comm port no such port error */
   VIGIL_COMM_RANGE_ERROR              = -103,  /* Comm port range error */
   VIGIL_ERROR_CREATING_COMM           = -104,  /* Error creating comm */
   VIGIL_ERROR_STARTING_COMM           = -105,  /* Error starting comm */
   VIGIL_ERROR_CLOSING_COMM            = -106,  /* Error closing comm */
   VIGIL_COMM_CHECKSUM_ERROR           = -107,  /* Comm checksum error */
   VIGIL_COMM_NO_DEV                   = -108,  /* No comm device */
   VIGIL_TIMEOUT_ERROR                 = -109,  /* Comm timeout error */
   VIGIL_COMM_ERROR_WRITING_COMM       = -110,  /* Error writing comm */
   VIGIL_COMM_ERROR_READING_COMM       = -111,  /* Error reading comm */
   VIGIL_COMM_COUNT_ERROR              = -112,  /* Comm byte count error */

   /* Other Errors */
   VIGIL_OPERATION_CANCELED            = -126,  /* Camera operation canceled */
   VIGIL_UNDEFINED_ERROR_CODE          = -127   /* Undefined error */

} VIGIL_RESULT;

typedef enum tagVigilFuncOffset
{
	VIGIL_FUNC_T						= 0,	/* Read module temperatures: Return digital 16bit value and floating point calculation in °C. */
	VIGIL_FUNC_C,								/* toggle (on/off) a Central Cross on the image, re-sending this command toggle this setting, canceling the display of the central cross. */
	VIGIL_FUNC_Y,								/* toggle On/Off the bad pixel correction */
	VIGIL_FUNC_D,								/* get Unique ID */
	VIGIL_FUNC_REBOOT,							/* reboot the module */
	VIGIL_FUNC_E, 								/* toggle the Experimental mode */
	VIGIL_FUNC_P,								/* toggle on/off a test pattern output */
	VIGIL_FUNC_N,								/* send displayed image through */
	VIGIL_FUNC_M,								/* color map */
	VIGIL_FUNC_END
}VIGIL_FUNC_OFFSET;

//uint16_t vc_terminal_id_to_module_base(VC_TERMINAL_ID entity_id);
//
//int8_t VC_LEP_GetAttribute (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf, uint16_t length);
//int8_t VC_LEP_SetAttribute (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf, uint16_t length);
//int8_t VC_LEP_GetAttributeLen (VC_TERMINAL_ID entity_id, uint16_t offset, uint16_t* pbuf);
//int8_t VC_LEP_GetMaxValue (VC_TERMINAL_ID entity_id, uint16_t offset, void* pbuf, uint16_t len);
//int8_t VC_LEP_RunCommand (VC_TERMINAL_ID entity_id, uint16_t offset);

int8_t VC_VIGIL_GetAttribute (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf, uint16_t length);
int8_t VC_VIGIL_SetAttribute (VC_TERMINAL_ID entity_id, uint16_t offset, uint8_t* pbuf, uint16_t length);
int8_t VC_VIGIL_GetAttributeLen (VC_TERMINAL_ID entity_id, uint16_t offset, uint16_t* pbuf);
int8_t VC_VIGIL_GetMaxValue (VC_TERMINAL_ID entity_id, uint16_t offset, void* pbuf, uint16_t len);
int8_t VC_VIGIL_RunCommand (VC_TERMINAL_ID entity_id, uint16_t offset);

#endif
