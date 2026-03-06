/**
  ******************************************************************************
  * @file    usbd_midi.c
  * @author  Illia Pikin, MCD Application Team
  * @brief   This file provides the MIDI core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                MIDI Class  Description
  *          =================================================================== 
  *           This module manages the MIDI class V1.0 following the "Universal Serial Bus
  *           Device Class Definition for MIDI Devices. Release 1.0 Nov 1, 1999".
  *      
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *           
  *      
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2025 Illia Pikin</center></h2>
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  * Licensed under BSD 2-Clause License
  *
  * Copyright (c) 2025, Illia Pikin a.k.a Hypnotriod
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice, this
  *    list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_MIDI 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_MIDI_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_MIDI_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_MIDI_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 




/** @defgroup USBD_MIDI_Private_FunctionPrototypes
  * @{
  */

static uint8_t  USBD_MIDI_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_MIDI_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length);
static uint8_t  *USBD_MIDI_GetDeviceQualifierDesc (uint16_t *length);
static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
/**
  * @}
  */ 

/** @defgroup USBD_MIDI_Private_Variables
  * @{
  */ 

static uint8_t usb_rx_buffer[MIDI_EPOUT_SIZE] = {0};


/* USB MIDI class type definition */
USBD_ClassTypeDef  USBD_MIDI = 
{
  USBD_MIDI_Init,
  USBD_MIDI_DeInit,
  USBD_MIDI_Setup,
  NULL, /*EP0_TxSent*/  
  NULL, /*EP0_RxReady*/
  USBD_MIDI_DataIn, /*DataIn*/
  USBD_MIDI_DataOut, /*DataOut*/
  NULL, /*SOF */
  NULL,
  NULL,      
  USBD_MIDI_GetCfgDesc,
  USBD_MIDI_GetCfgDesc, 
  USBD_MIDI_GetCfgDesc,
  USBD_MIDI_GetDeviceQualifierDesc,
};

/* USB MIDI device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CfgDesc[]  __ALIGN_END =
{
  /* MIDI Adapter Configuration Descriptor: 9Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 37,38 */
  0x09,		// Length of the Descriptor (1Byte)
  0x02,		// Descriptor Type: Configuration (1Byte)
  0x65,    	// Total Length of the config. block including this descriptor: length is 101 bytes (2bytes Low-byte first)
  0x00,   	// Total Length high-byte, continuing from above
  0x02,		// Number of Interfaces: 2 interfaces: Standard AC and Standard MIDI-streaming (1Byte)
  0x01,		// Configuration Value: ID of this configuration is 1 (1Byte)
  0x00,		// iConfiguration: Unused (1Byte)
  0x80,		// bmAttributes:   BUS Powered and not Battery/Self powered and no remote wake-up (1Byte)
  0x32,		// MaxPower = 100 mA, in steps of 2mA (1Byte)


  /* MIDI Adapter Standard Audio Control (AC) Interface Descriptor: 9Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 38 */
  0x09,		// Length of the Descriptor (1Byte)
  0x04,		// Descriptor Type: Interface (1Byte)
  0x00,		// Index of this interface (1Byte)
  0x00,		// Alternate Setting: Index of this Setting (1Byte)
  0x00,		// Number of End-points (1Byte)
  0x01,		// Interface Class: Audio (1Byte)
  0x01,		// Interface Sub-Class: Audio Control (1Byte)
  0x00,		// Interface Protocol: Unused (1Byte)
  0x00,		// iInterface: Unused (1Byte)


  /* MIDI Adapter Class-specific AC Interface Descriptor: 9Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 39 */
  0x09,		// Length of the Descriptor (1Byte)
  0x24,		// Descriptor Type: Class specific interface (1Byte)
  0x01,		// Descriptor Sub-type: Class Specific Interface Header (1Byte)
  0x00,		// Class Specification Revision No.: 1.00 (2Bytes Low-byte first)
  0x01,		// Class Specification revision No.: High-byte, continuing from above
  0x09,		// Total Length of class-specific descriptor: 9-bytes (2Bytes Low-byte first)
  0x00,		// Total Length of class-specific descriptor: High-byte, Continuing from above
  0x01,     // Number of streaming interfaces: 1 (1Byte)
  0x01,		// baInterfaceNr: MIDI-Streaming interface 1 belongs to this AudioControl interface. (1Byte)


  /* MIDI Adapter Standard MIDI Streaming (MS) Interface Descriptor: 9Bytes  */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 39 */
  0x09,		// Length of the Descriptor (1Byte)
  0x04,		// Descriptor Type: Interface (1Byte)
  0x01,		// Index of this interface (1Byte)
  0x00,		// Alternate Setting: Index of this Setting (1Byte)
  0x02,		// Number of End-points (1Byte)
  0x01,		// Interface Class: Audio (1Byte)
  0x03,		// Interface Sub-Class: MIDI-Streaming (1Byte)
  0x00,		// Interface Protocol: Unused (1Byte)
  0x00,		// iInterface: Unused (1Byte)


  /*  MIDI Adapter Class-specific MS Interface Descriptor: 7Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 40 */
  0x07,		// Length of the Descriptor (1Byte)
  0x24,		// Descriptor Type: Class specific interface (1Byte)
  0x01,		// Descriptor Sub-type: Class Specific Interface Header (1Byte)
  0x00,		// Class Specification Revision No.: 1.00 (2Bytes Low-byte first)
  0x01,		// Class Specification revision No.: High-byte, continuing from above
  0x41,		// Total length of class specific descriptor: length is 65bytes (2bytes Low-byte first)
  0x00,		// Total Length high-byte, continuing from above


  /* MIDI Adapter MIDI IN Jack Descriptor (Embedded): 6Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 40 */
  0x06,		// Length of the Descriptor (1Byte)
  0x24,		// Descriptor Type: Class specific interface (1Byte)
  0x02,		// Descriptor Sub-type: MIDI IN Jack (1Byte)
  0x01,		// Jack Type: Embedded (1Byte)
  0x01,		// Jack ID: 1 (1Byte)
  0x00,		// iJack: Unused (1Byte)


  /* MIDI Adapter MIDI IN Jack Descriptor (External): 6Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 40 */
  0x06,		// Length of the Descriptor (1Byte)
  0x24,		// Descriptor Type: Class specific interface (1Byte)
  0x02,		// Descriptor Sub-type: MIDI IN Jack (1Byte)
  0x02,		// Jack Type: External (1Byte)
  0x02,		// Jack ID: 2 (1Byte)
  0x00,		// iJack: Unused (1Byte)


  /* MIDI Adapter MIDI OUT Jack Descriptor (Embedded): 9Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 41 */
  0x09,		// Length of the Descriptor (1Byte)
  0x24,		// Descriptor Type: Class specific interface (1Byte)
  0x03,		// Descriptor Sub-type: MIDI OUT Jack (1Byte)
  0x01,		// Jack Type: Embedded (1Byte)
  0x03,		// Jack ID: 3 (1Byte)
  0x01,		// Number of Input Pins for this jack: 1 (1Byte)
  0x02,		// Source ID: ID of the Entity to which this Pin is connected: Connected to External MIDI In Jack??? (1Byte)
  0x01,		// Source Pin: Output Pin number of the Entity to which this Input Pin is connected (1Byte)
  0x00,		// iJack: Unused (1Byte)


  /* MIDI Adapter MIDI OUT Jack Descriptor (External): 9Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 41 */
  0x09,		// Length of the Descriptor (1Byte)
  0x24,		// Descriptor Type: Class specific interface (1Byte)
  0x03,		// Descriptor Sub-type: MIDI OUT Jack (1Byte)
  0x02,		// Jack Type: External (1Byte)
  0x04,		// Jack ID: 4 (1Byte)
  0x01,		// Number of Input Pins for this jack: 1 (1Byte)
  0x01,		// Source ID: ID of the Entity to which this Pin is connected: Connected to Embedded MIDI In Jack??? (1Byte)
  0x01,		// Source Pin: Output Pin number of the Entity to which this Input Pin is connected (1Byte)
  0x00,		// iJack: Unused (1Byte)


  /* MIDI Adapter Standard Bulk OUT Endpoint Descriptor: 9Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 42 */
  0x09,		// Length of the Descriptor (1Byte)
  0x05,		// Descriptor Type: Endpoint (1Byte)
  0x01,		// Endpoint Address: OUT Endpoint 1 (1Byte)
  0x02,		// Attributes: Bulk, Not shared (1Byte)
  0x40,		// Max Packet Size: 64 Bytes (2Bytes low-byte first)
  0x00,		// Max Packet Size: high-byte, continuing from above
  0x00,		// Interval: Ignored for bulk mode (1Byte)
  0x00,		// Refresh: Unused (1Byte)
  0x00,		// Synch. Address: Unused (1Byte)


  /* MIDI Adapter Class-specific Bulk OUT Endpoint Descriptor: 5Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 42 */
  0x05,		// Length of the Descriptor (1Byte)
  0x25,		// Descriptor Type: Class Specific Endpoint descriptor (1Byte)
  0x01,		// Descriptor Sub-type: MIDI-Streaming General sub-type (1Byte)
  0x01,		// No. of Embedded MIDI IN Jack: 1 (1Byte)
  0x01,		// ID of Embedded MIDI IN Jack: 1 (1Byte)


  /* MIDI Adapter Standard Bulk IN Endpoint Descriptor: 9Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 42,43 */
  0x09,		// Length of the Descriptor (1Byte)
  0x05,		// Descriptor Type: Endpoint (1Byte)
  0x81,		// Endpoint Address: IN Endpoint 1 (1Byte)
  0x02,		// Attributes: Bulk, Not shared (1Byte)
  0x40,		// Max Packet Size: 64 Bytes (2Bytes low-byte first)
  0x00,		// Max Packet Size: high-byte, continuing from above
  0x00,		// Interval: Ignored for bulk mode (1Byte)
  0x00,		// Refresh: Unused (1Byte)
  0x00,		// Synch. Address: Unused (1Byte)


  /* MIDI Adapter Class-specific Bulk IN Endpoint Descriptor: 5Bytes */
  /* Reference: https://www.usb.org/sites/default/files/midi10.pdf Page: 43 */
  0x05,		// Length of the Descriptor (1Byte)
  0x25,		// Descriptor Type: Class Specific Endpoint descriptor (1Byte)
  0x01,		// Descriptor Sub-type: MIDI-Streaming General sub-type (1Byte)
  0x01,		// No. of Embedded MIDI OUT Jack: 1 (1Byte)
  0x03		// ID of Embedded MIDI OUT Jack: 3 (1Byte)
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]  __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_MIDI_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_MIDI_Init
  *         Initialize the MIDI interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  
  // Open enpoint that sends data to host
  USBD_LL_OpenEP(pdev,
                 MIDI_EPIN_ADDR,
                 USBD_EP_TYPE_BULK,
                 MIDI_EPIN_SIZE);  
    
  // Open enpoint that receives data from host
  USBD_LL_OpenEP(pdev,
               MIDI_EPOUT_ADDR,
               USBD_EP_TYPE_BULK,
               MIDI_EPOUT_SIZE);
  
  //  Prepare the receiving endpoint with a buffer
  USBD_LL_PrepareReceive(pdev, 
               MIDI_EPOUT_ADDR,                                      
               usb_rx_buffer,
               MIDI_EPOUT_SIZE);    
  
  pdev->pClassData = USBD_malloc(sizeof (USBD_MIDI_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
  else
  {
    ((USBD_MIDI_HandleTypeDef *)pdev->pClassData)->state = MIDI_IDLE;
  }
  return ret;
}

/**
  * @brief  USBD_MIDI_DeInit
  *         DeInitialize the MIDI layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  /* Close MIDI EPs */
  USBD_LL_CloseEP(pdev, MIDI_EPIN_SIZE);
  
  /* FRee allocated memory */
  if(pdev->pClassData != NULL)
  {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  } 
  
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_Setup
  *         Handle the MIDI specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_MIDI_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  uint16_t len = 0;
  uint8_t  *pbuf = NULL;
  USBD_MIDI_HandleTypeDef     *hmidi = pdev->pClassData;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :  
    switch (req->bRequest)
    {
      case MIDI_REQ_SET_PROTOCOL:
        hmidi->Protocol = (uint8_t)(req->wValue);
        break;
        
      case MIDI_REQ_GET_PROTOCOL:
        USBD_CtlSendData (pdev, 
                          (uint8_t *)&hmidi->Protocol,
                          1);    
        break;
        
      case MIDI_REQ_SET_IDLE:
        hmidi->IdleState = (uint8_t)(req->wValue >> 8);
        break;
        
      case MIDI_REQ_GET_IDLE:
        USBD_CtlSendData (pdev, 
                          (uint8_t *)&hmidi->IdleState,
                          1);        
        break;      
        
      default:
        USBD_CtlError (pdev, req);
        return USBD_FAIL; 
    }
    break;
    
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
      case USB_REQ_GET_DESCRIPTOR: 
        if( req->wValue >> 8 == MIDI_DESCRIPTOR_TYPE)
        {
          pbuf = USBD_MIDI_CfgDesc + USB_MIDI_CLASS_DESC_SHIFT;
          len = MIN(USB_MIDI_DESC_SIZE , req->wLength);
        }
        
        USBD_CtlSendData (pdev, pbuf, len);
        break;
        
      case USB_REQ_GET_INTERFACE :
        USBD_CtlSendData (pdev,
                          (uint8_t *)&hmidi->AltSetting,
                          1);
        break;
        
      case USB_REQ_SET_INTERFACE :
        hmidi->AltSetting = (uint8_t)(req->wValue);
        break;
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_GetDeviceState 
  *         Get MIDI State
  * @param  pdev: device instance
  * @retval usb device state  (USBD_STATE_DEFAULT, USBD_STATE_ADDRESSED, USBD_STATE_CONFIGURED, USBD_STATE_SUSPENDED)
  */
uint8_t USBD_MIDI_GetDeviceState(USBD_HandleTypeDef  *pdev)
{
  return pdev->dev_state;
}

/**
  * @brief  USBD_MIDI_GetState 
  *         Get MIDI State
  * @param  pdev: device instance
  * @retval usb state  (MIDI_IDLE, MIDI_BUSY)
  */
uint8_t USBD_MIDI_GetState(USBD_HandleTypeDef  *pdev)
{
  return ((USBD_MIDI_HandleTypeDef *)pdev->pClassData)->state;
}

/**
  * @brief  USBD_MIDI_SendPackets 
  *         Send MIDI event packets to the host
  * @param  pdev: device instance
  * @param  data: pointer to the packets data
  * @param  len: size of the data
  * @retval status
  */
uint8_t USBD_MIDI_SendPackets(USBD_HandleTypeDef  *pdev, 
                                 uint8_t *data,
                                 uint16_t len)
{
  USBD_MIDI_HandleTypeDef *hmidi = pdev->pClassData;
  
  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if(hmidi->state == MIDI_IDLE)
    {
      hmidi->state = MIDI_BUSY;
      USBD_LL_Transmit(pdev, MIDI_EPIN_ADDR, data, len);
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_CfgDesc);
  return USBD_MIDI_CfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_MIDI_DeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_DeviceQualifierDesc);
  return USBD_MIDI_DeviceQualifierDesc;
}


/**
  * @brief  USBD_MIDI_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  UNUSED(epnum);
  /* Ensure that the FIFO is empty before a new transfer, this condition could 
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_MIDI_HandleTypeDef *)pdev->pClassData)->state = MIDI_IDLE;

  USBD_MIDI_OnPacketsSent();

  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_OnPacketsSent
  *         on usb midi packets sent to the host callback
  */
__weak extern void USBD_MIDI_OnPacketsSent(void)
{
}

/**
  * @brief  USBD_MIDI_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t len;

  if (epnum != (MIDI_EPOUT_ADDR & 0x0F)) return USBD_FAIL;
  
  len = (uint8_t)HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, epnum);

  USBD_MIDI_OnPacketsReceived(usb_rx_buffer, len);
  
  USBD_LL_PrepareReceive(pdev, MIDI_EPOUT_ADDR, usb_rx_buffer, MIDI_EPOUT_SIZE);  
  
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_OnPacketsReceived
  *         on usb midi packets received from the host callback
  * @param  data: pointer to the data packet
  * @param  len: size of the data
  */
__weak extern void USBD_MIDI_OnPacketsReceived(uint8_t *data, uint8_t len)
{
  UNUSED(data);
  UNUSED(len);
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_MIDI_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_DeviceQualifierDesc);
  return USBD_MIDI_DeviceQualifierDesc;
}

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
