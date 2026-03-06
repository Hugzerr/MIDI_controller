/**
  ******************************************************************************
  * @file    usbd_hid.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @verbatim
  *
  *          ===================================================================
  *                                HID Class  Description
  *          ===================================================================
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse protocol
  *             - Usage Page : Generic Desktop
  *             - Usage : Joystick
  *             - Collection : Application
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_HID
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_HID_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_HID_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_HID_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_HID_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);

/**
  * @}
  */

/** @defgroup USBD_HID_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL,              /* EP0_TxSent */
  NULL,              /* EP0_RxReady */
  USBD_HID_DataIn,   /* DataIn */
  NULL,              /* DataOut */
  NULL,              /* SOF */
  NULL,
  NULL,
  USBD_HID_GetHSCfgDesc,
  USBD_HID_GetFSCfgDesc,
  USBD_HID_GetOtherSpeedCfgDesc,
  USBD_HID_GetDeviceQualifierDesc,

};


/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[] __ALIGN_END =
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
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

static uint8_t HIDInEpAdd = HID_EPIN_ADDR;

/**
  * @}
  */

/** @defgroup USBD_HID_Private_Functions
  * @{
  */

/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_HID_HandleTypeDef *hhid;

  hhid = (USBD_HID_HandleTypeDef *)USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

  if (hhid == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassDataCmsit[pdev->classId] = (void *)hhid;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];


  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, HIDInEpAdd, USBD_EP_TYPE_BULK, 0x40);
  pdev->ep_in[HIDInEpAdd & 0xFU].is_used = 1U;

  hhid->state = USBD_HID_IDLE;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_DeInit
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Close HID EPs */
  (void)USBD_LL_CloseEP(pdev, HIDInEpAdd);
  pdev->ep_in[HIDInEpAdd & 0xFU].is_used = 0U;

  /* Free allocated memory */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        case USBD_HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case USBD_HID_REQ_GET_PROTOCOL:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
          break;

        case USBD_HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case USBD_HID_REQ_GET_IDLE:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_HID_CfgDesc;
            len = MIN(USB_HID_DESC_SIZ, req->wLength);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          (void)USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            hhid->AltSetting = (uint8_t)(req->wValue);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}


/**
  * @brief  USBD_HID_SendReport
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @param  ClassId: The Class ID
  * @retval status
  */

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hhid->state == USBD_HID_IDLE)
    {
      hhid->state = USBD_HID_BUSY;
      (void)USBD_LL_Transmit(pdev, HIDInEpAdd, report, len);
    }
  }

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_HID_GetCfgFSDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

/**
  * @brief  USBD_HID_GetCfgHSDesc
  *         return HS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length)
{

  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

/**
  * @brief  USBD_HID_GetOtherSpeedCfgDesc
  *         return other speed configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

/**
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);
  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId])->state = USBD_HID_IDLE;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_DeviceQualifierDesc);

  return USBD_HID_DeviceQualifierDesc;
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

