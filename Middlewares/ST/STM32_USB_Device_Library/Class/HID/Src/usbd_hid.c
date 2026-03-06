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
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */
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
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_HID_GetHSCfgDesc,
  USBD_HID_GetFSCfgDesc,
  USBD_HID_GetOtherSpeedCfgDesc,
  USBD_HID_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
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
#endif /* USE_USBD_COMPOSITE  */

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ] __ALIGN_END =
{
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
};

#ifndef USE_USBD_COMPOSITE
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
#endif /* USE_USBD_COMPOSITE  */

__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] __ALIGN_END =
{
  0x05, 0x01,        /* Usage Page (Generic Desktop Ctrls)     */
  0x09, 0x02,        /* Usage (Mouse)                          */
  0xA1, 0x01,        /* Collection (Application)               */
  0x09, 0x01,        /*   Usage (Pointer)                      */
  0xA1, 0x00,        /*   Collection (Physical)                */
  0x05, 0x09,        /*     Usage Page (Button)                */
  0x19, 0x01,        /*     Usage Minimum (0x01)               */
  0x29, 0x03,        /*     Usage Maximum (0x03)               */
  0x15, 0x00,        /*     Logical Minimum (0)                */
  0x25, 0x01,        /*     Logical Maximum (1)                */
  0x95, 0x03,        /*     Report Count (3)                   */
  0x75, 0x01,        /*     Report Size (1)                    */
  0x81, 0x02,        /*     Input (Data,Var,Abs)               */
  0x95, 0x01,        /*     Report Count (1)                   */
  0x75, 0x05,        /*     Report Size (5)                    */
  0x81, 0x01,        /*     Input (Const,Array,Abs)            */
  0x05, 0x01,        /*     Usage Page (Generic Desktop Ctrls) */
  0x09, 0x30,        /*     Usage (X)                          */
  0x09, 0x31,        /*     Usage (Y)                          */
  0x09, 0x38,        /*     Usage (Wheel)                      */
  0x15, 0x81,        /*     Logical Minimum (-127)             */
  0x25, 0x7F,        /*     Logical Maximum (127)              */
  0x75, 0x08,        /*     Report Size (8)                    */
  0x95, 0x03,        /*     Report Count (3)                   */
  0x81, 0x06,        /*     Input (Data,Var,Rel)               */
  0xC0,              /*   End Collection                       */
  0x09, 0x3C,        /*   Usage (Motion Wakeup)                */
  0x05, 0xFF,        /*   Usage Page (Reserved 0xFF)           */
  0x09, 0x01,        /*   Usage (0x01)                         */
  0x15, 0x00,        /*   Logical Minimum (0)                  */
  0x25, 0x01,        /*   Logical Maximum (1)                  */
  0x75, 0x01,        /*   Report Size (1)                      */
  0x95, 0x02,        /*   Report Count (2)                     */
  0xB1, 0x22,        /*   Feature (Data,Var,Abs,NoWrp)         */
  0x75, 0x06,        /*   Report Size (6)                      */
  0x95, 0x01,        /*   Report Count (1)                     */
  0xB1, 0x01,        /*   Feature (Const,Array,Abs,NoWrp)      */
  0xC0               /* End Collection                         */
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

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  HIDInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_in[HIDInEpAdd & 0xFU].bInterval = HID_HS_BINTERVAL;
  }
  else   /* LOW and FULL-speed endpoints */
  {
    pdev->ep_in[HIDInEpAdd & 0xFU].bInterval = HID_FS_BINTERVAL;
  }

  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, HIDInEpAdd, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
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

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  HIDInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* Close HID EPs */
  (void)USBD_LL_CloseEP(pdev, HIDInEpAdd);
  pdev->ep_in[HIDInEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[HIDInEpAdd & 0xFU].bInterval = 0U;

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
          if ((req->wValue >> 8) == HID_REPORT_DESC)
          {
            len = MIN(HID_MOUSE_REPORT_DESC_SIZE, req->wLength);
            pbuf = HID_MOUSE_ReportDesc;
          }
          else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_HID_Desc;
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
#ifdef USE_USBD_COMPOSITE
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len, uint8_t ClassId)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[ClassId];
#else
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
#endif /* USE_USBD_COMPOSITE */

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  HIDInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, ClassId);
#endif /* USE_USBD_COMPOSITE */

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
  * @brief  USBD_HID_GetPollingInterval
  *         return polling interval from endpoint descriptor
  * @param  pdev: device instance
  * @retval polling interval
  */
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev)
{
  uint32_t polling_interval;

  /* HIGH-speed endpoints */
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Sets the data transfer polling interval for high speed transfers.
     Values between 1..16 are allowed. Values correspond to interval
     of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
    polling_interval = (((1U << (HID_HS_BINTERVAL - 1U))) / 8U);
  }
  else   /* LOW and FULL-speed endpoints */
  {
    /* Sets the data transfer polling interval for low and full
    speed transfers */
    polling_interval =  HID_FS_BINTERVAL;
  }

  return ((uint32_t)(polling_interval));
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_HID_GetCfgFSDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_FS_BINTERVAL;
  }

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
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_HS_BINTERVAL;
  }

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
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_FS_BINTERVAL;
  }

  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE  */

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

#ifndef USE_USBD_COMPOSITE
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
#endif /* USE_USBD_COMPOSITE  */
/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

