/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
  *           This driver manages the Audio Class 1.0 following the "USB Device Class
  *           Definition for Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (stereo, PCM)
  *             - 1 Audio Streaming OUT Endpoint (isochronous, async)
  *             - 1 Audio Feedback IN Endpoint (3 bytes, 10.14 format)
  *             - Audio Feature Unit (Mute + Volume control)
  *             - Audio Synchronization type: Asynchronous
  *
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48 kHz
  *             - Bit resolution: 24
  *             - Number of channels: 2
  *             - Volume control 0 dB .. -96 dB (3 dB steps)
  *             - Mute/Unmute
  *             - Asynchronous Endpoints
  *
  *          Data path:
  *             USB OUT (24-bit L/R interleaved bytes) -> decode -> float32 DSP hook
  *             -> volume/mute -> encode -> 32-bit I2S ring buffer -> SAI1 DMA.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"
#include "bsp_audio.h"
#include "audio_dsp.h"
#include "stm32h7xx_ll_usb.h"
#include <math.h>

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
#define AUDIO_SAMPLE_FREQ(frq) \
  (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE_24B(frq) \
  (uint8_t)(((frq / 1000U + 1) * 2U * 3U) & 0xFFU), \
  (uint8_t)((((frq / 1000U + 1) * 2U * 3U) >> 8) & 0xFFU)

/* Nominal feedback (10.14 format, stored << 8) for 48 kHz full speed:
   48 samples/ms * 2^22 = 0x0C000000.  The actual SAI frame rate is
   SAI1_CK (12,288,001.443 Hz) / 256 ~= 48,000.0056 Hz; the PID loop
   self-corrects around this nominal value. */
#define AUDIO_FB_DEFAULT 0x0C000000U

/* Clamp feedback to nominal +/- 1 kHz */
#define AUDIO_FB_DELTA_MAX (uint32_t)(1 << 22)

/* float32 conversion scale: 24-bit signed sample -> [-1, 1) */
#define AUDIO_FLOAT_SCALE_INV (1.0f / 8388608.0f)
#define AUDIO_FLOAT_SCALE     (8388608.0f)

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_OUT_StopAndReset(USBD_HandleTypeDef *pdev);
static void AUDIO_OUT_Restart(USBD_HandleTypeDef *pdev);
static float USBD_AUDIO_VolFactor(int16_t volume);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  NULL,
#endif /* USBD_SUPPORT_USER_STRING_DESC */
};

#ifndef USE_USBD_COMPOSITE
/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration 1 */
  0x09,                                 /* bLength */
  USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
  LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),    /* wTotalLength */
  HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
  0x02,                                 /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0x80,                                 /* bmAttributes: Bus Powered */
  0x32,                                 /* bMaxPower = 100 mA */
  /* 09 byte*/

  /* USB Speaker Standard interface descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Class-specific AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  0x27,                                 /* wTotalLength = 39 */
  0x00,
  0x01,                                 /* bInCollection */
  0x01,                                 /* baInterfaceNr */
  /* 09 byte*/

  /* USB Speaker Input Terminal Descriptor */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
  0x01,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING 0x0101 */
  0x01,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bNrChannels */
  0x03,                                 /* wChannelConfig 0x0003 FL FR */
  0x00,
  0x00,                                 /* iChannelNames */
  0x00,                                 /* iTerminal */
  /* 12 byte*/

  /* USB Speaker Audio Feature Unit Descriptor */
  0x09,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
  0x01,                                 /* bSourceID */
  0x01,                                 /* bControlSize */
  (AUDIO_CONTROL_MUTE | AUDIO_CONTROL_VOL), /* bmaControls(0) */
  0,                                    /* bmaControls(1) */
  0x00,                                 /* iTerminal */
  /* 09 byte*/

  /* USB Speaker Output Terminal Descriptor */
  0x09,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  0x03,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType 0x0301 */
  0x03,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bSourceID */
  0x00,                                 /* iTerminal */
  /* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Zero Bandwidth */
  /* Interface 1, Alternate Setting 0 */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Operational */
  /* Interface 1, Alternate Setting 1 */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x01,                                 /* bAlternateSetting */
  0x02,                                 /* bNumEndpoints: 1 output + 1 feedback */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
  0x01,                                 /* bTerminalLink */
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM 0x0001 */
  0x00,
  /* 07 byte*/

  /* USB Speaker Audio Type I Format Interface Descriptor */
  0x11,                                 /* bLength = 17 (3 frequencies) */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_I,                  /* bFormatType */
  0x02,                                 /* bNrChannels */
  0x03,                                 /* bSubFrameSize: 3 bytes per frame (24-bit) */
  24,                                   /* bBitResolution (24 bits per sample) */
  0x03,                                 /* bSamFreqType: 3 frequencies supported */
  AUDIO_SAMPLE_FREQ(44100U),            /* 44.1 kHz (3 bytes) */
  AUDIO_SAMPLE_FREQ(48000U),            /* 48 kHz (3 bytes) */
  AUDIO_SAMPLE_FREQ(96000U),            /* 96 kHz (3 bytes) */
  /* 17 byte*/

  /* Endpoint 1 - Standard Descriptor */
  /* Isochronous Async endpoint for audio packets */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  AUDIO_OUT_EP,                         /* bEndpointAddress: 1 OUT */
  0x0D,                                 /* bmAttributes: isochronous, async */
  AUDIO_PACKET_SZE_24B(USBD_AUDIO_FREQ_MAX), /* wMaxPacketSize (bytes) */
  0x01,                                 /* bInterval */
  0x00,                                 /* bRefresh */
  AUDIO_IN_EP,                          /* bSynchAddress */
  /* 09 byte*/

  /* Endpoint - Audio Streaming Descriptor */
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
  0x01,                                 /* bmAttributes: Sampling Frequency control */
  0x00,                                 /* bLockDelayUnits */
  0x00,                                 /* wLockDelay */
  0x00,
  /* 07 byte*/

  /* Endpoint 2 - Standard Descriptor */
  /* 3 byte 10.14 sampling frequency feedback to host */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  AUDIO_IN_EP,                          /* bEndpointAddress */
  0x11,                                 /* bmAttributes */
  0x03, 0x00,                           /* wMaxPacketSize = 3 bytes */
  0x01,                                 /* bInterval 1 ms */
  SOF_RATE,                             /* bRefresh 4 ms = 2^2 */
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/
};

/* USB Standard Device Qualifier Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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
#endif /* USE_USBD_COMPOSITE */

/* DMA-accessible ring buffer (CPU writes in DataOut, SAI1 DMA reads).
   Placed in RAM_D2 via the .audio_buf linker section and marked
   non-cacheable by an MPU region in main.c. */
__attribute__((section(".audio_buf"), aligned(32)))
uint32_t AudioRingBuffer[AUDIO_BUF_WORDS];

volatile uint32_t tx_flag = 1U;
volatile uint32_t is_playing = 0U;
volatile uint32_t all_ready = 0U;

volatile uint32_t fb_nom = AUDIO_FB_DEFAULT;
volatile uint32_t fb_value = AUDIO_FB_DEFAULT;

/* Current audio sample rate (Hz), exposed for the status display. Updated on
   init and whenever the host switches rate. */
volatile uint32_t audio_freq_hz = USBD_AUDIO_FREQ_DEFAULT;
volatile uint32_t audio_buf_writable_samples_last = AUDIO_BUF_WORDS / 4U;
volatile uint32_t audio_sound_frames = 0U;

volatile uint8_t fb_data[3] = {
  (uint8_t)((AUDIO_FB_DEFAULT >> 8) & 0x000000FFU),
  (uint8_t)((AUDIO_FB_DEFAULT >> 16) & 0x000000FFU),
  (uint8_t)((AUDIO_FB_DEFAULT >> 24) & 0x000000FFU)
};

/* FNSOF is critical for the frequency-change feedback alignment */
volatile uint32_t fnsof = 0U;

/* OUT endpoint receive buffer (armed via USBD_LL_PrepareReceive) */
static uint8_t tmpbuf[AUDIO_OUT_PACKET_24B];

/* DSP staging buffer (interleaved L/R float32) */
static float dspbuf[AUDIO_SAMPLES_PER_PACKET * 2U];

#ifdef AUDIO_DEBUG
volatile uint8_t  audio_dbg_danger = 0U;
volatile uint32_t audio_dbg_drop_count = 0U;
volatile uint32_t audio_dbg_writable_samples = 0U;
volatile uint32_t audio_dbg_min_free = AUDIO_BUF_WORDS / 2U;
volatile uint32_t audio_dbg_max_free = 0U;
volatile uint32_t audio_dbg_freq = USBD_AUDIO_FREQ_DEFAULT;
#endif /* AUDIO_DEBUG */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */

/* Linear volume gain from the UAC 1.0 attenuation value.
   volume is in 1/256 dB units, 0 dB (max) .. -96 dB (min). Computed only on
   a SET_CUR, so the powf cost is off the per-sample path. */
static float USBD_AUDIO_VolFactor(int16_t volume)
{
  if (volume < (int16_t)USBD_AUDIO_VOL_MIN) volume = (int16_t)USBD_AUDIO_VOL_MIN;
  if (volume > (int16_t)USBD_AUDIO_VOL_MAX) volume = (int16_t)USBD_AUDIO_VOL_MAX;
  return powf(10.0f, ((float)volume / 256.0f) / 20.0f);
}

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_AUDIO_HandleTypeDef *haudio;

  /* Open EP OUT */
  (void)USBD_LL_OpenEP(pdev, AUDIO_OUT_EP, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET_24B);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;

  /* Open EP IN (feedback) */
  (void)USBD_LL_OpenEP(pdev, AUDIO_IN_EP, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

  /* Flush feedback endpoint */
  (void)USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  /* Block feedback transmission until the device is ready */
  tx_flag = 1U;

  /* Allocate Audio structure */
  haudio = (USBD_AUDIO_HandleTypeDef *)USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

  if (haudio == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassDataCmsit[pdev->classId] = (void *)haudio;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

  haudio->alt_setting = 0U;
  haudio->offset = AUDIO_OFFSET_UNKNOWN;
  haudio->wr_ptr = 0U;
  haudio->rd_enable = 0U;
  haudio->freq = USBD_AUDIO_FREQ_DEFAULT;
  haudio->volume = USBD_AUDIO_VOL_DEFAULT;
  haudio->vol_factor = USBD_AUDIO_VolFactor(USBD_AUDIO_VOL_DEFAULT);
  haudio->mute = USBD_AUDIO_MUTE_DEFAULT;

  audio_freq_hz = haudio->freq;

  /* Initialize the Audio output Hardware layer */
  if (((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init(haudio->freq,
                                                                      haudio->volume,
                                                                      haudio->mute) != 0)
  {
    return (uint8_t)USBD_FAIL;
  }

  AudioDSP_Init(haudio->freq);

  /* Arm the OUT endpoint to receive the first audio packet */
  (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, tmpbuf, AUDIO_OUT_PACKET_24B);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DeInit
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Flush and close endpoints */
  (void)USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);
  (void)USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  (void)USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;

  (void)USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;

  tx_flag = 0U;

  /* DeInit physical Interface components */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit(0U);
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: device instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case AUDIO_REQ_GET_CUR:
          AUDIO_REQ_GetCurrent(pdev, req);
          break;

        case AUDIO_REQ_GET_MAX:
          AUDIO_REQ_GetMax(pdev, req);
          break;

        case AUDIO_REQ_GET_MIN:
          AUDIO_REQ_GetMin(pdev, req);
          break;

        case AUDIO_REQ_GET_RES:
          AUDIO_REQ_GetRes(pdev, req);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCurrent(pdev, req);
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
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_AUDIO_CfgDesc + 18;
            len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);
            (void)USBD_CtlSendData(pdev, pbuf, len);
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&haudio->alt_setting, 1U);
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
            if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES)
            {
              if (haudio->alt_setting != (uint8_t)(req->wValue))
              {
                haudio->alt_setting = (uint8_t)(req->wValue);
                if (haudio->alt_setting == 0U)
                {
                  AUDIO_OUT_StopAndReset(pdev);
                }
                else
                {
                  AUDIO_OUT_Restart(pdev);
                }
              }
              (void)USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
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

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  length: pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AUDIO_CfgDesc);
  return USBD_AUDIO_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(pdev);

  /* Feedback endpoint finished transmitting: allow next one */
  if (epnum == (AUDIO_IN_EP & 0xFU))
  {
    tx_flag = 0U;
  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {
    if (haudio->control.req_type == AUDIO_CONTROL_REQ)
    {
      switch (haudio->control.cs)
      {
        case AUDIO_CONTROL_REQ_FU_MUTE:
          haudio->mute = haudio->control.data[0];
          ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->MuteCtl(haudio->control.data[0]);
          break;

        case AUDIO_CONTROL_REQ_FU_VOL:
        {
          int16_t volume = *(int16_t *)&haudio->control.data[0];
          haudio->volume = volume;
          haudio->vol_factor = USBD_AUDIO_VolFactor(volume);
          ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->VolumeCtl(volume);
          break;
        }

        default:
          break;
      }
    }
    else if (haudio->control.req_type == AUDIO_STREAMING_REQ)
    {
      if (haudio->control.cs == AUDIO_STREAMING_REQ_FREQ_CTRL)
      {
        uint32_t new_freq = *(uint32_t *)&haudio->control.data & 0x00FFFFFFU;
        if (haudio->freq != new_freq)
        {
          /* Only accept a rate we can actually generate (PLL2/SAI table). */
          if (BSP_AUDIO_OUT_GetRateConfig(new_freq) != NULL)
          {
            haudio->freq = new_freq;
            AUDIO_OUT_Restart(pdev);
          }
        }
      }
    }

    haudio->control.req_type = 0U;
    haudio->control.cs = 0U;
    haudio->control.cn = 0U;
    haudio->control.cmd = 0U;
    haudio->control.len = 0U;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 Tx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event (1 ms): PID feedback for async clock sync
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  static volatile uint32_t sof_count = 0;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  if (haudio == NULL)
  {
    return (uint8_t)USBD_OK;
  }

  if (haudio->rd_enable == 1U && all_ready == 1U)
  {
    /* Read pointer from DMA remaining count (NDTR, in words) */
    uint32_t rd_words = AUDIO_BUF_WORDS - BSP_AUDIO_OUT_GetRemainingDataSize();

    uint32_t writable_words;
    if (rd_words >= haudio->wr_ptr)
    {
      writable_words = rd_words - haudio->wr_ptr;
    }
    else
    {
      writable_words = rd_words + AUDIO_BUF_WORDS - haudio->wr_ptr;
    }
    uint32_t writable_samples = writable_words / 2U;

#ifdef AUDIO_DEBUG
    audio_dbg_writable_samples = writable_samples;
    if (writable_samples < audio_dbg_min_free)
    {
      audio_dbg_min_free = writable_samples;
    }
    if (writable_samples > audio_dbg_max_free)
    {
      audio_dbg_max_free = writable_samples;
    }
    audio_dbg_danger = (writable_samples < AUDIO_BUF_SAFEZONE_SAMPLES) ? 1U : 0U;
#endif /* AUDIO_DEBUG */

    sof_count++;
    if (sof_count == 1U)
    {
      sof_count = 0U;

      /* Optimal writable size at steady state is half the buffer.
         PID-style correction of the nominal feedback based on the deviation. */
      int32_t dev = (int32_t)writable_samples - (int32_t)(AUDIO_BUF_WORDS / 4U);
      uint64_t tmp = (uint64_t)((int32_t)(1 << 22) + (dev * 256));
      uint64_t pid_k = ((uint64_t)fb_nom) * tmp;
      fb_value = (uint32_t)(pid_k >> 22);

      /* Clamp feedback to nominal +/- 1 kHz */
      if (fb_value > fb_nom + AUDIO_FB_DELTA_MAX)
      {
        fb_value = fb_nom + AUDIO_FB_DELTA_MAX;
      }
      else if (fb_value < fb_nom - AUDIO_FB_DELTA_MAX)
      {
        fb_value = fb_nom - AUDIO_FB_DELTA_MAX;
      }

      /* 10.14 format feedback data { LO, MID, HI } */
      fb_data[0] = (uint8_t)((fb_value >> 8) & 0x000000FFU);
      fb_data[1] = (uint8_t)((fb_value >> 16) & 0x000000FFU);
      fb_data[2] = (uint8_t)((fb_value >> 24) & 0x000000FFU);
    }

    /* Transmit feedback only when the previous one completed */
    if (tx_flag == 0U)
    {
      USB_OTG_GlobalTypeDef *USBx = USB_OTG_HS;
      uint32_t USBx_BASE = (uint32_t)USBx;
      uint32_t fnsof_new = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

      if ((fnsof & 0x1U) == (fnsof_new & 0x1U))
      {
        (void)USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t *)fb_data, 3U);
        tx_flag = 1U;
      }
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Sync
  *         Handle Sync event called from usbd_audio_if.c.
  *         Read pointer is tracked in the SOF handler via the DMA counter,
  *         so this is intentionally a no-op.
  * @param  pdev: device instance
  * @param  offset: audio offset
  * @retval None
  */
void USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{
  UNUSED(pdev);
  UNUSED(offset);
}

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(pdev);
  UNUSED(epnum);

  USB_OTG_GlobalTypeDef *USBx = USB_OTG_HS;
  uint32_t USBx_BASE = (uint32_t)USBx;
  fnsof = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

  if (tx_flag == 1U)
  {
    tx_flag = 0U;
    (void)USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);

  (void)USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);

  /* Prepare Out endpoint to receive next audio packet */
  (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, tmpbuf, AUDIO_OUT_PACKET_24B);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage: decode, DSP, volume, encode into ring buffer
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if ((all_ready == 1U) && (epnum == (AUDIO_OUT_EP & 0xFU)))
  {
    uint32_t curr_length = USBD_LL_GetRxDataSize(pdev, epnum);
    uint32_t i;
    uint32_t num_frames;

    /* Ignore strangely large packets */
    if (curr_length > AUDIO_OUT_PACKET_24B)
    {
      curr_length = 0U;
    }

    num_frames = curr_length / 6U; /* 3 bytes per sample, 2 samples per frame */

    /* Safe-zone check: if the ring buffer has insufficient free space, drop this
       packet rather than overwrite unplayed audio. Only relevant once playback
       has started (before that the DMA read pointer is not advancing). */
    if (is_playing == 1U)
    {
      uint32_t rd_words = AUDIO_BUF_WORDS - BSP_AUDIO_OUT_GetRemainingDataSize();
      uint32_t free_words;
      uint32_t needed_words = num_frames * 2U + AUDIO_BUF_SAFEZONE_SAMPLES * 2U;

      if (rd_words >= haudio->wr_ptr)
      {
        free_words = rd_words - haudio->wr_ptr;
      }
      else
      {
        free_words = rd_words + AUDIO_BUF_WORDS - haudio->wr_ptr;
      }

      if (free_words < needed_words)
      {
#ifdef AUDIO_DEBUG
        audio_dbg_drop_count++;
#endif /* AUDIO_DEBUG */
        /* Re-arm the endpoint and discard the packet */
        (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, tmpbuf, AUDIO_OUT_PACKET_24B);
        return (uint8_t)USBD_OK;
      }
    }

    /* Decode 24-bit LE bytes -> float32 ([-1, 1)) */
    for (i = 0; i < num_frames; i++)
    {
      uint8_t *p = &tmpbuf[i * 6U];
      int32_t l = (int32_t)(((uint32_t)p[2] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[0] << 8)) >> 8;
      int32_t r = (int32_t)(((uint32_t)p[5] << 24) | ((uint32_t)p[4] << 16) | ((uint32_t)p[3] << 8)) >> 8;
      if ((l != 0) || (r != 0))
      {
        audio_sound_frames++;
      }
      dspbuf[2U * i]     = (float)l * AUDIO_FLOAT_SCALE_INV;
      dspbuf[2U * i + 1U] = (float)r * AUDIO_FLOAT_SCALE_INV;
    }

    /* DSP hook (pass-through by default) */
    AudioDSP_ProcessBlock(dspbuf, num_frames);

    /* Combined volume/mute + float->int24 scale (loop-invariant) */
    float scale = (haudio->mute ? 0.0f : haudio->vol_factor) * AUDIO_FLOAT_SCALE;

    /* float32 -> int32 (24-bit) with volume/mute -> left-justified I2S word */
    for (i = 0; i < num_frames; i++)
    {
      int32_t l = (int32_t)(dspbuf[2U * i] * scale);
      int32_t r = (int32_t)(dspbuf[2U * i + 1U] * scale);

      if (l > 8388607)  { l = 8388607;  }
      else if (l < -8388608) { l = -8388608; }
      if (r > 8388607)  { r = 8388607;  }
      else if (r < -8388608) { r = -8388608; }

      AudioRingBuffer[haudio->wr_ptr++] = (uint32_t)(l << 8);
      AudioRingBuffer[haudio->wr_ptr++] = (uint32_t)(r << 8);

      if (haudio->wr_ptr >= AUDIO_BUF_WORDS)
      {
        haudio->wr_ptr = 0U;
      }
    }

    /* Start playback once the buffer is half full */
    if (haudio->offset == AUDIO_OFFSET_UNKNOWN && is_playing == 0U)
    {
      if (haudio->wr_ptr >= (AUDIO_BUF_WORDS / 2U))
      {
        haudio->offset = AUDIO_OFFSET_NONE;
        is_playing = 1U;

        haudio->rd_enable = 1U;
        audio_buf_writable_samples_last = (AUDIO_BUF_WORDS - haudio->wr_ptr) / 2U;

        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->AudioCmd(AudioRingBuffer,
                                                                            (uint16_t)AUDIO_BUF_WORDS,
                                                                            AUDIO_CMD_START);
      }
    }

    /* Prepare Out endpoint to receive next audio packet */
    (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, tmpbuf, AUDIO_OUT_PACKET_24B);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  AUDIO_REQ_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  if ((req->bmRequest & 0x1FU) == AUDIO_CONTROL_REQ)
  {
    switch (HIBYTE(req->wValue))
    {
      case AUDIO_CONTROL_REQ_FU_MUTE:
        (void)USBD_CtlSendData(pdev, (uint8_t *)&haudio->mute, 1U);
        break;

      case AUDIO_CONTROL_REQ_FU_VOL:
        (void)USBD_CtlSendData(pdev, (uint8_t *)&haudio->volume, 2U);
        break;

      default:
        break;
    }
  }
  else if ((req->bmRequest & 0x1FU) == AUDIO_STREAMING_REQ)
  {
    if (HIBYTE(req->wValue) == AUDIO_STREAMING_REQ_FREQ_CTRL)
    {
      uint32_t freq __attribute__((aligned(4))) = haudio->freq;
      (void)USBD_CtlSendData(pdev, (uint8_t *)&freq, 3U);
    }
  }
}

/**
  * @brief  AUDIO_REQ_GetMax
  *         Handles the GET_MAX Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  if ((req->bmRequest & 0x1FU) == AUDIO_CONTROL_REQ)
  {
    if (HIBYTE(req->wValue) == AUDIO_CONTROL_REQ_FU_VOL)
    {
      int16_t vol_max = (int16_t)USBD_AUDIO_VOL_MAX;
      (void)USBD_CtlSendData(pdev, (uint8_t *)&vol_max, 2U);
    }
  }
}

/**
  * @brief  AUDIO_REQ_GetMin
  *         Handles the GET_MIN Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  if ((req->bmRequest & 0x1FU) == AUDIO_CONTROL_REQ)
  {
    if (HIBYTE(req->wValue) == AUDIO_CONTROL_REQ_FU_VOL)
    {
      int16_t vol_min = (int16_t)USBD_AUDIO_VOL_MIN;
      (void)USBD_CtlSendData(pdev, (uint8_t *)&vol_min, 2U);
    }
  }
}

/**
  * @brief  AUDIO_REQ_GetRes
  *         Handles the GET_RES Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  if ((req->bmRequest & 0x1FU) == AUDIO_CONTROL_REQ)
  {
    if (HIBYTE(req->wValue) == AUDIO_CONTROL_REQ_FU_VOL)
    {
      int16_t vol_res = (int16_t)USBD_AUDIO_VOL_STEP;
      (void)USBD_CtlSendData(pdev, (uint8_t *)&vol_res, 2U);
    }
  }
}

/**
  * @brief  AUDIO_REQ_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  if (req->wLength != 0U)
  {
    (void)USBD_CtlPrepareRx(pdev, haudio->control.data, req->wLength);

    haudio->control.cmd = AUDIO_REQ_SET_CUR;
    haudio->control.req_type = req->bmRequest & 0x1FU;
    haudio->control.len = (uint8_t)req->wLength;
    haudio->control.unit = HIBYTE(req->wIndex);
    haudio->control.cs = HIBYTE(req->wValue);
    haudio->control.cn = LOBYTE(req->wValue);
  }
}

/**
  * @brief  Stop playing and reset buffer pointers
  * @param  pdev: device instance
  */
static void AUDIO_OUT_StopAndReset(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  all_ready = 0U;
  tx_flag = 1U;
  is_playing = 0U;
  audio_buf_writable_samples_last = AUDIO_BUF_WORDS / 4U;

#ifdef AUDIO_DEBUG
  audio_dbg_danger = 0U;
#endif /* AUDIO_DEBUG */

  haudio->offset = AUDIO_OFFSET_UNKNOWN;
  haudio->rd_enable = 0U;
  haudio->wr_ptr = 0U;

  (void)USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  (void)USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);

  ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit(0U);
}

/**
  * @brief  Restart playing with new parameters
  * @param  pdev: device instance
  */
static void AUDIO_OUT_Restart(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  AUDIO_OUT_StopAndReset(pdev);

  audio_freq_hz = haudio->freq;

  /* Set the nominal feedback to the actual SAI rate for the selected frequency */
  {
    const AUDIO_RateConfig_t *cfg = BSP_AUDIO_OUT_GetRateConfig(haudio->freq);
    if (cfg != NULL)
    {
      fb_nom = fb_value = cfg->fb_nom;
    }
    else
    {
      fb_nom = fb_value = AUDIO_FB_DEFAULT;
    }

    /* Refresh the feedback packet immediately so the first SOF transmit after
       restart does not send the previous rate's value. */
    fb_data[0] = (uint8_t)((fb_value >> 8) & 0x000000FFU);
    fb_data[1] = (uint8_t)((fb_value >> 16) & 0x000000FFU);
    fb_data[2] = (uint8_t)((fb_value >> 24) & 0x000000FFU);
  }

#ifdef AUDIO_DEBUG
  audio_dbg_freq = haudio->freq;
  audio_dbg_min_free = AUDIO_BUF_WORDS / 2U;
  audio_dbg_max_free = 0U;
  audio_dbg_drop_count = 0U;
  audio_dbg_danger = 0U;
#endif /* AUDIO_DEBUG */

  ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init(haudio->freq,
                                                                  haudio->volume,
                                                                  haudio->mute);
  AudioDSP_Init(haudio->freq);

  /* Re-arm the OUT endpoint after the StopAndReset flush */
  (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, tmpbuf, AUDIO_OUT_PACKET_24B);

  tx_flag = 0U;
  all_ready = 1U;
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length: pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_AUDIO_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: Audio interface callback
  * @retval status
  */
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}

#ifdef USE_USBD_COMPOSITE
/**
  * @brief  USBD_AUDIO_GetEpPcktSze
  * @param  pdev: device instance (reserved for future use)
  * @param  If: Interface number (reserved for future use)
  * @param  Ep: Endpoint number (reserved for future use)
  * @retval mps
  */
uint32_t USBD_AUDIO_GetEpPcktSze(USBD_HandleTypeDef *pdev, uint8_t If, uint8_t Ep)
{
  uint32_t mps;

  UNUSED(pdev);
  UNUSED(If);
  UNUSED(Ep);

  mps = (uint32_t)((USBD_AUDIO_FREQ_MAX / 1000U + 1U) * 2U * 3U);
  return mps;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
