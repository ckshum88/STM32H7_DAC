/**
  ******************************************************************************
  * @file    usbd_audio.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_audio.c file.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief This file is the Header file for usbd_audio.c
  * @{
  */

/** @defgroup USBD_AUDIO_Exported_Defines
  * @{
  */
#ifndef USBD_AUDIO_FREQ
/* AUDIO Class Config */
#define USBD_AUDIO_FREQ                               48000U
#endif /* USBD_AUDIO_FREQ */

#ifndef USBD_AUDIO_FREQ_MAX
#define USBD_AUDIO_FREQ_MAX                           96000U
#endif /* USBD_AUDIO_FREQ_MAX */

/* Initial/default sampling frequency. Supported rates: 44100 / 48000 / 96000. */
#ifndef USBD_AUDIO_FREQ_DEFAULT
#define USBD_AUDIO_FREQ_DEFAULT                       48000U
#endif /* USBD_AUDIO_FREQ_DEFAULT */

/* See USB Device Class Definition for Audio Devices v1.0 p.77:
   max volume is 0 dB (no attenuation), min is -96 dB, 3 dB steps.
   1 dB <=> 0x100, so -96 dB = 0xA000 */
#ifndef USBD_AUDIO_VOL_MAX
#define USBD_AUDIO_VOL_MAX                            0x0000U
#endif /* USBD_AUDIO_VOL_MAX */

#ifndef USBD_AUDIO_VOL_MIN
#define USBD_AUDIO_VOL_MIN                            0xA000U
#endif /* USBD_AUDIO_VOL_MIN */

#ifndef USBD_AUDIO_VOL_DEFAULT
#define USBD_AUDIO_VOL_DEFAULT                        0x0000U
#endif /* USBD_AUDIO_VOL_DEFAULT */

#ifndef USBD_AUDIO_VOL_STEP
#define USBD_AUDIO_VOL_STEP                           0x0300U
#endif /* USBD_AUDIO_VOL_STEP */

/* default mute state: 0 = unmuted */
#ifndef USBD_AUDIO_MUTE_DEFAULT
#define USBD_AUDIO_MUTE_DEFAULT                       0x00U
#endif /* USBD_AUDIO_MUTE_DEFAULT */

/* Interface */
#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES                       1U
#endif /* USBD_MAX_NUM_INTERFACES */

/* bEndpointAddress, see UAC 1.0 spec, p.61 */
#define AUDIO_OUT_EP                                  0x01U
#define AUDIO_IN_EP                                   0x81U

#define SOF_RATE                                      0x02U

#define USB_AUDIO_CONFIG_DESC_SIZ                     124U

#define AUDIO_INTERFACE_DESC_SIZE                     0x09U
#define USB_AUDIO_DESC_SIZ                            0x09U
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09U
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07U

#define AUDIO_DESCRIPTOR_TYPE                         0x21U
#define USB_DEVICE_CLASS_AUDIO                        0x01U
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01U
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02U
#define AUDIO_PROTOCOL_UNDEFINED                      0x00U
#define AUDIO_STREAMING_GENERAL                       0x01U
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02U

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24U
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25U

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01U
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02U
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03U
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06U

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0CU
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09U
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07U

#define AUDIO_CONTROL_MUTE                            0x0001U
#define AUDIO_CONTROL_VOL                             0x0002U

#define AUDIO_FORMAT_TYPE_I                           0x01U
#define AUDIO_FORMAT_TYPE_III                         0x03U

#define AUDIO_ENDPOINT_GENERAL                        0x01U

/* Audio Requests */
#define AUDIO_REQ_GET_CUR                             0x81U
#define AUDIO_REQ_GET_MIN                             0x82U
#define AUDIO_REQ_GET_MAX                             0x83U
#define AUDIO_REQ_GET_RES                             0x84U
#define AUDIO_REQ_SET_CUR                             0x01U
#define AUDIO_REQ_SET_MIN                             0x02U
#define AUDIO_REQ_SET_MAX                             0x03U
#define AUDIO_REQ_SET_RES                             0x04U

#define AUDIO_OUT_STREAMING_CTRL                      0x02U

/* Audio Control Requests (bmRequest & 0x1F) */
#define AUDIO_CONTROL_REQ                             0x01U
/* Feature Unit, UAC Spec 1.0 p.102 */
#define AUDIO_CONTROL_REQ_FU_MUTE                     0x01U
#define AUDIO_CONTROL_REQ_FU_VOL                      0x02U

/* Audio Streaming Requests */
#define AUDIO_STREAMING_REQ                           0x02U
#define AUDIO_STREAMING_REQ_FREQ_CTRL                 0x01U
#define AUDIO_STREAMING_REQ_PITCH_CTRL                0x02U

/* Max packet size: (freq/1000 + extra) * channels * bytes_per_sample
   e.g. 48kHz, 24bit: (48 + 1) * 2(stereo) * 3(24bit) = 294 bytes */
#define AUDIO_SAMPLES_PER_PACKET                      ((USBD_AUDIO_FREQ_MAX / 1000U) + 1U)
#define AUDIO_OUT_PACKET_24B                          (AUDIO_SAMPLES_PER_PACKET * 2U * 3U)

/* Feedback endpoint (IN) packet size: 3 bytes, 10.14 format */
#define AUDIO_IN_PACKET                               3U

/* Number of sub-packets in the audio transfer buffer (even, > 3).
   Larger values increase latency: playback starts at half-fill. */
#define AUDIO_OUT_PACKET_NUM                          16U

/* Ring buffer size, in 32-bit SAI words (2 words per stereo frame). */
#define AUDIO_BUF_WORDS                               (AUDIO_SAMPLES_PER_PACKET * 2U * AUDIO_OUT_PACKET_NUM)

/* Minimum writable frames before the writer risks overwriting unplayed data */
#define AUDIO_BUF_SAFEZONE_SAMPLES                    AUDIO_SAMPLES_PER_PACKET

/* Audio Commands enumeration */
typedef enum
{
  AUDIO_CMD_START = 1,
  AUDIO_CMD_PLAY,
  AUDIO_CMD_STOP,
} AUDIO_CMD_TypeDef;

typedef enum
{
  AUDIO_OFFSET_NONE = 0,
  AUDIO_OFFSET_HALF,
  AUDIO_OFFSET_FULL,
  AUDIO_OFFSET_UNKNOWN,
} AUDIO_OffsetTypeDef;
/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef struct
{
  uint8_t cmd;                    /* bRequest */
  uint8_t req_type;               /* bmRequest */
  uint8_t cs;                     /* wValue (high byte): Control Selector */
  uint8_t cn;                     /* wValue (low byte): Control Number */
  uint8_t unit;                   /* wIndex: Feature Unit / Interface / Endpoint */
  uint8_t len;                    /* wLength */
  uint8_t data[USB_MAX_EP0_SIZE]; /* Data */
} USBD_AUDIO_ControlTypeDef;

typedef struct
{
  uint32_t                  alt_setting;
  AUDIO_OffsetTypeDef       offset;
  uint8_t                   rd_enable;
  uint16_t                  wr_ptr;       /* ring buffer write index, in 32-bit words */
  uint32_t                  freq;
  int16_t                   volume;
  int32_t                   vol_3dB_shift; /* 3 dB attenuation steps equivalent to volume */
  uint8_t                   mute;          /* 0 = unmuted, 1 = muted */
  USBD_AUDIO_ControlTypeDef control;
} USBD_AUDIO_HandleTypeDef;

typedef struct
{
  int8_t (*Init)(uint32_t AudioFreq, int16_t Volume, uint8_t options);
  int8_t (*DeInit)(uint8_t options);
  int8_t (*AudioCmd)(uint32_t *pbuf, uint16_t size, uint8_t cmd);
  int8_t (*VolumeCtl)(int16_t vol);
  int8_t (*MuteCtl)(uint8_t cmd);
  int8_t (*PeriodicTC)(uint8_t cmd);
  int8_t (*GetState)(void);
} USBD_AUDIO_ItfTypeDef;
/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */
extern USBD_ClassTypeDef USBD_AUDIO;
#define USBD_AUDIO_CLASS &USBD_AUDIO
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops);

void USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset);

#ifdef USE_USBD_COMPOSITE
uint32_t USBD_AUDIO_GetEpPcktSze(USBD_HandleTypeDef *pdev, uint8_t If, uint8_t Ep);
#endif /* USE_USBD_COMPOSITE */

/**
  * @}
  */

/* Debug instrumentation (LED danger-zone indicator + LCD readout in main.c).
   Define AUDIO_DEBUG to enable; leave undefined for a clean production build. */
#ifdef AUDIO_DEBUG
extern volatile uint8_t  audio_dbg_danger;          /* 1 = ring buffer inside safe zone */
extern volatile uint32_t audio_dbg_drop_count;      /* packets dropped by safe-zone check */
extern volatile uint32_t audio_dbg_writable_samples;/* last writable sample count */
extern volatile uint32_t audio_dbg_min_free;        /* min writable samples observed */
extern volatile uint32_t audio_dbg_max_free;        /* max writable samples observed */
extern volatile uint32_t audio_dbg_freq;            /* current sample rate (Hz) */
#endif /* AUDIO_DEBUG */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_AUDIO_H */
/**
  * @}
  */

/**
  * @}
  */
