/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_audio_if.c
  * @version        : v1.0_Cube
  * @brief          : Generic media access layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
#include <math.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_AUDIO_IF
  * @{
  */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_FS(uint32_t options);
static int8_t AUDIO_AudioCmd_FS(uint8_t** packets, uint32_t count, uint8_t cmd, uint8_t sync);
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol);
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_FS(uint8_t cmd);
static int8_t AUDIO_GetState_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_FS =
{
  AUDIO_Init_FS,
  AUDIO_DeInit_FS,
  AUDIO_AudioCmd_FS,
  AUDIO_VolumeCtl_FS,
  AUDIO_MuteCtl_FS,
  AUDIO_PeriodicTC_FS,
  AUDIO_GetState_FS
};


#define IDLE_TIMEOUT_MS 1000
int idleCount = 0;
uint16_t theta = 0;
#define ZERO_LEVEL 2047U
#define MS_PER_BATCH AUDIO_PACKET_BATCH

static uint16_t __attribute__((aligned(8))) dmaLeftBuffer[2 * AUDIO_PACKET_BATCH * AUDIO_OUT_PACKET / 2 / 2];
static uint16_t __attribute__((aligned(8))) dmaRightBuffer[2 * AUDIO_PACKET_BATCH * AUDIO_OUT_PACKET / 2 / 2];

void updateDMABuffersIdle(int halve)
{
	const uint32_t samples = sizeof(dmaLeftBuffer) / sizeof(uint16_t) / 2;
		uint16_t *dstl = (uint16_t*)dmaLeftBuffer;
		dstl += halve ? samples : 0;
		uint16_t *dstr = (uint16_t*)dmaRightBuffer;
		dstr+= halve ? samples : 0;

	for(uint32_t i=0; i < samples; i++) {
		dstl[i] = theta;
		dstr[i] = theta;
		theta += 4;
	}
	/*
	// MCU is bit too slow to compute this
	const float stepf = 2 * M_PI / (USBD_AUDIO_FREQ / 50);
	for(uint32_t i=0; i < samples; i++) {
		dstl[i] = sinf(theta) * ZERO_LEVEL + ZERO_LEVEL;
		dstr[i] = cosf(theta) * ZERO_LEVEL + ZERO_LEVEL;
		theta += stepf;
	}
	*/
}

int updateDMABuffers(uint8_t* packets[], uint32_t count, int halve)
{
#define _16BTO12B(s) ((((int32_t)s + 32767) >> 4) & 0x0FFF)

	const int samplesPerPacket = AUDIO_OUT_PACKET / 2 / 2;
	uint16_t *dstl = (uint16_t*)dmaLeftBuffer;
	dstl += halve ? sizeof(dmaLeftBuffer) / sizeof(uint16_t) / 2 : 0;
	uint16_t *dstr = (uint16_t*)dmaRightBuffer;
	dstr += halve ? sizeof(dmaLeftBuffer) / sizeof(uint16_t) / 2 : 0;

	for(int i=0; i < count; i++) {
		uint16_t *packet = (uint16_t*)(packets[i]);
		for(int i=0; i < samplesPerPacket;i++) {
			dstl[i] = _16BTO12B(packet[i*2]);
			dstr[i] = _16BTO12B(packet[i*2+1]);
		}
		dstl += samplesPerPacket;
		dstr += samplesPerPacket;
	}
	for(int i=count; i < AUDIO_PACKET_BATCH; i++) {
		for(int j=0; j < samplesPerPacket; j++) {
			dstl[j] = dstr[j] = ZERO_LEVEL;
		}
		dstl += samplesPerPacket;
		dstr += samplesPerPacket;
	}
	return AUDIO_PACKET_BATCH * samplesPerPacket;

#undef _16BTO12B
}

void stopDMA()
{
	extern DAC_HandleTypeDef hdac;
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
}


void submitDMABuffers(int samples)
{
	extern DAC_HandleTypeDef hdac;
	stopDMA();
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dmaLeftBuffer, samples, DAC_ALIGN_12B_R);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)dmaRightBuffer, samples, DAC_ALIGN_12B_R);
}

void initDMA()
{
	const uint32_t n = sizeof(dmaLeftBuffer) / sizeof(uint16_t);

	for (uint32_t i = 0; i < n; i++) {
		dmaLeftBuffer[i] = ZERO_LEVEL;
		dmaRightBuffer[i] = ZERO_LEVEL;
	}
	submitDMABuffers(n);
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the AUDIO media low layer over USB FS IP
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 0 */
  return (USBD_OK);
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes the AUDIO media low layer
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_DeInit_FS(uint32_t options)
{
  /* USER CODE BEGIN 1 */
  return (USBD_OK);
  /* USER CODE END 1 */
}


/**
  * @brief  Handles AUDIO command.
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_AudioCmd_FS(uint8_t** packets, uint32_t count, uint8_t cmd, uint8_t sync)
{
  /* USER CODE BEGIN 2 */
	switch (cmd) {
	case AUDIO_CMD_START: // start from scratch
		//samples = updateDMABuffers(packets, count);
		updateDMABuffers(packets, count, 0);
		idleCount = 0;
		break;

	case AUDIO_CMD_IDLE:
	case AUDIO_CMD_PLAY: // update current buffer
		if (count == 0 && idleCount >= IDLE_TIMEOUT_MS) {
			idleCount = IDLE_TIMEOUT_MS;
			updateDMABuffersIdle(sync == AUDIO_SYNC_COMPLETE ? 1 : 0);
		} else {
			updateDMABuffers(packets, count, sync == AUDIO_SYNC_COMPLETE ? 1 : 0);
			if (count)
				idleCount = 0;
		}

		break;

	case AUDIO_CMD_STOP:
		break;
	}
	return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Controls AUDIO Volume.
  * @param  vol: volume level (0..100)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol)
{
  /* USER CODE BEGIN 3 */
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  Controls AUDIO Mute.
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  AUDIO_PeriodicT_FS
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_PeriodicTC_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Gets AUDIO State.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_GetState_FS(void)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Manages the DMA full transfer complete event.
  * @retval None
  */
void TransferComplete_CallBack_FS(void)
{
  /* USER CODE BEGIN 7 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_SYNC_COMPLETE);
  /* USER CODE END 7 */
}

/**
  * @brief  Manages the DMA Half transfer complete event.
  * @retval None
  */
void HalfTransfer_CallBack_FS(void)
{
  /* USER CODE BEGIN 8 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_SYNC_HALF);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);
  HalfTransfer_CallBack_FS();
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hdac);

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // Debug
	idleCount += MS_PER_BATCH;

	TransferComplete_CallBack_FS();
}

void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
