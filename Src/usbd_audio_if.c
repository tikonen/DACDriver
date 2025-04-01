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
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
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
int idleTimer = 0;
int isIdle = 0;
#define ZERO_LEVEL (2047U << 4) // 0x7FF
#define MS_PER_BATCH AUDIO_PACKET_BATCH
int fIdleDisabled = 0;

#define AUDIO_CHANNELS 2
#define AUDIO_SAMPLES_PER_CHANNEL  (AUDIO_PACKET_BATCH * AUDIO_OUT_PACKET / AUDIO_CHANNELS / sizeof(uint16_t))
#define DMA_SAMPLE_COUNT (INTERPOLATION_MUL * AUDIO_SAMPLES_PER_CHANNEL)

typedef struct
{
	uint16_t l;
	uint16_t r;
} DMASample;

// Interleaved dma buffer
static DMASample __attribute__((aligned(4))) dmaBuffer[2 /* circular double buffer */ * DMA_SAMPLE_COUNT];

// Sin wave
static const uint16_t __attribute__((aligned(4))) dmaIdleSinWaveBuffer[(USBD_AUDIO_FREQ
		/ 50)] = { 32752, 32960, 33168, 33392, 33600, 33808, 34032, 34240, 34464, 34672, 34880, 35104,
				   35312, 35520, 35744, 35952, 36160, 36384, 36592, 36800, 37024, 37232, 37440, 37648,
				   37872, 38080, 38288, 38496, 38720, 38928, 39136, 39344, 39552, 39760, 39968, 40176,
				   40384, 40592, 40800, 41008, 41216, 41424, 41632, 41840, 42048, 42256, 42464, 42656,
				   42864, 43072, 43264, 43472, 43680, 43872, 44080, 44288, 44480, 44688, 44880, 45072,
				   45280, 45472, 45680, 45872, 46064, 46256, 46448, 46656, 46848, 47040, 47232, 47424,
				   47616, 47808, 48000, 48176, 48368, 48560, 48752, 48928, 49120, 49312, 49488, 49680,
				   49856, 50032, 50224, 50400, 50576, 50768, 50944, 51120, 51296, 51472, 51648, 51824,
				   52000, 52176, 52336, 52512, 52688, 52848, 53024, 53184, 53360, 53520, 53680, 53856,
				   54016, 54176, 54336, 54496, 54656, 54816, 54976, 55136, 55296, 55440, 55600, 55744,
				   55904, 56048, 56208, 56352, 56496, 56656, 56800, 56944, 57088, 57232, 57376, 57504,
				   57648, 57792, 57920, 58064, 58192, 58336, 58464, 58592, 58720, 58864, 58992, 59120,
				   59248, 59360, 59488, 59616, 59728, 59856, 59984, 60096, 60208, 60336, 60448, 60560,
				   60672, 60784, 60896, 61008, 61104, 61216, 61328, 61424, 61520, 61632, 61728, 61824,
				   61920, 62016, 62112, 62208, 62304, 62400, 62480, 62576, 62672, 62752, 62832, 62928,
				   63008, 63088, 63168, 63248, 63328, 63392, 63472, 63552, 63616, 63696, 63760, 63824,
				   63888, 63952, 64016, 64080, 64144, 64208, 64272, 64320, 64384, 64432, 64496, 64544,
				   64592, 64640, 64688, 64736, 64784, 64832, 64864, 64912, 64944, 64992, 65024, 65056,
				   65088, 65120, 65152, 65184, 65216, 65248, 65264, 65296, 65312, 65344, 65360, 65376,
				   65392, 65408, 65424, 65440, 65456, 65456, 65472, 65472, 65488, 65488, 65488, 65488,
				   65504, 65488, 65488, 65488, 65488, 65472, 65472, 65456, 65456, 65440, 65424, 65408,
				   65392, 65376, 65360, 65344, 65312, 65296, 65264, 65248, 65216, 65184, 65152, 65120,
				   65088, 65056, 65024, 64992, 64944, 64912, 64864, 64832, 64784, 64736, 64688, 64640,
				   64592, 64544, 64496, 64432, 64384, 64320, 64272, 64208, 64144, 64080, 64016, 63952,
				   63888, 63824, 63760, 63696, 63616, 63552, 63472, 63392, 63328, 63248, 63168, 63088,
				   63008, 62928, 62832, 62752, 62672, 62576, 62480, 62400, 62304, 62208, 62112, 62016,
				   61920, 61824, 61728, 61632, 61520, 61424, 61328, 61216, 61104, 61008, 60896, 60784,
				   60672, 60560, 60448, 60336, 60208, 60096, 59984, 59856, 59728, 59616, 59488, 59360,
				   59248, 59120, 58992, 58864, 58720, 58592, 58464, 58336, 58192, 58064, 57920, 57792,
				   57648, 57504, 57376, 57232, 57088, 56944, 56800, 56656, 56496, 56352, 56208, 56048,
				   55904, 55744, 55600, 55440, 55296, 55136, 54976, 54816, 54656, 54496, 54336, 54176,
				   54016, 53856, 53680, 53520, 53360, 53184, 53024, 52848, 52688, 52512, 52336, 52176,
				   52000, 51824, 51648, 51472, 51296, 51120, 50944, 50768, 50576, 50400, 50224, 50032,
				   49856, 49680, 49488, 49312, 49120, 48928, 48752, 48560, 48368, 48176, 48000, 47808,
				   47616, 47424, 47232, 47040, 46848, 46656, 46448, 46256, 46064, 45872, 45680, 45472,
				   45280, 45072, 44880, 44688, 44480, 44288, 44080, 43872, 43680, 43472, 43264, 43072,
				   42864, 42656, 42464, 42256, 42048, 41840, 41632, 41424, 41216, 41008, 40800, 40592,
				   40384, 40176, 39968, 39760, 39552, 39344, 39136, 38928, 38720, 38496, 38288, 38080,
				   37872, 37648, 37440, 37232, 37024, 36800, 36592, 36384, 36160, 35952, 35744, 35520,
				   35312, 35104, 34880, 34672, 34464, 34240, 34032, 33808, 33600, 33392, 33168, 32960,
				   32736, 32528, 32320, 32096, 31888, 31680, 31456, 31248, 31024, 30816, 30608, 30384,
				   30176, 29968, 29744, 29536, 29328, 29104, 28896, 28688, 28464, 28256, 28048, 27824,
				   27616, 27408, 27200, 26992, 26768, 26560, 26352, 26144, 25936, 25728, 25520, 25312,
				   25104, 24896, 24688, 24480, 24272, 24064, 23856, 23648, 23440, 23232, 23024, 22832,
				   22624, 22416, 22208, 22016, 21808, 21616, 21408, 21200, 21008, 20800, 20608, 20416,
				   20208, 20016, 19808, 19616, 19424, 19232, 19024, 18832, 18640, 18448, 18256, 18064,
				   17872, 17680, 17488, 17312, 17120, 16928, 16736, 16560, 16368, 16176, 16000, 15808,
				   15632, 15456, 15264, 15088, 14912, 14720, 14544, 14368, 14192, 14016, 13840, 13664,
				   13488, 13312, 13152, 12976, 12800, 12640, 12464, 12304, 12128, 11968, 11808, 11632,
				   11472, 11312, 11152, 10992, 10832, 10672, 10512, 10352, 10192, 10048, 9888, 9744,
				   9584, 9440, 9280, 9136, 8992, 8832, 8688, 8544, 8400, 8256, 8112, 7984, 7840, 7696,
				   7568, 7424, 7296, 7152, 7024, 6896, 6752, 6624, 6496, 6368, 6240, 6128, 6000, 5872,
				   5744, 5632, 5504, 5392, 5280, 5152, 5040, 4928, 4816, 4704, 4592, 4480, 4384, 4272,
				   4160, 4064, 3968, 3856, 3760, 3664, 3568, 3472, 3376, 3280, 3184, 3088, 3008, 2912,
				   2816, 2736, 2656, 2560, 2480, 2400, 2320, 2240, 2160, 2096, 2016, 1936, 1872, 1792,
				   1728, 1664, 1600, 1536, 1472, 1408, 1344, 1280, 1216, 1168, 1104, 1056, 992, 944, 896,
				   848, 800, 752, 704, 656, 624, 576, 544, 496, 464, 432, 400, 368, 336, 304, 272, 240,
				   224, 192, 176, 144, 128, 112, 96, 80, 64, 48, 32, 32, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0,
				   0, 16, 16, 32, 32, 48, 64, 80, 96, 112, 128, 144, 176, 192, 224, 240, 272, 304, 336,
				   368, 400, 432, 464, 496, 544, 576, 624, 672, 704, 752, 800, 848, 896, 944, 1008, 1056,
				   1104, 1168, 1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728, 1808, 1872, 1936,
				   2016, 2096, 2160, 2240, 2320, 2400, 2480, 2576, 2656, 2736, 2832, 2912, 3008, 3088,
				   3184, 3280, 3376, 3472, 3568, 3664, 3760, 3856, 3968, 4064, 4176, 4272, 4384, 4496,
				   4592, 4704, 4816, 4928, 5040, 5168, 5280, 5392, 5520, 5632, 5760, 5872, 6000, 6128,
				   6240, 6368, 6496, 6624, 6768, 6896, 7024, 7152, 7296, 7424, 7568, 7696, 7840, 7984,
				   8128, 8256, 8400, 8544, 8688, 8848, 8992, 9136, 9280, 9440, 9584, 9744, 9888, 10048,
				   10192, 10352, 10512, 10672, 10832, 10992, 11152, 11312, 11472, 11632, 11808, 11968,
				   12128, 12304, 12464, 12640, 12800, 12976, 13152, 13328, 13488, 13664, 13840, 14016,
				   14192, 14368, 14544, 14720, 14912, 15088, 15264, 15456, 15632, 15808, 16000, 16176,
				   16368, 16560, 16736, 16928, 17120, 17312, 17488, 17680, 17872, 18064, 18256, 18448,
				   18640, 18832, 19040, 19232, 19424, 19616, 19824, 20016, 20208, 20416, 20608, 20816,
				   21008, 21216, 21408, 21616, 21808, 22016, 22224, 22416, 22624, 22832, 23040, 23232,
				   23440, 23648, 23856, 24064, 24272, 24480, 24688, 24896, 25104, 25312, 25520, 25728,
				   25936, 26144, 26352, 26560, 26784, 26992, 27200, 27408, 27616, 27840, 28048, 28256,
				   28464, 28688, 28896, 29104, 29328, 29536, 29744, 29968, 30176, 30384, 30608, 30816,
				   31024, 31248, 31456, 31680, 31888, 32096, 32320, 32528 };


// fills buffer with sine-wave when there is nothing coming from USB
void updateDMABuffersIdle(int halve)
{
	static int thetaSin = 0;
	static int thetaCos = ARRAYSIZE(dmaIdleSinWaveBuffer) / 4; // 90 degrees phase delay. cos(t) = sin(t + pi/2)
	static int skipAmount = 4;
	static int skipRotation = 0;

	DMASample *dst = dmaBuffer;
	dst += halve ? DMA_SAMPLE_COUNT : 0;

	for (int i = 0; i < DMA_SAMPLE_COUNT; i += INTERPOLATION_MUL)
	{
#if 1  // chain animation
	    if((thetaSin + skipRotation) % skipAmount == 0) {
	        thetaSin = (thetaSin + skipAmount / 2) % ARRAYSIZE(dmaIdleSinWaveBuffer);
	        thetaCos = (thetaCos + skipAmount / 2) % ARRAYSIZE(dmaIdleSinWaveBuffer);
	    }
#endif

		uint16_t sv = dst[i].l = dmaIdleSinWaveBuffer[thetaSin];
		uint16_t cv = dst[i].r = dmaIdleSinWaveBuffer[thetaCos];
		thetaSin++;
		thetaCos++;
		if(thetaSin == ARRAYSIZE(dmaIdleSinWaveBuffer)) thetaSin = 0;
		if(thetaCos == ARRAYSIZE(dmaIdleSinWaveBuffer)) thetaCos = 0;
		int16_t ld = dmaIdleSinWaveBuffer[thetaSin] - sv;
		int16_t rd = dmaIdleSinWaveBuffer[thetaCos] - cv;
		for(int j=1; j < INTERPOLATION_MUL; j++) {
			dst[i+j].l = (j*ld)/INTERPOLATION_MUL + sv;
			dst[i+j].r = (j*rd)/INTERPOLATION_MUL + cv;
		}
	}
	skipRotation++;
	if(skipRotation >= skipAmount) skipRotation = 0;
}

typedef struct {
    int16_t l; // left channel
    int16_t r; // right channel
} AudioSample;

// Generates steps to draw a box following edges
void buildFrameSteps(uint16_t steps[][2], int count) {

    const int min = 0;
    const int max = (1 << 16) - 1; // max 16bit
    int s = 0;
    const int step = (max - min)/(count / 4);
    for(int i=0 ; i < count / 4; i++, s++) {
        steps[s][0] = min;
        steps[s][1] = step * i + min;
        steps[s+count/2][0] = max;
        steps[s+count/2][1] = max - step * i + min;
    }
    for(int i=0 ; i < count / 4; i++, s++) {
        steps[s][0] = step * i + min;
        steps[s][1] = max;
        steps[s+count/2][0] = max - step * i + min;
        steps[s+count/2][1] = min;
    }
}

#define DISABLE_INTERPOLATE 0
#define SYNC_THRESHOLD 1
#define FRAMESTEPCOUNT 64
uint16_t frameSteps[FRAMESTEPCOUNT][2];

// This function can process a batch or double batch
int updateDMABuffers(uint8_t* packets[], uint32_t count, int halve)
{
//#define _16BTO12B(s) ((((int32_t)(s) + 32767) >> 4) & 0x0FFF)
#define _NORMALIZE(s) ((s) + 32767)

	const int samplesPerPacket = AUDIO_OUT_PACKET / sizeof(uint16_t) / AUDIO_CHANNELS;
	//static_assert(samplesPerPacket % INTERPOLATION_MUL == 0);

	DMASample *dst = dmaBuffer;
	dst += halve ? DMA_SAMPLE_COUNT : 0;

	static AudioSample prev = { .l = 0, .r = 0 };
	unsigned int dstidx = 0;
	static int syncPoint = 0;
	for (unsigned int p = 0; p < count; p++)
	{
		const AudioSample *packet = (AudioSample*) (packets[p]);

		for(int i = 0; i < samplesPerPacket; i++) {
           const AudioSample *sample = &packet[i];
#if DISABLE_INTERPOLATE || (INTERPOLATION_MUL == 1)
           for (int k = 0; k < INTERPOLATION_MUL; k++) {
                dst[dstidx + k].l = _NORMALIZE(sample->l);
                dst[dstidx + k].r = _NORMALIZE(sample->r);
           }
           dstidx += INTERPOLATION_MUL;
#else
           // if point is very close to 0 then treat it as a sync point signal
           if(abs(sample->l) <= SYNC_THRESHOLD && abs(sample->r) <= SYNC_THRESHOLD) {
               syncPoint = 1;
               for(int k=0; k < INTERPOLATION_MUL; k++) {
                   dst[dstidx + k].l = _NORMALIZE(prev.l);
                   dst[dstidx + k].r = _NORMALIZE(prev.r);
               }
               dstidx += INTERPOLATION_MUL;
               continue;
           }
            if(!syncPoint) {
                // sample has low bit on, interpolate line from the previous point.
            	const int16_t ld = (sample->l - prev.l);
            	const int16_t rd = (sample->r - prev.r);
                for(int k=1; k < INTERPOLATION_MUL; k++) {
                    dst[dstidx+k-1].l = _NORMALIZE((k * ld)/INTERPOLATION_MUL + prev.l);
                    dst[dstidx+k-1].r = _NORMALIZE((k * rd)/INTERPOLATION_MUL + prev.r);
                }
                dst[dstidx + INTERPOLATION_MUL - 1].l = _NORMALIZE(sample->l);
                dst[dstidx + INTERPOLATION_MUL - 1].r = _NORMALIZE(sample->r);
                dstidx += INTERPOLATION_MUL;
            } else {
                // sample starts a new path, skip interpolation
                for (int k = 0; k < INTERPOLATION_MUL; k++) {
                    dst[dstidx + k].l = _NORMALIZE(sample->l);
                    dst[dstidx + k].r = _NORMALIZE(sample->r);
                }
                dstidx += INTERPOLATION_MUL;
                syncPoint = 0;
            }

            prev = *sample;
#endif
        }

	}

	// zero out rest of the samples in DMA buffer if there is not enough packets in the batch
	if(count > AUDIO_PACKET_BATCH) count -= AUDIO_PACKET_BATCH;
	const int zeroidle = 0;
    unsigned int total = samplesPerPacket * INTERPOLATION_MUL * AUDIO_PACKET_BATCH;
    if(dstidx < total) {
        if(zeroidle) {
            while(dstidx < total) {
                dst[dstidx].l = ZERO_LEVEL;
                dst[dstidx].r = ZERO_LEVEL;
                dstidx++;
            }
        } else {
            static unsigned int step = 0;

            while(dstidx < total) {
                dst[dstidx].l = frameSteps[step % FRAMESTEPCOUNT][0];
                dst[dstidx].r = frameSteps[step % FRAMESTEPCOUNT][1];
                step++;
                dstidx++;
            }
        }

        prev.l = 0;
        prev.r = 0;
    }

	if (count <= AUDIO_PACKET_BATCH)
		return INTERPOLATION_MUL * AUDIO_PACKET_BATCH * samplesPerPacket * 2;
	else
		return INTERPOLATION_MUL * 2 * AUDIO_PACKET_BATCH * samplesPerPacket * 2;

#undef _NORMALIZE
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
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dmaBuffer, samples, DAC_ALIGN_12B_LD);
	HAL_DAC_Start_Dual_DMA(&hdac, (uint32_t*)dmaBuffer, samples, DAC_ALIGN_12B_LD);
	//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dmaLeftBuffer, samples, DAC_ALIGN_12B_L);
	//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)dmaRightBuffer, samples, DAC_ALIGN_12B_L);
}

void initDMA(int idleDisabled)
{
    buildFrameSteps(frameSteps, FRAMESTEPCOUNT);

	fIdleDisabled = idleDisabled;
	const uint32_t n = DMA_SAMPLE_COUNT * 2;

	for (uint32_t i = 0; i < n; i++)
	{
		dmaBuffer[i].l = ZERO_LEVEL;
		dmaBuffer[i].r = ZERO_LEVEL;
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

#define CMD_QUEUE_LEN 4

struct AudioCommand
{
	uint8_t *packets[AUDIO_PACKET_BATCH*2];
	uint32_t count;
	uint8_t sync;
	uint8_t cmd;
} audioCommands[CMD_QUEUE_LEN] __attribute__((section(".ccmram")));
uint32_t cmdReadIdx = 0;
uint32_t cmdWriteIdx = 0;

// Called from main loop, interrupt may happen at any time.
void Process_Audio_Command()
{
	if(cmdWriteIdx - cmdReadIdx == 0) return;

	struct AudioCommand *cmd = &audioCommands[cmdReadIdx % CMD_QUEUE_LEN];
	cmdReadIdx++;

	switch (cmd->cmd)
	{
	case AUDIO_CMD_START: // start from scratch
		LOG("START");

		//NOTE we assume that start provides 2 * AUDIO_PACKET_BATCH of packets, otherwise
		// the dma half/full interrupt update logic won't work.
		stopDMA();
		int samples = updateDMABuffers(cmd->packets, cmd->count, 0);
		submitDMABuffers(samples);
		idleTimer = 0;
		isIdle = 0;
		break;

	case AUDIO_CMD_IDLE:
	case AUDIO_CMD_PLAY: // update current buffer

		if(!fIdleDisabled) {
			if(idleTimer >= IDLE_TIMEOUT_MS && !isIdle && cmd->count == 0) {
				isIdle = 1;
				LOG("IDLE");
			} else if(cmd->count != 0 && isIdle) {
				isIdle = 0;
				LOG("RESUME");
			}
		}

		if (isIdle)
		{
			idleTimer = IDLE_TIMEOUT_MS;
			updateDMABuffersIdle(cmd->sync == AUDIO_SYNC_COMPLETE ? 1 : 0);
		}
		else
		{
			//LOG(".");
			updateDMABuffers(cmd->packets, cmd->count, cmd->sync == AUDIO_SYNC_COMPLETE ? 1 : 0);
			if (cmd->count)
				idleTimer = 0;
		}
		HAL_GPIO_WritePin(LED_PORT, BLUE_LED_PIN, isIdle);
		HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, !isIdle);
		break;

	case AUDIO_CMD_STOP:
		break;
	}
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
	// Audio commands needs heavy data processing so put it away in ring buffer and let main
	// loop handle it. Don't use cycles in an interrupt routine.

	struct AudioCommand *acmd = &audioCommands[cmdWriteIdx % CMD_QUEUE_LEN];

	acmd->count = count;
	acmd->sync = sync;
	acmd->cmd = cmd;
	if(packets) memcpy(acmd->packets, packets, sizeof(uint8_t*)*count);

	cmdWriteIdx++;

	//Process_Audio_Command();

  	return (USBD_OK);
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

// Only channel1 callbacks are used as both DAC channels are run with the same
// DMA transfer.

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  //HAL_GPIO_TogglePin(LED_PORT, ORANGE_LED_PIN); // Debug
  idleTimer += MS_PER_BATCH;

  HalfTransfer_CallBack_FS();
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hdac);

	HAL_GPIO_TogglePin(LED_PORT, ORANGE_LED_PIN); // Debug
	idleTimer += MS_PER_BATCH;

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
