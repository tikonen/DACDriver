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
int idleTimer = 0;
#define ZERO_LEVEL 2047U
#define MS_PER_BATCH AUDIO_PACKET_BATCH
int fIdleDisabled = 0;

#define AUDIO_SAMPLES_PER_CHANNEL  (AUDIO_PACKET_BATCH * AUDIO_OUT_PACKET / 2 / 2)
static uint16_t __attribute__((aligned(4))) dmaLeftBuffer[2 * 2 * AUDIO_SAMPLES_PER_CHANNEL];
static uint16_t __attribute__((aligned(4))) dmaRightBuffer[2 * 2 * AUDIO_SAMPLES_PER_CHANNEL];

/*
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define ZERO_LEVEL 2047U

void foo()
{
    const int USBD_AUDIO_FREQ = 48000;
    const unsigned int samples = (USBD_AUDIO_FREQ / 50);
    const float stepf = 2 * M_PI / samples;
    float theta = 0;
    printf("SIN = {");
    for (unsigned int i = 0; i < samples; i++) {
        printf("%u,", (uint16_t)(sinf(theta) * ZERO_LEVEL + ZERO_LEVEL));
        theta += stepf;
    }
    printf("};\n");
    printf("\nCOS = {\n");
    theta = 0;
    for (unsigned int i = 0; i < samples; i++) {
        printf("%u,", (uint16_t)(cosf(theta) * ZERO_LEVEL + ZERO_LEVEL));
        theta += stepf;
    }
    printf("};\n");
}
*/

// Sin wave
static const uint16_t __attribute__((aligned(4))) dmaIdleSinWaveBuffer[(USBD_AUDIO_FREQ
		/ 50)] = { 2047, 2060, 2073, 2087, 2100, 2113, 2127, 2140, 2154, 2167,
		2180, 2194, 2207, 2220, 2234, 2247, 2260, 2274, 2287, 2300, 2314, 2327,
		2340, 2353, 2367, 2380, 2393, 2406, 2420, 2433, 2446, 2459, 2472, 2485,
		2498, 2511, 2524, 2537, 2550, 2563, 2576, 2589, 2602, 2615, 2628, 2641,
		2654, 2666, 2679, 2692, 2704, 2717, 2730, 2742, 2755, 2768, 2780, 2793,
		2805, 2817, 2830, 2842, 2855, 2867, 2879, 2891, 2903, 2916, 2928, 2940,
		2952, 2964, 2976, 2988, 3000, 3011, 3023, 3035, 3047, 3058, 3070, 3082,
		3093, 3105, 3116, 3127, 3139, 3150, 3161, 3173, 3184, 3195, 3206, 3217,
		3228, 3239, 3250, 3261, 3271, 3282, 3293, 3303, 3314, 3324, 3335, 3345,
		3355, 3366, 3376, 3386, 3396, 3406, 3416, 3426, 3436, 3446, 3456, 3465,
		3475, 3484, 3494, 3503, 3513, 3522, 3531, 3541, 3550, 3559, 3568, 3577,
		3586, 3594, 3603, 3612, 3620, 3629, 3637, 3646, 3654, 3662, 3670, 3679,
		3687, 3695, 3703, 3710, 3718, 3726, 3733, 3741, 3749, 3756, 3763, 3771,
		3778, 3785, 3792, 3799, 3806, 3813, 3819, 3826, 3833, 3839, 3845, 3852,
		3858, 3864, 3870, 3876, 3882, 3888, 3894, 3900, 3905, 3911, 3917, 3922,
		3927, 3933, 3938, 3943, 3948, 3953, 3958, 3962, 3967, 3972, 3976, 3981,
		3985, 3989, 3993, 3997, 4001, 4005, 4009, 4013, 4017, 4020, 4024, 4027,
		4031, 4034, 4037, 4040, 4043, 4046, 4049, 4052, 4054, 4057, 4059, 4062,
		4064, 4066, 4068, 4070, 4072, 4074, 4076, 4078, 4079, 4081, 4082, 4084,
		4085, 4086, 4087, 4088, 4089, 4090, 4091, 4091, 4092, 4092, 4093, 4093,
		4093, 4093, 4094, 4093, 4093, 4093, 4093, 4092, 4092, 4091, 4091, 4090,
		4089, 4088, 4087, 4086, 4085, 4084, 4082, 4081, 4079, 4078, 4076, 4074,
		4072, 4070, 4068, 4066, 4064, 4062, 4059, 4057, 4054, 4052, 4049, 4046,
		4043, 4040, 4037, 4034, 4031, 4027, 4024, 4020, 4017, 4013, 4009, 4005,
		4001, 3997, 3993, 3989, 3985, 3981, 3976, 3972, 3967, 3962, 3958, 3953,
		3948, 3943, 3938, 3933, 3927, 3922, 3917, 3911, 3905, 3900, 3894, 3888,
		3882, 3876, 3870, 3864, 3858, 3852, 3845, 3839, 3833, 3826, 3819, 3813,
		3806, 3799, 3792, 3785, 3778, 3771, 3763, 3756, 3749, 3741, 3733, 3726,
		3718, 3710, 3703, 3695, 3687, 3679, 3670, 3662, 3654, 3646, 3637, 3629,
		3620, 3612, 3603, 3594, 3586, 3577, 3568, 3559, 3550, 3541, 3531, 3522,
		3513, 3503, 3494, 3484, 3475, 3465, 3456, 3446, 3436, 3426, 3416, 3406,
		3396, 3386, 3376, 3366, 3355, 3345, 3335, 3324, 3314, 3303, 3293, 3282,
		3271, 3261, 3250, 3239, 3228, 3217, 3206, 3195, 3184, 3173, 3161, 3150,
		3139, 3127, 3116, 3105, 3093, 3082, 3070, 3058, 3047, 3035, 3023, 3011,
		3000, 2988, 2976, 2964, 2952, 2940, 2928, 2916, 2903, 2891, 2879, 2867,
		2855, 2842, 2830, 2817, 2805, 2793, 2780, 2768, 2755, 2742, 2730, 2717,
		2704, 2692, 2679, 2666, 2654, 2641, 2628, 2615, 2602, 2589, 2576, 2563,
		2550, 2537, 2524, 2511, 2498, 2485, 2472, 2459, 2446, 2433, 2420, 2406,
		2393, 2380, 2367, 2353, 2340, 2327, 2314, 2300, 2287, 2274, 2260, 2247,
		2234, 2220, 2207, 2194, 2180, 2167, 2154, 2140, 2127, 2113, 2100, 2087,
		2073, 2060, 2046, 2033, 2020, 2006, 1993, 1980, 1966, 1953, 1939, 1926,
		1913, 1899, 1886, 1873, 1859, 1846, 1833, 1819, 1806, 1793, 1779, 1766,
		1753, 1739, 1726, 1713, 1700, 1687, 1673, 1660, 1647, 1634, 1621, 1608,
		1595, 1582, 1569, 1556, 1543, 1530, 1517, 1504, 1491, 1478, 1465, 1452,
		1439, 1427, 1414, 1401, 1388, 1376, 1363, 1351, 1338, 1325, 1313, 1300,
		1288, 1276, 1263, 1251, 1238, 1226, 1214, 1202, 1189, 1177, 1165, 1153,
		1141, 1129, 1117, 1105, 1093, 1082, 1070, 1058, 1046, 1035, 1023, 1011,
		1000, 988, 977, 966, 954, 943, 932, 920, 909, 898, 887, 876, 865, 854,
		843, 832, 822, 811, 800, 790, 779, 769, 758, 748, 738, 727, 717, 707,
		697, 687, 677, 667, 657, 647, 637, 628, 618, 609, 599, 590, 580, 571,
		562, 552, 543, 534, 525, 516, 507, 499, 490, 481, 473, 464, 456, 447,
		439, 431, 422, 414, 406, 398, 390, 383, 375, 367, 359, 352, 344, 337,
		330, 322, 315, 308, 301, 294, 287, 280, 274, 267, 260, 254, 248, 241,
		235, 229, 223, 217, 211, 205, 199, 193, 188, 182, 176, 171, 166, 160,
		155, 150, 145, 140, 135, 131, 126, 121, 117, 112, 108, 104, 100, 96, 92,
		88, 84, 80, 76, 73, 69, 66, 62, 59, 56, 53, 50, 47, 44, 41, 39, 36, 34,
		31, 29, 27, 25, 23, 21, 19, 17, 15, 14, 12, 11, 9, 8, 7, 6, 5, 4, 3, 2,
		2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 11,
		12, 14, 15, 17, 19, 21, 23, 25, 27, 29, 31, 34, 36, 39, 42, 44, 47, 50,
		53, 56, 59, 63, 66, 69, 73, 76, 80, 84, 88, 92, 96, 100, 104, 108, 113,
		117, 121, 126, 131, 135, 140, 145, 150, 155, 161, 166, 171, 177, 182,
		188, 193, 199, 205, 211, 217, 223, 229, 235, 241, 248, 254, 261, 267,
		274, 281, 287, 294, 301, 308, 315, 323, 330, 337, 345, 352, 360, 367,
		375, 383, 390, 398, 406, 414, 423, 431, 439, 447, 456, 464, 473, 481,
		490, 499, 508, 516, 525, 534, 543, 553, 562, 571, 580, 590, 599, 609,
		618, 628, 637, 647, 657, 667, 677, 687, 697, 707, 717, 727, 738, 748,
		758, 769, 779, 790, 800, 811, 822, 833, 843, 854, 865, 876, 887, 898,
		909, 920, 932, 943, 954, 966, 977, 988, 1000, 1011, 1023, 1035, 1046,
		1058, 1070, 1082, 1093, 1105, 1117, 1129, 1141, 1153, 1165, 1177, 1190,
		1202, 1214, 1226, 1239, 1251, 1263, 1276, 1288, 1301, 1313, 1326, 1338,
		1351, 1363, 1376, 1389, 1401, 1414, 1427, 1440, 1452, 1465, 1478, 1491,
		1504, 1517, 1530, 1543, 1556, 1569, 1582, 1595, 1608, 1621, 1634, 1647,
		1660, 1674, 1687, 1700, 1713, 1726, 1740, 1753, 1766, 1779, 1793, 1806,
		1819, 1833, 1846, 1859, 1873, 1886, 1899, 1913, 1926, 1939, 1953, 1966,
		1980, 1993, 2006, 2020, 2033 };

#if 0
// 50Hz Cos wave
static const uint16_t __attribute__((aligned(4))) dmaIdleCosWaveBuffer[(USBD_AUDIO_FREQ
		/ 50)] = { 4094, 4093, 4093, 4093, 4093, 4092, 4092, 4091, 4091, 4090,
		4089, 4088, 4087, 4086, 4085, 4084, 4082, 4081, 4079, 4078, 4076, 4074,
		4072, 4070, 4068, 4066, 4064, 4062, 4059, 4057, 4054, 4052, 4049, 4046,
		4043, 4040, 4037, 4034, 4031, 4027, 4024, 4020, 4017, 4013, 4009, 4005,
		4001, 3997, 3993, 3989, 3985, 3981, 3976, 3972, 3967, 3962, 3958, 3953,
		3948, 3943, 3938, 3933, 3927, 3922, 3917, 3911, 3905, 3900, 3894, 3888,
		3882, 3876, 3870, 3864, 3858, 3852, 3845, 3839, 3832, 3826, 3819, 3813,
		3806, 3799, 3792, 3785, 3778, 3771, 3763, 3756, 3749, 3741, 3733, 3726,
		3718, 3710, 3703, 3695, 3687, 3679, 3670, 3662, 3654, 3646, 3637, 3629,
		3620, 3612, 3603, 3594, 3586, 3577, 3568, 3559, 3550, 3541, 3531, 3522,
		3513, 3503, 3494, 3484, 3475, 3465, 3456, 3446, 3436, 3426, 3416, 3406,
		3396, 3386, 3376, 3366, 3355, 3345, 3335, 3324, 3314, 3303, 3293, 3282,
		3271, 3261, 3250, 3239, 3228, 3217, 3206, 3195, 3184, 3173, 3161, 3150,
		3139, 3127, 3116, 3105, 3093, 3082, 3070, 3058, 3047, 3035, 3023, 3011,
		3000, 2988, 2976, 2964, 2952, 2940, 2928, 2916, 2903, 2891, 2879, 2867,
		2855, 2842, 2830, 2817, 2805, 2793, 2780, 2768, 2755, 2742, 2730, 2717,
		2704, 2692, 2679, 2666, 2654, 2641, 2628, 2615, 2602, 2589, 2576, 2563,
		2550, 2537, 2524, 2511, 2498, 2485, 2472, 2459, 2446, 2433, 2420, 2406,
		2393, 2380, 2367, 2353, 2340, 2327, 2314, 2300, 2287, 2274, 2260, 2247,
		2234, 2220, 2207, 2194, 2180, 2167, 2154, 2140, 2127, 2113, 2100, 2087,
		2073, 2060, 2047, 2033, 2020, 2006, 1993, 1980, 1966, 1953, 1939, 1926,
		1913, 1899, 1886, 1873, 1859, 1846, 1833, 1819, 1806, 1793, 1779, 1766,
		1753, 1740, 1726, 1713, 1700, 1687, 1673, 1660, 1647, 1634, 1621, 1608,
		1595, 1582, 1569, 1556, 1543, 1530, 1517, 1504, 1491, 1478, 1465, 1452,
		1439, 1427, 1414, 1401, 1389, 1376, 1363, 1351, 1338, 1325, 1313, 1300,
		1288, 1276, 1263, 1251, 1238, 1226, 1214, 1202, 1190, 1177, 1165, 1153,
		1141, 1129, 1117, 1105, 1093, 1082, 1070, 1058, 1046, 1035, 1023, 1011,
		1000, 988, 977, 966, 954, 943, 932, 920, 909, 898, 887, 876, 865, 854,
		843, 832, 822, 811, 800, 790, 779, 769, 758, 748, 738, 727, 717, 707,
		697, 687, 677, 667, 657, 647, 637, 628, 618, 609, 599, 590, 580, 571,
		562, 552, 543, 534, 525, 516, 507, 499, 490, 481, 473, 464, 456, 447,
		439, 431, 423, 414, 406, 398, 390, 383, 375, 367, 360, 352, 344, 337,
		330, 322, 315, 308, 301, 294, 287, 280, 274, 267, 260, 254, 248, 241,
		235, 229, 223, 217, 211, 205, 199, 193, 188, 182, 176, 171, 166, 160,
		155, 150, 145, 140, 135, 131, 126, 121, 117, 112, 108, 104, 100, 96, 92,
		88, 84, 80, 76, 73, 69, 66, 62, 59, 56, 53, 50, 47, 44, 41, 39, 36, 34,
		31, 29, 27, 25, 23, 21, 19, 17, 15, 14, 12, 11, 9, 8, 7, 6, 5, 4, 3, 2,
		2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 11,
		12, 14, 15, 17, 19, 21, 23, 25, 27, 29, 31, 34, 36, 39, 41, 44, 47, 50,
		53, 56, 59, 62, 66, 69, 73, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112,
		117, 121, 126, 131, 135, 140, 145, 150, 155, 160, 166, 171, 176, 182,
		188, 193, 199, 205, 211, 217, 223, 229, 235, 241, 248, 254, 261, 267,
		274, 280, 287, 294, 301, 308, 315, 323, 330, 337, 345, 352, 360, 367,
		375, 383, 390, 398, 406, 414, 423, 431, 439, 447, 456, 464, 473, 481,
		490, 499, 508, 516, 525, 534, 543, 552, 562, 571, 580, 590, 599, 609,
		618, 628, 637, 647, 657, 667, 677, 687, 697, 707, 717, 727, 738, 748,
		758, 769, 779, 790, 800, 811, 822, 833, 843, 854, 865, 876, 887, 898,
		909, 920, 932, 943, 954, 966, 977, 988, 1000, 1011, 1023, 1035, 1046,
		1058, 1070, 1082, 1093, 1105, 1117, 1129, 1141, 1153, 1165, 1177, 1190,
		1202, 1214, 1226, 1239, 1251, 1263, 1276, 1288, 1300, 1313, 1325, 1338,
		1351, 1363, 1376, 1389, 1401, 1414, 1427, 1440, 1452, 1465, 1478, 1491,
		1504, 1517, 1530, 1543, 1556, 1569, 1582, 1595, 1608, 1621, 1634, 1647,
		1660, 1674, 1687, 1700, 1713, 1726, 1740, 1753, 1766, 1779, 1793, 1806,
		1819, 1833, 1846, 1859, 1873, 1886, 1899, 1913, 1926, 1939, 1953, 1966,
		1980, 1993, 2006, 2020, 2033, 2047, 2060, 2073, 2087, 2100, 2114, 2127,
		2140, 2154, 2167, 2180, 2194, 2207, 2221, 2234, 2247, 2261, 2274, 2287,
		2300, 2314, 2327, 2340, 2354, 2367, 2380, 2393, 2406, 2420, 2433, 2446,
		2459, 2472, 2485, 2498, 2511, 2524, 2537, 2550, 2563, 2576, 2589, 2602,
		2615, 2628, 2641, 2654, 2666, 2679, 2692, 2705, 2717, 2730, 2742, 2755,
		2768, 2780, 2793, 2805, 2818, 2830, 2842, 2855, 2867, 2879, 2891, 2904,
		2916, 2928, 2940, 2952, 2964, 2976, 2988, 3000, 3012, 3023, 3035, 3047,
		3058, 3070, 3082, 3093, 3105, 3116, 3128, 3139, 3150, 3161, 3173, 3184,
		3195, 3206, 3217, 3228, 3239, 3250, 3261, 3271, 3282, 3293, 3303, 3314,
		3324, 3335, 3345, 3355, 3366, 3376, 3386, 3396, 3406, 3416, 3426, 3436,
		3446, 3456, 3465, 3475, 3485, 3494, 3503, 3513, 3522, 3531, 3541, 3550,
		3559, 3568, 3577, 3586, 3594, 3603, 3612, 3620, 3629, 3637, 3646, 3654,
		3662, 3671, 3679, 3687, 3695, 3703, 3710, 3718, 3726, 3734, 3741, 3749,
		3756, 3763, 3771, 3778, 3785, 3792, 3799, 3806, 3813, 3819, 3826, 3833,
		3839, 3845, 3852, 3858, 3864, 3870, 3876, 3882, 3888, 3894, 3900, 3906,
		3911, 3917, 3922, 3927, 3933, 3938, 3943, 3948, 3953, 3958, 3962, 3967,
		3972, 3976, 3981, 3985, 3989, 3993, 3997, 4001, 4005, 4009, 4013, 4017,
		4020, 4024, 4027, 4031, 4034, 4037, 4040, 4043, 4046, 4049, 4052, 4054,
		4057, 4059, 4062, 4064, 4066, 4068, 4070, 4072, 4074, 4076, 4078, 4079,
		4081, 4082, 4084, 4085, 4086, 4087, 4088, 4089, 4090, 4091, 4091, 4092,
		4092, 4093, 4093, 4093, 4093 };



int theta = 0;

void updateDMABuffersIdle(int halve)
{
	const int samples = sizeof(dmaLeftBuffer) / sizeof(uint16_t) / 2;
	uint16_t *dstl = (uint16_t*) dmaLeftBuffer;
	dstl += halve ? samples : 0;
	uint16_t *dstr = (uint16_t*) dmaRightBuffer;
	dstr += halve ? samples : 0;

	for (int i = 0; i < samples; i++) {
		dstl[i] = dmaIdleSinWaveBuffer[theta];
		dstr[i] = dmaIdleCosWaveBuffer[theta];
		theta++;
		theta %= sizeof(dmaIdleSinWaveBuffer) / 2;
	}
}

#else
int thetaSin = 0;
int thetaCos = (USBD_AUDIO_FREQ / 50) / 4; // 90 degrees phase delay. cos(t) = sin(t + pi/2)

// fills buffer with sine-wave when there is nothing coming from USB
void updateDMABuffersIdle(int halve)
{
	const int samples = ARRAYSIZE(dmaLeftBuffer) / 2;
	uint16_t *dstl = (uint16_t*) dmaLeftBuffer;
	dstl += halve ? samples : 0;
	uint16_t *dstr = (uint16_t*) dmaRightBuffer;
	dstr += halve ? samples : 0;

	for (int i = 0; i < samples; i += 2)
	{
		// Interpolate samples to twice the frequency
		dstl[i] = dmaIdleSinWaveBuffer[thetaSin];
		dstr[i] = dmaIdleSinWaveBuffer[thetaCos];
		int oldThetaSin = thetaSin;
		int oldThetaCos = thetaCos;
		thetaSin++;
		thetaCos++;
		thetaSin %= ARRAYSIZE(dmaIdleSinWaveBuffer);
		thetaCos %= ARRAYSIZE(dmaIdleSinWaveBuffer);
		dstl[i+1] = (dmaIdleSinWaveBuffer[thetaSin] - dmaIdleSinWaveBuffer[oldThetaSin])/2 + dmaIdleSinWaveBuffer[oldThetaSin];
		dstr[i+1] = (dmaIdleSinWaveBuffer[thetaCos] - dmaIdleSinWaveBuffer[oldThetaCos])/2 + dmaIdleSinWaveBuffer[oldThetaCos];
	}
}
#endif

int updateDMABuffers(uint8_t* packets[], uint32_t count, int halve)
{
#define _16BTO12B(s) ((((int32_t)s + 32767) >> 4) & 0x0FFF)

	const int samplesPerPacket = AUDIO_OUT_PACKET / 2 / 2;
	uint16_t *dstl = (uint16_t*) dmaLeftBuffer;
	dstl += halve ? ARRAYSIZE(dmaLeftBuffer) / 2 : 0;
	uint16_t *dstr = (uint16_t*) dmaRightBuffer;
	dstr += halve ? ARRAYSIZE(dmaLeftBuffer) / 2 : 0;

	for (int i = 0; i < count; i++)
	{
		const uint16_t *packet = (uint16_t*) (packets[i]);
		for (int i = 0; i < samplesPerPacket * 2; i += 4)
		{
#if 0
			dstl[i] = dstl[i+1] = _16BTO12B(packet[i]);
			dstr[i] = dstr[i+1] = _16BTO12B(packet[i + 1]);
			dstl[i+3] = dstl[i+2] = _16BTO12B(packet[i + 2]);
			dstr[i+3] = dstr[i+2] = _16BTO12B(packet[i + 3]);
#else

			// Interpolate, this is lazy and assumes that we're rendering more or less vectors, nothing smooth
			dstl[i] = _16BTO12B(packet[i]);
			dstr[i] = _16BTO12B(packet[i + 1]);
			dstl[i+3] = _16BTO12B(packet[i + 2]);
			dstr[i+3] = _16BTO12B(packet[i + 3]);

			const int16_t deltal = (dstl[i+3] - dstl[i])/3;
			const int16_t deltar = (dstr[i+3] - dstr[i])/3;

			dstl[i+1] = deltal + dstl[i];
			dstr[i+1] = deltar + dstr[i];

			dstl[i+2] = dstl[i+3] - deltal;
			dstr[i+2] = dstr[i+3] - deltar;
#endif
		}
		dstl += samplesPerPacket * 2;
		dstr += samplesPerPacket * 2;
	}
	// zero out rest of the samples if there is not enough packets in the batch
	for (int i = count; i < AUDIO_PACKET_BATCH; i++)
	{
		for (int j = 0; j < samplesPerPacket * 2; j++)
		{
			dstl[j] = dstr[j] = ZERO_LEVEL;
		}
		dstl += samplesPerPacket * 2;
		dstr += samplesPerPacket * 2;
	}
	if (count <= AUDIO_PACKET_BATCH)
		return AUDIO_PACKET_BATCH * samplesPerPacket * 2;
	else
		return AUDIO_PACKET_BATCH * 2 * samplesPerPacket * 2;

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

void initDMA(int idleDisabled)
{
	fIdleDisabled = idleDisabled;
	const uint32_t n = sizeof(dmaLeftBuffer) / sizeof(uint16_t);

	for (uint32_t i = 0; i < n; i++)
	{
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

struct AudioCommand
{
	uint8_t *packets[AUDIO_PACKET_BATCH*2];
	uint32_t count;
	uint8_t sync;
	uint8_t cmd;
} audioCommands[2];
uint8_t cmdReadIdx = 0;
uint8_t cmdWriteIdx = 0;

// Called from main loop, interrupt may happen at any time.
void Process_Audio_Command()
{
	if(cmdWriteIdx - cmdReadIdx == 0) return;

	struct AudioCommand *cmd = &audioCommands[cmdReadIdx % 2];
	cmdReadIdx++;

	switch (cmd->cmd)
	{
	case AUDIO_CMD_START: // start from scratch
		//NOTE we assume that start provides 2 * AUDIO_PACKET_BATCH of packets, otherwise
		// the dma half/full interrupt update logic won't work.
		stopDMA();
		int samples = updateDMABuffers(cmd->packets, cmd->count,
				0);
		submitDMABuffers(samples);
		idleTimer = 0;
		break;

	case AUDIO_CMD_IDLE:
	case AUDIO_CMD_PLAY: // update current buffer
		if (cmd->count == 0 && idleTimer >= IDLE_TIMEOUT_MS
				&& !fIdleDisabled)
		{
			idleTimer = IDLE_TIMEOUT_MS;
			updateDMABuffersIdle(
					cmd->sync == AUDIO_SYNC_COMPLETE ? 1 : 0);
		}
		else
		{
			updateDMABuffers(cmd->packets, cmd->count,
					cmd->sync == AUDIO_SYNC_COMPLETE ? 1 : 0);
			if (cmd->count)
				idleTimer = 0;
		}

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

	struct AudioCommand *acmd = &audioCommands[cmdWriteIdx % 2];
	cmdWriteIdx++;

	acmd->count = count;
	acmd->sync = sync;
	acmd->cmd = cmd;
	if(packets) memcpy(acmd->packets, packets, sizeof(uint8_t*)*count);

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
// speed and buffer size.
// The interrupt for ch2 may come little bit before or after but it does not matter.

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);

  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // Debug
  idleTimer += MS_PER_BATCH;

  HalfTransfer_CallBack_FS();
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hdac);

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // Debug
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
