/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration of the RTC instance.
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* Global RTC handle ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* Private prototypes --------------------------------------------------------*/
/* USER CODE BEGIN PFP */
static void RTC_SetTimeDateOnce(void);
/* USER CODE END PFP */

/* RTC init function ---------------------------------------------------------*/
void MX_RTC_Init(void)
{
  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat      = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv    = 127;   /* 128 * 256 = 32768 nominal with LSI/LSE */
  hrtc.Init.SynchPrediv     = 255;
  hrtc.Init.OutPut          = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap     = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity  = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType      = RTC_OUTPUT_TYPE_OPENDRAIN;
#if defined(RTC_OUTPUT_PULLUP_NONE)
  hrtc.Init.OutPutPullUp    = RTC_OUTPUT_PULLUP_NONE;
#endif
#if defined(RTC_BINARY_NONE)
  hrtc.Init.BinMode         = RTC_BINARY_NONE;
#endif

  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN RTC_Init 2 */
  /* Only set a default time/date on the very first boot (backup magic not set). */
  RTC_SetTimeDateOnce();
  /* USER CODE END RTC_Init 2 */
}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{
  if (rtcHandle->Instance == RTC)
  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;  // was LSI
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_RTC_ENABLE();
#if defined(__HAL_RCC_RTCAPB_CLK_ENABLE)
    __HAL_RCC_RTCAPB_CLK_ENABLE();
#endif
  }
}


void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{
  if(rtcHandle->Instance == RTC)
  {
    /* USER CODE BEGIN RTC_MspDeInit 0 */
    /* USER CODE END RTC_MspDeInit 0 */

    /* Disable RTC peripheral clocks */
    __HAL_RCC_RTC_DISABLE();
#if defined(__HAL_RCC_RTCAPB_CLK_DISABLE)
    __HAL_RCC_RTCAPB_CLK_DISABLE();
#endif

    /* USER CODE BEGIN RTC_MspDeInit 1 */
    /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Set default time/date once on first boot.
  *         Uses RTC_BKP_DR0 == 0x32F2 as a “magic” flag.
  *         Safe to call on every boot.
  */
static void RTC_SetTimeDateOnce(void)
{
  /* Allow access to backup domain */
  HAL_PWR_EnableBkUpAccess();

  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
  {
    /* ---- Set your desired initial time/date here (24h, binary format) ---- */
    RTC_TimeTypeDef sTime = {
      .Hours          = 12,
      .Minutes        = 0,
      .Seconds        = 0,
      .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
      .StoreOperation = RTC_STOREOPERATION_RESET
    };

    RTC_DateTypeDef sDate = {
      /* DD/MM/YY -> 15/05/25 for example */
      .Year  = 25,   /* 2000 + 25 = 2025 */
      .Month = 5,    /* May */
      .Date  = 15
      /* .WeekDay is optional; HAL will compute if needed */
    };

    (void)HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    (void)HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    /* Mark as initialized so we don’t overwrite user-set time later */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
  }
}
/* USER CODE END 1 */
