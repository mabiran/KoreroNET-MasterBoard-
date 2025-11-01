/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* CubeMX peripheral headers (provide extern handles + MX_*_Init decls) */
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#if defined(HAL_RTC_MODULE_ENABLED)
#include "rtc.h"
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>   // strtof, strtod
#include <strings.h>  // strcasestr, strncasecmp
#include <ctype.h>
#include <stddef.h>   // size_t
#include <stdint.h>   // uint*_t
#include <stdio.h>    // snprintf, sscanf
#include "pins_config.h"
#include "battery_adc.h"
#include "battery_flow.h"
#include <stdbool.h>   // for bool, true, false
#include <math.h>      // if you use fabsf()

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOC_WAKE_THRESHOLD_PCT        20.0f

/* Wake pulse length (keep your existing define if present) */
#ifndef RPI_WAKE_PULSE_MS
#define RPI_WAKE_PULSE_MS             200u
#endif
#ifndef RPI_WAKE_DELAY_MS
#define RPI_WAKE_DELAY_MS             60000u     // 60 seconds
#endif

/* Restart notifier to help Pi re-init after MCU resets */
#define RESTART_BROADCAST_COUNT       20
#define RESTART_BROADCAST_PERIOD_MS   2000

#if defined(HAL_RTC_MODULE_ENABLED)
/* -------- Adaptive RTC BKP layout (STM32U083 has 9 regs: DR0..DR8) ---------
   Important: DR0 is RESERVED for CubeMX RTC "first-boot" magic (0x32F2).
   We never touch DR0 to avoid unintended RTC resets.
*/

/* Detect availability */
#if defined(RTC_BKP_NUMBER)
  #define BKP_COUNT RTC_BKP_NUMBER
#else
  #define BKP_COUNT 0
#endif

/* ---------------- Timetable persistence ------------------------------------
   We support two layouts:

   A) Rich layout (BKP_COUNT >= 9): uses DR1..DR7 (DR0 kept for RTC, DR8 spare)
      DR1: TT_MAGIC (0xAC1DDA7A)
      DR2: TT_FLAGS (bit0: valid)
      DR3: PACK0  (modes bits [31:0], 2 bits/hour)
      DR4: PACK1  (modes bits [47:32] in low 16b)
      DR5: REC_MS
      DR6: SLEEP_MS
      DR7: CRC32  over [PACK0|PACK1|REC_MS|SLEEP_MS]

   B) Compact layout (5 <= BKP_COUNT < 9): uses DR1..DR4 (no DR0 usage)
      DR1: TT_MAGIC (0xAC1DDA7A)
      DR2: PACK0
      DR3: PACK1 with CRC16 in high 16b
      DR4: DUR     (uint16_t REC_S | uint16_t SLEEP_S)  <-- seconds granularity

   If BKP_COUNT < 5, persistence is disabled.
---------------------------------------------------------------------------- */
#define TT_MAGIC                       0xAC1DDA7Au

#if BKP_COUNT >= 9
  #define TT_HAVE_RICH                 1
  #define TT_REG_MAGIC                 RTC_BKP_DR1
  #define TT_REG_FLAGS                 RTC_BKP_DR2
  #define TT_FLAG_VALID                (1u << 0)
  #define TT_REG_PACK0                 RTC_BKP_DR3
  #define TT_REG_PACK1                 RTC_BKP_DR4
  #define TT_REG_REC_MS                RTC_BKP_DR5
  #define TT_REG_SLEEP_MS              RTC_BKP_DR6
  #define TT_REG_CRC32                 RTC_BKP_DR7
#elif BKP_COUNT >= 5
  #define TT_HAVE_RICH                 0
  #define TT_REG_MAGIC                 RTC_BKP_DR1
  #define TT_REG_PACK0                 RTC_BKP_DR2
  #define TT_REG_PACK1                 RTC_BKP_DR3   /* high16 = CRC16, low16 = modes[47:32] */
  #define TT_REG_DUR_COMBO             RTC_BKP_DR4   /* low16 = rec_s, high16 = sleep_s */
#else
  #define TT_HAVE_RICH                 0
  /* No BKP persistence available */
#endif

#endif /* HAL_RTC_MODULE_ENABLED */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static float soc_from_voltage(float vbat) {
  /*
   * Revised LiFePO₄ 4S voltage map.
   */
  typedef struct { float v, p; } vp_t;
  static const vp_t map[] = {
    {14.27f, 100.0f}, {14.00f, 95.0f}, {13.80f, 90.0f}, {13.60f, 75.0f},
    {13.20f, 55.0f},  {12.80f, 30.0f}, {12.40f, 10.0f}, {12.00f,  0.0f}
  };
  if (vbat >= map[0].v) return 100.0f;
  for (size_t i = 1; i < sizeof(map)/sizeof(map[0]); ++i) {
    if (vbat >= map[i].v) {
      float dv = (map[i-1].v - map[i].v);
      float t  = (dv > 0.0f) ? (vbat - map[i].v) / dv : 0.0f;
      float p  = map[i].p + t * (map[i-1].p - map[i].p);
      if (p < 0.0f) p = 0.0f; if (p > 100.0f) p = 100.0f;
      return p;
    }
  }
  return 0.0f;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* NOTE: Peripheral handles are defined in adc.c/i2c.c/usart.c/rtc.c by CubeMX.
   Do NOT define them here to avoid multiple-definition link errors. */

/* USER CODE BEGIN PV */
/* UART1 line buffer (single definition) */
static uint8_t  uart1_rx_byte;
static char     uart1_line[128];
static uint32_t uart1_len = 0;

/* Defer command handling to main loop (ISR just sets this) */
static volatile uint8_t cmd_ready = 0;
static char             cmd_buf[128];

/* Battery “full” tracking state */
static uint8_t  full_marked   = 0;
static uint8_t  cond_active   = 0;
static uint32_t cond_start_ms = 0;

/* RPi wake scheduler (runtime, ms-based) */
typedef enum { RPI_WAKE_IDLE=0, RPI_WAKE_SCHEDULED, RPI_WAKE_PULSING } rpi_wake_state_t;
static rpi_wake_state_t rpi_wake_state = RPI_WAKE_IDLE;
static uint32_t rpi_wake_due_ms = 0;
static uint32_t rpi_wake_pulse_end_ms = 0;

/* Persistent wake (absolute, seconds since 2000-01-01) */
#if defined(HAL_RTC_MODULE_ENABLED)
static uint32_t wake_epoch = 0;
static uint8_t  wake_active = 0;
#endif

/* Restart notifier */
static uint8_t  restart_left = 0;
static uint32_t restart_next_ms = 0;

/* Power stats helper */
typedef struct {
  int    bus_ok;
  int    shunt_ok;
  int    use_ina;          /* 1=INA bus voltage chosen, 0=ADC fallback */
  float  v_bus;            /* INA bus voltage (V) */
  float  v_shunt;          /* INA shunt voltage (V) */
  float  current;          /* A (sign depends on shunt orientation) */
  float  v_adc;            /* ADC backup voltage (V) */
  float  v_src;            /* chosen source voltage (V) */
  float  power_W;          /* v_src * current (W) valid only if shunt_ok */
  float  soc_v;            /* SoC estimate from voltage (%) */
} PowerStats_t;

/* Latest power snapshot (silently maintained) */
static volatile PowerStats_t g_ps;
static volatile float        g_used_mAh = 0.0f;
static volatile float        g_soc_i    = 0.0f;

/* -----------------------------------------------------------------------
 * Daily recording scheduler
 * --------------------------------------------------------------------- */
static char     g_day_schedule[24] = {0};
static uint32_t g_rec_ms = 0;             /* record duration in ms */
static uint32_t g_sleep_ms = 0;           /* sleep duration in ms */
static uint8_t  g_sched_enabled = 0;      /* 1 when a timetable has been set */
static uint8_t  g_sched_paused  = 0;      /* 1 if waiting on Pi processing */
static uint32_t g_sched_pause_deadline_ms = 0; /* resume by this time */
static uint8_t  g_sched_hour = 0;         /* current hour we are executing */
static uint8_t  g_sched_recording = 0;    /* 1 if currently recording */
static uint32_t g_sched_next_toggle_ms = 0; /* when to toggle record state */

/* ---------------- Power history (rolling, hourly, last 25 hours) --------- */
#define PH_BINS            25u          /* 25 hours: oldest..current */
static float   g_ph_wh[PH_BINS]  = {0}; /* energy per hour (Wh) */
static float   g_ph_mAh[PH_BINS] = {0}; /* charge per hour (mAh) */

static float   g_ph_soc_i[PH_BINS] = {0}; /* time-weighted avg SoC (coulomb counter, %) */
static float   g_ph_soc_v[PH_BINS] = {0}; /* time-weighted avg SoC (voltage map, %) */
static float   g_ph_dt_h [PH_BINS] = {0}; /* accumulated hours in current bin (for averaging) */
static uint8_t g_ph_head = 0;           /* index of CURRENT hour bin [0..24] */
static uint8_t g_ph_inited = 0;         /* lazy init on first tick */
#if defined(HAL_RTC_MODULE_ENABLED)
static uint8_t g_ph_hourRTC = 0xFF;     /* last RTC hour used */
#else
static uint32_t g_ph_uptimeHour = 0;    /* last uptime-derived hour */
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void UART1_Send(const char *s);
static void RPi_HandleLine(const char *line);
static void RPi_ScheduleWake(uint32_t delay_ms);
static void RPi_WakeTick(void);
static void normalize_cmd(const char *in, char *out, size_t out_sz);

/* Parse a daily timetable from a command string.  Returns 1 on success. */
static int  parse_timetable(const char *line_in);

/* Execute the recording schedule.  Called periodically from the main loop. */
static void scheduler_tick(void);

static int  ReadPowerStats(PowerStats_t *ps);
static void PrintPowerSnapshot(void);
static void PowerStats_Tick(void);
static int  parse_wake_hours(const char *line_in, float *out_hours, uint32_t *out_ms);

/* Power history helpers */
static void PH_AdvanceHour(void);
static void PH_TickIntegrate(float power_W, float current_A, float dt_s, float soc_i_pct, float soc_v_pct);
static void PH_MaybeRollHour(void);
static void PH_PrintHistory(void);

/* RTC helpers (compiled only if RTC enabled) */
#if defined(HAL_RTC_MODULE_ENABLED)
/* BCD helpers */
static inline uint8_t bcd2bin(uint8_t v){ return (uint8_t)((v>>4)*10 + (v&0x0F)); }
static inline uint8_t bin2bcd(uint8_t v){ return (uint8_t)(((v/10)<<4) | (v%10)); }

/* *** CHANGED: prototype now supports DD/MM/YYYY HH:MM:SS (and still DD/MM/YY) *** */
static uint8_t  parse_datetime_ddmmyyyy_hhmmss(const char *p, RTC_DateTypeDef *d, RTC_TimeTypeDef *t);
static int      is_leap(int y);                  /* y in full years, e.g., 2000 */
static uint32_t rtc_now_epoch2000(void);
static uint32_t make_epoch2000(const RTC_DateTypeDef *d, const RTC_TimeTypeDef *t);

/* Persistence via RTC backup: wake + timetable */
#if HAVE_PERSIST_WAKE
static void     Persist_SaveWakeEpoch(uint32_t epoch);
static int      Persist_LoadWakeEpoch(uint32_t *epoch);
static void     Persist_ClearWake(void);
#endif

/* Timetable persistence */
static uint32_t TT_CalcCRC32(const uint8_t *data, size_t len);
static uint16_t TT_CalcCRC16(const uint8_t *data, size_t len);
static uint8_t  TT_ModeToBits(char c);
static char     TT_BitsToMode(uint8_t b);
static void     TT_SaveToBKP(const char modes[24], uint32_t rec_ms, uint32_t sleep_ms);
static int      TT_LoadFromBKP(char modes_out[24], uint32_t *rec_ms, uint32_t *sleep_ms);
static void     TT_PrintCurrent(void);
#endif

/* Restart notifier */
static void RestartNotifier_Tick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock (LSI/LSE per config) */
  SystemClock_Config();

  /* Initialize all configured peripherals (implementations are in CubeMX .c files) */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
#if defined(HAL_RTC_MODULE_ENABLED)
  MX_RTC_Init();
#endif

  /* Start UART1 RX interrupt (1 byte at a time) */
  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);

  /* Initialize leds and button via BSP */
  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Optional COM settings (not used directly below) */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;

  /* USER CODE BEGIN BSP */
  /* Configure INA219 (short timeout) */
  uint8_t ina_cfg[3] = { INA_REG_CONFIG, 0x39, 0x9F }; /* 32V, ±320mV, 12b, cont */
  (void)HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, ina_cfg, 3, 10);

  /* Seed coulomb counter from ADC voltage on cold boot */
  float v_bat_boot = Battery_ReadVoltage(32);
  float soc_v_boot = soc_from_voltage(v_bat_boot);
  float used_mAh_guess = (1.0f - (soc_v_boot / 100.0f)) * BATTERY_NOMINAL_mAh;
  BatteryFlow_Reset(used_mAh_guess);

  /* Set default levels for control lines after MX_GPIO_Init() */
#ifdef RPI_WAKE_GPIO_Port
  HAL_GPIO_WritePin(RPI_WAKE_GPIO_Port, RPI_WAKE_Pin, GPIO_PIN_SET);
#endif
#ifdef AM_REC_Port
  HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_SET);
#endif
#ifdef AM_CONFIG_Port
  HAL_GPIO_WritePin(AM_CONFIG_Port, AM_CONFIG_Pin, GPIO_PIN_SET);
#endif

  /* Load persisted wake schedule (RTC only, if supported) */
#if defined(HAL_RTC_MODULE_ENABLED) && HAVE_PERSIST_WAKE
  if (Persist_LoadWakeEpoch(&wake_epoch)) {
    wake_active = 1;
  } else {
    wake_active = 0;
  }

  /* Load persisted timetable if present */
  {
    char modes[24];
    uint32_t r_ms = 0, s_ms = 0;
    if (TT_LoadFromBKP(modes, &r_ms, &s_ms)) {
      for (int i = 0; i < 24; ++i) g_day_schedule[i] = modes[i];
      g_rec_ms = r_ms;
      g_sleep_ms = s_ms;
      g_sched_enabled = 1;
      g_sched_paused  = 0;
      g_sched_recording = 0;
      g_sched_next_toggle_ms = 0;
      UART1_Send("INFO: timetable restored from backup\r\n");
    }
  }
#elif defined(HAL_RTC_MODULE_ENABLED)
  /* Even if persisted wake isn't available, we can still restore timetable if regs exist */
  {
    char modes[24];
    uint32_t r_ms = 0, s_ms = 0;
    if (TT_LoadFromBKP(modes, &r_ms, &s_ms)) {
      for (int i = 0; i < 24; ++i) g_day_schedule[i] = modes[i];
      g_rec_ms = r_ms;
      g_sleep_ms = s_ms;
      g_sched_enabled = 1;
      g_sched_paused  = 0;
      g_sched_recording = 0;
      g_sched_next_toggle_ms = 0;
      UART1_Send("INFO: timetable restored from backup\r\n");
    }
  }
#endif

  /* Restart notifier */
  restart_left = RESTART_BROADCAST_COUNT;
  restart_next_ms = HAL_GetTick() + 500;  // first ping shortly after boot

  UART1_Send("BOOT: Nucleo ready\r\n");
  /* USER CODE END BSP */

  /* Infinite loop */
  while (1)
  {
    /* Keep RX armed (in case of error) */
    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX) {
      HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }

    /* RPi wake scheduler tick (handles both runtime and persisted wake) */
    RPi_WakeTick();

    /* Single silent call that samples INA/ADC, handles FULL detection, and updates coulomb counter */
    PowerStats_Tick();

    /* Drive the hourly recording scheduler */
    scheduler_tick();

    /* Process any received command (handled outside ISR) */
    if (cmd_ready) {
      cmd_ready = 0;
      RPi_HandleLine(cmd_buf);
    }

    /* Restart notifier (periodic) */
    RestartNotifier_Tick();

    /* Small pacing slice to keep ISR latency low */
    uint32_t until = HAL_GetTick() + 50;
    while ((int32_t)(HAL_GetTick() - until) < 0) {
      RPi_WakeTick();
      if (cmd_ready) {
        cmd_ready = 0;
        RPi_HandleLine(cmd_buf);
      }
      /* Invoke scheduler during pacing loop */
      scheduler_tick();
      HAL_Delay(5);  /* Does NOT disable UART interrupts — only adds latency */
    }

    /* Heartbeat blink (BSP LED) */
   // BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(250);
  }
}

/* -------------------- Clocks (prefer LSE for RTC; fallback to LSI) -------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Try LSE first for RTC accuracy */
  HAL_PWR_EnableBkUpAccess();                               // allow backup-domain writes
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    /* Fallback: disable LSE, enable LSI for RTC */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
    }
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  /* Route RTC clock */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
#if defined(RCC_OSCILLATORTYPE_LSE)
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) {
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  } else
#endif
  {
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
  }
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */

/* --- I2C utility with short timeouts --- */
static int i2c_read_reg(uint8_t reg, uint8_t *buf, uint32_t len) {
  const uint32_t TO = 10; // ms
  if (HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, &reg, 1, TO) != HAL_OK) return 0;
  if (HAL_I2C_Master_Receive (&hi2c1, INA219_ADDR, buf, len, TO)   != HAL_OK) return 0;
  return 1;
}

/* --- Power stats --- */
static int ReadPowerStats(PowerStats_t *ps) {
  if (!ps) return 0;
  memset(ps, 0, sizeof(*ps));

  uint8_t data[2];
  uint8_t reg = INA_REG_BUS_V;
  if (i2c_read_reg(reg, data, 2)) {
    uint16_t bus_raw = ((uint16_t)data[0] << 8) | data[1];
    bus_raw >>= 3;
    ps->v_bus = bus_raw * 0.004f;
    ps->bus_ok = 1;

    reg = INA_REG_SHUNT_V;
    if (i2c_read_reg(reg, data, 2)) {
      int16_t shunt_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
      ps->v_shunt = shunt_raw * 0.00001f;
      ps->current = ps->v_shunt / SHUNT_OHMS;
      ps->shunt_ok = 1;
    }
  }

  /* backup ADC path always available */
  ps->v_adc   = Battery_ReadVoltage(8);
  ps->v_src   = ps->bus_ok ? ps->v_bus : ps->v_adc;
  ps->use_ina = ps->bus_ok;

  if (ps->shunt_ok) ps->power_W = ps->v_src * ps->current;
  ps->soc_v = soc_from_voltage(ps->v_src);

  return (ps->bus_ok || ps->shunt_ok);
}

static void PH_MaybeRollHour(void){
#if defined(HAL_RTC_MODULE_ENABLED)
  RTC_TimeTypeDef t_bcd; RTC_DateTypeDef d_bcd;
  HAL_RTC_GetTime(&hrtc, &t_bcd, RTC_FORMAT_BCD);
  HAL_RTC_GetDate(&hrtc, &d_bcd, RTC_FORMAT_BCD);
  uint8_t h = bcd2bin(t_bcd.Hours);
  if (g_ph_hourRTC == 0xFF) g_ph_hourRTC = h;
  if (h != g_ph_hourRTC) {
    g_ph_hourRTC = h;
    g_ph_head = (uint8_t)((g_ph_head + 1u) % PH_BINS);
    g_ph_wh[g_ph_head]   = 0.0f;
    g_ph_mAh[g_ph_head]  = 0.0f;
    g_ph_soc_i[g_ph_head]= 0.0f;
    g_ph_soc_v[g_ph_head]= 0.0f;
    g_ph_dt_h [g_ph_head]= 0.0f;
  }
#else
  uint32_t h = (HAL_GetTick() / 3600000u) % 24u;
  if (h != g_ph_uptimeHour) {
    g_ph_uptimeHour = h;
    g_ph_head = (uint8_t)((g_ph_head + 1u) % PH_BINS);
    g_ph_wh[g_ph_head]   = 0.0f;
    g_ph_mAh[g_ph_head]  = 0.0f;
    g_ph_soc_i[g_ph_head]= 0.0f;
    g_ph_soc_v[g_ph_head]= 0.0f;
    g_ph_dt_h [g_ph_head]= 0.0f;
  }
#endif
}


static void PH_TickIntegrate(float power_W, float current_A, float dt_s,
                             float soc_i_pct, float soc_v_pct)
{
  if (!g_ph_inited) {
#if defined(HAL_RTC_MODULE_ENABLED)
    RTC_TimeTypeDef t_bcd; RTC_DateTypeDef d_bcd;
    HAL_RTC_GetTime(&hrtc, &t_bcd, RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc, &d_bcd, RTC_FORMAT_BCD);
    g_ph_hourRTC = bcd2bin(t_bcd.Hours);
#else
    g_ph_uptimeHour = (HAL_GetTick() / 3600000u) % 24u;
#endif
    g_ph_inited = 1;
  }

  if (dt_s <= 0.0f) return;

  const float dt_h = dt_s / 3600.0f;

  /* Energy & charge */
  g_ph_wh[g_ph_head]  += (power_W   * dt_h);            /* Wh (signed) */
  g_ph_mAh[g_ph_head] += (current_A * dt_h * 1000.0f);  /* mAh (signed) */

  /* Time-weighted SoC averages */
  float prev_dt = g_ph_dt_h[g_ph_head];
  float new_dt  = prev_dt + dt_h;
  g_ph_dt_h[g_ph_head] = new_dt;
  if (new_dt > 0.0f) {
    g_ph_soc_i[g_ph_head] = (g_ph_soc_i[g_ph_head] * prev_dt + soc_i_pct * dt_h) / new_dt;
    g_ph_soc_v[g_ph_head] = (g_ph_soc_v[g_ph_head] * prev_dt + soc_v_pct * dt_h) / new_dt;
  }
}


static void PH_PrintHistory(void){
  char line[128];

  /* Wh row */
  UART1_Send("PH_WH,");
  for (uint32_t i = 0; i < PH_BINS; ++i) {
    uint32_t idx = (g_ph_head + 1u + i) % PH_BINS;
    int n = snprintf(line, sizeof(line), (i + 1 < PH_BINS) ? "%.6f," : "%.6f\r\n", (double)g_ph_wh[idx]);
    HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)n, 50);
  }

  /* mAh row */
  UART1_Send("PH_mAh,");
  for (uint32_t i = 0; i < PH_BINS; ++i) {
    uint32_t idx = (g_ph_head + 1u + i) % PH_BINS;
    int n = snprintf(line, sizeof(line), (i + 1 < PH_BINS) ? "%.6f," : "%.6f\r\n", (double)g_ph_mAh[idx]);
    HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)n, 50);
  }

  /* SoC_i (%) row */
  UART1_Send("PH_SoCi,");
  for (uint32_t i = 0; i < PH_BINS; ++i) {
    uint32_t idx = (g_ph_head + 1u + i) % PH_BINS;
    int n = snprintf(line, sizeof(line), (i + 1 < PH_BINS) ? "%.3f," : "%.3f\r\n", (double)g_ph_soc_i[idx]);
    HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)n, 50);
  }

  /* SoC_v (%) row */
  UART1_Send("PH_SoCv,");
  for (uint32_t i = 0; i < PH_BINS; ++i) {
    uint32_t idx = (g_ph_head + 1u + i) % PH_BINS;
    int n = snprintf(line, sizeof(line), (i + 1 < PH_BINS) ? "%.3f," : "%.3f\r\n", (double)g_ph_soc_v[idx]);
    HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)n, 50);
  }
}


static void PowerStats_Tick(void)
{
  static uint32_t last_ms = 0;
  if (last_ms == 0) last_ms = HAL_GetTick();

  PowerStats_t tmp;
  ReadPowerStats(&tmp);
  g_ps = tmp;

  /* FULL detection */
  /* --- Replace the FULL detection block --- */
  uint32_t tnow = HAL_GetTick();

  /* Stage 1: are we actually charging? */
  const float CHARGE_PRESENT_V = 13.5f;     // LiFePO4 4S under charge
  const float CHARGE_PRESENT_A = 0.20f;     // ~0.2 A charging present threshold

  bool charging_present =
      (tmp.shunt_ok) &&
      (tmp.v_src >= CHARGE_PRESENT_V) &&
      (tmp.current < -CHARGE_PRESENT_A);    // negative = charging

  /* Stage 2: end-of-charge (tail current + high voltage) */
  const float CHARGE_FULL_V  = 14.2f;       // set to your charger absorb/cv voltage
  const float EOC_TAIL_A     = 0.15f;       // ~C/80 for 12 Ah pack ≈ 0.15 A

  bool end_of_charge =
      (tmp.shunt_ok) &&
      (tmp.v_src >= CHARGE_FULL_V) &&
      (fabsf(tmp.current) <= EOC_TAIL_A);

  /* Only this condition can trip the dwell timer and mark FULL */
  bool meets = end_of_charge;
  if (meets) {
      if (!cond_active) { cond_active = 1; cond_start_ms = tnow; }
      if (!full_marked && (tnow - cond_start_ms) >= CHARGE_CONFIRM_MS) {
          BatteryFlow_Reset(0.0f);          // mark 100% right at the tail
          full_marked = 1;
          /* ... keep your EVENT: FULL_MARKED print ... */
      }
  } else {
      cond_active = 0;
  }


  /* Coulomb counting */
  uint32_t now = HAL_GetTick();
  float dt_s = (now - last_ms) / 1000.0f;
  if (dt_s <= 0.0f) dt_s = 0.001f;
  if (dt_s >  5.0f) dt_s = 5.0f;
  last_ms = now;

  if (tmp.shunt_ok) {
    BatteryFlow_Update(tmp.current, dt_s);
  }

  float used_mAh = BatteryFlow_Get_mAh();
  if (used_mAh < 0.0f) used_mAh = 0.0f;
  float soc_i = 100.0f * (1.0f - (used_mAh / BATTERY_NOMINAL_mAh));
  if (soc_i < 0.0f)   soc_i = 0.0f;
  if (soc_i > 100.0f) soc_i = 100.0f;

  g_used_mAh = used_mAh;
  g_soc_i    = soc_i;

  /* ---- Power history integration & hour roll ---- */
  PH_MaybeRollHour();
  if (tmp.shunt_ok) {
    PH_TickIntegrate(tmp.power_W, tmp.current, dt_s, soc_i, tmp.soc_v);
  }
}

/* One-line printer for on-demand snapshot */
static void PrintPowerSnapshot(void)
{
  PowerStats_t latest;
  if (ReadPowerStats(&latest)) {
    g_ps = latest;
  }

  PowerStats_t ps  = g_ps;
  float used_mAh   = g_used_mAh;
  float soc_i      = g_soc_i;

  const char *vsrc_tag = ps.use_ina ? "INA" : "ADC";
  char status_msg[256];

  if (ps.shunt_ok) {
    float pW = ps.v_src * ps.current;
    snprintf(status_msg, sizeof(status_msg),
      "Vsrc[%s]=%.3f V | I=%+.3f A | used=%5.0f/%5.0f mAh | "
      "SoC_i=%3.0f%% | SoC_v=%3.0f%% | %s\r\n",
      vsrc_tag, ps.v_src, ps.current, used_mAh,
      (float)BATTERY_NOMINAL_mAh, soc_i, ps.soc_v,
      full_marked ? "FULL_MARKED" : "—");
    UART1_Send(status_msg);

    char power_msg[96];
    snprintf(power_msg, sizeof(power_msg),
      "Power=%.3f W (%s)\r\n",
      pW, (pW >= 0.0f ? "discharge" : "charge"));
    UART1_Send(power_msg);
  } else {
    snprintf(status_msg, sizeof(status_msg),
      "Vsrc[%s]=%.3f V | I=NA (INA fail) | used=%5.0f/%5.0f mAh | "
      "SoC_i=%3.0f%% | SoC_v=%3.0f%% | %s\r\n",
      vsrc_tag, ps.v_src, used_mAh,
      (float)BATTERY_NOMINAL_mAh, soc_i, ps.soc_v,
      full_marked ? "FULL_MARKED" : "—");
    UART1_Send(status_msg);
  }
}

/* --- UART helpers --- */
static void UART1_Send(const char *s) {
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 50);
}

static void normalize_cmd(const char *in, char *out, size_t out_sz) {
  size_t n = 0;
  for (const char *p = in; *p && n + 1 < out_sz; ++p) {
    unsigned char c = (unsigned char)*p;
    if (c==' ' || c=='\t' || c==',' || c=='.' || c=='!' || c=='?' ||
        c==':' || c==';' || c=='\'' || c=='\"')
      continue;                 // skip whitespace & punctuation
    out[n++] = (char)tolower(c); // lowercase everything
  }
  out[n] = '\0';
}

/* --- Parse a daily timetable from an incoming command --- */
static int parse_timetable(const char *line_in)
{
  if (!line_in) return 0;
  /* Find first and second brace pairs */
  const char *p1 = strchr(line_in, '{');
  if (!p1) return 0;
  const char *p2 = strchr(p1 + 1, '}');
  if (!p2) return 0;
  /* Parse the first list: up to 24 mode characters */
  char modes[24];
  memset(modes, '0', sizeof(modes));
  unsigned idx = 0;
  const char *s = p1 + 1;
  while (s < p2 && idx < 24) {
    /* Skip whitespace */
    while (s < p2 && (*s == ' ' || *s == '\t')) s++;
    if (s >= p2) break;
    char c = *s;
    char upc;
    /* Accept valid characters */
    if (c == 'S' || c == 's') upc = 'S';
    else if (c == 'U' || c == 'u') upc = 'U';
    else if (c == 'P' || c == 'p') upc = 'P';
    else if (c == '0' || c == 'o' || c == 'O') upc = '0';
    else {
      /* Unrecognised token: treat as zero */
      upc = '0';
    }
    modes[idx++] = upc;
    /* Move to next comma or end */
    while (s < p2 && *s != ',') s++;
    if (s < p2 && *s == ',') s++;
  }
  /* Fill remaining hours with '0' */
  for (; idx < 24; ++idx) modes[idx] = '0';

  /* Parse the second list: record and sleep durations */
  const char *p3 = strchr(p2 + 1, '{');
  if (!p3) return 0;
  const char *p4 = strchr(p3 + 1, '}');
  if (!p4) return 0;
  char buf[64];
  int len = (int)(p4 - (p3 + 1));
  if (len <= 0 || len >= (int)sizeof(buf)) return 0;
  strncpy(buf, p3 + 1, (size_t)len);
  buf[len] = '\0';
  /* Extract two floating point numbers */
  char *endptr;
  double d1 = strtod(buf, &endptr);
  if (endptr == buf) return 0;
  while (*endptr == ' ' || *endptr == '\t' || *endptr == ',') endptr++;
  double d2 = strtod(endptr, NULL);
  if (d1 <= 0.0 || d2 <= 0.0) return 0;
  /* Convert seconds to milliseconds */
  g_rec_ms   = (uint32_t)(d1 * 1000.0 + 0.5);
  g_sleep_ms = (uint32_t)(d2 * 1000.0 + 0.5);
  if (g_rec_ms < 100) g_rec_ms = 100;
  if (g_sleep_ms < 100) g_sleep_ms = 100;

  /* Commit the new schedule */
  for (int i = 0; i < 24; ++i) g_day_schedule[i] = modes[i];
  g_sched_enabled       = 1;
  g_sched_paused        = 0;
  g_sched_recording     = 0;
  g_sched_next_toggle_ms = 0;

#if defined(HAL_RTC_MODULE_ENABLED)
  /* Persist it if backup regs exist */
  TT_SaveToBKP(g_day_schedule, g_rec_ms, g_sleep_ms);
#endif
  return 1;
}

/* --- Scheduler tick: drives hour-by-hour recording behaviour --- */
static void scheduler_tick(void)
{

  if (!g_sched_enabled) {
#if defined(HAL_RTC_MODULE_ENABLED)
    /* Plan B: wake Raspberry Pi at 11:00 local time once per day */
    static uint8_t _last_planb_day = 0xFF;
    RTC_TimeTypeDef t_bcd; RTC_DateTypeDef d_bcd;
    HAL_RTC_GetTime(&hrtc, &t_bcd, RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc, &d_bcd, RTC_FORMAT_BCD);
    uint8_t hour = bcd2bin(t_bcd.Hours);
    uint8_t day  = bcd2bin(d_bcd.Date);
    if (hour == 11 && _last_planb_day != day) {
#ifdef RPI_WAKE_GPIO_Port
      UART1_Send("PLANB: no timetable; waking Pi at 11:00\r\n");
      RPi_ScheduleWake(0);
#endif
      _last_planb_day = day;
    }
#endif
    return;
  }

  uint32_t now_ms = HAL_GetTick();
  /* If paused waiting for Pi */
  if (g_sched_paused) {
    /* Resume if timeout reached */
    if ((int32_t)(now_ms - g_sched_pause_deadline_ms) >= 0) {
      g_sched_paused = 0;
      UART1_Send("WARN: schedule timeout elapsed, resuming\r\n");
    } else {
      return;
    }
  }

  /* Determine the current hour.  Prefer RTC if available, otherwise use uptime. */
#if defined(HAL_RTC_MODULE_ENABLED)
  RTC_TimeTypeDef t_bcd;
  RTC_DateTypeDef d_bcd;
  HAL_RTC_GetTime(&hrtc, &t_bcd, RTC_FORMAT_BCD);
  HAL_RTC_GetDate(&hrtc, &d_bcd, RTC_FORMAT_BCD);
  uint8_t hour = bcd2bin(t_bcd.Hours);
#else
  uint8_t hour = (uint8_t)((HAL_GetTick() / 3600000u) % 24u);
#endif

  /* On hour change initialise recording state */
  if (g_sched_next_toggle_ms == 0 || hour != g_sched_hour) {
    g_sched_hour = hour;
    char mode = g_day_schedule[hour];
    /* Normalise to uppercase */
    if (mode >= 'a' && mode <= 'z') {
      mode = (char)(mode - ('a' - 'A'));
    }

    /* Configure ultrasound based on mode */
    if (mode == 'S') {
#ifdef AM_CONFIG_Port
      HAL_GPIO_WritePin(AM_CONFIG_Port, AM_CONFIG_Pin, GPIO_PIN_SET);
#endif
    } else if (mode == 'U') {
#ifdef AM_CONFIG_Port
      HAL_GPIO_WritePin(AM_CONFIG_Port, AM_CONFIG_Pin, GPIO_PIN_RESET);
#endif
    }

    /* Standby: ensure AM_REC is high and no toggling */
    if (mode == '0') {
#ifdef AM_REC_Port
      HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_SET);
#endif
      g_sched_recording = 0;
      g_sched_next_toggle_ms = 0;
      return;
    }
    /* Process: stop recording, wake Pi and pause */
    if (mode == 'P') {
#ifdef AM_REC_Port
      HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_SET);
#endif
#ifdef RPI_WAKE_GPIO_Port
      /* Schedule an immediate wake pulse */
      rpi_wake_state = RPI_WAKE_SCHEDULED;
      rpi_wake_due_ms = now_ms;
#endif
      /* Pause scheduler and set timeout for 5 hours */
      g_sched_paused = 1;
      g_sched_pause_deadline_ms = now_ms + 5U * 3600000U;
      UART1_Send("EVENT: schedule paused for processing\r\n");
      return;
    }
    /* For S or U: start with recording low */
    if (mode == 'S' || mode == 'U') {
      g_sched_recording = 1;
#ifdef AM_REC_Port
      HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_RESET);
#endif
      g_sched_next_toggle_ms = now_ms + g_rec_ms;
      return;
    }
    /* Unknown mode: treat as standby */
    g_sched_recording = 0;
    g_sched_next_toggle_ms = 0;
    return;
  }

  /* Within the hour: handle toggling for S/U modes */
  char mode = g_day_schedule[g_sched_hour];
  if (mode >= 'a' && mode <= 'z') {
    mode = (char)(mode - ('a' - 'A'));
  }
  if (mode == 'S' || mode == 'U') {
    if ((int32_t)(now_ms - g_sched_next_toggle_ms) >= 0) {
      g_sched_recording = (uint8_t)!g_sched_recording;
      if (g_sched_recording) {
        /* Begin recording: AM_REC low */
#ifdef AM_REC_Port
        HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_RESET);
#endif
        g_sched_next_toggle_ms = now_ms + g_rec_ms;
      } else {
        /* Sleep: AM_REC high */
#ifdef AM_REC_Port
        HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_SET);
#endif
        g_sched_next_toggle_ms = now_ms + g_sleep_ms;
      }
    }
  } else {
    /* For any other mode ensure AM_REC high */
#ifdef AM_REC_Port
    HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_SET);
#endif
    g_sched_recording = 0;
    g_sched_next_toggle_ms = 0;
  }
}


/* --- Parse "wake me up in <fp> h/min" --- */
static int parse_wake_hours(const char *line_in, float *out_hours, uint32_t *out_ms)
{
  if (!line_in || !out_hours || !out_ms) return 0;

  const char *key = "wake me up in";
  const char *q = strcasestr(line_in, key);
  if (!q) return 0;

  q += strlen(key);
  while (*q && !( (*q>='0'&&*q<='9') || *q=='-' || *q=='+' || *q=='.' )) q++;

  char *endp = NULL;
  double val = strtod(q, &endp);
  while (*endp == ' ') endp++;

  int unit_is_hours = 1;  /* default hours */
  if (*endp) {
    if      (!strncasecmp(endp, "h",     1)) unit_is_hours = 1;
    else if (!strncasecmp(endp, "hr",    2)) unit_is_hours = 1;
    else if (!strncasecmp(endp, "hrs",   3)) unit_is_hours = 1;
    else if (!strncasecmp(endp, "hour",  4)) unit_is_hours = 1;
    else if (!strncasecmp(endp, "hours", 5)) unit_is_hours = 1;
    else if (!strncasecmp(endp, "m",     1)  ||
             !strncasecmp(endp, "min",   3)  ||
             !strncasecmp(endp, "mins",  4)  ||
             !strncasecmp(endp, "minute",6)  ||
             !strncasecmp(endp, "minutes",7)) {
      unit_is_hours = 0;
    }
  }

  if (val <= 0.0) return 0;

  double msd = unit_is_hours ? (val * 3600000.0) : (val * 60000.0);
  if (msd > 4.29e9) msd = 4.29e9;           // clamp to ~49 days
  uint32_t delay_ms = (uint32_t)(msd + 0.5);

  *out_hours = unit_is_hours ? (float)val : (float)(val / 60.0);
  *out_ms = delay_ms;
  return 1;
}

/* --- UART RX ISR: line assembly --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    uint8_t c = uart1_rx_byte;
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);  /* re-arm */

    if (c == '\r' || c == '\n') {
      if (uart1_len && !cmd_ready) {
        uart1_line[uart1_len] = '\0';
        strncpy(cmd_buf, uart1_line, sizeof(cmd_buf));
        cmd_buf[sizeof(cmd_buf)-1] = '\0';
        cmd_ready = 1;
      }
      uart1_len = 0;
      return;
    }
    if (uart1_len < sizeof(uart1_line) - 1) {
      uart1_line[uart1_len++] = (char)c;
    } else {
      uart1_len = 0;  // reset on overflow
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  }
}

/* --- Command handler --- */
static void RPi_HandleLine(const char *line_in) {
  char cmd[128];
  normalize_cmd(line_in, cmd, sizeof(cmd));

  /* --- AudioMoth controls --- */
  if (strstr(cmd, "nucleostartrecording")) {
#ifdef AM_REC_Port
    HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_RESET);
#endif
    UART1_Send("ACK: AM_REC -> LOW (Start Recording)\r\n");
    return;
  }
  if (strstr(cmd, "nucleoineedfiles")) {
#ifdef AM_REC_Port
    HAL_GPIO_WritePin(AM_REC_Port, AM_REC_Pin, GPIO_PIN_SET);
#endif
    UART1_Send("ACK: AM_REC -> HIGH (Need files)\r\n");
    return;
  }
  if (strstr(cmd, "nucleoidonotneedultrasound")) {
#ifdef AM_CONFIG_Port
    HAL_GPIO_WritePin(AM_CONFIG_Port, AM_CONFIG_Pin, GPIO_PIN_SET);
#endif
    UART1_Send("ACK: AM_CONFIG -> HIGH (No ultrasound)\r\n");
    return;
  }
  if (strstr(cmd, "nucleoineedultrasound")) {
#ifdef AM_CONFIG_Port
    HAL_GPIO_WritePin(AM_CONFIG_Port, AM_CONFIG_Pin, GPIO_PIN_RESET);
#endif
    UART1_Send("ACK: AM_CONFIG -> LOW (Need ultrasound)\r\n");
    return;
  }

  /* Power snapshot */
  if (strstr(cmd, "nucleopowerstats")) {
    UART1_Send("ACK: power snapshot\r\n");
    PrintPowerSnapshot();
    return;
  }

  /* Power history dump */
  if (strstr(cmd, "nucleopowerhistory")) {
    UART1_Send("ACK: power history\r\n");
    PH_PrintHistory();
    return;
  }

  /* Print timetable saved in BKP or runtime */
  if (strstr(cmd, "nucleogivemetimetable") || strstr(cmd, "nucleoprinttimetable")) {
    UART1_Send("ACK: timetable dump\r\n");
    TT_PrintCurrent();
    return;
  }

  /* Time query: "nucleo tell me time" */
  if (strstr(cmd, "nucleotellmetime")) {
#if defined(HAL_RTC_MODULE_ENABLED)
    RTC_TimeTypeDef t_bcd;
    RTC_DateTypeDef d_bcd;
    HAL_RTC_GetTime(&hrtc, &t_bcd, RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc, &d_bcd, RTC_FORMAT_BCD); /* must read Date after Time */

    uint8_t dd = bcd2bin(d_bcd.Date);
    uint8_t mm = bcd2bin(d_bcd.Month);
    uint8_t yy = bcd2bin(d_bcd.Year);
    uint8_t hh = bcd2bin(t_bcd.Hours);
    uint8_t mi = bcd2bin(t_bcd.Minutes);
    uint8_t ss = bcd2bin(t_bcd.Seconds);

    char msg[64];
    snprintf(msg, sizeof(msg), "%02u/%02u/%04u %02u:%02u:%02u\r\n",
             (unsigned)dd, (unsigned)mm, (unsigned)(2000u + yy),
             (unsigned)hh, (unsigned)mi, (unsigned)ss);
    UART1_Send(msg);
#else
    UART1_Send("ERR: RTC not enabled\r\n");
#endif
    return;
  }

  /* Time set: "nucleo time is DD/MM/YYYY HH:MM:SS" (4-digit; 2-digit still accepted) */
  /* Time set: "nucleo time is DD/MM/YYYY HH:MM:SS" */
  {
    const char *k = "nucleo time is";
    const char *p = strcasestr(line_in, k);
    if (p) {
  #if defined(HAL_RTC_MODULE_ENABLED)
      p += strlen(k);
      while (*p==' ' || *p=='\t') p++;

      RTC_DateTypeDef d_bin; RTC_TimeTypeDef t_bin;
      if (parse_datetime_ddmmyyyy_hhmmss(p, &d_bin, &t_bin)) {
        /* Convert BIN -> BCD */
        RTC_DateTypeDef d_bcd = {0};
        RTC_TimeTypeDef t_bcd = {0};
        d_bcd.Date   = bin2bcd(d_bin.Date);
        d_bcd.Month  = bin2bcd(d_bin.Month);
        d_bcd.Year   = bin2bcd(d_bin.Year);
        /* >>> set a valid weekday (compute or set any 1..7) <<< */
        d_bcd.WeekDay = RTC_WEEKDAY_MONDAY;   // TODO: compute real weekday if you care

        t_bcd.Hours   = bin2bcd(t_bin.Hours);
        t_bcd.Minutes = bin2bcd(t_bin.Minutes);
        t_bcd.Seconds = bin2bcd(t_bin.Seconds);

        /* Recommended order: TIME then DATE */
        if (HAL_RTC_SetTime(&hrtc, &t_bcd, RTC_FORMAT_BCD) == HAL_OK &&
            HAL_RTC_SetDate(&hrtc, &d_bcd, RTC_FORMAT_BCD) == HAL_OK) {
          UART1_Send("ACK: time set\r\n");
        } else {
          UART1_Send("ERR: time set failed (weekday or format)\r\n");
        }
      } else {
        UART1_Send("ERR: bad time format, use DD/MM/YYYY HH:MM:SS\r\n");
      }
  #else
      UART1_Send("ERR: RTC not enabled\r\n");
  #endif
      return;
    }
  }


  /* Wake me up in ... */
  {
    float hours = 0.0f;
    uint32_t delay_ms = 0;
    if (parse_wake_hours(line_in, &hours, &delay_ms)) {
      /* runtime relative schedule */
      RPi_ScheduleWake(delay_ms);

#if defined(HAL_RTC_MODULE_ENABLED) && HAVE_PERSIST_WAKE
      /* persistent absolute time */
      uint32_t now_epoch = rtc_now_epoch2000();
      uint32_t due_epoch = now_epoch + (delay_ms / 1000u);
      Persist_SaveWakeEpoch(due_epoch);
      wake_epoch = due_epoch;
      wake_active = 1;
#else
      UART1_Send("WARN: persisted wake not available; keeping in RAM only\r\n");
#endif

      char resp[96];
      snprintf(resp, sizeof(resp),
               "OK, wake scheduled in %.3f h (~%lu ms).\r\n",
               (double)hours, (unsigned long)delay_ms);
      UART1_Send(resp);
      return;
    }
  }

  /* Goodnight shortcut */
  if (strcmp(cmd, "goodnight") == 0 || strcmp(cmd, "goodnight!") == 0
      || strcmp(cmd, "goodnightnow") == 0) {
    UART1_Send("OK, sleeping now. I will wake you in 60s.\r\n");
    RPi_ScheduleWake(RPI_WAKE_DELAY_MS);
#if defined(HAL_RTC_MODULE_ENABLED) && HAVE_PERSIST_WAKE
    Persist_SaveWakeEpoch(rtc_now_epoch2000() + (RPI_WAKE_DELAY_MS/1000u));
    wake_active = 1;
#endif
    return;
  }

  /* Set daily timetable: expect "nucleotimetable{<24 chars>}{rec,sleep}" */
  if (strstr(cmd, "nucleotimetable")) {
    if (parse_timetable(line_in)) {
      UART1_Send("ACK: timetable set\r\n");
    } else {
      UART1_Send("ERR: invalid timetable format\r\n");
    }
    return;
  }

  /* Resume schedule after processing: "nucleoprocessingcompleted" */
  if (strstr(cmd, "nucleoprocessingcompleted")) {
    if (g_sched_enabled) {
      g_sched_paused = 0;
      UART1_Send("ACK: processing complete, schedule resumed\r\n");
    } else {
      UART1_Send("WARN: no active schedule\r\n");
    }
    return;
  }

  /* fallback: echo */
  UART1_Send("ACK: ");
  UART1_Send(line_in);
  UART1_Send("\r\n");
}

/* --- Runtime relative scheduler (ms-based) --- */
static void RPi_ScheduleWake(uint32_t delay_ms) {
#ifdef RPI_WAKE_GPIO_Port
  rpi_wake_state  = RPI_WAKE_SCHEDULED;
  rpi_wake_due_ms = HAL_GetTick() + delay_ms;
#else
  UART1_Send("WARN: RPISwitch pin not mapped in .ioc\r\n");
#endif
}

/* --- Wake tick: handles persisted & runtime schedules --- */
static void RPi_WakeTick(void) {
#ifdef RPI_WAKE_GPIO_Port
  /* Persisted absolute schedule: fire when due & SoC ok */
#if defined(HAL_RTC_MODULE_ENABLED) && HAVE_PERSIST_WAKE
  if (wake_active) {
    uint32_t now_epoch = rtc_now_epoch2000();
    if ((int32_t)(now_epoch - wake_epoch) >= 0) {
      float v   = Battery_ReadVoltage(8);
      float soc = soc_from_voltage(v);
      if (soc >= SOC_WAKE_THRESHOLD_PCT) {
        /* Fire wake pulse now */
        HAL_GPIO_WritePin(RPI_WAKE_GPIO_Port, RPI_WAKE_Pin, GPIO_PIN_RESET);
        rpi_wake_pulse_end_ms = HAL_GetTick() + RPI_WAKE_PULSE_MS;
        rpi_wake_state = RPI_WAKE_PULSING;
        Persist_ClearWake();
        wake_active = 0;
        UART1_Send("Wake pulse (persisted) sent.\r\n");
      } /* else: wait until SoC improves */
    }
  }
#endif

  /* Runtime relative schedule (as before) */
  uint32_t now = HAL_GetTick();
  switch (rpi_wake_state) {
    case RPI_WAKE_SCHEDULED:
      if ((int32_t)(now - rpi_wake_due_ms) >= 0) {
        HAL_GPIO_WritePin(RPI_WAKE_GPIO_Port, RPI_WAKE_Pin, GPIO_PIN_RESET); // pull low
        rpi_wake_pulse_end_ms = now + RPI_WAKE_PULSE_MS;
        rpi_wake_state = RPI_WAKE_PULSING;
      }
      break;
    case RPI_WAKE_PULSING:
      if ((int32_t)(now - rpi_wake_pulse_end_ms) >= 0) {
        HAL_GPIO_WritePin(RPI_WAKE_GPIO_Port, RPI_WAKE_Pin, GPIO_PIN_SET);   // release (high via pull-up)
        rpi_wake_state = RPI_WAKE_IDLE;
        UART1_Send("Wake pulse sent.\r\n");
      }
      break;
    default: break;
  }
#endif
}

/* --- Restart notifier --- */
static void RestartNotifier_Tick(void) {
  if (!restart_left) return;
  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - restart_next_ms) >= 0) {
    UART1_Send("EVENT: MCU_RESTARTED\r\n");
    restart_left--;
    restart_next_ms = now + RESTART_BROADCAST_PERIOD_MS;
  }
}

/* -------------------- RTC helpers & persistence (only if enabled) -------------------- */
#if defined(HAL_RTC_MODULE_ENABLED)
static int is_leap(int y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}
static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

/* seconds since 2000-01-01 00:00:00 (not Unix epoch); expects BIN fields (yy=0..99) */
static uint32_t make_epoch2000(const RTC_DateTypeDef *d, const RTC_TimeTypeDef *t) {
  int yy = 2000 + d->Year;
  int mm = d->Month;
  int dd = d->Date;

  uint32_t days = 0;
  for (int y = 2000; y < yy; ++y) days += is_leap(y) ? 366 : 365;
  for (int m = 1; m < mm; ++m) {
    days += mdays[m-1];
    if (m == 2 && is_leap(yy)) days += 1;
  }
  days += (dd - 1);
  uint32_t secs = days*86400u + (uint32_t)t->Hours*3600u + (uint32_t)t->Minutes*60u + (uint32_t)t->Seconds;
  return secs;
}

/* Read now() in BCD, convert to BIN locally, then compute epoch */
static uint32_t rtc_now_epoch2000(void) {
  RTC_TimeTypeDef t_bcd; RTC_DateTypeDef d_bcd;
  HAL_RTC_GetTime(&hrtc, &t_bcd, RTC_FORMAT_BCD);
  HAL_RTC_GetDate(&hrtc, &d_bcd, RTC_FORMAT_BCD);
  RTC_TimeTypeDef t_bin = {0};
  RTC_DateTypeDef d_bin = {0};
  d_bin.Year  = bcd2bin(d_bcd.Year);
  d_bin.Month = bcd2bin(d_bcd.Month);
  d_bin.Date  = bcd2bin(d_bcd.Date);
  t_bin.Hours   = bcd2bin(t_bcd.Hours);
  t_bin.Minutes = bcd2bin(t_bcd.Minutes);
  t_bin.Seconds = bcd2bin(t_bcd.Seconds);
  return make_epoch2000(&d_bin, &t_bin);
}

/* *** CHANGED: parser that accepts DD/MM/YYYY HH:MM:SS; falls back to DD/MM/YY *** */
static uint8_t parse_datetime_ddmmyyyy_hhmmss(const char *p,
                                              RTC_DateTypeDef *d,
                                              RTC_TimeTypeDef *t)
{
  /* Accepts:
     - "DD/MM/YYYY HH:MM:SS"  (e.g., 18/10/2025 16:42:05)
     - "DD/MM/YY   HH:MM:SS"  (e.g., 18/10/25   16:42:05)  <-- still supported
  */
  unsigned int DD=0, MM=0, YYYY=0, YY=0, hh=0, mm=0, ss=0;

  /* Try 4-digit year first */
  int n = sscanf(p, " %2u/%2u/%4u %2u:%2u:%2u", &DD, &MM, &YYYY, &hh, &mm, &ss);
  if (n == 6) {
    if (!(DD>=1 && DD<=31 && MM>=1 && MM<=12 && hh<=23 && mm<=59 && ss<=59))
      return 0;

    /* Map YYYY to RTC 0..99 (offset from 2000) */
    if (YYYY < 2000 || YYYY > 2099) return 0; /* RTC holds 0..99 => 2000..2099 */
    unsigned int YYoff = YYYY - 2000;         /* 0..99 */

    d->Date  = (uint8_t)DD;
    d->Month = (uint8_t)MM;
    d->Year  = (uint8_t)YYoff;

    t->Hours          = (uint8_t)hh;
    t->Minutes        = (uint8_t)mm;
    t->Seconds        = (uint8_t)ss;
    t->SubSeconds     = 0;
    t->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    t->StoreOperation = RTC_STOREOPERATION_RESET;
    return 1;
  }

  /* Fallback: 2-digit year */
  n = sscanf(p, " %2u/%2u/%2u %2u:%2u:%2u", &DD, &MM, &YY, &hh, &mm, &ss);
  if (n != 6) return 0;
  if (!(DD>=1 && DD<=31 && MM>=1 && MM<=12 && hh<=23 && mm<=59 && ss<=59))
    return 0;

  YY %= 100;

  d->Date  = (uint8_t)DD;
  d->Month = (uint8_t)MM;
  d->Year  = (uint8_t)YY;

  t->Hours          = (uint8_t)hh;
  t->Minutes        = (uint8_t)mm;
  t->Seconds        = (uint8_t)ss;
  t->SubSeconds     = 0;
  t->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  t->StoreOperation = RTC_STOREOPERATION_RESET;
  return 1;
}

#if HAVE_PERSIST_WAKE
/* Persisted wake using RTC backup registers */
static void Persist_SaveWakeEpoch(uint32_t epoch) {
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_WAKE_EPOCH, epoch);
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_FLAGS, BKP_FLAG_WAKE_ACTIVE);
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_MAGIC, BKP_MAGIC);
}

static int Persist_LoadWakeEpoch(uint32_t *epoch) {
  HAL_PWR_EnableBkUpAccess();
  uint32_t magic = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_MAGIC);
  uint32_t flags = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_FLAGS);
  if (magic == BKP_MAGIC && (flags & BKP_FLAG_WAKE_ACTIVE)) {
    if (epoch) *epoch = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_WAKE_EPOCH);
    return 1;
  }
  return 0;
}

static void Persist_ClearWake(void) {
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_FLAGS, 0);
}
#endif /* HAVE_PERSIST_WAKE */

/* ---------------- Timetable persistence ---------------- */
static uint32_t TT_CalcCRC32(const uint8_t *data, size_t len)
{
  /* Simple 32-bit rolling checksum */
  uint32_t s = 0x12345678u;
  for (size_t i = 0; i < len; ++i) {
    s += data[i];
    s = (s << 5) | (s >> 27);
  }
  return s;
}

static uint16_t TT_CalcCRC16(const uint8_t *data, size_t len)
{
  /* CRC-16/CCITT-FALSE like (simple variant) */
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000u) crc = (crc << 1) ^ 0x1021u;
      else               crc = (crc << 1);
    }
  }
  return crc;
}

static uint8_t TT_ModeToBits(char c)
{
  switch (c) {
    case 'S': case 's': return 1u;
    case 'U': case 'u': return 2u;
    case 'P': case 'p': return 3u;
    default: return 0u;
  }
}

static char TT_BitsToMode(uint8_t b)
{
  switch (b & 3u) {
    case 1u: return 'S';
    case 2u: return 'U';
    case 3u: return 'P';
    default: return '0';
  }
}

static void TT_SaveToBKP(const char modes[24], uint32_t rec_ms, uint32_t sleep_ms)
{
#if (defined(HAL_RTC_MODULE_ENABLED) && (BKP_COUNT >= 5))
  HAL_PWR_EnableBkUpAccess();

  /* Pack 24 modes into 48 bits (2 bits/hour): 00=0,01=S,10=U,11=P */
  uint64_t bits = 0;
  for (int i = 0; i < 24; ++i) {
    uint8_t b = TT_ModeToBits(modes[i]) & 3u;
    bits |= ((uint64_t)b) << (i * 2);
  }
  uint32_t pack0 = (uint32_t)(bits & 0xFFFFFFFFu);
  uint32_t pack1 = (uint32_t)((bits >> 32) & 0xFFFFFFFFu); /* only low 16 bits used */

#if TT_HAVE_RICH
  /* Invalidate first */
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_MAGIC, 0);
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_FLAGS, 0);

  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_PACK0,   pack0);
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_PACK1,   pack1);
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_REC_MS,  rec_ms);
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_SLEEP_MS, sleep_ms);

  /* CRC32 over [pack0 | pack1 | rec_ms | sleep_ms] */
  uint8_t blob[16];
  memcpy(blob+0,  &pack0,   4);
  memcpy(blob+4,  &pack1,   4);
  memcpy(blob+8,  &rec_ms,  4);
  memcpy(blob+12, &sleep_ms,4);
  uint32_t crc = TT_CalcCRC32(blob, sizeof(blob));
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_CRC32, crc);

  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_MAGIC, TT_MAGIC);
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_FLAGS, TT_FLAG_VALID);
#else
  /* Compact layout: store CRC16 in high 16 bits of PACK1; durations packed as seconds in TT_REG_DUR_COMBO */
  uint8_t blob[8];
  memcpy(blob+0,  &pack0,   4);
  memcpy(blob+4,  &pack1,   4);
  uint16_t crc16 = TT_CalcCRC16(blob, sizeof(blob));
  uint32_t pack1_crc = (pack1 & 0x0000FFFFu) | ((uint32_t)crc16 << 16);

  /* Pack durations as seconds into a single 32-bit word */
  uint32_t rec_s   = (rec_ms   + 500u) / 1000u;
  uint32_t sleep_s = (sleep_ms + 500u) / 1000u;
  if (rec_s   > 65535u) rec_s   = 65535u;
  if (sleep_s > 65535u) sleep_s = 65535u;
  uint32_t dur_combo = (sleep_s << 16) | (rec_s & 0xFFFFu);

  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_MAGIC,      TT_MAGIC);
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_PACK0,      pack0);
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_PACK1,      pack1_crc);
  HAL_RTCEx_BKUPWrite(&hrtc, TT_REG_DUR_COMBO,  dur_combo);
#endif

#else
  (void)modes; (void)rec_ms; (void)sleep_ms;
#endif
}

static int TT_LoadFromBKP(char modes_out[24], uint32_t *rec_ms, uint32_t *sleep_ms)
{
#if (defined(HAL_RTC_MODULE_ENABLED) && (BKP_COUNT >= 5))
  HAL_PWR_EnableBkUpAccess();
  /* Check magic */
  if (HAL_RTCEx_BKUPRead(&hrtc, TT_REG_MAGIC) != TT_MAGIC) return 0;

  uint32_t pack0 = 0, pack1 = 0, r = 0, s = 0;

#if TT_HAVE_RICH
  if (!(HAL_RTCEx_BKUPRead(&hrtc, TT_REG_FLAGS) & TT_FLAG_VALID)) return 0;

  pack0 = HAL_RTCEx_BKUPRead(&hrtc, TT_REG_PACK0);
  pack1 = HAL_RTCEx_BKUPRead(&hrtc, TT_REG_PACK1);
  r     = HAL_RTCEx_BKUPRead(&hrtc, TT_REG_REC_MS);
  s     = HAL_RTCEx_BKUPRead(&hrtc, TT_REG_SLEEP_MS);

  uint32_t crc_s = HAL_RTCEx_BKUPRead(&hrtc, TT_REG_CRC32);
  uint8_t blob[16];
  memcpy(blob+0,  &pack0, 4);
  memcpy(blob+4,  &pack1, 4);
  memcpy(blob+8,  &r,     4);
  memcpy(blob+12, &s,     4);
  uint32_t crc_c = TT_CalcCRC32(blob, sizeof(blob));
  if (crc_c != crc_s) return 0;
#else
  pack0 = HAL_RTCEx_BKUPRead(&hrtc, TT_REG_PACK0);
  pack1 = HAL_RTCEx_BKUPRead(&hrtc, TT_REG_PACK1);
  uint32_t dur_combo = HAL_RTCEx_BKUPRead(&hrtc, TT_REG_DUR_COMBO);

  /* Validate CRC16 from high 16 bits of PACK1 */
  uint16_t crc_s = (uint16_t)((pack1 >> 16) & 0xFFFFu);
  uint32_t pack1_low16 = (pack1 & 0x0000FFFFu);
  uint8_t blob[8];
  memcpy(blob+0, &pack0, 4);
  memcpy(blob+4, &pack1_low16, 4);
  uint16_t crc_c = TT_CalcCRC16(blob, sizeof(blob));
  if (crc_c != crc_s) return 0;

  /* Unpack durations (seconds) */
  uint32_t rec_s   = (dur_combo & 0x0000FFFFu);
  uint32_t sleep_s = (dur_combo >> 16) & 0x0000FFFFu;
  r = rec_s * 1000u;
  s = sleep_s * 1000u;
#endif

  /* Unpack to 24 chars */
  uint64_t bits = ((uint64_t)pack0) | (((uint64_t)pack1) << 32);
  for (int i = 0; i < 24; ++i) {
    modes_out[i] = TT_BitsToMode((uint8_t)((bits >> (i*2)) & 3u));
  }
  if (rec_ms)   *rec_ms   = r;
  if (sleep_ms) *sleep_ms = s;
  return 1;
#else
  (void)modes_out; (void)rec_ms; (void)sleep_ms;
  return 0;
#endif
}

/* Pretty-printer: show timetable from BKP (if present) or current runtime */
static void TT_PrintCurrent(void)
{
#if defined(HAL_RTC_MODULE_ENABLED)
  char m[24]; uint32_t r=0, s=0;
  if (TT_LoadFromBKP(m, &r, &s)) {
    UART1_Send("TTABLE FROM BKP: ");
    /* Print as "{...}{rec_s,sleep_s}" */
    UART1_Send("{");
    for (int i=0;i<24;i++){
      char c = m[i];
      char out[4];
      if (i<23) snprintf(out, sizeof(out), "%c,", c);
      else      snprintf(out, sizeof(out), "%c",  c);
      HAL_UART_Transmit(&huart1, (uint8_t*)out, (uint16_t)strlen(out), 50);
    }
    UART1_Send("},{");
    char num[32];
    snprintf(num, sizeof(num), "%lu,%lu", (unsigned long)((r+500)/1000), (unsigned long)((s+500)/1000));
    UART1_Send(num);
    UART1_Send("}\r\n");
    return;
  }
#endif
  /* Fallback: print current runtime timetable if enabled */
  if (g_sched_enabled) {
    UART1_Send("TTABLE (RUNTIME ONLY): ");
    UART1_Send("{");
    for (int i=0;i<24;i++){
      char c = g_day_schedule[i];
      char out[4];
      if (i<23) snprintf(out, sizeof(out), "%c,", c);
      else      snprintf(out, sizeof(out), "%c",  c);
      HAL_UART_Transmit(&huart1, (uint8_t*)out, (uint16_t)strlen(out), 50);
    }
    UART1_Send("},{");
    char num[32];
    snprintf(num, sizeof(num), "%lu,%lu", (unsigned long)((g_rec_ms+500)/1000), (unsigned long)((g_sleep_ms+500)/1000));
    UART1_Send(num);
    UART1_Send("}\r\n");
  } else {
    UART1_Send("TTABLE: none saved; Plan B active (wake Pi at 11:00 local time).\r\n");
  }
}

#endif /* HAL_RTC_MODULE_ENABLED */

/* USER CODE END 4 */

/**
  * @brief  BSP Push Button callback
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER) {
    BspButtonState = BUTTON_PRESSED;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* You can toggle an error LED here */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Optional: report file/line */
}
#endif /* USE_FULL_ASSERT */
