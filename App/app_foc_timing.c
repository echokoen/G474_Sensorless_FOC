#include "app_foc_timing.h"
#include "foc_config.h"
#include "stm32g4xx_hal.h"

#if (APP_DWT_TIMING_ENABLE != 0u)
static volatile FOC_HfTimingSnapshot_t s_hf_timing_snapshot;
static volatile uint32_t s_hf_timing_seq = 0u;

static uint32_t app_foc_timing_threshold_cycles(void)
{
  uint32_t cycles_per_us = SystemCoreClock / 1000000u;

  if (cycles_per_us == 0u)
  {
    cycles_per_us = 1u;
  }

  return cycles_per_us * APP_DWT_TIMING_OVERRUN_US;
}
#endif

void AppFocTiming_Reset(void)
{
#if (APP_DWT_TIMING_ENABLE != 0u)
  s_hf_timing_seq++;
  s_hf_timing_snapshot.enabled = 1u;
  s_hf_timing_snapshot.sample_count = 0u;
  s_hf_timing_snapshot.last_cycles = 0u;
  s_hf_timing_snapshot.min_cycles = 0u;
  s_hf_timing_snapshot.max_cycles = 0u;
  s_hf_timing_snapshot.avg_cycles = 0u;
  s_hf_timing_snapshot.threshold_cycles = app_foc_timing_threshold_cycles();
  s_hf_timing_snapshot.overrun_count = 0u;
  s_hf_timing_seq++;
#endif
}

void AppFocTiming_Init(void)
{
#if (APP_DWT_TIMING_ENABLE != 0u)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  AppFocTiming_Reset();
#endif
}

uint32_t AppFocTiming_GetCycle(void)
{
#if (APP_DWT_TIMING_ENABLE != 0u)
  return DWT->CYCCNT;
#else
  return 0u;
#endif
}

void AppFocTiming_Update(uint32_t cycles)
{
#if (APP_DWT_TIMING_ENABLE != 0u)
  uint32_t avg;

  s_hf_timing_seq++;
  s_hf_timing_snapshot.last_cycles = cycles;

  if (s_hf_timing_snapshot.sample_count == 0u)
  {
    s_hf_timing_snapshot.min_cycles = cycles;
    s_hf_timing_snapshot.max_cycles = cycles;
    s_hf_timing_snapshot.avg_cycles = cycles;
  }
  else
  {
    if (cycles < s_hf_timing_snapshot.min_cycles)
    {
      s_hf_timing_snapshot.min_cycles = cycles;
    }
    if (cycles > s_hf_timing_snapshot.max_cycles)
    {
      s_hf_timing_snapshot.max_cycles = cycles;
    }

    avg = s_hf_timing_snapshot.avg_cycles;
    if (cycles >= avg)
    {
      avg += ((cycles - avg) >> 4);
    }
    else
    {
      avg -= ((avg - cycles) >> 4);
    }
    s_hf_timing_snapshot.avg_cycles = avg;
  }

  if (s_hf_timing_snapshot.sample_count < 0xFFFFFFFFu)
  {
    s_hf_timing_snapshot.sample_count++;
  }

  if ((s_hf_timing_snapshot.threshold_cycles != 0u) &&
      (cycles > s_hf_timing_snapshot.threshold_cycles))
  {
    if (s_hf_timing_snapshot.overrun_count < 0xFFFFFFFFu)
    {
      s_hf_timing_snapshot.overrun_count++;
    }
  }

  s_hf_timing_seq++;
#else
  (void)cycles;
#endif
}

uint8_t AppFocTiming_GetSnapshot(FOC_HfTimingSnapshot_t *snapshot)
{
#if (APP_DWT_TIMING_ENABLE != 0u)
  uint32_t seq_start;
  uint32_t seq_end;
  uint8_t retry = 8u;

  if (snapshot == 0)
  {
    return 0u;
  }

  do
  {
    seq_start = s_hf_timing_seq;
    if ((seq_start & 1u) != 0u)
    {
      continue;
    }

    *snapshot = s_hf_timing_snapshot;
    seq_end = s_hf_timing_seq;

    if ((seq_start == seq_end) && ((seq_end & 1u) == 0u))
    {
      return 1u;
    }
  } while (--retry != 0u);
#else
  (void)snapshot;
#endif

  return 0u;
}
