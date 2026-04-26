#include "foc_openloop.h"
#include "foc_config.h"
#include "foc_math.h"

/* 开环轨迹实现。
 *
 * 负责对齐阶段和开环运行阶段的角度、频率推进。
 * 它的输出只是一条“角度时间轨迹”，实际电压/电流怎么给，
 * 由 app_foc 根据当前控制模式决定。
 */
void FOC_OpenLoopInit(FOC_OpenLoopData_t *openloop)
{
  if (openloop == 0)
  {
    return;
  }

  openloop->align_ticks = 0u;
  openloop->mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ;
  openloop->theta_e_rad = 0.0f;
}

void FOC_OpenLoopResetAlign(FOC_OpenLoopData_t *openloop)
{
  if (openloop != 0)
  {
    openloop->align_ticks = 0u;
  }
}

void FOC_OpenLoopResetRun(FOC_OpenLoopData_t *openloop)
{
  if (openloop == 0)
  {
    return;
  }

  openloop->mech_freq_hz = FOC_OPENLOOP_START_FREQ_HZ;
  openloop->theta_e_rad = 0.0f;
}

void FOC_OpenLoopAdvance(FOC_OpenLoopData_t *openloop)
{
  /* 开环频率按线性斜坡从起始频率爬到目标频率。
   * 斜坡越慢，observer 越容易跟上；斜坡越快，启动更快但更容易丢步。
   */
  const float inv_ramp_time_s = 1.0f / (FOC_OPENLOOP_RAMP_TIME_MS * 0.001f);
  const float ramp_rate_hz_per_s = (FOC_OPENLOOP_TARGET_FREQ_HZ - FOC_OPENLOOP_START_FREQ_HZ) * inv_ramp_time_s;
  const float ramp_step_hz = ramp_rate_hz_per_s * FOC_TS_SEC;
  const float two_pi = 6.2831853071795864769f;

  if (openloop == 0)
  {
    return;
  }

  if (openloop->mech_freq_hz < FOC_OPENLOOP_TARGET_FREQ_HZ)
  {
    openloop->mech_freq_hz += ramp_step_hz;
    if (openloop->mech_freq_hz > FOC_OPENLOOP_TARGET_FREQ_HZ)
    {
      openloop->mech_freq_hz = FOC_OPENLOOP_TARGET_FREQ_HZ;
    }
  }

  /* 机械频率乘极对数得到电频率，再乘 2pi 和控制周期得到本拍角度增量。 */
  openloop->theta_e_rad += two_pi * (openloop->mech_freq_hz * FOC_POLE_PAIRS) * FOC_TS_SEC;
  openloop->theta_e_rad = FOC_MathWrap2Pi(openloop->theta_e_rad);
}

float FOC_OpenLoopGetElecSpeedRadPerSec(const FOC_OpenLoopData_t *openloop)
{
  const float two_pi = 6.2831853071795864769f;
  return (openloop != 0) ? (two_pi * openloop->mech_freq_hz * FOC_POLE_PAIRS) : 0.0f;
}

float FOC_OpenLoopGetFreqHz(const FOC_OpenLoopData_t *openloop)
{
  return (openloop != 0) ? openloop->mech_freq_hz : 0.0f;
}

float FOC_OpenLoopGetThetaErad(const FOC_OpenLoopData_t *openloop)
{
  return (openloop != 0) ? openloop->theta_e_rad : 0.0f;
}
