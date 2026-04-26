#include "foc_debug.h"

/* 调试快照聚合实现。
 *
 * 把采样、PWM 与控制链路中的关键数据收敛成一致快照，
 * 方便后台低频打印时读取同一拍的数据。
 */
void FOC_DebugModule_Init(FOC_DebugData_t *debug_data)
{
  if (debug_data == 0)
  {
    return;
  }

  debug_data->snapshot.seq = 0u;
  debug_data->snapshot.sec = 0u;
  debug_data->snapshot.pwm_sec = 0u;
  debug_data->snapshot.pair = 0u;
  debug_data->snapshot.fb = 0u;
  debug_data->snapshot.hold = 0u;
  debug_data->snapshot.dyn = 0u;
  debug_data->snapshot.ccru = 0u;
  debug_data->snapshot.ccrv = 0u;
  debug_data->snapshot.ccrw = 0u;
  debug_data->snapshot.t1 = 0.0f;
  debug_data->snapshot.t2 = 0.0f;
  debug_data->snapshot.t0 = 0.0f;
  debug_data->snapshot.win1 = 0u;
  debug_data->snapshot.win2 = 0u;
  debug_data->snapshot.smp = 0u;
  debug_data->snapshot.rawu = 0u;
  debug_data->snapshot.rawv = 0u;
  debug_data->snapshot.raww = 0u;
  debug_data->snapshot.iu = 0.0f;
  debug_data->snapshot.iv = 0.0f;
  debug_data->snapshot.iw = 0.0f;
  debug_data->snapshot.isum = 0.0f;
}

void FOC_DebugModule_UpdateSnapshot(FOC_DebugData_t *debug_data,
                                    const FOC_SamplingData_t *sampling,
                                    const FOC_PwmData_t *pwm)
{
  if ((debug_data == 0) || (sampling == 0) || (pwm == 0))
  {
    return;
  }

  /* seq 使用奇偶保护：
   * - 写入开始时置为奇数；
   * - 字段全部写完后置为偶数；
   * - 读取侧只有看到前后 seq 相同且为偶数，才认为快照一致。
   */
  const uint32_t next_seq = debug_data->snapshot.seq + 1u;
  debug_data->snapshot.seq = next_seq;
  debug_data->snapshot.sec = sampling->sample_sector;
  debug_data->snapshot.pwm_sec = pwm->svpwm_sector;
  debug_data->snapshot.pair = sampling->trusted_current_pair;
  debug_data->snapshot.fb = pwm->adc_trigger_fallback;
  debug_data->snapshot.hold = sampling->used_hold_last_current;
  debug_data->snapshot.dyn = pwm->adc_dynamic_sampling;
  debug_data->snapshot.ccru = pwm->ccr_u;
  debug_data->snapshot.ccrv = pwm->ccr_v;
  debug_data->snapshot.ccrw = pwm->ccr_w;
  debug_data->snapshot.t1 = pwm->svpwm_t1;
  debug_data->snapshot.t2 = pwm->svpwm_t2;
  debug_data->snapshot.t0 = pwm->svpwm_t0;
  debug_data->snapshot.win1 = pwm->adc_win1_ccr;
  debug_data->snapshot.win2 = pwm->adc_win2_ccr;
  debug_data->snapshot.smp = pwm->adc_sample_ccr;
  debug_data->snapshot.rawu = sampling->raw_u;
  debug_data->snapshot.rawv = sampling->raw_v;
  debug_data->snapshot.raww = sampling->raw_w;
  debug_data->snapshot.iu = sampling->iu_a;
  debug_data->snapshot.iv = sampling->iv_a;
  debug_data->snapshot.iw = sampling->iw_a;
  debug_data->snapshot.isum = sampling->iu_a + sampling->iv_a + sampling->iw_a;
  debug_data->snapshot.seq = next_seq + 1u;
}

uint8_t FOC_DebugModule_GetSnapshot(const FOC_DebugData_t *debug_data, FOC_DebugSnapshot_t *snapshot)
{
  if ((debug_data == 0) || (snapshot == 0))
  {
    return 0u;
  }

  {
    uint8_t retry = 8u;

    do
    {
      const uint32_t seq1 = debug_data->snapshot.seq;

      /* 高频侧正在写入时 seq 为奇数，低频侧本轮不死等。 */
      if ((seq1 & 1u) != 0u)
      {
        continue;
      }

      *snapshot = debug_data->snapshot;

      /* 复制前后 seq 不变，说明没有读到半包数据。 */
      if ((seq1 == debug_data->snapshot.seq) && ((debug_data->snapshot.seq & 1u) == 0u))
      {
        return 1u;
      }
    } while (--retry != 0u);
  }

  return 0u;
}
