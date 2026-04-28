#include "foc_trace.h"

/* 调试快照聚合实现。
 *
 * 把采样、PWM 与控制链路中的关键数据收敛成一致快照，
 * 方便后台低频打印时读取同一拍的数据。
 */
void FOC_Trace_Init(FOC_Trace_t *trace)
{
  if (trace == 0)
  {
    return;
  }

  trace->snapshot.seq = 0u;
  trace->snapshot.sec = 0u;
  trace->snapshot.pwm_sec = 0u;
  trace->snapshot.pair = 0u;
  trace->snapshot.fb = 0u;
  trace->snapshot.hold = 0u;
  trace->snapshot.dyn = 0u;
  trace->snapshot.ccru = 0u;
  trace->snapshot.ccrv = 0u;
  trace->snapshot.ccrw = 0u;
  trace->snapshot.t1 = 0.0f;
  trace->snapshot.t2 = 0.0f;
  trace->snapshot.t0 = 0.0f;
  trace->snapshot.win1 = 0u;
  trace->snapshot.win2 = 0u;
  trace->snapshot.smp = 0u;
  trace->snapshot.rawu = 0u;
  trace->snapshot.rawv = 0u;
  trace->snapshot.raww = 0u;
  trace->snapshot.iu = 0.0f;
  trace->snapshot.iv = 0.0f;
  trace->snapshot.iw = 0.0f;
  trace->snapshot.isum = 0.0f;
}

void FOC_Trace_UpdateSnapshot(FOC_Trace_t *trace,
                                    const FOC_CurrentSense_t *current_sense,
                                    const FOC_Svpwm_t *svpwm)
{
  if ((trace == 0) || (current_sense == 0) || (svpwm == 0))
  {
    return;
  }

  /* seq 使用奇偶保护：
   * - 写入开始时置为奇数；
   * - 字段全部写完后置为偶数；
   * - 读取侧只有看到前后 seq 相同且为偶数，才认为快照一致。
   */
  const uint32_t next_seq = trace->snapshot.seq + 1u;
  trace->snapshot.seq = next_seq;
  trace->snapshot.sec = current_sense->sample_sector;
  trace->snapshot.pwm_sec = svpwm->svpwm_sector;
  trace->snapshot.pair = current_sense->trusted_current_pair;
  trace->snapshot.fb = svpwm->adc_trigger_fallback;
  trace->snapshot.hold = current_sense->used_hold_last_current;
  trace->snapshot.dyn = svpwm->adc_dynamic_sampling;
  trace->snapshot.ccru = svpwm->ccr_u;
  trace->snapshot.ccrv = svpwm->ccr_v;
  trace->snapshot.ccrw = svpwm->ccr_w;
  trace->snapshot.t1 = svpwm->svpwm_t1;
  trace->snapshot.t2 = svpwm->svpwm_t2;
  trace->snapshot.t0 = svpwm->svpwm_t0;
  trace->snapshot.win1 = svpwm->adc_win1_ccr;
  trace->snapshot.win2 = svpwm->adc_win2_ccr;
  trace->snapshot.smp = svpwm->adc_sample_ccr;
  trace->snapshot.rawu = current_sense->raw_u;
  trace->snapshot.rawv = current_sense->raw_v;
  trace->snapshot.raww = current_sense->raw_w;
  trace->snapshot.iu = current_sense->iu_a;
  trace->snapshot.iv = current_sense->iv_a;
  trace->snapshot.iw = current_sense->iw_a;
  trace->snapshot.isum = current_sense->iu_a + current_sense->iv_a + current_sense->iw_a;
  trace->snapshot.seq = next_seq + 1u;
}

uint8_t FOC_Trace_GetSnapshot(const FOC_Trace_t *trace, FOC_TraceSnapshot_t *snapshot)
{
  if ((trace == 0) || (snapshot == 0))
  {
    return 0u;
  }

  {
    uint8_t retry = 8u;

    do
    {
      const uint32_t seq1 = trace->snapshot.seq;

      /* 高频侧正在写入时 seq 为奇数，低频侧本轮不死等。 */
      if ((seq1 & 1u) != 0u)
      {
        continue;
      }

      *snapshot = trace->snapshot;

      /* 复制前后 seq 不变，说明没有读到半包数据。 */
      if ((seq1 == trace->snapshot.seq) && ((trace->snapshot.seq & 1u) == 0u))
      {
        return 1u;
      }
    } while (--retry != 0u);
  }

  return 0u;
}
