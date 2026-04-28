#ifndef __FOC_SPEED_LOOP_H__
#define __FOC_SPEED_LOOP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_pi_controller.h"
#include "foc_types.h"
#include <stdint.h>

/* RUN-state mechanical speed loop. */
typedef struct
{
  PIController_t pi;
  float speed_ref_cmd_mech_rad_s;
  float speed_target_mech_rad_s;
  float speed_fdb_mech_rad_s;
  float speed_fdb_raw_mech_rad_s;
  float iq_ref_cmd_a;
  uint32_t run_entry_tick_ms;
  uint32_t speed_enable_tick_ms;
  uint8_t enabled;
} FOC_SpeedLoopData_t;

void FOC_SpeedLoopInit(FOC_SpeedLoopData_t *speed_loop);
void FOC_SpeedLoopReset(FOC_SpeedLoopData_t *speed_loop, float speed_init_mech_rad_s);
void FOC_SpeedLoopPrepareRunEntry(FOC_SpeedLoopData_t *speed_loop,
                                  float observer_elec_rad_s,
                                  float fallback_open_elec_rad_s);
void FOC_SpeedLoopSetTargetMechRadS(FOC_SpeedLoopData_t *speed_loop, float target_mech_rad_s);
void FOC_SpeedLoopStep(FOC_SpeedLoopData_t *speed_loop,
                       float observer_elec_rad_s,
                       float dt_sec);
uint8_t FOC_SpeedLoopIsEnabled(const FOC_SpeedLoopData_t *speed_loop);
void FOC_SpeedLoopGetSnapshot(const FOC_SpeedLoopData_t *speed_loop,
                              FOC_SpeedLoopSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
