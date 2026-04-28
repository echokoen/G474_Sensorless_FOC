#include "foc_math.h"
#include <math.h>

void FOC_Math_SinCos(float theta_rad, float *sin_out, float *cos_out)
{
  const float sin_v = sinf(theta_rad);
  const float cos_v = cosf(theta_rad);

  if (sin_out != 0)
  {
    *sin_out = sin_v;
  }

  if (cos_out != 0)
  {
    *cos_out = cos_v;
  }
}
