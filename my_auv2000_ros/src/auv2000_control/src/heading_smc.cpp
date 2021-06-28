#include "heading_smc.h"

HeadingSMC::HeadingSMC() {}

void HeadingSMC::runSMC()
{
  double f = -(theta[1] * u * r + theta[2] * u * v + theta[3] * v + theta[4] * abs(v) * v + theta[5] * r +
               theta[6] * abs(r) * r) /
             theta[0];
  double b = 1 / theta[0];
  error_dot = r - psid_dot;
  surface = k * error + error_dot;
  output = -(f + k * error + nuy * tanh(surface) - psid_2dot) / b;

  output = -output;
}
