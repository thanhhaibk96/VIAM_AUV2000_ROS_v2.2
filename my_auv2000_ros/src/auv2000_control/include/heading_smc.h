#ifndef HEADING_SMC_H
#define HEADING_SMC_H

#include <cmath>
#include <iostream>

using namespace std;

class HeadingSMC
{
public:
  HeadingSMC();

  double k;
  double nuy;

  double surface;
  double error;
  double error_dot;
  double psid_dot = 0;
  double psid_2dot = 0;
  double u, v, r;
  double output;

  double theta[7];

  void runSMC();
};

#endif // HEADING_SMC_H
