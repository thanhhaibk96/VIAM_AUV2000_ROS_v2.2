#ifndef SBG_LOS_H
#define SBG_LOS_H

#include <iostream>

#include "base_los.h"

class SbgLOS : public BaseLOS
{
public:
  SbgLOS();

  float Dsafe = 4;  // Khoang cach an toan
  float DLA = 15;    // Khoang cach bat dau ne
  float dz_b = 0;   // (khong su dung)
  float dz_rl = 2;  // Can duoi cua lidar
  float dz_rh = 30; // Can tren lidar

  float u;
  float v0;
  float heading0;
  bool isStatic;
  bool doTrick = false;
  bool isASBG;

  bool isCol = false;
  bool stop = true;
  int k = 0;
  std::vector<double> WPx, WPy;

  void setupLOS();
  void resetLOS();

  float norm(float* v);
  float calmuy(float D, float Dsafe);
  float caldis(float* ctr, int N);
  void cone_space(float Rs, float* ctr, int N, float& g_up, float& g_low);
  int set_deadzone(float dz_b, float dz_rl, float dz_rh, float Rs, float* ctr, int N, float* min_dist, float* g_up,
                   float* g_low);
  bool in_cone_space(float g_up, float g_low);
  void Tsolve(float &psi_ud, float vu, float vo, float psi_o, float psi_uod);
  int in_Colreg(float psi_u, float psi_o);
  bool col_assess(float DLA, float* sigma, float* g_up, float* g_low, int noz, float _g_up, float _g_low, float _dis);
  bool set_deadzone_2(float dz_b, float dz_rl, float dz_rh, float Rs, const float* ctr, int N, float angle_min,
                      float angle_res, float& min_dist, float& g_up, float& g_low);
  bool col_assess_2(float DLA, float sigma, float g_up, float g_low);
  bool col_assess_2(float DLA, float sigma, float g_up, float g_low, float psi_uod);
  float relative_angle(float v1,float psi1,float v2,float psi2);
  float LOSGuidance_sbg(float x, float y, double WPx[], double WPy[], int& k, float& alpha_k, float beta, float& ye,
                        int n_wp);
  float* ASBG(float xu, float yu, float psi_u, const float* ctr, int N, float angle_min, float angle_res);
  float* ASBG(float xu, float yu, float vu, float psi_u, float vo, float psi_o, const float *ctr, int N, float angle_min, float angle_res);
  float* SBG(float xu, float yu, float vu, float psi_u, float vo, float psi_o, const float *ctr, int N, float angle_min, float angle_res);
};
#endif // SBG_LOS_H
