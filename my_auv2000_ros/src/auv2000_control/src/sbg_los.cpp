#include "sbg_los.h"

SbgLOS::SbgLOS() {}

void SbgLOS::setupLOS()
{
  WPx.clear();
  WPy.clear();
  for (auto it = waypoints.begin(); it != waypoints.end(); it++)
  {
    WPx.push_back(it->x);
    WPy.push_back(it->y);
  }
}

void SbgLOS::resetLOS()
{
  waypoints.clear();
  k = 0;
}

float SbgLOS::norm(float* v) { return sqrtf(v[0] * v[0] + v[1] * v[1]); }

float SbgLOS::calmuy(float D, float Dsafe) // Modified
{
  if (Dsafe / D > 1)
    return M_PI / 2;
  else
    return asinf(Dsafe / D);
}
float SbgLOS::caldis(float* ctr, int N)
{
  float dis = 100;
  for (int i = 0; i < N; i += 2)
  {
    float tmp = norm(&(ctr[i]));
    if (dis > tmp)
      dis = tmp;
  }
  return dis;
}

void SbgLOS::cone_space(float Rs, float* ctr, int N, float& g_up, float& g_low) //-> test ham nay
{
  // Rs : safety radius
  // ctr: array of point got from Lidar
  // N: length of ctr
  // g_up and g_low: two edge of cone space
  g_up = atan2f(ctr[1], ctr[0]) + calmuy(norm(ctr), Rs);
  g_low = atan2f(ctr[1], ctr[0]) - calmuy(norm(ctr), Rs);
  float c_up, c_low;
  for (int i = 0; i < N; i += 2)
  {
    float tmp[2] = {ctr[i], ctr[i + 1]};
    c_up = atan2f(tmp[1], tmp[0]) + calmuy(norm(tmp), Rs);
    c_low = atan2f(tmp[1], tmp[0]) - calmuy(norm(tmp), Rs);
    // std::cout << "g_up = "<< c_up<<" g_low = "<<c_low << std::endl;
    if (c_up > g_up)
      g_up = c_up;
    if (c_low < g_low)
      g_low = c_low;
  }
}

int SbgLOS::set_deadzone(float dz_b, float dz_rl, float dz_rh, float Rs, float* ctr, int N, float* min_dist,
                         float* g_up, float* g_low)
{
  bool init = true;
  float pre_ctr[2] = {0}, pre_t = 0, pre_n = 0, c_up, c_low;
  int noz = -1;
  min_dist[0] = 1412;
  for (int i = 0; i < N; i += 2)
  {
    float tmp = atan2f(-ctr[i + 1], ctr[i]);
    float n_tmp = norm(&(ctr[i]));
    if ((fabsf(tmp) < M_PI - dz_b / 2) && (n_tmp > dz_rl) && (n_tmp < dz_rh))
    {
      if (fabsf(pre_n - n_tmp) > 0.5f || fabsf(pre_t - tmp) > 5 * M_PI / 180)
      {
        init = true;
        noz++;
      }
      if (init)
      {
        g_up[noz] = tmp + calmuy(n_tmp, Rs);
        g_low[noz] = tmp - calmuy(n_tmp, Rs);
        min_dist[noz] = n_tmp;
        init = false;
      }
      else
      {
        if (min_dist[noz] > n_tmp)
          min_dist[noz] = n_tmp;
        c_up = tmp + calmuy(n_tmp, Rs);
        c_low = tmp - calmuy(n_tmp, Rs);
        if (c_up > g_up[noz])
          g_up[noz] = c_up;
        if (c_low < g_low[noz])
          g_low[noz] = c_low;
      }
      pre_t = tmp;
      pre_n = n_tmp;
      pre_ctr[0] = ctr[i];
      pre_ctr[1] = ctr[i + 1];
    }
  }
  return noz;
}

bool SbgLOS::in_cone_space(float g_up, float g_low)
{
  if (g_up > 0 && g_low < 0)
    return true;
  else
    return false;
}

void SbgLOS::Tsolve(float& psi_ud, float vu, float vo, float psi_o, float psi_uod)
{
  psi_ud = psi_uod;
  if (vo * sin(psi_o - psi_uod) / vu > 1)
    std::cout << "error in Tsolve" << std::endl;
  else
    psi_ud = asin(vo * sin(psi_o - psi_uod) / vu) + psi_uod;
}

int SbgLOS::in_Colreg(float psi_u, float psi_o)
{
  int direct = 1;
  float beta = (psi_u - psi_o) * 180 / M_PI;

  while (beta > 360)
    beta = beta - 360;
  while (beta < 0)
    beta = beta + 360;
  if (beta > 200 && beta < 360)
    direct = -1;
  std::cout << "beta = " << beta << std::endl;
  return direct;
}

bool SbgLOS::col_assess(float DLA, float* sigma, float* g_up, float* g_low, int noz, float _g_up, float _g_low,
                        float _dis)
{
  bool F_C = false;
  static bool FT = false;
  float mtmp = 100, tmp;
  int i_track = 0;
  for (int ii = 0; ii <= noz; ii++)
  {
    if (FT)
    {
      tmp = fabsf(g_up[ii] - _g_up) * 4 + fabsf(g_low[ii] - _g_low) * 4 + fabsf(sigma[ii] - _dis);
      if (tmp < mtmp)
      {
        i_track = ii;
        mtmp = tmp;
      }
    }
    else
    {
      if (sigma[ii] < DLA && in_cone_space(g_up[ii], g_low[ii]))
      {
        FT = true;
        F_C = true;
        i_track = ii;
        break;
      }
    }
  }

  _g_up = g_up[i_track];
  _g_low = g_low[i_track];
  _dis = sigma[i_track];
  F_C = in_cone_space(_g_up, _g_low);
  return F_C;
}

bool SbgLOS::set_deadzone_2(float dz_b, float dz_rl, float dz_rh, float Rs, const float* ctr, int N, float angle_min,
                            float angle_res, float& min_dist, float& g_up, float& g_low)
{
  min_dist = 1412;
  bool ii = true;
  // g_up = -M_PI/2;
  // g_low = M_PI/2;
  for (int i = 0; i < N; i += 2)
  {
    float tmp = -(angle_min + i * angle_res); // atan2(-ctr[i + 1], ctr[i]);
    float n_tmp = ctr[i];                     // norm(&(ctr[i]));
    if ((fabsf(tmp) < M_PI - dz_b / 2) && (n_tmp > dz_rl) && (n_tmp < dz_rh))
    {
      if (ii)
      {
        g_up = tmp + calmuy(n_tmp, Rs);
        g_low = tmp - calmuy(n_tmp, Rs);
        ii = false;
      }
      if (min_dist > n_tmp)
        min_dist = n_tmp;
      float c_up = tmp + calmuy(n_tmp, Rs);
      float c_low = tmp - calmuy(n_tmp, Rs);
      if (c_up > g_up)
        g_up = c_up;
      if (c_low < g_low)
        g_low = c_low;
    }
  }

  return false;
}

bool SbgLOS::col_assess_2(float DLA, float sigma, float g_up, float g_low)
{
  bool F_C = false;
  if (sigma < DLA)
  {
    if (in_cone_space(g_up, g_low))
      F_C = true;
  }
  return F_C;
}
float SbgLOS::LOSGuidance_sbg(float x, float y, double WPx[], double WPy[], int& k, float& alpha_k, float beta,
                              float& ye, int n_wp)
{
  alpha_k = atan2f(WPy[k + 1] - WPy[k], WPx[k + 1] - WPx[k]);
  float s_k_1 = (WPx[k + 1] - WPx[k]) * cosf(alpha_k) + (WPy[k + 1] - WPy[k]) * sinf(alpha_k);
  // Calculate the cross-track error
  ye = -(x - WPx[k]) * sinf(alpha_k) + (y - WPy[k]) * cosf(alpha_k);
  // Calculate the along-track error
  float xe = (x - WPx[k]) * cosf(alpha_k) + (y - WPy[k]) * sinf(alpha_k);
  // Find course angle
  float delta = (maxDelta - minDelta) * exp(-0.3 * (ye * ye)) + minDelta;
  float Chi_r = atan(-ye / delta);
  float psi_d = alpha_k + Chi_r - beta * exp(-10 * fabs(ye));
  if (fabsf(s_k_1 - xe) <= radius)
    k++;
  if (k == n_wp - 1)
    stop = true;
  return psi_d;
}

bool SbgLOS::col_assess_2(float DLA, float sigma, float g_up, float g_low, float psi_uod)
{
  bool F_C = false;
  float _g_up = g_up - psi_uod;
  float _g_low = g_low - psi_uod;
  if (sigma < DLA)
  {
    if (in_cone_space(_g_up, _g_low))
      F_C = true;
  }
  return F_C;
}

float SbgLOS::relative_angle(float v1, float psi1, float v2, float psi2)
{
  return atan2(v1 * sin(psi1) - v2 * sin(psi2), v1 * cos(psi1) - v2 * cos(psi2));
}

float* SbgLOS::ASBG(float xu, float yu, float psi_u, const float* ctr, int N, float angle_min, float angle_res)
{
  auto x_wp = WPx.data();
  auto y_wp = WPy.data();
  auto n_wp = WPx.size();

  /* Input */
  /* psi_los: goc guidance tu bo LOS
  alpha_k: goc cua duong thang ma USV dang bam theo
  F_C: Flag bao va cham
  */
  /*Output*/
  /*
  psi_d: goc dat cho bo dieu khien
  FF: co bao ket thuc giai thuat tranh va cham
  */
  static double WP_oax[4] = {0};
  static double WP_oay[4] = {0};
  static float _g_up = 0, _g_low = 0, dis = 0, alpha_k, a, psi_ud;
  static int prev_mode = 1;
  static int k_oa = 0;
  static int direct = 1;
  static int F_OA = 0, Fdo = 0; // Flag bao dang trong che do tranh vat can
  int mode = 1;
  float ye, psi_U, g_up, g_low, ye_los = 0;
  //  float min_dist[100], ag_up[100], ag_low[100];
  float _dz_b = dz_b;
  float _dz_rh = dz_rh;
  // prev_mode = 1;
  // int noz = set_deadzone(dz_b,dz_rl,dz_rh,Dsafe,ctr,N,min_dist,ag_up,ag_low);
  // bool F_C = col_assess(DLA,min_dist,ag_up,ag_low,noz,_g_up,_g_low,dis);

  // Dirty trick
  if (prev_mode == 1)
  {
    _dz_b = 4 * M_PI / 3;
  }
  else
  {
    _dz_rh = 7.5;
  }

  set_deadzone_2(_dz_b, dz_rl, _dz_rh, Dsafe, ctr, N, angle_min, angle_res, dis, _g_up, _g_low);
  bool F_C = col_assess_2(DLA, dis, _g_up, _g_low);
  float psi_d;
  if (prev_mode != 1)
    psi_d = LOSGuidance_sbg(xu, yu, WP_oax, WP_oay, k_oa, a, 0, ye_los, 4); // Obstacle avoidance path
  else
    psi_d = LOSGuidance_sbg(xu, yu, x_wp, y_wp, k, alpha_k, 0, ye_los, n_wp);

  //  float Rsc = Dsafe;
  bool FF = 1;
  if (F_OA == 0 && F_C == true && Fdo == 0)
  {
    F_OA = 1;
    Fdo = 1;
  }

  if (F_OA)
  {
    FF = 0;
    isCol = true;
    if (prev_mode == 1)
    {
      if (fabsf(_g_up) < fabsf(_g_low))
      {
        direct = 1;
        psi_ud = psi_u + _g_up;
      }
      else
      {
        direct = -1;
        psi_ud = psi_u + _g_low;
      }
      k_oa = 0;
      mode = 2;
      WP_oax[0] = xu;
      WP_oay[0] = yu;
      WP_oax[1] = xu + 100 * cos(psi_ud);
      WP_oay[1] = yu + 100 * sin(psi_ud);
      std::cout << "mode = " << mode << " | g_up = " << _g_up << " | g_low = " << _g_low << " | dis = " << dis
                << std::endl;
    }
    else if (prev_mode == 2)
    {
      psi_U = alpha_k - psi_u;
      while (psi_U < -4 * M_PI / 3)
        psi_U = psi_U + 2 * M_PI;
      while (psi_U > 4 * M_PI / 3)
        psi_U = psi_U - 2 * M_PI;
      g_up = _g_up - psi_U;
      g_low = _g_low - psi_U;
      float x_tmp = (xu - WP_oax[0]) * cos(psi_ud) + (yu - WP_oay[0]) * sin(psi_ud);
      if (!in_cone_space(g_up, g_low) && x_tmp > Dsafe + 2)
      {
        mode = 3;
        //
        WP_oax[1] = WP_oax[0] + (x_tmp + 1) * cos(psi_ud); //
        WP_oay[1] = WP_oay[0] + (x_tmp + 1) * sin(psi_ud); //
        WP_oax[2] = WP_oax[1] + 100 * cos(alpha_k);        //
        WP_oay[2] = WP_oay[1] + 100 * sin(alpha_k);        //
        std::cout << "mode = " << mode << " | x_tmp = " << x_tmp << std::endl;
      }
      else
        mode = 2;
    }
    else if (prev_mode == 3)
    {
      psi_ud = alpha_k - direct * M_PI / 6;
      psi_U = psi_ud - psi_u;
      while (psi_U < -4 * M_PI / 3)
        psi_U = psi_U + 2 * M_PI;
      while (psi_U > 4 * M_PI / 3)
        psi_U = psi_U - 2 * M_PI;
      g_up = _g_up - psi_U;
      g_low = _g_low - psi_U;
      float x_tmp = (xu - WP_oax[1]) * cos(alpha_k) + (yu - WP_oay[1]) * sin(alpha_k);
      if ((!in_cone_space(g_up, g_low)) && fabs(x_tmp) > 3)
      {
        mode = 4;
        WP_oax[2] = WP_oax[1] + (x_tmp + 1) * cos(alpha_k); //
        WP_oay[2] = WP_oay[1] + (x_tmp + 1) * sin(alpha_k); //
        WP_oax[3] = WP_oax[2] + 50 * cos(psi_ud);           //
        WP_oay[3] = WP_oay[2] + 50 * sin(psi_ud);           //
      }
      else
        mode = 3;
    }
    else if (prev_mode == 4)
    {
      ye = -(xu - x_wp[k]) * sin(alpha_k) + (yu - y_wp[k]) * cos(alpha_k);
      if (fabs(ye) < 1)
      {
        mode = 1;
        F_OA = 0;
        isCol = false;
      }
      else
        mode = 4;
    }
  }
  // Adjust psi_d response to psi_u
  while (psi_d - psi_u < -4 * M_PI / 3)
    psi_d = psi_d + 2 * M_PI;

  while (psi_d - psi_u > 4 * M_PI / 3)
    psi_d = psi_d - 2 * M_PI;
  prev_mode = mode;
  // std::cout<<"mode="<<mode<<std::endl;
  static float return_SBG[20] = {0};
  return_SBG[0] = psi_d;
  return_SBG[1] = ye_los;
  return_SBG[2] = dis;
  return_SBG[3] = _g_up;
  return_SBG[4] = _g_low;
  return_SBG[5] = x_wp[0];
  return_SBG[6] = x_wp[1];
  return_SBG[7] = y_wp[0];
  return_SBG[8] = y_wp[1];
  return_SBG[9] = WP_oax[0];
  return_SBG[10] = WP_oax[1];
  return_SBG[11] = WP_oax[2];
  return_SBG[12] = WP_oax[3];
  return_SBG[13] = WP_oay[0];
  return_SBG[14] = WP_oay[1];
  return_SBG[15] = WP_oay[2];
  return_SBG[16] = WP_oay[3];
  return_SBG[17] = xu;
  return_SBG[18] = yu;
  return_SBG[19] = psi_u;
  desiredHeading = psi_d;

  return return_SBG;
}

float* SbgLOS::ASBG(float xu, float yu, float vu, float psi_u, float vo, float psi_o, const float* ctr, int N, float angle_min, float angle_res)
{
  auto x_wp = WPx.data();
  auto y_wp = WPy.data();
  auto n_wp = WPx.size();

  /* Input */
  /* psi_los: goc guidance tu bo LOS
  alpha_k: goc cua duong thang ma USV dang bam theo
  F_C: Flag bao va cham
  */
  /*Output*/
  /*
  psi_d: goc dat cho bo dieu khien
  FF: co bao ket thuc giai thuat tranh va cham
  */
  static double WP_oax[5] = {0};
  static double WP_oay[5] = {0};
  static float _g_up = 0, _g_low = 0, dis = 0, alpha_k, a, psi_ud;
  static int prev_mode = 1;
  static int k_oa = 0, k = 0;
  static int direct = 1;
  static int F_OA = 0, Fdo = 0; // Flag bao dang trong che do tranh vat can
  int mode = 1;
  float ye, psi_U, g_up, g_low, ye_los = 0, psi_uod;
  // float min_dist[100],ag_up[100],ag_low[100];
  float _dz_b = dz_b;
  float _dz_rh = dz_rh;
  // prev_mode = 1;
  // int noz = set_deadzone(dz_b,dz_rl,dz_rh,Dsafe,ctr,N,min_dist,ag_up,ag_low);
  // bool F_C = col_assess(DLA,min_dist,ag_up,ag_low,noz,_g_up,_g_low,dis);

  // Dirty trick
  if (doTrick)
  {
    if (prev_mode == 1)
    {
      _dz_b = 4 * M_PI / 3;
    }
    else
    {
      _dz_rh = 35;
    }
  }

  float psi_uo = relative_angle(vu, alpha_k, vo, psi_o);
  set_deadzone_2(_dz_b, dz_rl, _dz_rh, Dsafe, ctr, N, angle_min, angle_res, dis, _g_up, _g_low);
  bool F_C = col_assess_2(DLA, dis, _g_up, _g_low, psi_uo - psi_u);
  //  std::cout << _g_up << " " << _g_low << " " << dis << " " << psi_uo << std::endl;
  //  std::cout << alpha_k << std::endl;
  float psi_d;
  if (prev_mode != 1)
    psi_d = LOSGuidance_sbg(xu, yu, WP_oax, WP_oay, k_oa, a, 0, ye_los, 4); // Obstacle avoidance path
  else
    psi_d = LOSGuidance_sbg(xu, yu, x_wp, y_wp, k, alpha_k, 0, ye_los, n_wp);
  bool FF = 1;
  if (F_OA == 0 && F_C == true && Fdo == 0)
  {
    F_OA = 1;
    Fdo = 1;
  }

  if (F_OA)
  {
    FF = 0;
    if (prev_mode == 1)
    {
      direct = in_Colreg(alpha_k, psi_o);
      if (direct == 1)
      {
        psi_uod = psi_u + _g_up;
      }
      else
      {
        psi_uod = psi_u + _g_low;
      }
      Tsolve(psi_ud, vu, vo, psi_o, psi_uod);
      k_oa = 0;
      mode = 2;
      WP_oax[0] = xu;
      WP_oay[0] = yu;
      WP_oax[1] = xu + 100 * cos(psi_ud);
      WP_oay[1] = yu + 100 * sin(psi_ud);
      //      WP_oax[2] = WP_oax[1] + 4 * cos(alpha_k);
      //      WP_oay[2] = WP_oax[1] + 4 * sin(alpha_k);
      //      WP_oax[3] = WP_oax[0] + (7.5 * cos(psi_ud - alpha_k) + 4 + 4) * cos(alpha_k);
      //      WP_oay[3] = WP_oax[0] + (7.5 * cos(psi_ud - alpha_k) + 4 + 4) * sin(alpha_k);
      //      WP_oax[4] = x_wp[1];
      //      WP_oay[4] = y_wp[1];
      std::cout << "mode = " << mode << " | g_up = " << _g_up << " | g_low = " << _g_low << " | dis = " << dis
                << " psi_ud = " << psi_ud << " psi_uod = " << psi_uod << "direct = " << direct << std::endl;
    }
    else if (prev_mode == 2)
    {
      psi_U = alpha_k - psi_u;
      while (psi_U < -4 * M_PI / 3)
        psi_U = psi_U + 2 * M_PI;
      while (psi_U > 4 * M_PI / 3)
        psi_U = psi_U - 2 * M_PI;
      g_up = _g_up - psi_U - (psi_uo - alpha_k);
      g_low = _g_low - psi_U - (psi_uo - alpha_k);
      float x_tmp = (xu - WP_oax[0]) * cos(psi_ud) + (yu - WP_oay[0]) * sin(psi_ud);
      if (!in_cone_space(g_up, g_low))
      {
        mode = 3;
        //
        WP_oax[1] = WP_oax[0] + (x_tmp + 1) * cos(psi_ud); //
        WP_oay[1] = WP_oay[0] + (x_tmp + 1) * sin(psi_ud); //
        WP_oax[2] = WP_oax[1] + 100 * cos(alpha_k);        //
        WP_oay[2] = WP_oay[1] + 100 * sin(alpha_k);        //
        //        WP_oax[3] = WP_oax[0] + ((x_tmp + 1) * cos(psi_ud - alpha_k) + 4 + 4) * cos(alpha_k);
        //        WP_oay[3] = WP_oax[0] + ((x_tmp + 1) * cos(psi_ud - alpha_k) + 4 + 4) * sin(alpha_k);
        //        WP_oax[4] = x_wp[1];
        //        WP_oay[4] = y_wp[1];
        std::cout << "mode = " << mode << " | g_up = " << _g_up << " | g_low = " << _g_low << " | dis = " << dis
                  << std::endl;
      }
      else
        mode = 2;
    }
    else if (prev_mode == 3)
    {
      psi_ud = alpha_k - direct * M_PI / 6;
      psi_U = psi_ud - psi_u;
      while (psi_U < -4 * M_PI / 3)
        psi_U = psi_U + 2 * M_PI;
      while (psi_U > 4 * M_PI / 3)
        psi_U = psi_U - 2 * M_PI;
      psi_uo = relative_angle(vu, psi_ud, vo, psi_o);
      g_up = _g_up - psi_U - (psi_uo - psi_ud);
      g_low = _g_low - psi_U - (psi_uo - psi_ud);
      float x_tmp = (xu - WP_oax[1]) * cos(alpha_k) + (yu - WP_oay[1]) * sin(alpha_k);
      if ((!in_cone_space(g_up, g_low)) && fabs(x_tmp) > 2)
      {
        mode = 4;
        WP_oax[2] = WP_oax[1] + (x_tmp + 1) * cos(alpha_k); //
        WP_oay[2] = WP_oay[1] + (x_tmp + 1) * sin(alpha_k); //
        WP_oax[3] = WP_oax[2] + 50 * cos(psi_ud);           //
        WP_oay[3] = WP_oay[2] + 50 * sin(psi_ud);           //
        std::cout << "mode = " << mode << " | g_up = " << _g_up << " | g_low = " << _g_low << " | dis = " << dis
                  << std::endl;
      }
      else
        mode = 3;
    }
    else if (prev_mode == 4)
    {
      ye = -(xu - x_wp[k]) * sin(alpha_k) + (yu - y_wp[k]) * cos(alpha_k);
      if (fabs(ye) < 1)
      {
        mode = 1;
        F_OA = 0;
      }
      else
        mode = 4;
    }
  }
  // Adjust psi_d response to psi_u
  while (psi_d - psi_u < -4 * M_PI / 3)
    psi_d = psi_d + 2 * M_PI;

  while (psi_d - psi_u > 4 * M_PI / 3)
    psi_d = psi_d - 2 * M_PI;
  prev_mode = mode;
  // std::cout<<"mode="<<mode<<std::endl;
  static float return_SBG[20] = {0};
  return_SBG[0] = psi_d;
  return_SBG[1] = ye_los;
  return_SBG[2] = dis;
  return_SBG[3] = _g_up;
  return_SBG[4] = _g_low;
  return_SBG[5] = x_wp[0];
  return_SBG[6] = x_wp[1];
  return_SBG[7] = y_wp[0];
  return_SBG[8] = y_wp[1];
  return_SBG[9] = WP_oax[0];
  return_SBG[10] = WP_oax[1];
  return_SBG[11] = WP_oax[2];
  return_SBG[12] = WP_oax[3];
  return_SBG[13] = WP_oay[0];
  return_SBG[14] = WP_oay[1];
  return_SBG[15] = WP_oay[2];
  return_SBG[16] = WP_oay[3];
  return_SBG[17] = xu;
  return_SBG[18] = yu;
  return_SBG[19] = psi_u;
  desiredHeading = psi_d;
  return return_SBG;
}

float* SbgLOS::SBG(float xu, float yu, float vu, float psi_u, float vo, float psi_o, const float* ctr, int N, float angle_min, float angle_res)
{
  auto x_wp = WPx.data();
  auto y_wp = WPy.data();
  auto n_wp = WPx.size();

  /* Input */
  /* psi_los: goc guidance tu bo LOS
  alpha_k: goc cua duong thang ma USV dang bam theo
  F_C: Flag bao va cham
  */
  /*Output*/
  /*
  psi_d: goc dat cho bo dieu khien
  FF: co bao ket thuc giai thuat tranh va cham
  */
  static double WP_oax[4] = {0};
  static double WP_oay[4] = {0};
  static float _g_up = 0, _g_low = 0, dis = 0, alpha_k, a, psi_ud;
  static int prev_mode = 1;
  static int k_oa = 0, k = 0;
  static int direct = 1;
  static int F_OA = 0, Fdo = 0; // Flag bao dang trong che do tranh vat can
  int mode = 1;
  float ye, psi_U, g_up, g_low, ye_los = 0, psi_uod;
  // float min_dist[100],ag_up[100],ag_low[100];
  float _dz_b = dz_b;
  float _dz_rh = dz_rh;
  // prev_mode = 1;
  // int noz = set_deadzone(dz_b,dz_rl,dz_rh,Dsafe,ctr,N,min_dist,ag_up,ag_low);
  // bool F_C = col_assess(DLA,min_dist,ag_up,ag_low,noz,_g_up,_g_low,dis);

  // Dirty trick
  if (prev_mode == 1)
  {
    _dz_b = 4 * M_PI / 3;
  }
  else
  {
    _dz_rh = 6.5;
  }

  float psi_uo = relative_angle(vu, alpha_k, vo, psi_o);
  set_deadzone_2(_dz_b, dz_rl, _dz_rh, Dsafe, ctr, N, angle_min, angle_res, dis, _g_up, _g_low);
  bool F_C = col_assess_2(DLA, dis, _g_up, _g_low, psi_uo);
  // std::cout << _g_up << " " << _g_low << std::endl;
  float psi_d = 0;
  if (prev_mode != 1)
    psi_d = LOSGuidance_sbg(xu, yu, WP_oax, WP_oay, k_oa, a, 0, ye_los, 4); // Obstacle avoidance path
  else
    psi_d = LOSGuidance_sbg(xu, yu, x_wp, y_wp, k, alpha_k, 0, ye_los, n_wp);
  bool FF = 1;
  if (F_OA == 0 && F_C == true && Fdo == 0)
  {
    F_OA = 1;
    Fdo = 1;
  }

  if (F_OA)
  {
    FF = 0;
    if (prev_mode == 1)
    {
      direct = in_Colreg(psi_u, psi_o);
      if (direct == 1)
      {
        psi_uod = psi_u + _g_up;
      }
      else
      {
        psi_uod = psi_u + _g_low;
      }
      Tsolve(psi_ud, vu, vo, psi_o, psi_uod);
      k_oa = 0;
      mode = 2;
      WP_oax[0] = xu;
      WP_oay[0] = yu;
      WP_oax[1] = xu + DLA * cos(psi_ud);
      WP_oay[1] = yu + DLA * sin(psi_ud);
      WP_oax[2] = x_wp[1];
      WP_oay[2] = y_wp[1];
      std::cout << "mode = " << mode << " | g_up = " << _g_up << " | g_low = " << _g_low << " | dis = " << dis
                << " psi_ud = " << psi_ud << " psi_uo = " << psi_uo << "direct = " << direct << std::endl;
    }
    else if (prev_mode == 2)
    {
      mode = 2;
    }
  }
  // Adjust psi_d response to psi_u
  while (psi_d - psi_u < -4 * M_PI / 3)
    psi_d = psi_d + 2 * M_PI;

  while (psi_d - psi_u > 4 * M_PI / 3)
    psi_d = psi_d - 2 * M_PI;
  prev_mode = mode;
  // std::cout<<"mode="<<mode<<std::endl;
  static float return_SBG[20] = {0};
  return_SBG[0] = psi_d;
  return_SBG[1] = ye_los;
  return_SBG[2] = dis;
  return_SBG[3] = _g_up;
  return_SBG[4] = _g_low;
  return_SBG[5] = x_wp[0];
  return_SBG[6] = x_wp[1];
  return_SBG[7] = y_wp[0];
  return_SBG[8] = y_wp[1];
  return_SBG[9] = WP_oax[0];
  return_SBG[10] = WP_oax[1];
  return_SBG[11] = WP_oax[2];
  return_SBG[12] = WP_oax[3];
  return_SBG[13] = WP_oay[0];
  return_SBG[14] = WP_oay[1];
  return_SBG[15] = WP_oay[2];
  return_SBG[16] = WP_oay[3];
  return_SBG[17] = xu;
  return_SBG[18] = yu;
  return_SBG[19] = psi_u;
  desiredHeading = psi_d;
  return return_SBG;
}
