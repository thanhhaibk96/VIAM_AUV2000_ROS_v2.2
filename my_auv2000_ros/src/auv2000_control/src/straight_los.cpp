#include "straight_los.h"
#include <algorithm>
#include <cmath>
#include <vector>
void StraightLOS::setupLOS()
{
  numPoints = waypoints.size();
  if (numPoints > 1)
  {
    alpha_P.resize(numPoints - 1);
    s.resize(numPoints - 1);
    alt.resize(numPoints - 1);
    for (unsigned i = 0; i < numPoints - 1; i++)
    {
      double diff_x = waypoints[i + 1].x - waypoints[i].x;
      double diff_y = waypoints[i + 1].y - waypoints[i].y;
      alpha_P[i] = atan2(diff_y, diff_x);
      s[i] = diff_x * cos(alpha_P[i]) + diff_y * sin(alpha_P[i]);
      alt[i] = waypoints[i+1].z;
    }
  }
}

void StraightLOS::resetLOS()
{
  waypoints.clear();
  pointId = 0;
}
float StraightLOS::Calc_psi_oa(const double& currHeading, const float* ctr, int N, float angle_min, float angle_res)
{
float left_sum = 0, center_sum = 0, right_sum = 0 ;
int cnt_left = 0 , cnt_center = 0 , cnt_right = 0 ;
for(int i = 80 ; i < 130; i++)
{
  float temp = ctr[i];
  if (temp > 100)
  temp = 100;

  cnt_left += 1;
  left_sum += temp;
  
}

for(int i = 50; i < 80; i++)
{
  float temp = ctr[i];
  if (temp > 100)
  temp = 100;
    cnt_center += 1;
  center_sum += temp;
  
}

for(int i = 0 ; i < 80 ; i++)
{
  float temp = ctr[i];
  if (temp > 100)
  temp = 100;

    cnt_right += 1;
    right_sum += temp;
  
}

left_bar = left_sum / cnt_left ;
center_bar = center_sum / cnt_center ;
right_bar = right_sum / cnt_right ;

Left_Sector_TD Left_Sector;
Center_Sector_TD Center_Sector;
Right_Sector_TD Right_Sector;


Left_Sector.Near = mftrap(left_bar ,-1, 0, 10, 45);
Left_Sector.Medium = mftrap(left_bar ,10, 45 , 45 ,80);
Left_Sector.Far = mftrap(left_bar , 45, 80, 100, 120) ; 

Right_Sector.Near = mftrap(right_bar ,-1, 0, 10, 45);
Right_Sector.Medium = mftrap(right_bar ,10, 45 , 45 ,50);
Right_Sector.Far = mftrap(right_bar , 45, 80, 100, 120) ; 

Center_Sector.Near = mftrap(center_bar ,-1, 0, 10, 45);
Center_Sector.Medium = mftrap(center_bar ,10, 45 , 45 ,50);
Center_Sector.Far = mftrap(center_bar ,  45, 80, 100, 120) ; 



float f1 = std::min({Left_Sector.Near , Center_Sector.Near , Right_Sector.Near});
float f2 = std::min({Left_Sector.Near , Center_Sector.Near , Right_Sector.Medium});
float f3 = std::min({Left_Sector.Near , Center_Sector.Near , Right_Sector.Far});
float f4 = std::min({Left_Sector.Near , Center_Sector.Medium , Right_Sector.Near});
float f5 = std::min({Left_Sector.Near , Center_Sector.Medium , Right_Sector.Medium});
float f6 = std::min({Left_Sector.Near , Center_Sector.Medium , Right_Sector.Far});
float f7 = std::min({Left_Sector.Near , Center_Sector.Far , Right_Sector.Near});
float f8 = std::min({Left_Sector.Near , Center_Sector.Far , Right_Sector.Medium});
float f9 = std::min({Left_Sector.Near , Center_Sector.Far , Right_Sector.Far});
 
float f10 = std::min({Left_Sector.Medium , Center_Sector.Near , Right_Sector.Near});
float f11 = std::min({Left_Sector.Medium , Center_Sector.Near , Right_Sector.Medium});
float f12 = std::min({Left_Sector.Medium , Center_Sector.Near , Right_Sector.Far});
float f13 = std::min({Left_Sector.Medium , Center_Sector.Medium , Right_Sector.Near});
float f14 = std::min({Left_Sector.Medium , Center_Sector.Medium , Right_Sector.Medium});
float f15 = std::min({Left_Sector.Medium , Center_Sector.Medium , Right_Sector.Far});
float f16 = std::min({Left_Sector.Medium , Center_Sector.Far , Right_Sector.Near});
float f17 = std::min({Left_Sector.Medium , Center_Sector.Far , Right_Sector.Medium});
float f18 = std::min({Left_Sector.Medium , Center_Sector.Far , Right_Sector.Far});

float f19 = std::min({Left_Sector.Far , Center_Sector.Near , Right_Sector.Near});
float f20 = std::min({Left_Sector.Far , Center_Sector.Near , Right_Sector.Medium});
float f21 = std::min({Left_Sector.Far , Center_Sector.Near , Right_Sector.Far});
float f22 = std::min({Left_Sector.Far , Center_Sector.Medium , Right_Sector.Near});
float f23 = std::min({Left_Sector.Far , Center_Sector.Medium , Right_Sector.Medium});
float f24 = std::min({Left_Sector.Far , Center_Sector.Medium , Right_Sector.Far});
float f25 = std::min({Left_Sector.Far , Center_Sector.Far , Right_Sector.Near});
float f26 = std::min({Left_Sector.Far , Center_Sector.Far , Right_Sector.Medium});
float f27 = std::min({Left_Sector.Far , Center_Sector.Far , Right_Sector.Far});

float PB = std::max({f1, f2, f3, f11, f12});
float PS = std::max({f4, f5, f6, f14, f15});
float ZE = std::max({f7, f8, f9, f16, f17, f18 , f25, f26, f27});
float NS = std::max({f13, f23, f24, f25});
float NB = std::max({f10, f19, f20, f21});

return (PB * 3.14 + PS * 1.57 + ZE * 0 - NS * 1.57  - NB * 3.14)/(1); 

}

float StraightLOS::mftrap(float x,float L,float C1,float C2,float R)
{
  float y; 
    if (x <= L)
    {
        y = 0;
    }
    else if (x <= C1){
        y = (x- L)/(C1-L);}
    else if (x <= C2){
        y = 1;}
    else if (x <= R){
        y = (R-x)/(R-C2);}
    else{
        y = 0;}
}
bool StraightLOS::runLOS(const double& currX, const double& currY, const double& currZ)
{
  double c_alpha = cos(alpha_P[pointId]);
  double s_alpha = sin(alpha_P[pointId]);
  double diff_x = currX - waypoints[pointId].x;
  double diff_y = currY - waypoints[pointId].y;

  // Calculate the along-track and cross-track errors
  alongTrackError = diff_x * c_alpha + diff_y * s_alpha;
  crossTrackError = -diff_x * s_alpha + diff_y * c_alpha;

  // Find desired heading
  double delta = (maxDelta - minDelta) * exp(-0.3* (crossTrackError * crossTrackError)) + minDelta;
  desiredHeading = alpha_P[pointId] + atan2(-crossTrackError, delta);
  desiredHeading = atan2(sin(desiredHeading), cos(desiredHeading));

  desiredPitch = atan2(currZ - alt[pointId],10);
  if (desiredPitch > 0.69)
     desiredPitch = 0.69 ;
  else if (desiredPitch < - 0.69)
     desiredPitch  = - 0.69 ;
  
  // Head toward the next waypoint. For the nearly final point, discard point-switching scheme
  if (pointId < numPoints - 2)
  {
    if (fabs(s[pointId] - alongTrackError) < radius)
      pointId++;
  }
  else {
    if (fabs(s[pointId] - alongTrackError) < 1.0)
      pointId++;
  }

  // Terminate LOS if the final point has been reached
  return pointId < numPoints - 1;
}
