#ifndef STRAIGHT_LOS_H
#define STRAIGHT_LOS_H

#include <base_los.h>

class StraightLOS : public BaseLOS
{
public:
  std::vector<double> alpha_P;
  std::vector<double> s;
  std::vector<double> alt;


  float w1, w2 , bearing;
  float psi_oa_look;

  float left_bar , center_bar , right_bar;
  unsigned pointId;
  unsigned long numPoints;

  struct Left_Sector_TD
  {
    float Near;
    float Medium;
    float Far;
  };

 struct Center_Sector_TD
  {
    float Near;
    float Medium;
    float Far;
  };
   struct Right_Sector_TD
  {
    float Near;
    float Medium;
    float Far;
  };
  void setupLOS();
  void resetLOS();
  bool runLOS(const double& currX, const double& currY, const double& currZ);
  float mftrap(float x,float L,float C1,float C2,float R);
  float Calc_psi_oa(const double& currHeading, const float* ctr, int N, float angle_min, float angle_res);
};

#endif // STRAIGHT_LOS_H


