#ifndef BASE_LOS_H
#define BASE_LOS_H

#include <cmath>
#include <vector>

class BaseLOS
{
public:
  BaseLOS();
  virtual ~BaseLOS();

  struct Point
  {
    inline Point() : x(0), y(0), z(0) {}
    inline Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

    double x;
    double y;
    double z;
  };
  static std::vector<Point> waypoints;

  static double radius;
  static double minDelta ;
  static double maxDelta;
  static double beta;

  double crossTrackError;
  double alongTrackError;
  double desiredHeading;
  double desiredPitch;

  virtual void setupLOS();
  virtual void resetLOS();
  virtual bool runLOS(const double& currX, const double& currY, const double& currZ);
};

#endif // BASE_LOS_H
