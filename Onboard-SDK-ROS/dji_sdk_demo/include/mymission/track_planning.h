#ifndef TRACK_PLANNING_H
#define TRACK_PLANNING_H

#include <string.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mymission/flydata.h"

#define PI 3.1415926
class GetWay{
public:
  double nowtime;
  //FlyData FD;
  void setPointAim(FlyData *FD,double mLat,double mLong);
  void setLat(FlyData *FD,double mLat);
  void setLong(FlyData *FD,double mLong);
  void trackHeight(FlyData *FD,double Hs,double He,double ts,double te );
  void trackLat(FlyData *FD,double ts,double te,double vs,double vm,double ve,double Ps,double Pe);
  void trackLong(FlyData *FD,double ts,double te,double vs,double vm,double ve,double Ps,double Pe);
  void trackcircleLat(FlyData *FD,double ts,double te,double V1,double Vm,double V2,double thetas,double thetae,double R,double Plat);
  void trackcircleLong(FlyData *FD,double ts,double te,double V1,double Vm,double V2,double thetas,double thetae,double R,double Plong);



  double changetomLat(double v);
  double changetomLong(double v);
  double changeturelat(double v);
  double changeturelong(double v);
  double specialforLong(double oldLong);
  double specialforLongr(double newLong);


};

#endif
