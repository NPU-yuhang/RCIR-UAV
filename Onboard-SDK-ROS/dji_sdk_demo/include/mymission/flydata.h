#ifndef FLYDATA_H
#define FLYDATA_H
#include <string.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>

class FlyData{
public:
  FlyData();
  double Lat,Long;
  double dLat,dLong;
  double ddLat,ddLong;
  double Height;
  FlyData(double La,double Lo,double dLa,double dLo,double ddLa,double ddLo,double H);

  FlyData operator +(const FlyData &c)
  {

      return FlyData(this->Lat    + c.Lat,
                     this->Long   + c.Long,
                     this->dLat   + c.dLat,
                     this->dLong  + c.dLong,
                     this->ddLat  + c.ddLat,
                     this->ddLong + c.ddLong,
                     this->Height + c.Height);
  }



};

#endif
