#include"mymission/flydata.h"

FlyData::FlyData(double La,double Lo,double dLa,double dLo,double ddLa,double ddLo,double H)
{
    Lat    = La;
    Long   = Lo;
    dLat   = dLa;
    dLong  = dLo;
    ddLat  = ddLa;
    ddLong = ddLo;
    Height = H;
}
FlyData::FlyData()
{

}

/*
inline FlyData FlyData::operator+(const FlyData &c)
{
    FlyData tp;

    tp.Lat    = this->Lat    + c.Lat;
    tp.Long   = this->Long   + c.Long,
    tp.dLat   = this->dLat   + c.dLat,
    tp.dLong  = this->dLong  + c.dLong,
    tp.ddLat  = this->ddLat  + c.ddLat,
    tp.ddLong = this->ddLong + c.ddLong,
    tp.Height = this->Height + c.Height;
    return tp;
}
*/

