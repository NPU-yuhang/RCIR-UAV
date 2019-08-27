#include "mymission/track_planning.h"


void GetWay::setPointAim(FlyData *FD,double mLat,double mLong)
{
    FD->Lat  = mLat;
    FD->dLat    = 0;
    FD->ddLat   = 0;
    FD->Long = mLong;
    FD->dLong   = 0;
    FD->ddLong  = 0;
}
void GetWay::setLat(FlyData *FD,double mLat)
{
    FD->Lat=mLat;
    FD->dLat=0;
    FD->ddLat=0;
}
void GetWay::setLong(FlyData *FD,double mLong)
{
    FD->Long = mLong;
    FD->dLong = 0;
    FD->ddLong = 0;
}
void GetWay::trackHeight(FlyData *FD,double Hs,double He,double ts,double te )
{
    double t = nowtime;
    if(t<ts)
    {
        FD->Height = Hs;
    } else if(t<te)
    {
        FD->Height = Hs + (He - Hs)/(te - ts)*(t-ts);
    } else
    {
        FD->Height = He;
    }
}
void GetWay::trackLat(FlyData *FD,double ts,double te,double vs,double vm,double ve,double Ps,double Pe)
{
    double t = nowtime;
    double t1 = abs(vm-vs) + ts;
    double t2 = 3*(Pe-Ps)/(vm-ve)+3.0*(vs+vm)*ts/2.0/(vm-ve)+3*(vm-vs)*t1/2.0/(vm-ve)-(2.0*vm+ve)*te/(vm-ve);
    if(t1<=ts||t2<t1)
    {
        FD->Lat = Pe;
        FD->dLat   = 0;
        FD->ddLat  = 0;
        return;
    }
    if(t<=t1){
        t=t-ts;
        FD->Lat = Ps + (0.5*t*(vm-vs)+t*vs);//x
        FD->dLat   = vs+(vm-vs)*t/(t1-ts);
        FD->ddLat  = (vm-vs)/(t1-ts);
    } else if(t<=t2){
        t=t-t1;
        FD->Lat = Ps + (0.5*(vm+vs)*(t1-ts)) + (vm * t);//x
        FD->dLat   = vm;
        FD->ddLat  = 0;
    }
    else {
        t=t-t2;
        double tpk = -(vm-ve) / pow((t2 - te), 2);
        FD->Lat = Ps + (0.5*(vm+vs)*(t1-ts) + (t2-t1)*vm + tpk*t*t*t/3+vm*t);
        FD->dLat   = tpk * t * t + vm;
        FD->ddLat  = 2.0*tpk*t;
    }
}
void GetWay::trackLong(FlyData *FD,double ts,double te,double vs,double vm,double ve,double Ps,double Pe)
{
    double t = nowtime;
    double t1 = abs(vm-vs) + ts;
    double t2 = 3*(Pe-Ps)/(vm-ve)+3.0*(vs+vm)*ts/2.0/(vm-ve)+3*(vm-vs)*t1/2.0/(vm-ve)-(2.0*vm+ve)*te/(vm-ve);
    if(t1<=ts||t2<t1)
    {
        FD->Long = Pe;
        FD->dLong   = 0;
        FD->ddLong  = 0;
        return;
    }
    if((t<=t1)&&(t>=ts)){
        t=t-ts;
        FD->Long = Ps + (0.5*t*(vm-vs)+t*vs);//x
        FD->dLong   = vs+(vm-vs)*t/(t1-ts);
        FD->ddLong  = (vm-vs)/(t1-ts);
    } else if(t<=t2){
        t=t-t1;
        FD->Long = Ps + (0.5*(vm+vs)*(t1-ts))+ (vm * t);//x
        FD->dLong   = vm;
        FD->ddLong  = 0;
    }
    else {
        t = t - t2;
        double tpk = -(vm-ve) / pow((t2 - te), 2);
        FD->Long = Ps + (0.5*(vm+vs)*(t1-ts) + (t2-t1)*vm + tpk*t*t*t/3+vm*t);
        FD->dLong   = tpk*t*t+vm;
        FD->ddLong  = 2.0*tpk*t;
    }
}
void GetWay::trackcircleLat(FlyData *FD,double ts,double te,double V1,double Vm,double V2,double thetas,double thetae,double R,double Plat)
{

    double t=nowtime;
    double theta0 = thetae-thetas;
    double W1 = V1/R;
    double Wm = Vm/R;
    double W2 = V2/R;
    double t2 = (pow((Wm-W1),2)/(2*(Wm-W2))*te-(W2+Wm)*te/2+ts*Wm+theta0)/((pow((Wm-W1),2)+pow((Wm-W2),2))/(2*(Wm-W2)));
    double t1 = (te-t2)*(Wm-W1)/(Wm-W2)+ts;

    if(t1<=ts||t2<t1)
    {
        FD->Lat = Plat+(R)*cos(theta0);
        FD->dLat   = 0;
        FD->ddLat  = 0;
        return;
    }
    if((t<=t1)&&(t>=ts)){
        t=t-ts;
        FD->Lat = Plat + (R)*cos(thetas+0.5*t*t*(Wm-W1)/(t1-ts)+W1*t);//x
        FD->dLat   = -1*(((Wm-W1)*t/(t1-ts)+W1)*R*sin(thetas+0.5*t*t*(Wm-W1)/(t1-ts)+W1*t));
        FD->ddLat  = -1*R*(((Wm-W1)/(t1-ts)*sin(thetas+0.5*t*t*(Wm-W1)/(t1-ts)+W1*t))+pow((Wm-W1)*t/(t1-ts)+W1,2)*cos(thetas+0.5*t*t*(Wm-W1)/(t1-ts)+W1*t));
    } else if(t<=t2){
        t=t-t1;
        FD->Lat = Plat + (R)*cos(thetas+Wm*t+0.5*(Wm+W1)*(t1-ts));//x
        FD->dLat   = -1*(Wm*R*sin(thetas+Wm*t+0.5*(Wm+W1)*(t1-ts)));
        FD->ddLat  = -1*(Wm*Wm*R*cos(thetas+Wm*t+0.5*(Wm+W1)*(t1-ts)));
    }
    else {
        t = t - t2;
        FD->Lat = Plat + (R)*cos(thetas+Wm*t-0.5*t*t*(Wm-W2)/(te-t2)+Wm*(t2-t1)+0.5*(Wm+W1)*(t1-ts));
        FD->dLat   = -1*((-(Wm-W2)*t/(te-t2)+Wm)*R*sin(thetas+Wm*t-0.5*t*t*(Wm-W2)/(te-t2)+Wm*(t2-t1)+0.5*(Wm+W1)*(t1-ts)));
        FD->ddLat  = -1*(R*((-(Wm-W2)/(te-t2))*sin(thetas+Wm*t-0.5*t*t*(Wm-W2)/(te-t2)+Wm*(t2-t1)+0.5*(Wm+W1)*(t1-ts))+pow((-(Wm-W2)*t/(te-t2)+Wm),2)*cos(thetas+Wm*t-0.5*t*t*(Wm-W2)/(te-t2)+Wm*(t2-t1)+0.5*(Wm+W1)*(t1-ts))));
    }
}
void GetWay::trackcircleLong(FlyData *FD,double ts,double te,double V1,double Vm,double V2,double thetas,double thetae,double R,double Plong)
{

    double t=nowtime;
    double theta0 = thetae-thetas;
    double W1 = V1/R;
    double Wm = Vm/R;
    double W2 = V2/R;
    double t2 = (pow((Wm-W1),2)/(2*(Wm-W2))*te-(W2+Wm)*te/2+ts*Wm+theta0)/((pow((Wm-W1),2)+pow((Wm-W2),2))/(2*(Wm-W2)));
    double t1 = (te-t2)*(Wm-W1)/(Wm-W2)+ts;

    if(t1<=ts||t2<t1)
    {
        FD->Long = Plong+(R)*sin(theta0);
        FD->dLong   = 0;
        FD->ddLong  = 0;
        return;
    }
    if((t<=t1)&&(t>=ts)){
        t=t-ts;
        FD->Long = Plong + (R)*sin(thetas+0.5*t*t*(Wm-W1)/(t1-ts)+W1*t);//x
        FD->dLong   = ((Wm-W1)*t/(t1-ts)+W1)*R*cos(thetas+0.5*t*t*(Wm-W1)/(t1-ts)+W1*t);
        FD->ddLong  = R*(((Wm-W1)/(t1-ts)*cos(thetas+0.5*t*t*(Wm-W1)/(t1-ts)+W1*t)) - pow((Wm-W1)*t/(t1-ts)+W1,2)*sin(thetas+0.5*t*t*(Wm-W1)/(t1-ts)+W1*t));
    } else if(t<=t2){
        t=t-t1;
        FD->Long = Plong + (R)*sin(thetas+Wm*t+0.5*(Wm+W1)*(t1-ts));//x
        FD->dLong   = Wm*R*cos(thetas+Wm*t+0.5*(Wm+W1)*(t1-ts));
        FD->ddLong  = -Wm*Wm*R*sin(thetas+Wm*t+0.5*(Wm+W1)*(t1-ts));
    }
    else {
        t = t - t2;
        FD->Long = Plong + (R)*sin(thetas+Wm*t-0.5*t*t*(Wm-W2)/(te-t2)+Wm*(t2-t1)+0.5*(Wm+W1)*(t1-ts));
        FD->dLong   = (-(Wm-W2)*t/(te-t2)+Wm)*R*cos(thetas+Wm*t-0.5*t*t*(Wm-W2)/(te-t2)+Wm*(t2-t1)+0.5*(Wm+W1)*(t1-ts));
        FD->ddLong  = R*((-(Wm-W2)/(te-t2))*cos(thetas+Wm*t-0.5*t*t*(Wm-W2)/(te-t2)+Wm*(t2-t1)+0.5*(Wm+W1)*(t1-ts)) - pow((-(Wm-W2)*t/(te-t2)+Wm),2)*sin(thetas+Wm*t-0.5*t*t*(Wm-W2)/(te-t2)+Wm*(t2-t1)+0.5*(Wm+W1)*(t1-ts)));
    }

}




















