#include <iostream>

class Control_law
{
public:
  double Kp;
  double Ki;
  double Kd;
  double Kv;
  double Ka;

  Control_law()
  {

  }

  void set_control_law(double kp, double ki, double kd, double kv, double ka)
  {
    Kp = kp;
    Ki = ki;
    Kd = Kd;
    Kv = kv;
    Ka = Ka;
  }
};

class Control_data
{
public:
  double mission_x;
  double mission_y;
  double mission_h;
  double mission_dx;
  double mission_dy;
  double mission_ddx;
  double mission_ddy;

  Control_data()
  {

  }

};
