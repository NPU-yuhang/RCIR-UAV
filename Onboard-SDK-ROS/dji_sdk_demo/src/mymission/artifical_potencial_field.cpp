#include "mymission/artifical_potencial_field.h"

artifical_potencial_field::artifical_potencial_field() : count(0), P0(2.5), att_gain_coe(15), rep_gain_coe(5),
  a(0.5), l(0.1), iteration(200)
{
  X0.x = 0;
  X0.y = 0;
  current_x = X0;
}

std::vector<float> artifical_potencial_field::compute_angle(cv::Point2f X, std::vector<cv::Point2f> Xs, int n_obs)
{
    std::vector<float> Y;
    std::vector<float> deltaX, deltaY, r;
    for(int i = 0; i < n_obs+1; i++)
    {
      deltaX.push_back(Xs[i].x - X.x);
      deltaY.push_back(Xs[i].y - X.y);
      r.push_back(sqrt(deltaX[i]*deltaX[i] + deltaY[i]*deltaY[i]));

      float theta;
      if(deltaY[i] >= 0)
          theta = std::acos(deltaX[i]/r[i]);
      else
          theta = -std::acos(deltaX[i]/r[i]);
      float angle = theta;
      Y.push_back(angle);
    }

    return Y;
}

cv::Point2f artifical_potencial_field::compute_Attact(cv::Point2f X, std::vector<cv::Point2f> Xs, int att_gain,
                                                      float angle, int b, float P0, int n_obs)
{
    float R = (X.x - Xs[0].x)*(X.x - Xs[0].x) + (X.y - Xs[0].y)*(X.y - Xs[0].y);
    float r = sqrt(R);

    cv::Point2f Yat;
    if(r <= 8)
    {
      Yat.x = att_gain*r*std::cos(angle);
      Yat.y = att_gain*r*std::sin(angle);
    }
    else
    {
      Yat.x = 8*att_gain*std::cos(angle);
      Yat.y = 8*att_gain*std::sin(angle);
    }

    return Yat;
}

std::vector<cv::Point2f> artifical_potencial_field::compute_repulsion(cv::Point2f X,
                                                                      std::vector<cv::Point2f> Xs,
                                                                      int rep_gain, float angle_at,
                                                                      std::vector<float> angle_re,
                                                                      int n_obs, float P0, float a)
{
  float Rat = (X.x - Xs[0].x)*(X.x - Xs[0].x) + (X.y - Xs[0].y)*(X.y - Xs[0].y);
  float rat = sqrt(Rat);

  std::vector<float> Rrei, rre;
  std::vector<float> Yrerx, Yrery, Yatax, Yatay;
  float Yrer, Yata;
  std::vector<cv::Point2f> Rep;
  cv::Point2f rep1, rep2;
  float rep1x, rep1y, rep2x, rep2y;
  for(int i = 0; i < n_obs; i++)
  {
    Rrei.push_back((X.x - Xs[i+1].x)*(X.x - Xs[i+1].x) + (X.y - Xs[i+1].y)*(X.y - Xs[i+1].y));
    rre.push_back(std::abs(sqrt(Rrei[i]) - 0.3));
    float R0 = (Xs[0].x - Xs[i+1].x)*(Xs[0].x - Xs[i+1].x) + (Xs[0].y - Xs[i+1].y)*(Xs[0].y - Xs[i+1].y);
    float r0 = sqrt(R0);

    if(rre[i] > (P0 - 0.3))
    {
      Yrerx.push_back(0);
      Yrery.push_back(0);
      Yatax.push_back(0);
      Yatay.push_back(0);
    }
    else
    {
      if(rre[i] < (P0/2 - 0.3))
      {
        Yrer = rep_gain*(1/rre[i] - 1/P0)*(1/(rre[i]*rre[i]))*pow(rat, a);
        Yata = a*rep_gain*(1/rre[i] - 1/P0)*(1/rre[i] - 1/P0)*pow(rat, a-1) / 2;
        Yrerx.push_back(Yrer*std::cos(angle_re[i] + PI));
        Yrery.push_back(Yrer*std::sin(angle_re[i] + PI));
        Yatax.push_back(Yata*std::cos(angle_at));
        Yatay.push_back(Yata*std::sin(angle_at));
      }

      else {
        Yrer = rep_gain*(1/rre[i] - 1/P0)*(1/(rre[i]*rre[i]))*Rat;
        Yata = rep_gain*(1/rre[i] - 1/P0)*(1/rre[i] - 1/P0)*rat;
        Yrerx.push_back(Yrer*std::cos(angle_re[i] + PI));
        Yrery.push_back(Yrer*std::sin(angle_re[i] + PI));
        Yatax.push_back(Yata*std::cos(angle_at));
        Yatay.push_back(Yata*std::sin(angle_at));
      }
    }
  }
  for(int i = 0; i < n_obs; i++)
  {
    rep1x = rep1x + Yrerx[i];
    rep1.x = rep1x;
    rep1y = rep1y + Yrery[i];
    rep1.y = rep1y;
    rep2x = rep2x + Yatax[i];
    rep2.x = rep2x;
    rep2y = rep2y + Yatay[i];
    rep2.y = rep2y;
  }

  Rep.push_back(rep1);
  Rep.push_back(rep2);

  return Rep;
}

void artifical_potencial_field::path_planning()
{
  float position_angle;
  int j;
  for(j = 0; j<iteration; j++)
  {
    cv::Point2f goi;
    goi.x = current_x.x;
    goi.y = current_x.y;
    Goal.push_back(goi);
    std::cout<<"goalx: "<<goi.x<<"---goaly: "<<goi.y<<std::endl;
    std::vector<float> theta = compute_angle(current_x, Xsum, obstacle);
    //std::cout<<"theta: "<<theta[1]<<std::endl;
    float angle = theta[0];
    float angle_at = theta[0];
    cv::Point2f Fat = compute_Attact(current_x, Xsum, att_gain_coe, angle, 0, P0, obstacle);
    //std::cout<<"Fat: "<<Fat<<std::endl;
    std::vector<float> angle_re;
    for(int i = 0; i < obstacle; i++)
    {
      angle_re.push_back(theta[i+1]);
    }

    std::vector<cv::Point2f> Frep = compute_repulsion(current_x, Xsum, rep_gain_coe, angle_at,
                                                      angle_re, obstacle, P0, a);
    float Fsumyj = Fat.y + Frep[0].y + Frep[1].y;
    float Fsumxj = Fat.x + Frep[0].x + Frep[1].x;
    //std::cout<<"Fsumyj: "<<Fsumyj<<"--Fsumxj: "<<Fsumxj<<std::endl;
    if(Fsumxj >= 0)
      position_angle = std::atan(Fsumyj/Fsumxj);
    else
      position_angle = PI + std::atan(Fsumyj/Fsumxj);
    //std::cout<<"Fsumyj: "<<Fsumyj<<"---Fsumxj: "<<Fsumxj<<std::endl;
    next_x.x = current_x.x + l*std::cos(position_angle);
    next_x.y = current_x.y + l*std::sin(position_angle);
    //std::cout<<next_x<<std::endl;
    current_x = next_x;

    if(((current_x.x - Xsum[0].x) > 0)&&((current_x.y - Xsum[0].y) > 0))
    {
      count = j;
      break;
    }
  }
  std::cout<<j<<std::endl;
  count = j;
  Goal.push_back(Xsum[0]);
  //std::cout<<Goal.back().x<<std::endl;
}

void artifical_potencial_field::SetObstacle(std::vector<cv::Point2f> obs)
{
  Xsum = obs;
  obstacle = obs.size() - 1;
}

void artifical_potencial_field::SetStartPoint(cv::Point2f X_s)
{
  X0 = X_s;
}

float artifical_potencial_field::get_path_length()
{
  return count*l;
}
