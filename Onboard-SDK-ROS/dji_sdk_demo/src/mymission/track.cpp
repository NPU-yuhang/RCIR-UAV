#include "mymission/track.h"

float send_data_time = 0;
int count = 0;
FlyData leader;
GetWay getway;

std_msgs::Float32MultiArray send_data;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "track");
    ros::NodeHandle nh;
    ros::Publisher mission_track = nh.advertise<std_msgs::Float32MultiArray> ("mission_track", 1000);
    ros::Rate loop_rate(20);

    while(ros::ok())
    {
      count ++ ;
      send_data_time = count / 20.00;
      getway.nowtime = send_data_time;
      std::cout<<send_data_time<<std::endl;
      if(send_data_time <= 20)
      {
        getway.setPointAim(&leader, 0, 0);
        getway.trackHeight(&leader, 1, 10, 0, 20);
      }
      else if(send_data_time <= 30){
        getway.trackLat(&leader, 20, 30, 0, 2, 0, 0, 15);
        getway.trackLong(&leader, 20, 30, 0, 2, 0, 0, 15);
        getway.trackHeight(&leader, 10, 10, 20, 30);
      }
      else if(send_data_time <= 35){
        getway.setPointAim(&leader, 15, 15);
        getway.trackHeight(&leader, 10, 10, 30, 35);
      }
      else if(send_data_time <= 55){
        getway.setPointAim(&leader, 15, 15);
        getway.trackHeight(&leader, 10, 1, 35, 55);
      }
      else{
        getway.setPointAim(&leader, 15, 15);
      }

      send_data.data.resize(0);

      send_data.data.push_back(leader.Lat);
      send_data.data.push_back(leader.Long);
      send_data.data.push_back(leader.Height);
      send_data.data.push_back(leader.dLat);
      send_data.data.push_back(leader.dLong);
      send_data.data.push_back(leader.ddLat);
      send_data.data.push_back(leader.ddLong);
      mission_track.publish(send_data);

      ros::spinOnce();
      loop_rate.sleep();
    }

}
