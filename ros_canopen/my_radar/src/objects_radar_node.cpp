#include "objects_radar_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_radar_node");
  ros::NodeHandle nh;
  ros::start();
  my_frame_pub = nh.advertise<my_radar::objects>("objects/msg", 1);
  is_first_frame = false;
  message_filters::Subscriber<can_msgs::Frame> frame_sub(nh, "/received_messages", 100);
  frame_sub.registerCallback(frame_callback);
  ros::spin();
  ros::shutdown();
}

void frame_callback(const can_msgs::FrameConstPtr &frame)
{
  int frame_id = frame->id;
  int obj_num;
  my_radar::object my_object;
  GeneralInfo obj_info;
  can_msgs::Frame Frame;
  Frame.data = frame->data;
  Frame.dlc = frame->dlc;
  Frame.header = frame->header;
  Frame.id = frame->id;
  Frame.is_error = frame->is_error;
  Frame.is_extended = frame->is_extended;
  Frame.is_rtr = frame->is_rtr;

  if(frame_id == 0x60A && is_first_frame)
  {
    is_first_frame = false;
    my_frame_pub.publish(my_objects);
  }

  if(frame_id == 0x60A)
  {
    std::cout<<"60A"<<std::endl;
    is_first_frame = true;
    Obj_list_status.unPackBytes(Frame);
    obj_num = Obj_list_status.getNumOfObj();
    my_objects.header.stamp = frame->header.stamp;
    my_objects.num = obj_num;
    my_objects.objs.resize(0);
  }
  if(frame_id == 0x60B && is_first_frame)
  {
    std::cout<<"60B"<<std::endl;
    Obj_gene_info.unPackBytes(Frame);
    obj_info = Obj_gene_info.getGeneralInfo();

    my_object.id = obj_info.object_id_;
    my_object.Long = obj_info.object_dist_long_;
    my_object.Lat = obj_info.object_dist_lat_;
    my_object.velo_Long = obj_info.object_vre_long_;
    my_object.velo_Lat = obj_info.object_vre_lat_;
    my_object.dyn_prop = obj_info.object_dyn_prop_;
    my_object.rcs= obj_info.object_rcs_;

    my_objects.objs.push_back(my_object);
  }
  if(frame_id == 0x60C && is_first_frame)
  {
    std::cout<<"60C"<<std::endl;
  }
  if(frame_id == 0x60D && is_first_frame)
  {
    std::cout<<"60D"<<std::endl;
  }
  if(frame_id == 0x201)
  {
    Radar_state.unPackBytes(Frame);
    std::cout<<Radar_state.getMaxDistanceCfg()<<std::endl;
  }
}
