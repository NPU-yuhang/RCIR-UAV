#include "clusters_radar_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clusters_radar_node");
  ros::NodeHandle nh;
  ros::start();
  my_frame_pub = nh.advertise<my_radar::objects>("clusters/msg", 1);
  is_first_frame = false;
  message_filters::Subscriber<can_msgs::Frame> frame_sub(nh, "/received_messages", 100);
  frame_sub.registerCallback(frame_callback);
  ros::spin();
  ros::shutdown();
}

void frame_callback(const can_msgs::FrameConstPtr &frame)
{
  int frame_id = frame->id;
  int clus_num;
  my_radar::object my_cluster;
  GeneralInfo_Clus clus_info;
  can_msgs::Frame Frame;
  Frame.data = frame->data;
  Frame.dlc = frame->dlc;
  Frame.header = frame->header;
  Frame.id = frame->id;
  Frame.is_error = frame->is_error;
  Frame.is_extended = frame->is_extended;
  Frame.is_rtr = frame->is_rtr;

  if(frame_id == 0x600 && is_first_frame)
  {
    is_first_frame = false;
    my_frame_pub.publish(my_clusters);
  }

  if(frame_id == 0x600)
  {
    std::cout<<"600"<<std::endl;
    is_first_frame = true;
    Clus_list_status.unPackBytes(Frame);
    clus_num = Clus_list_status.getNumOfClus();
    my_clusters.header.stamp = frame->header.stamp;
    my_clusters.num = clus_num;
    my_clusters.objs.resize(0);
  }
  if(frame_id == 0x701 && is_first_frame)
  {
    std::cout<<"701"<<std::endl;
    Clus_gene_info.unPackBytes(Frame);
    clus_info = Clus_gene_info.getGeneralInfo_Clus();

    my_cluster.id = clus_info.cluster_id_;
    my_cluster.Long = clus_info.cluster_dist_long_;
    my_cluster.Lat = clus_info.cluster_dist_lat_;
    my_cluster.velo_Long = clus_info.cluster_vre_long_;
    my_cluster.velo_Lat = clus_info.cluster_vre_lat_;
    my_cluster.dyn_prop = clus_info.cluster_dyn_prop_;
    my_cluster.rcs= clus_info.cluster_rcs_;

    my_clusters.objs.push_back(my_cluster);
  }
  if(frame_id == 0x702 && is_first_frame)
  {
    std::cout<<"702"<<std::endl;
  }
}
