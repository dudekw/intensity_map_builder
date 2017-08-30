#include <intensity_map_builder/intensity_map_builder.h>
#include <intensity_map_msgs/IntensityMarker.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
tf::StampedTransform tf_transform;
std::auto_ptr<IntensityMapBuilder> intensity_map_builder;
std::auto_ptr<tf::TransformListener> listener_ptr;

void updateMapMarkers(const intensity_map_msgs::IntensityMarker& markers_msg)
{

  listener_ptr->waitForTransform("map", "base_link",
                                 markers_msg.header.stamp, ros::Duration(3.0));
  listener_ptr->lookupTransform("map", "base_link",
                           markers_msg.header.stamp, tf_transform);

  intensity_map_builder->updateMapMarkers(markers_msg, tf_transform);
}
  
int main(int argc, char** argv)
{
  std::vector<float> marker_position_tresh_;

  ros::init(argc, argv, "intensity_map_builder");

  ros::NodeHandle nh("my_namespace");
  listener_ptr.reset(new tf::TransformListener);
  ros::Rate loop_rate(10);
  marker_position_tresh_.resize(2);

    if(!nh.getParam("/VELMWHEEL_OROCOS_ROBOT/velmobil_global_localization/marker_position_tresh", marker_position_tresh_))
    {
      marker_position_tresh_ = {0.4, 0.1};
    }
  intensity_map_builder.reset(new IntensityMapBuilder);

  ros::Subscriber intensity_markers_subscriber = nh.subscribe("/intensity_map_markers", 1000, updateMapMarkers);
  ros::Publisher publish_markers = nh.advertise<visualization_msgs::MarkerArray>("/intensity_map_markers/visualization", 1000);
  while (ros::ok())
  {

   publish_markers.publish(intensity_map_builder->getVisMarkerArray());
    ros::spinOnce(); 
    loop_rate.sleep();
  }
  return 0;
}