#include <intensity_map_builder/intensity_map_builder.h>
#include <intensity_map_builder/intensity_map_handler.h>
#include <intensity_map_msgs/IntensityMarker.h>
#include <intensity_map_msgs/SaveMap.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
tf::StampedTransform tf_transform;
std::auto_ptr<IntensityMapBuilder> intensity_map_builder;
std::auto_ptr<IntensityMapHandler> intensity_map_handler;
std::auto_ptr<tf::TransformListener> listener_ptr;
visualization_msgs::MarkerArray vis_marker_array;
void updateMapMarkers(const intensity_map_msgs::IntensityMarker& markers_msg)
{
    std::cout << "in update: \n";

  listener_ptr->waitForTransform("map", "base_link",
                                 markers_msg.header.stamp, ros::Duration(10.0));
  listener_ptr->lookupTransform("map", "base_link",
                           markers_msg.header.stamp, tf_transform);

  intensity_map_builder->updateMapMarkers(markers_msg, tf_transform);
}

bool save_current_map(intensity_map_msgs::SaveMap::Request  &req,
         intensity_map_msgs::SaveMap::Response &res)
{
  vis_marker_array = intensity_map_builder->getVisMarkerArray();
  intensity_map_handler.reset(new IntensityMapHandler(vis_marker_array.markers[1].points.size()));
  intensity_map_handler->save_map(req.file_path, vis_marker_array.markers[1]);
  res.status = true;
  return res.status;
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

  ros::ServiceServer service = nh.advertiseService("intensity_map_builder/save_map", save_current_map);

  ros::Subscriber intensity_markers_subscriber = nh.subscribe("/intensity_map_markers", 1000, updateMapMarkers);
  ros::Publisher publish_markers = nh.advertise<visualization_msgs::MarkerArray>("/intensity_map_markers/visualization", 1000);
  while (ros::ok())
  {
    vis_marker_array = intensity_map_builder->getVisMarkerArray();

    std::cout << "in IF: "<< vis_marker_array.markers[1].points.size()<< "\n";
    if (vis_marker_array.markers[1].points.size() > 0)
    {

          for (int i = 0; i < vis_marker_array.markers[1].points.size(); ++i)
          {
            std::cout << i <<", "; 
            vis_marker_array.markers[1].points[i].z = 0;
          }
          std::cout<<"\n"; 
         publish_markers.publish(vis_marker_array);
    }
    ros::spinOnce(); 
    loop_rate.sleep();
  }
  return 0;
}