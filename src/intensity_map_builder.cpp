
#include <string>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
//XML
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <intensity_map_builder/intensity_map_builder.h>
#include <intensity_map_msgs/IntensityMarker.h>

#define MAX_SCORE 100000
#define SCORE_THRESH 1000
std::ofstream my_file;
Eigen::Matrix<float, 3,3> odom_transform;
 std::vector<Eigen::Vector3f> markers;
  std::vector<Eigen::Vector3f> map_markers;
  std::vector<Eigen::Vector2f> markers_in_map;
  size_t global_iterator;
  size_t global_iterator_2;
  size_t marker_counter;
  size_t map_marker_counter;
      std::vector<float> marker_position_tresh_;
visualization_msgs::Marker vis_current_markers;
visualization_msgs::Marker vis_map_markers;
std::vector<geometry_msgs::Point> map_positions;
std_msgs::ColorRGBA vis_color;
geometry_msgs::Point map_point;

visualization_msgs::MarkerArray vis_markers_array;

IntensityMapBuilder::IntensityMapBuilder(std::vector<float> tresh)
{
  my_file.open("/tmp/intensity_map_builder_log.txt");
  map_marker_counter = 0;
    my_file<<"in builder\n";

  markers_in_map.resize(30);
  map_markers.resize(30);
  vis_markers_array.markers.resize(2);
  marker_position_tresh_.resize(2);
  marker_position_tresh_ = tresh;
}

IntensityMapBuilder::IntensityMapBuilder()
{
  IntensityMapBuilder({0.1,0.1});
}

bool IntensityMapBuilder::visualizationInitialization(visualization_msgs::Marker &vis_markers, const size_t &marker_id , const std::vector<geometry_msgs::Point> &positions, 
                                                        const std_msgs::ColorRGBA &color, const std::string &frame , const std::string &ns)
{
   std::cout<<"in visualizationInitialization\n";
   std::cout<<"marker_id: "<<marker_id<<"\n";
   std::cout<<"frame: "<<frame<<"\n";
   std::cout<<"positions.size = "<<positions.size()<<"\n";

  vis_markers.header.frame_id = frame;

  vis_markers.ns = ns;
  vis_markers.id = 1;
  // type = POINTS
  vis_markers.type = 8;
  // action add object
  vis_markers.action = 0;
  vis_markers.lifetime = ros::Duration(0);
  vis_markers.scale.x = 0.1;
  vis_markers.scale.y = 0.1;
  vis_markers.frame_locked = true;
  vis_markers.points.clear();
  vis_markers.colors.clear();

  for (int i = 0; i < marker_id; i++)
  {
    vis_markers.points.push_back(positions.at(i));
    vis_markers.colors.push_back(color);
  }  
  std::cout <<"EXIT initialization \n"; 

}
 visualization_msgs::MarkerArray IntensityMapBuilder::getVisMarkerArray()
{
     my_file<<"in getVisMarkerArray\n";

  return vis_markers_array;
}

void IntensityMapBuilder::updateMapMarkers(const intensity_map_msgs::IntensityMarker& markers_msg, tf::StampedTransform &tf_transform)
{
   my_file<<"in updateMapMarkers\n";


  marker_counter = markers_msg.marker_count;
  vis_color.r = 0;
  vis_color.g = 1;
  vis_color.b = 0;
  vis_color.a = 1;

  visualizationInitialization(vis_current_markers, marker_counter, markers_msg.points, vis_color, "base_link", "current_markers");

  odom_transform(0,2) = tf_transform.getOrigin().x();
  odom_transform(1,2) = tf_transform.getOrigin().y();

  odom_transform(0,0) = tf_transform.getBasis()[0][0];
  odom_transform(0,1) = tf_transform.getBasis()[0][1];
  odom_transform(1,0) = tf_transform.getBasis()[1][0];
  odom_transform(1,1) = tf_transform.getBasis()[1][1];

  std::cout <<"Using localization transform:"<<"\n"; 
  std::cout  <<odom_transform<<"\n";

  for (global_iterator = 0; global_iterator < marker_counter; ++global_iterator)
  {
      std::cout  <<global_iterator<<"\n";

    markers_in_map.at(global_iterator)(0) = (odom_transform(0,0) * markers_msg.points.at(global_iterator).x + odom_transform(0,1) * markers_msg.points.at(global_iterator).y + odom_transform(0,2));
    markers_in_map.at(global_iterator)(1) = (odom_transform(1,0) * markers_msg.points.at(global_iterator).x + odom_transform(1,1) * markers_msg.points.at(global_iterator).y + odom_transform(1,2));
  }
  for (global_iterator = 0; global_iterator < marker_counter; global_iterator++)
  {
    for (global_iterator_2 = 0; global_iterator_2 < map_marker_counter; global_iterator_2++)
    {
      if (fabs(markers_in_map.at(global_iterator)(0) - map_markers.at(global_iterator_2)(0)) < marker_position_tresh_[0] 
          && fabs(markers_in_map.at(global_iterator)(1) - map_markers.at(global_iterator_2)(1)) < marker_position_tresh_[1])
      {
        // needed to determine if the marker was found -> global_iterator_2 greater then map_marker_counter
        my_file <<"Marker in TRESH"<<"\n"; 
        my_file <<"ODOM:"<<"\n"; 
        my_file <<markers_in_map.at(global_iterator)<<"\n"; 
        my_file <<"MAP:"<<"\n"; 
        my_file <<map_markers.at(global_iterator_2)<<"\n"; 
        my_file <<"global_iterator_2: "<< global_iterator_2 <<"\n"; 

        global_iterator_2 = global_iterator_2 + map_marker_counter + 1;
        break;
      }
    }
    // marker found in map_markers, change pose of known marker
    if (global_iterator_2 > map_marker_counter)
    {
      // get the true global_iterator_2 count
      global_iterator_2 = global_iterator_2 - map_marker_counter - 1;
      map_markers.at(global_iterator_2).head(2) = map_markers.at(global_iterator_2).head(2) + (markers_in_map.at(global_iterator).head(2) - map_markers.at(global_iterator_2).head(2))/2;
      
      if (map_markers.at(global_iterator_2)(2) > MAX_SCORE)
        map_markers.at(global_iterator_2)(2) = MAX_SCORE;
      else
        map_markers.at(global_iterator_2)(2) = map_markers.at(global_iterator_2)(2) + 1;

      my_file <<"new MAP:"<<"\n"; 
        my_file <<map_markers.at(global_iterator_2)<<"\n"; 

      my_file <<"global_iterator_2: "<< global_iterator_2 <<"\n"; 
      my_file <<"global_iterator: "<< global_iterator<<"\n"; 
      my_file <<"map_marker_counter: "<< map_marker_counter <<"\n"; 
    }
    //marker not found in map_markers -> adding this marker to map
    else
    {
      my_file <<"Adding new marker to map"<<"\n"; 
      if (map_marker_counter > map_markers.size())
          my_file <<"ERROR"<<"\n";
        //ROS_ERROR_STREAM ("[Global localization] -- reached max global markers count ( "<<map_markers.size()<<" )!!!! \n");
      else
      {   
        map_markers.at(map_marker_counter).head(2) = markers_in_map.at(global_iterator);
        map_markers.at(global_iterator_2)(2) = 0;
        map_marker_counter += 1;
      }
    }
  }
  map_positions.clear();

    for (global_iterator_2 = 0; global_iterator_2 < map_marker_counter; global_iterator_2++)
    {
      if (map_markers.at(global_iterator_2)(2) > SCORE_THRESH)
      {
        map_point.x = map_markers.at(global_iterator_2)(0);
        map_point.y = map_markers.at(global_iterator_2)(1);
        map_point.z = map_markers.at(global_iterator_2)(2);
        map_positions.push_back(map_point);
      }
    }
  vis_color.r = 1;
  vis_color.g = 0;
  vis_color.b = 0;
  vis_color.a = 1;

  visualizationInitialization(vis_map_markers, map_positions.size(), map_positions, vis_color, "map", "map_markers");
  vis_markers_array.markers = {vis_current_markers, vis_map_markers};
}