#include <intensity_map_msgs/IntensityMarker.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
class IntensityMapBuilder
{
public:

 visualization_msgs::MarkerArray getVisMarkerArray();
IntensityMapBuilder(std::vector<float> tresh);
IntensityMapBuilder();
void updateMapMarkers(const intensity_map_msgs::IntensityMarker& markers_msg, tf::StampedTransform &tf_transform);

private:

bool visualizationInitialization(visualization_msgs::Marker &vis_markers, const size_t &marker_id , const std::vector<geometry_msgs::Point> &positions, 
                                                        const std_msgs::ColorRGBA &color, const std::string &frame , const std::string &ns);

//visualization_msgs::MarkerArray vis_markers_array;
};