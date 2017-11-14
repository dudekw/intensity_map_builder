
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>

class IntensityMapHandler
{
public:
    IntensityMapHandler(size_t markers_size);
    IntensityMapHandler();
    ~IntensityMapHandler();
    bool load_map(const std::string &file_path, std::vector<Eigen::Vector3f> &map_markers, size_t &used_markers_count);
    bool save_map(const std::string &file_path, const std::vector<Eigen::Vector3f> &map_markers, size_t &used_markers_count);
    bool save_map(const std::string &file_path, const visualization_msgs::Marker &map_markers);
    bool getMarkers(std::vector<Eigen::Vector3f> &markers);
private:
bool calcMarkDistEIGEN(const std::vector<Eigen::Vector3f> &input_markers, const int &marker_size, const int &respect_marker, Eigen::Matrix<float,Eigen::Dynamic,3> &distances);
bool calcMarkDistVISUAL(const visualization_msgs::Marker &input_markers, const int &respect_marker, Eigen::Matrix<float,Eigen::Dynamic,3> &distances);
    std::vector<Eigen::Vector3f> map_markers;
  Eigen::Matrix<float,1,3>  distance_memory;
    Eigen::Matrix<float,Eigen::Dynamic,3> marker_distances;
    size_t global_iterator;
    size_t global_iterator_2;
    size_t calcMarkDist_iter;
    size_t calcMarkDist_iter_2;
    size_t calcMarkDist_iter_3;
    boost::property_tree::ptree xml_tree;
    boost::property_tree::ptree save_xml_tree;
    const boost::property_tree::ptree& marker_tree = save_xml_tree;
    std::ostringstream marker_path;
    Eigen::Vector3f tmp_marker;
};