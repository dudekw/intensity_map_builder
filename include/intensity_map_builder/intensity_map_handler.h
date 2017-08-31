
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
    std::vector<Eigen::Vector3f> map_markers;
    size_t global_iterator;

    boost::property_tree::ptree xml_tree;
    boost::property_tree::ptree save_xml_tree;
    const boost::property_tree::ptree& marker_tree = save_xml_tree;
    std::ostringstream marker_path;
    Eigen::Vector3f tmp_marker;
};