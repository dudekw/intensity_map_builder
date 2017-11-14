#include <intensity_map_builder/intensity_map_handler.h>
#include <boost/foreach.hpp>

#define SCORE_TRESH 200

IntensityMapHandler::IntensityMapHandler(size_t markers_size = 0)
{
	std::vector<Eigen::Vector3f> map_markers;
	map_markers.resize(markers_size);
	size_t global_iterator = 0;
	boost::property_tree::ptree xml_tree;
	boost::property_tree::ptree save_xml_tree;
	const boost::property_tree::ptree& marker_tree = save_xml_tree;
	std::ostringstream marker_path;
	Eigen::Vector3f tmp_marker;
	Eigen::Matrix<float,Eigen::Dynamic,3> marker_distances;
  Eigen::Matrix<float,1,3>  distance_memory;
}

IntensityMapHandler::IntensityMapHandler()
{
	IntensityMapHandler(0);
}
IntensityMapHandler::~IntensityMapHandler()
{

}
bool IntensityMapHandler::load_map(const std::string &file_path, std::vector<Eigen::Vector3f> &map_markers, size_t &used_markers_count)
{

	boost::property_tree::read_xml(file_path, xml_tree);
	used_markers_count = 0;
	BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, xml_tree.get_child("map"))
    {
    	tmp_marker.head(2) << v.second.get<float>("x"), v.second.get<float>("y");
    	tmp_marker(2) = v.second.get<float>("score");
    	map_markers.at(used_markers_count) = tmp_marker;
    	++used_markers_count;
    }
    return true;
}
bool IntensityMapHandler::calcMarkDistEIGEN(const std::vector<Eigen::Vector3f> &input_markers, const int &marker_size, const int &respect_marker, Eigen::Matrix<float,Eigen::Dynamic,3> &distances)
{
	for (calcMarkDist_iter = 0; calcMarkDist_iter < marker_size; ++calcMarkDist_iter)
	{
  	// distances [ [distance, point_id, isMatched]
  	//			               ...
  	//             [distance, point_id, isMatched]	]
    distances(calcMarkDist_iter,0) = sqrt(pow(input_markers.at(calcMarkDist_iter)(0) - input_markers.at(respect_marker)(0), 2) + 
                                        pow(input_markers.at(calcMarkDist_iter)(1) - input_markers.at(respect_marker)(1), 2));
    distances(calcMarkDist_iter,1) = calcMarkDist_iter; 
    distances(calcMarkDist_iter,2) = 0;
	for (calcMarkDist_iter_2 = 0; calcMarkDist_iter_2 < calcMarkDist_iter; ++calcMarkDist_iter_2)
	{
		if (distances(calcMarkDist_iter,0) < distances(calcMarkDist_iter_2,0))
		{
			distance_memory << distances.row(calcMarkDist_iter);
			for (calcMarkDist_iter_3 = calcMarkDist_iter; calcMarkDist_iter_3 > calcMarkDist_iter_2; --calcMarkDist_iter_3)
			{			
				distances.row(calcMarkDist_iter_3) = distances.row(calcMarkDist_iter_3-1);
			}
			distances.row(calcMarkDist_iter_2) << distance_memory;
		}
	}

  }

  return true;
}

bool IntensityMapHandler::calcMarkDistVISUAL(const visualization_msgs::Marker &input_markers, const int &respect_marker, Eigen::Matrix<float,Eigen::Dynamic,3> &distances)
{
std::cout<<"input_markers.points.size()\n";
std::cout<<input_markers.points.size()<<"\n";
std::cout<<"distances.rows()\n";
std::cout<<distances.rows()<<"\n";

	for (calcMarkDist_iter = 0; calcMarkDist_iter < input_markers.points.size(); ++calcMarkDist_iter)
	{
std::cout<<"calcMarkDist_iter\n";
std::cout<<calcMarkDist_iter<<"\n";
  	// distances [ [distance, point_id, isMatched] 
  	//			               ...
  	//             [distance, point_id, isMatched]	]
    distances(calcMarkDist_iter,0) = sqrt(pow(input_markers.points.at(calcMarkDist_iter).x - input_markers.points.at(respect_marker).x, 2) + 
                                        pow(input_markers.points.at(calcMarkDist_iter).y - input_markers.points.at(respect_marker).y, 2));
    distances(calcMarkDist_iter,1) = calcMarkDist_iter; 
    distances(calcMarkDist_iter,2) = 0;
	for (calcMarkDist_iter_2 = 0; calcMarkDist_iter_2 < calcMarkDist_iter; ++calcMarkDist_iter_2)
	{
std::cout<<"calcMarkDist_iter_2\n";
std::cout<<calcMarkDist_iter_2<<"\n";
		if (distances(calcMarkDist_iter,0) < distances(calcMarkDist_iter_2,0))
		{
			std::cout<<"IF\n";

			distance_memory << distances.row(calcMarkDist_iter);
			for (calcMarkDist_iter_3 = calcMarkDist_iter; calcMarkDist_iter_3 > calcMarkDist_iter_2; --calcMarkDist_iter_3)
			{			
std::cout<<"calcMarkDist_iter_3\n";
std::cout<<calcMarkDist_iter_3<<"\n";
				distances.row(calcMarkDist_iter_3) = distances.row(calcMarkDist_iter_3-1);
std::cout<<"END FOR\n";
			}
			distances.row(calcMarkDist_iter_2) << distance_memory;
std::cout<<"END IF\n";
		}
	}

  }

  return true;
}

bool IntensityMapHandler::save_map(const std::string &file_path, const std::vector<Eigen::Vector3f> &map_markers, size_t &used_markers_count)
{
std::cout<<"in save_map\n"; 
	marker_distances.resize(used_markers_count,3);
    for (global_iterator = 0; global_iterator < used_markers_count; global_iterator++ )
    {
    	if (map_markers.at(global_iterator)(2) > SCORE_TRESH)
    	{
	        marker_path.str("");
	        marker_path << "marker_" << global_iterator;

	        //marker_tree = xml_tree.add_child(marker_path.str(), boost::property_tree::ptree{});
	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".x";
	        save_xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(0));
	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".y";
	        save_xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(1));

	    	calcMarkDistEIGEN(map_markers, used_markers_count, global_iterator, marker_distances);

			for (global_iterator_2 = 0; global_iterator_2 < used_markers_count; ++global_iterator_2)
			{
		        marker_path.str("");
		        marker_path << "map.marker_"<<global_iterator<<".distances."<<marker_distances(global_iterator_2,1);
		        save_xml_tree.put(marker_path.str(), marker_distances(global_iterator_2,0));

			}
	        
	        save_xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(1));

	        marker_path << "map.marker_"<<global_iterator<<".y";
	        save_xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(1));

	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".score";
	        save_xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(2));
    	}

    }

    boost::property_tree::write_xml(file_path, save_xml_tree,
        std::locale(),
        boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>('\t', 1u)); 
	
	return true;
}
bool IntensityMapHandler::save_map(const std::string &file_path, const visualization_msgs::Marker &map_markers)
{
std::cout<<"in save_map\n";
	marker_distances.resize(map_markers.points.size(),3);

    for (global_iterator = 0; global_iterator < map_markers.points.size(); global_iterator++ )
    {
    	if (map_markers.points.at(global_iterator).z > SCORE_TRESH)
    	{
	        marker_path.str("");
	        marker_path << "marker_" << global_iterator;

	        //marker_tree = xml_tree.add_child(marker_path.str(), boost::property_tree::ptree{});
	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".x";
	        save_xml_tree.put(marker_path.str(), map_markers.points.at(global_iterator).x);
	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".y";
	        save_xml_tree.put(marker_path.str(), map_markers.points.at(global_iterator).y);

std::cout<<"calcMarkDistVISUAL\n";
	    	calcMarkDistVISUAL(map_markers, global_iterator, marker_distances);
std::cout<<"--- END calcMarkDistVISUAL\n";

			for (global_iterator_2 = 0; global_iterator_2 < map_markers.points.size(); ++global_iterator_2)
			{
		        marker_path.str("");
		        marker_path << "map.marker_"<<global_iterator<<".distances."<<marker_distances(global_iterator_2,1);
		        save_xml_tree.put(marker_path.str(), marker_distances(global_iterator_2,0));

			}
	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".score";
	        save_xml_tree.put(marker_path.str(), map_markers.points.at(global_iterator).z);
    	}

    }

    boost::property_tree::write_xml(file_path, save_xml_tree,
        std::locale(),
        boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>('\t', 1u)); 
	
	return true;
}
bool IntensityMapHandler::getMarkers(std::vector<Eigen::Vector3f> &markers)
{
	markers = map_markers;
	return true;
}