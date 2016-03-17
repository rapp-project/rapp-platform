#include <vector>
#include <string>

#include <geometry_msgs/Point.h>

namespace FindObjects {
	
	int findObjects(std::string fname, std::vector<std::string> names, std::vector<std::string> files, unsigned int limit, 
	                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores);
	
};
