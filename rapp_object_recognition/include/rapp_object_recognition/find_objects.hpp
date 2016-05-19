#include <vector>
#include <string>

#include <geometry_msgs/Point.h>

namespace FindObjects {
	int findObjects(const std::string & fname, const std::vector<std::string> & names, const std::vector<std::string> & files, unsigned int limit, 
	                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores);
	
	void learnObject(const std::string & fname, const std::string & name, const std::string & user, int & result);
	
	void clearModels();
	
	void loadModel(const std::string & user, const std::string & name);
	
	void loadModels(const std::string & user, const std::vector<std::string> & names, int & result);
	
};
