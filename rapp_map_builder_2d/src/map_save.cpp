#include <string>
#include <sstream>

int main(int argc, char** argv){

	std::string ID = argv[1];
	std::string map_name = argv[2];
    std::string map_start = "rosrun map_server map_saver -f "+map_name+" map:=map_"+ID+" __name:=map_saver_"+ID;

    system(map_start.c_str());
	return 0;
}