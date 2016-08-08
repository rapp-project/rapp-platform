#ifndef INTERCHANGE_HPP_
#define INTERCHANGE_HPP_

struct Interchange {
	cv::Mat img;
	int dx;
	int dz;
	int dd;

	double belief;
	int best_x;
	int best_z;
	int best_d;
};


#endif /* INTERCHANGE_HPP_ */
