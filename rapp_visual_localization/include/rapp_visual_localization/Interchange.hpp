#ifndef INTERCHANGE_HPP_
#define INTERCHANGE_HPP_

#include <condition_variable>
#include <thread>

class Interchange {
public:
	Interchange() {
		image_flag = false;
		prediction_flag = false;
	}

	struct pos2d {
		int x, y, d;
	};

	void setImage(cv::Mat i) {
		std::unique_lock<std::mutex> l(lock);
		
		std::cout << "Setting image.\n";

		prediction_flag = false;
		image_flag = true;
		img = i;

		has_image.notify_one();
	}

	cv::Mat getImage() {
		std::unique_lock<std::mutex> l(lock);
		std::cout << "Waiting for image...\n";
		has_image.wait(l, [this](){ return image_flag; });
		std::cout << "Got image!\n";

		image_flag = false;
		return img;
	}

	void setStep(int x, int y, int d) {
		dx = x;
		dy = y;
		dd = d;
	}

	pos2d getStep() {
		return { dx, dy, dd };
	}


	void setPrediction(int x, int y, int d) {
		std::unique_lock<std::mutex>l(lock);

		std::cout << "Setting prediction.\n";

		best_x = x;
		best_y = y;
		best_d = d;

		prediction_flag = true;

		has_prediction.notify_one();
	}

	pos2d getPrediction() {
		std::unique_lock<std::mutex> l(lock);
		std::cout << "Waiting for prediction...\n";
		has_prediction.wait(l, [this](){ return prediction_flag; });
		std::cout << "Got prediction!\n";

		prediction_flag = false;

		return {best_x, best_y, best_d};
	}

	void setBelief(double b) {
		belief = b;
	}

	double getBelief() {
		return belief;
	}

private:	
	cv::Mat img;
	bool image_flag;

	int dx;
	int dy;
	int dd;

	double belief;
	int best_x;
	int best_y;
	int best_d;
	bool prediction_flag;

	std::mutex lock;
	std::condition_variable has_image;
	std::condition_variable has_prediction;
};


#endif /* INTERCHANGE_HPP_ */
