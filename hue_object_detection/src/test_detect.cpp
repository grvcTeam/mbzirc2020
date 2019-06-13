#include <hue_object_detection/HueDetection.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

int main(int _argc, char** _argv) {

	if (_argc != 3 ) {
		std::cout << "Syntax : " << _argv[0] << " [frame_file] [histogram_file]" << std::endl;
		std::cout << "Detects in [frame_file] objects whose hue is defined by [histogram_file]" << std::endl;
		return 0;
	}

	std::string frame_file = _argv[1];
	std::string histogram_file =  _argv[2];

	HueDetection detection;
	detection.addDetector("test", histogram_file, cvScalar(255, 0, 0));
	cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	while (true) {
		// Capture/load frames:
	 	cv::Mat frame = cv::imread(frame_file, CV_LOAD_IMAGE_UNCHANGED);

		// Pass frame to the hue-model-based tracker:
		detection.setFrame(frame);
		std::vector<HueItem> detected = detection.detect("test");

		// Print detected items:
		for (int i = 0; i < detected.size(); i++) {
			printf("[%d] Detected: centroid = {%d, %d}, area = %lf\n", i, detected[i].centroid.x, detected[i].centroid.y, detected[i].area);
		}

		cv::imshow("image", frame);
		cv::waitKey(1000);
	}

	return 0;
}
