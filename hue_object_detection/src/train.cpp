#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main( int _argc, char** _argv ) {

	if (_argc != 4 ) {
		std::cout << "Syntax : " << _argv[0] << " [image] [mask] [label]" << std::endl;
		std::cout << "Generates [label].yaml" << std::endl;
		return 0;
	}

    std::string rgb_filename = _argv[1];
	cv::Mat rgb  = cv::imread(rgb_filename, CV_LOAD_IMAGE_UNCHANGED);
    if (!rgb.data) {
		std::cout << "Unable to open " << rgb_filename << std::endl;
		return 0;
    }

    std::string mask_filename = _argv[2];
	cv::Mat mask = cv::imread(mask_filename, CV_LOAD_IMAGE_GRAYSCALE);
    if (!mask.data) {
		std::cout << "Unable to open " << mask_filename << std::endl;
		return 0;
    }

    std::string histogram_filename = std::string(_argv[3]) + ".yaml";

	cv::Mat	hls = cv::Mat(rgb.size(), CV_8UC3);
	cv::cvtColor(rgb, hls, CV_BGR2HLS);

    int hue_bins = 32;  // quantization levels
    int size[] = { hue_bins };
    float hue_ranges[] = { 0, 180 };  // Hue varies from 0 to 179
    const float* ranges[] = { hue_ranges };
    int channels[] = { 0 };  // Hue is the 0-th channel

    cv::Mat histogram;
    cv::calcHist(&hls, 1, channels, mask, histogram, 1, size, ranges);
	cv::normalize(histogram, histogram, 1.0, 0, cv::NORM_L1);

    cv::FileStorage fs(histogram_filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { std::cout << "Unable to open " << histogram_filename << std::endl; }
    fs << "histogram" << histogram;
    fs.release();

    cv::Mat train_img;
    rgb.copyTo(train_img, mask);
    cv::namedWindow("Train image");
	imshow("Train image", train_img);

	cv::normalize(histogram, histogram, 0, 255, CV_MINMAX);
    cv::Mat histogram_img = cv::Mat::zeros(200, 320, CV_8UC3);
	int bin_width = histogram_img.cols / hue_bins;

	cv::Mat buffer(1, hue_bins, CV_8UC3);
	for (int i = 0; i < hue_bins; i++) {
		buffer.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(180.0*i / hue_bins), 255, 255);
    }
	cvtColor(buffer, buffer, CV_HSV2BGR);

	for (int i = 0; i < hue_bins; i++) {
		int val = cv::saturate_cast<int>(histogram.at<float>(i) * histogram_img.rows / 255);
		rectangle(histogram_img, cv::Point(i*bin_width, histogram_img.rows),
			cv::Point((i+1)*bin_width, histogram_img.rows - val),
			cv::Scalar(buffer.at<cv::Vec3b>(i)), -1, 8);
	}
    cv::namedWindow("Histogram");
	imshow("Histogram", histogram_img);

	cv::waitKey();
	return 0;
}
