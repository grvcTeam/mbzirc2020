#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>

// using namespace cv;

int main(int argc, char** argv) {
    cv::Mat src, src_gray;

    /// Read the image
    src = cv::imread(argv[1], 1);

    if (!src.data) { return -1; }

    /// Convert it to gray
    cvtColor(src, src_gray, CV_BGR2GRAY);

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur(src_gray, src_gray, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 2, 10, 125, 105, 0, 0);

    /// Draw the circles detected
    printf("Total of %ld circles found!\n", circles.size());

    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle(src, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
        // circle outline
        circle(src, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
     }

    /// Show your results
    cv::namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
    imshow("Hough Circle Transform Demo", src);

    cv::waitKey(0);
    return 0;
}
