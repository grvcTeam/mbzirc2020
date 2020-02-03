int main{
Mat mask;
// specify the range of colours that you want to include, you can play with the borders here
cv::Scalar lowerb = cv::Scalar(38, 125, 250);
cv::Scalar upperb = cv::Scalar(42, 129, 255); 

cv::inRange(frame, lowerb, upperb, mask); // if the frame has any orange pixel, this will be painted in the mask as white

imshow("mask", mask); // show where orange pixels are located, then use this mask for further processing pass it for example to findNonZero() function in order to obtain the location of the pixels, etc...
}