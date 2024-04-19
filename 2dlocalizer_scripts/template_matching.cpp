#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

int main() {
    // Load the full image
    cv::Mat img = cv::imread("map499.png", cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Error reading the image" << std::endl;
        return -1;
    }

    // Load the template image
    cv::Mat templ = cv::imread("map499.png", cv::IMREAD_COLOR);
    if (templ.empty()) {
        std::cerr << "Error reading the template" << std::endl;
        return -1;
    }

    
    cv::Mat result;
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    result.create(result_rows, result_cols, CV_32FC1);

    // Do the matching
    cv::matchTemplate(img, templ, result, cv::TM_CCOEFF_NORMED);
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    // Localize the best match with minMaxLoc
    double minVal, maxVal;
    cv::Point minLoc, maxLoc, matchLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    // For TM_SQDIFF and TM_SQDIFF_NORMED, the best match are the lowest values
    matchLoc = maxLoc;

    // Draw a rectangle on the found location
    cv::rectangle(img, matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), cv::Scalar::all(0), 2, 8, 0);

    // Show the result
    cv::imshow("Matched Result", img);
    cv::waitKey(0);

    return 0;
}