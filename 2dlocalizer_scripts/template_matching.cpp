#include <chrono>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
// what is cuda stream?
// Try in python: img gradient

int main() {
  // Load the full image
  cv::Mat img = cv::imread("map499.png", cv::IMREAD_COLOR);
  if (img.empty()) {
    std::cerr << "Error reading the image" << std::endl;
    return -1;
  }

  // Load the template image
  cv::Mat templ = cv::imread("map499_bluth.png", cv::IMREAD_COLOR);
  if (templ.empty()) {
    std::cerr << "Error reading the template" << std::endl;
    return -1;
  }

  cv::Mat result;
  int result_cols = img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;
  result.create(result_rows, result_cols, CV_32FC1);

  // Do the matching
  auto start = std::chrono::high_resolution_clock::now(); // Start the timer
  for (int i = 0; i < 127; i++) {
    cv::matchTemplate(img, templ, result, cv::TM_CCOEFF_NORMED);
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    // Localize the best match with minMaxLoc
    double minVal, maxVal;
    cv::Point minLoc, maxLoc, matchLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    // For TM_SQDIFF and TM_SQDIFF_NORMED, the best match are the lowest values
    matchLoc = maxLoc;

    // Draw a rectangle on the found location
    cv::rectangle(img, matchLoc,
                  cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
                  cv::Scalar::all(0), 2, 8, 0);
    // std::cout<<"matchLoc: "<<matchLoc.x<<", "<<matchLoc.y<<std::endl;
  }
  auto end = std::chrono::high_resolution_clock::now(); // Stop the timer
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Matching time: " << elapsed.count() << " seconds." << std::endl;

  // Show the result
  cv::imshow("Matched Result", img);
  cv::waitKey(5000);

  return 0;
}