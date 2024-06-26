// docker run --rm -it -v /home/$USER/rjia:/home/rjia --workdir /home/rjia --runtime nvidia --network host nvcr.io/nvidia/l4t-ml:r32.6.1-py3
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <chrono>
#include <iostream>

int main() {
    try {
        // Load the image and template
        cv::Mat img = cv::imread("map499.png", cv::IMREAD_GRAYSCALE);
        cv::Mat templ = cv::imread("map499_bluth.png", cv::IMREAD_GRAYSCALE);

        if (img.empty() || templ.empty()) {
            std::cerr << "Could not read one of the images." << std::endl;
            return -1;
        }

        // Upload the images to GPU
        cv::cuda::GpuMat d_img, d_templ, d_result;
        d_img.upload(img);
        d_templ.upload(templ);
        

        int result_cols = img.cols - templ.cols + 1;
        int result_rows = img.rows - templ.rows + 1;
        d_result.create(result_rows, result_cols, CV_32FC1);

        auto start = std::chrono::high_resolution_clock::now();  // Start the timer
        // Create CUDA-based template matchers and streams
        std::vector<cv::Ptr<cv::cuda::TemplateMatching>> matchers;
        std::vector<cv::cuda::Stream> streams;
        for (int i = 0; i < 127; i++) {
            matchers.push_back(cv::cuda::createTemplateMatching(d_img.type(), cv::TM_CCOEFF_NORMED));
            streams.emplace_back();
        }
        for (int i = 0; i < 127; i++) {
            matchers[i]->match(d_img, d_templ, d_result, streams[i]);
        }

        // Wait for all operations to complete
        for (auto& stream : streams) {
            stream.waitForCompletion();
        }
        
        // Optionally, process each result
        for (int i = 0; i < 127; i++) {
            cv::Mat h_result;
            d_result.download(h_result, streams[i]);

            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(h_result, &minVal, &maxVal, &minLoc, &maxLoc);

            // Draw a rectangle around the matched region if necessary
            // cv::rectangle(img, maxLoc, cv::Point(maxLoc.x + templ.cols, maxLoc.y + templ.rows), cv::Scalar::all(0), 2, 8, 0);
        }
        auto end = std::chrono::high_resolution_clock::now();  // Stop the timer
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Matching time: " << elapsed.count() << " seconds." << std::endl;


    } catch (const cv::Exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    return 0;
}
