#pragma once
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <opencv2/features2d.hpp>
#include <algorithm>

namespace RgbdSlamRicoExercises{

// To Simple Robotics 
using cv::KeyPoint;
using SimpleRoboticsCppUtils::build_image_pyramid;
using SimpleRoboticsCppUtils::draw_feature_points;


constexpr const int BORDER_SIZE = 4;    // FAST Circle's radius is 3
constexpr const int DESIRED_KEYPOINT_BASE_LEVEL = 40;

void cull(std::vector<KeyPoint>& keypoints, const int& desired_features_num){
    if (keypoints.size() <= desired_features_num) return;
    // elements before the returned iterator has a value less than or equal to the value
    std::nth_element(keypoints.begin(),keypoints.begin() + desired_features_num - 1, keypoints.end(),
        [](const KeyPoint& k1, const KeyPoint& k2){
            //descending 
            return k1.response > k2.response;
        }
    );
    // we might still have elements equal to the nth element. So, we use partition to find them
    auto new_end = std::partition(keypoints.begin() + desired_features_num, keypoints.end(),
        [&keypoints, &desired_features_num](const KeyPoint& k){return k.response >= keypoints.at(desired_features_num-1).response;}
    );
    keypoints.erase(new_end, keypoints.end());
}

/**
 * @brief Compute CV::FAST on a gray scale image, then rate keypoints based on their cornerness
 * 
 * @param pyramid : image pyramid
 * @return std::vector<std::vector<KeyPoint>> 
 */
std::vector<std::vector<KeyPoint>> compute_keypoints(const std::vector<cv::Mat>& pyramid){
    std::vector<std::vector<KeyPoint>> all_keypoints;
    int octave = 0;
    for (const auto& image: pyramid){
        // Use CV::Fast to compute
        int threshold = 40; // threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.
        bool non_maximal_suppression = true; // Apply non-maximum suppression
        std::vector<KeyPoint> keypoints;
    /**
     * Fast Detection
        for (const auto& pixel : interesting_pixels) {
            std::vector<Pixel> pixels = draw_circle_of_16_pixels(pixel);
            bool has_consecutive = detect_consecutive_px(pixels, 12);   // FAST-12
            if (has_consecutive) {
                keypoints.push_back(std::move(pixel));
            }
        }
        non_maximal_suppression(keypoints);
     */
        // LOTS OF FEATURESSSSSSS. Definitely Needs to be pruned
        cv::FAST(image, keypoints, non_maximal_suppression, threshold, cv::FastFeatureDetector::TYPE_9_16);
        // Remove keypoints near the border
        cv::KeyPointsFilter::runByImageBorder(keypoints, image.size(), BORDER_SIZE);

        cull(keypoints, 2*DESIRED_KEYPOINT_BASE_LEVEL);
        // Calculates gradient variance matrix M on every pixel, and calculate the det(M)-k(tr(M))
        cv::Mat harris_response;
        constexpr const int harris_block_size=2;
        constexpr const int aperture_size=3;  //for sobel?
        constexpr const double k = 0.04;    // Harris detector free parameter?

        // Harris Corner provides a better scoring
        cv::cornerHarris(image, harris_response, harris_block_size, aperture_size, k);
        for (auto& k: keypoints){
            k.response = harris_response.at<float>(
                static_cast<int>(k.pt.y),
                static_cast<int>(k.pt.x)
            );
            //TODO
            std::cout<<"response: "<<k.response<<std::endl;
        }
        cull(keypoints, DESIRED_KEYPOINT_BASE_LEVEL);
        for(auto& k: keypoints){
            k.octave = octave;
        }
        // TODO
        draw_feature_points(image, keypoints);

        all_keypoints.emplace_back(std::move(keypoints));
        ++octave;
    }

    return all_keypoints;
}

// // void IC_Angle_Integral(){}

// // https://github.com/barak/opencv/blob/051e6bb8f6641e2be38ae3051d9079c0c6d5fdd4/modules/features2d/src/orb.cpp#L233-L234
// void compute_orientation(){
//     // opencv uses IC_Angle_Integral, to  begin with, we use the vanilla image moment calculation
// }

// // https://github.com/barak/opencv/blob/051e6bb8f6641e2be38ae3051d9079c0c6d5fdd4/modules/features2d/src/orb.cpp#L320
// // cv::Mat & descriptors
// void compute_descriptor(){
//     // retrieve the point pair pattern, 
//     // convert the angles back to bits.
// }


// https://github.com/barak/opencv/blob/051e6bb8f6641e2be38ae3051d9079c0c6d5fdd4/modules/features2d/src/orb.cpp#L533-L534
void handwritten_orb(const cv::Mat& image_in, 
    std::vector<KeyPoint>& keypoints, 
    cv::Mat& descriptors){
    cv::Mat image = image_in.clone();
    // Image is converted to gray
    if (image.type() != CV_8UC1)
        // dst can be the same as src
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
    auto pyramid = build_image_pyramid(image, 3, 1.2);
    auto all_keypoints = compute_keypoints(pyramid);

//     // remove keypoints close to border. 

//     // Create descriptor
//     std::vector<Descriptor> descriptors;
//     for (const auto& k : keypoints) {
//         // Get moments
//         double patch_moment_10 = get_patch_moment(k, image, 1, 0);  // I(x,y)
//         double patch_moment_01 = get_patch_moment(k, image, 0, 1);  // I(x,y)
//         double theta = atan2(patch_moment_01, patch_moment_10);

//         // Start from theta
//         std::vector<Pixel> pixels = draw_circle_of_16_pixels(k);
//         int starting_pixel_index = static_cast<int>(theta / (2 * M_PI / 16));

//         // Generate descriptor
//         Descriptor desc = round_robin_16_bit_intensity(pixels, starting_pixel_index);
//         descriptors.push_back(desc);
//     }

//     res.keypoints = std::move(keypoints);
//     res.descriptors = std::move(descriptors);
}
// https://github.com/barak/opencv/blob/051e6bb8f6641e2be38ae3051d9079c0c6d5fdd4/modules/features2d/include/opencv2/features2d/features2d.hpp#L2429-L2430
// void match_descriptor_bruteforce(){
// }
};