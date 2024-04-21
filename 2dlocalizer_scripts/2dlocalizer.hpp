#pragma once
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <regex>

#include <Eigen/Dense>
#include <cmath>

// ----------------------------------------------------------------
Eigen::IOFormat print_in_one_line_format(4, 0, ", ", ", ", "", "", "", "");
struct RoundToInt{
    // Functor to round a float to int
    int operator() (float coeff)const{
        return static_cast<int>(std::round(coeff));
    }
};

struct InputArgs{
    std::string map_image_filename;
    std::string map_metadata_filename;
    friend std::ostream& operator<<(std::ostream& os, const InputArgs& args){
        os << "map_image_filename: " << args.map_image_filename<<", "; 
        os << "map_metadata_filename: " << args.map_metadata_filename; 
        os << std::endl;
        return os;
    }
};

InputArgs parse_input_args(int argc, char** argv){
    if(argc != 2) throw std::runtime_error("usage: 2dlocalizer <map_image_filename>");
    InputArgs args;
    args.map_image_filename = argv[1];
    std::regex rgx("(.+)\\.(png|pgm)$"); // Regex to capture the name before ".pgm"
    std::smatch matches;
    if (std::regex_search(args.map_image_filename, matches, rgx) && matches.size() > 1){
        args.map_metadata_filename = matches[1].str()+".yaml";
    }
    else{
        throw std::runtime_error("Need '.' in filename: " + args.map_image_filename);
    }
    return args;
}

inline std::vector<std::vector<float>> read_laser_scans_from_csv(const std::string& filename){
    std::ifstream file(filename.c_str());
    std::vector<std::vector<float>> laser_scans;
    std::string line;

    while(std::getline(file, line)){
        std::stringstream ss(line);
        std::vector<float> scan;
        float value;

        // string stream can extract a number
        while (ss >> value) {
            scan.push_back(value);
            // ss.peek() will check the next character
            if (ss.peek() == ',') ss.ignore();
        }
        laser_scans.push_back(scan);
    }
    return laser_scans;
}

inline void print_scans(const std::vector<std::vector<float>>& laser_scans){
    // for(const auto& scan : laser_scans){
    //     for(const auto& scan_value : scan) std::cout<<scan_value<<", ";
    //     std::cout<<std::endl;
    // }
    std::cout<<"Number of scans: "<<laser_scans.size()<<std::endl<<std::endl;
}

struct MapMetadata{
    float resolution;
    Eigen::Vector3f origin;
    float width, height;
    float laser_max_range;
    friend std::ostream& operator<<(std::ostream& os, const MapMetadata& metadata){
        os<<"--------------------------------"<<std::endl;
        os<<"resolution: "<<metadata.resolution<<std::endl;
        os<<"origin: "<<"["<<metadata.origin[0]<<", "<<metadata.origin[1]<<", "<<metadata.origin[2]<<"]"<<std::endl;
        os<<"width: "<<metadata.width<<std::endl;
        os<<"height: "<<metadata.height<<std::endl;
        return os;
    }
};

inline MapMetadata load_map_metadata(const std::string& filename, const cv::Mat& map_image){
    YAML::Node config = YAML::LoadFile(filename);
    auto metadata = MapMetadata();
    metadata.resolution = config["resolution"].as<float>();
    std::vector<float> temp;
    for(const auto& origin : config["origin"]) temp.push_back(origin.as<float>());
    metadata.origin = Eigen::Map<Eigen::Vector3f>(temp.data(), temp.size());
    metadata.width = map_image.cols;
    metadata.height = map_image.rows;
    // TODO: to make this a parameter
    metadata.laser_max_range = 10.0;
    return metadata;
}

inline cv::Mat load_map_image(const std::string& filename){
    auto img = cv::imread(filename, cv::IMREAD_COLOR);
    if (img.empty()) throw std::runtime_error("Could not load image: " + filename);
    return img;
}

inline cv::Mat get_image_gradient(const cv::Mat& binary_image){
    cv::Mat gradients;
    cv::GaussianBlur(binary_image, gradients, cv::Size(3,3), 0, 0);
    cv::Laplacian(gradients, gradients, CV_8UC1);
    return gradients;
}

inline cv::Mat get_binary_image(const cv::Mat& image){
    cv::Mat new_image;
    double thresh = 0;
    double max_val = 255;
    cv::threshold(image, new_image, thresh, max_val, cv::THRESH_BINARY);  
    return new_image;
}

inline cv::Mat map_image_preprocessing(const cv::Mat& map_image){
    // to gray
    cv::Mat new_map_image;
    cv::cvtColor(map_image, new_map_image, cv::COLOR_RGBA2GRAY);
    new_map_image = get_binary_image(new_map_image);
    new_map_image = get_image_gradient(new_map_image);
    return new_map_image;
}

inline void visualize_map(const cv::Mat& map){
    cv::imshow("Map Display", map);
    cv::waitKey(0);
}