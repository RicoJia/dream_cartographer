// g++ -std=c++17 2dlocalizer.cpp -lyaml-cpp -I /usr/include/eigen3 `pkg-config --cflags --libs opencv4` && ./a.out map499.png 
#include "2dlocalizer.hpp"
#include <asm-generic/errno.h>


int main(int argc, char *argv[]){
    InputArgs input_args = parse_input_args(argc, argv);
    std::cout << input_args<<std::endl;
    auto laser_scans = read_laser_scans_from_csv("data/laser_scans.csv");
    print_scans(laser_scans);

    auto map_img = load_map_image(input_args.map_image_filename);
    map_img = map_image_preprocessing(map_img);
    auto metadata = load_map_metadata(input_args.map_metadata_filename, map_img);
    std::cout<<"metadata: "<<metadata<<std::endl;
    Eigen::Vector2i origin_xy_pixel = (-metadata.origin.head<2>() / metadata.resolution).unaryExpr(RoundToInt());
    std::cout<<"origin_xy_pixel: "<<origin_xy_pixel.format(print_in_one_line_format)<<std::endl;
    // visualize_map(map_img);
}