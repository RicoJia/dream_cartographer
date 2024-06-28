#include <ros/ros.h>
#include <ros/package.h>
#include "simple_robotics_ros_utils/rosbag_helpers.hpp"
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

void load_rgbd_images(SimpleRoboticsRosUtils::BagParser& bp, const std::string& image_topic_name){
    auto msg = bp.next<sensor_msgs::Image>(image_topic_name);
    //TODO
    std::cout<<"msg->image"<<msg->height <<std::endl;
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Can't load image, %s", e.what());
        return;
    }
    // High Gui provides simple GUI functionalities, like track bar, mouse event handling
    cv::namedWindow("Image window");
    cv::imshow("Image window", cv_ptr->image);
    cv::waitKey(0);
    cv::destroyWindow("Image window");
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "orb_test");
   ros::NodeHandle nh;
   constexpr auto PACKAGE_NAME = "rgbd_slam_rico";
   std::string package_path = ros::package::getPath(PACKAGE_NAME);
    if (package_path.empty()) throw std::runtime_error(PACKAGE_NAME);

   auto bag_file_path = std::string(package_path)+"/data"+"/rgbd_dataset_freiburg1_xyz.bag";
   SimpleRoboticsRosUtils::BagParser bp(nh, bag_file_path);
   constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
   bp.add_topic<sensor_msgs::Image>(RGB_TOPIC, false);

   load_rgbd_images(bp, RGB_TOPIC);

}