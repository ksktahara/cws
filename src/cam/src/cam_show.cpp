#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv)
{
    ros::init (argc, argv, "cam_show");
    ros::NodeHandle nh;

    cv::Mat image;
    cv::VideoCapture camera(0);

    if (!camera.isOpened()) {
        ROS_INFO("failed to open camera.");
        return -1;
    }

    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 360);

    ros::Rate rate (30);
    while(ros::ok()) {
        camera >> image;

        cv::imshow("image", image);
        cv::waitKey(1);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
