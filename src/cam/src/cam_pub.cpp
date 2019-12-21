#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv)
{
    ros::init (argc, argv, "cam_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("image", 1);

    cv::Mat image;
    cv::VideoCapture camera(0);

    if (!camera.isOpened()) {
        ROS_INFO("failed to open camera.");
        return -1;
    }

    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 360);

    ros::Rate rate(2);
    while(ros::ok()) {
        camera >> image;

        if(!image.empty()){
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                    "bgr8", image).toImageMsg();
            image_pub.publish(msg);
            // cv::waitKey(0);
        }

        // ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
