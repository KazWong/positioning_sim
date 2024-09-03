#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <guiding_before_positioning/uwb_msg.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "guiding and positioning simulation");
    ros::NodeHandle nd;

    ros::Subscriber uwb_sub = n.subscribe("uwb_data", 1000, UWB_DataCallback);

    return 0;
}