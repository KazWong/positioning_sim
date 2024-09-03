#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_beacon_broadcaster");

    tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "target";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.8;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    ros::Rate loop(10);
    while (ros::ok())
    {
        transformStamped.header.stamp = ros::Time::now();
        br.sendTransform(transformStamped);
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
};
