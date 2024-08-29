#include <math.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <guiding_before_positioning/uwb_msg.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_msg_pub");

    ros::NodeHandle node;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfl(tf_buffer);
    ros::Publisher uwb_data_pub = node.advertise<guiding_before_positioning::uwb_msg>("uwb_data", 10);

    ros::Rate rate(10.0);
    while (node.ok())
    {
        geometry_msgs::TransformStamped tf;
        try
        {
            tf = tf_buffer.lookupTransform("target", "source0", ros::Time(0));
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        guiding_before_positioning::uwb_msg uwb_msg;
        uwb_msg.id = 0;
        double x = tf.transform.translation.x;
        double y = tf.transform.translation.y;
        double z = tf.transform.translation.z;
        uwb_msg.distance = sqrt(x*x + y*y + z*z);
        uwb_msg.azimuth = 20.0;
        uwb_msg.elevation = 30.0;

        uwb_data_pub.publish(uwb_msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};