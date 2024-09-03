#include <math.h>
#include <string> 

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <guiding_before_positioning/uwb_msg.h>
#include <geometry_msgs/Point.h>

// Let azmith is rx
// Let elevation is ry
const int NUM_OF_BEACON = 4;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_msg_pub");

    ros::NodeHandle node;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfl(tf_buffer);
    ros::Publisher gps_data_pub = node.advertise<geometry_msgs::Point>("gps_data", 10);
    ros::Publisher uwb_data_pub = node.advertise<guiding_before_positioning::uwb_msg>("uwb_data", 10);

    ros::Rate rate(10.0);
    while (node.ok())
    {
        geometry_msgs::TransformStamped tg_s_tf[NUM_OF_BEACON], tg_map_tf, s_map_tf[NUM_OF_BEACON];
        std::string source = "source";

        try
        {
            tg_map_tf = tf_buffer.lookupTransform("target", "map", ros::Time(0));

            for (int i=0;i<NUM_OF_BEACON;i++) {
                std::string idx = source + std::to_string(i);
                tg_s_tf[i] = tf_buffer.lookupTransform("target", idx, ros::Time(0));
                s_map_tf[i] = tf_buffer.lookupTransform(idx, "map", ros::Time(0));
            }
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        double tg_map_x = tg_map_tf.transform.translation.x;
        double tg_map_y = tg_map_tf.transform.translation.y;
        double tg_map_z = tg_map_tf.transform.translation.z;

        // GPS msg
        // Just let 0,0 as the point
        geometry_msgs::Point gps_msg;

        gps_msg.x = 0.0;
        gps_msg.y = 0.0;
        gps_msg.z = 0.0;

        gps_data_pub.publish(gps_msg);

        // UWB msg
        for (int i=0; i<NUM_OF_BEACON;i++) {
            guiding_before_positioning::uwb_msg uwb_msg;
            uwb_msg.id = i;

            double tg_s_x = tg_s_tf[i].transform.translation.x;
            double tg_s_y = tg_s_tf[i].transform.translation.y;
            double tg_s_z = tg_s_tf[i].transform.translation.z;

            double s_map_x = s_map_tf[i].transform.translation.x;
            double s_map_y = s_map_tf[i].transform.translation.y;
            double s_map_z = s_map_tf[i].transform.translation.z;


            uwb_msg.distance = sqrt(tg_s_x*tg_s_x + tg_s_y*tg_s_y + tg_s_z*tg_s_z);

            double xa = sqrt(tg_map_x*tg_map_x + tg_map_z*tg_map_z);
            double xb = sqrt(s_map_x*s_map_x + s_map_z*s_map_z);
            double xc = sqrt(tg_s_x*tg_s_x + tg_s_z*tg_s_z);
            double signx = (s_map_x > tg_map_x)? 1.0: -1.0;
            uwb_msg.azimuth = signx * acos( ((xc*xc) + (xb*xb) - (xa*xa))/(2*xc*xb) ) / M_PI * 180.0;

            double ya = sqrt(tg_map_x*tg_map_x + tg_map_y*tg_map_y);
            double yb = sqrt(s_map_x*s_map_x + s_map_y*s_map_y);
            double yc = sqrt(tg_s_x*tg_s_x + tg_s_y*tg_s_y);
            double signy = (s_map_y > tg_map_y)? 1.0: -1.0;
            uwb_msg.elevation = signy * acos( ((xc*xc) + (xb*xb) - (xa*xa))/(2*xc*xb) ) / M_PI * 180.0;

            uwb_data_pub.publish(uwb_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};