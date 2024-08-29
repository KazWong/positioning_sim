#include <vector>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "source_beacon_broadcaster");

    geometry_msgs::TransformStamped static_transformStamped;
    std::vector<geometry_msgs::TransformStamped> static_tf;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    tf2::Quaternion q;
    tf2::Vector3 pos;
    ros::Time t = ros::Time::now();

    /* Soure 0 */
    static_transformStamped.header.stamp = t;
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "source0";

    static_transformStamped.transform.translation.x = 0.8;
    static_transformStamped.transform.translation.y = 0.8;
    static_transformStamped.transform.translation.z = 0.0;

    static_transformStamped.transform.rotation.x = 0.0;
    static_transformStamped.transform.rotation.y = 0.0;
    static_transformStamped.transform.rotation.z = 0.0;
    static_transformStamped.transform.rotation.w = 1.0;

    static_tf.push_back(static_transformStamped);

    /* Soure 1 */
    static_transformStamped.header.stamp = t;
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "source1";

    static_transformStamped.transform.translation.x = 0.8;
    static_transformStamped.transform.translation.y = -0.8;
    static_transformStamped.transform.translation.z = 0.0;

    static_transformStamped.transform.rotation.x = 0.0;
    static_transformStamped.transform.rotation.y = 0.0;
    static_transformStamped.transform.rotation.z = 0.0;
    static_transformStamped.transform.rotation.w = 1.0;

    static_tf.push_back(static_transformStamped);

    /* Soure 2 */
    static_transformStamped.header.stamp = t;
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "source2";

    static_transformStamped.transform.translation.x = -0.8;
    static_transformStamped.transform.translation.y = 0.8;
    static_transformStamped.transform.translation.z = 0.0;

    static_transformStamped.transform.rotation.x = 0.0;
    static_transformStamped.transform.rotation.y = 0.0;
    static_transformStamped.transform.rotation.z = 0.0;
    static_transformStamped.transform.rotation.w = 1.0;

    static_tf.push_back(static_transformStamped);

    /* Soure 3 */
    static_transformStamped.header.stamp = t;
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "source3";

    static_transformStamped.transform.translation.x = -0.8;
    static_transformStamped.transform.translation.y = -0.8;
    static_transformStamped.transform.translation.z = 0.0;

    static_transformStamped.transform.rotation.x = 0.0;
    static_transformStamped.transform.rotation.y = 0.0;
    static_transformStamped.transform.rotation.z = 0.0;
    static_transformStamped.transform.rotation.w = 1.0;

    static_tf.push_back(static_transformStamped);

    static_broadcaster.sendTransform(static_tf);

    ros::spin();
    return 0;
};