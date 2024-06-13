#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nlink_parser/LinktrackTagframe0.h>

class PathTfBroadcaster {
public:
    PathTfBroadcaster() {
        // Initialize the node handle
        nh_ = ros::NodeHandle();

        // Subscribe to the path topics
        uwb_path_sub_ = nh_.subscribe("/nlink_linktrack_tagframe0", 10, &PathTfBroadcaster::uwbpathCallback, this);
        estimated_path_sub_ = nh_.subscribe("/estimated_path", 10, &PathTfBroadcaster::estimatedpathCallback, this);

        // Initialize the transform broadcaster
        br_ = tf2_ros::TransformBroadcaster();
    }

    void uwbpathCallback(const nlink_parser::LinktrackTagframe0::ConstPtr& msg) {
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now(); // Use current time for the header
        transformStamped.header.frame_id = "map"; // Assuming the fixed frame is "map"
        transformStamped.child_frame_id = "uwb_frame";

        // Copy the UWB data to the transform
        transformStamped.transform.translation.x = msg->pos_3d[0];
        transformStamped.transform.translation.y = msg->pos_3d[1];
        transformStamped.transform.translation.z = msg->pos_3d[2];

        // Copy the quaternion orientation
        tf2::Quaternion q(
            msg->quaternion[0],
            msg->quaternion[1],
            msg->quaternion[2],
            msg->quaternion[3]
        );

        // Normalize the quaternion
        q.normalize();

        // Check if the quaternion is valid
        if (!isValidQuaternion(q)) {
            ROS_WARN("Received invalid quaternion, skipping this transform.");
            return;
        }

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // Send the transform
        br_.sendTransform(transformStamped);
    }

    void estimatedpathCallback(const nav_msgs::Path::ConstPtr& msg) {
        for (const auto& pose : msg->poses) {
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp = pose.header.stamp;
            transformStamped.header.frame_id = "map"; // assuming the frame_id is "map"
            transformStamped.child_frame_id = "estimated_frame";

            // Copy the pose data to the transform
            transformStamped.transform.translation.x = pose.pose.position.x;
            transformStamped.transform.translation.y = pose.pose.position.y;
            transformStamped.transform.translation.z = pose.pose.position.z;

            // Copy the quaternion orientation
            tf2::Quaternion q(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            );

            // Normalize the quaternion
            q.normalize();

            // Check if the quaternion is valid
            if (!isValidQuaternion(q)) {
                ROS_WARN("Received invalid quaternion, skipping this transform.");
                continue;
            }

            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            // Send the transform
            br_.sendTransform(transformStamped);
        }
    }

    bool isValidQuaternion(const tf2::Quaternion& q) {
        // A valid quaternion has a norm close to 1.0
        return std::abs(q.length() - 1.0) < 1e-6;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber uwb_path_sub_;
    ros::Subscriber estimated_path_sub_;
    tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "path_tf_broadcaster");

    PathTfBroadcaster broadcaster;

    ros::spin();
    return 0;
}
