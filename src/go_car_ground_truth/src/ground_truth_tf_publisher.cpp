#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

tf::Transform transform;

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z));
  transform.setRotation(tf::Quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_tf_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("ground_truth_pose",
                                                         10, odometryCallback);

  static tf::TransformBroadcaster br;
  transform.setOrigin(tf::Vector3(0, 0, 0));
  transform.setRotation(tf::Quaternion(1, 0, 0, 0));

  ros::Rate r(50);
  while (ros::ok()) {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world",
                                          "ground_truth"));
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
