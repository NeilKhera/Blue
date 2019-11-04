#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "blue_tf_broadcaster");
  ros::NodeHandle nh;

  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;
  
  while (nh.ok()) {
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.0, 0.18)),
	ros::Time::now(), "base_link", "camera_link"));
    r.sleep();
  }
}
