#include "ros/ros.h"
#include "Robotics_project1/odom.h" //pacchetto (turtlesim/Pose.h)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 

class TfBroad
{
public:
  TfBroad() {
    sub = n.subscribe("/odom", 1000, &TfBroad::callback, this); //cambiato da /turtle1/pose
    //sostituire turtle1/pose con la posizione base_link
  }

  void callback(const turtlesim::Pose::ConstPtr& msg){ //sostituire turtlesime::Pose con la posizione attuale
    // set header
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world"; //odom
    transformStamped.child_frame_id = "robot"; 
    // set x,y
    transformStamped.transform.translation.x = msg->x;
    transformStamped.transform.translation.y = msg->y;
    transformStamped.transform.translation.z = 0.0; 
    // set theta
    tf2::Quaternion q;  //tf2 da cambiare
    q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    // send transform
    br.sendTransform(transformStamped);
  }

private:
  ros::NodeHandle n; 
  tf2_ros::TransformBroadcaster br; //tf2_ros da cambiare
  geometry_msgs::TransformStamped transformStamped;
  ros::Subscriber input_sub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_broadcast");
  TfBroad my_tf_broadcaster;
  ros::spin();
  return 0;
}
