#include "ros/ros.h"
#include "Robotics_project1/Reset_odom.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Matrix3x3.h"

class Reset {
private:
    ros::NodeHandle m;
    ros::Subscriber subscriber;
    ros::ServiceClient client;
    Robotics_project1::Reset_odom srv;
    
    bool pub;
    
    double x,y,theta;

    void pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& true_pose) {
        
        this->x = true_pose->pose.position.x;
        this->y = true_pose->pose.position.y;
        double quaternion_x= true_pose->pose.orientation.x;
        double quaternion_y= true_pose->pose.orientation.y;
        double quaternion_z= true_pose->pose.orientation.z;
        double quaternion_w= true_pose->pose.orientation.w;
        tf2::Quaternion q(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        this->theta = yaw;
        resetpose();
    };
    
    
    void resetpose(){

        srv.request.x = this->x;
        srv.request.y = this->y;
        srv.request.theta = this->theta;

          
        if (client.call(srv)) {
          ROS_INFO("New pose is [%f, %f, %f], while old pose was: [%f, %f, %f]", this->x, this->y, this->theta, (double)srv.response.x, (double)srv.response.y, (double)srv.response.theta);
            this->pub = true;
        }
        else {
          ROS_ERROR("Failed to call service Reset_odom");
        }
    };
    
    
    
public:
    Reset() { // class constructor
      // all initializations here
        ros::NodeHandle m;
        this->pub = false;
        this->subscriber = this->m.subscribe("/robot/pose", 1, &Reset::pose_Callback, this);
//this->client = m.serviceClient<Robotics_project1::Reset_odom>("Reset_odom");
    };
    
    void main_loop(){
        sleep(1.0);
        ros::Rate looprate(2);
        //ROS_INFO("Node %s running.", ros::this_node::getName().c_str());
        while(!pub&&ros::ok()){
            ros::spinOnce();
            looprate.sleep();
        };
    };
};


int main(int argc, char **argv){
    ros::init(argc, argv, "reset_client");
    Reset reset_node;
    reset_node.main_loop();
    return 0;
}