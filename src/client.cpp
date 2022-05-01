#include "ros/ros.h"
#include "project1/Reset_Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Matrix3x3.h"


class Reset {
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::ServiceClient client;
    project1::Reset_Odometry srv;
    
    bool published;
    
    double x,y,theta;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& true_pose) {
        
        this->x = true_pose->pose.position.x;
        this->y = true_pose->pose.position.y;

        double quatx= true_pose->pose.orientation.x;
        double quaty= true_pose->pose.orientation.y;
        double quatz= true_pose->pose.orientation.z;
        double quatw= true_pose->pose.orientation.w;
    
        tf2::Quaternion q(quatx, quaty, quatz, quatw);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        this->theta = yaw;

        resetFunction();
    };
    
    
    void resetFunction(){
        srv.request.x = this->x;
        srv.request.y = this->y;
        srv.request.theta = this->theta;

          
        if (client.call(srv))
        {
          ROS_INFO("I heard pose [%f, %f, %f],\nold pose was: [%f, %f, %f]", this->x, this->y, this->theta, (double)srv.response.x, (double)srv.response.y, (double)srv.response.theta);
            this->published = true;
        }
        else
        {
          ROS_ERROR("Failed to call service reset_odometry, retry");
        }
    };
    
    
    
public:
    Reset() { // class constructor
      // all initializations here
        ros::NodeHandle n;
        this->published = false;
        this->sub = this->n.subscribe("/robot/pose", 1, &Reset::poseCallback, this);
        this->client = n.serviceClient<project1::Reset_Odometry>("reset_odometry");
        ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
    };
    
    void main_loop(){
        sleep(1.0);
        ros::Rate rate(1);
        
        ROS_INFO("Node %s running.", ros::this_node::getName().c_str());
        while(!published&&ros::ok()){
            
            ros::spinOnce();
            rate.sleep();
            
        };
        
    };
    
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_client");
    
    Reset my_reset;
    
    my_reset.main_loop();
    
    ROS_INFO("node reset_client is shutting down");

  return 0;
}