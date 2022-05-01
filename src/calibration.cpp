#include "ros/ros.h"
#include "project1/paramMsg.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class Calibrator {
private:
    ros::NodeHandle n;
    
    message_filters::Subscriber<geometry_msgs/PoseStamped> sub_pose;
    message_filters::Subscriber<sensor_msgs/JointState> sub_wheels;
    
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::JointState> MySyncPolicy;
    
    message_filters::Synchronizer<MySyncPolicy> sync;
    

    ros::Publisher  output_publisher;
    
    double x_past, y_past, theta_past;
    double tick_past[4];
    double lw, Tr, N;
    
    ros::time t_curr, t_prev;
    
    void Calibrator::Callback(const geometry_msgs::PoseStamped::ConstPtr& true_pose, const sensor_msgs/JointState wheels_tick) {
        
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
    };
    
    void compute(){
        
        
        
        
    };
    
    
    
    void Calibrator::publish(){
        project1::paramMsg msg;
        msg.lw = this->lw;
        msg.Tr = this->Tr;
        msg.N = this->N;
        output_publisher.publish(msg);
    };
    
public:
    
    Calibrator():sync(sub_pose, sub_wheels) { // class constructor
      // all initializations here
        sub_pose.subscribe(n, "/robot/pose", 1);
        sub_wheels.subscribe(n, "/wheel_states",1);
        
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_pose, sub_wheels);
        
        sync.registerCallback(boost::bind(&Calibrator::Callback, this, _1, _2));
                
        this->output_publisher = this->n.advertise<project1::paramMsg>("/est_params", 1);
        
        ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
    };
    
    void Calibrator::main_loop(){
        sleep(1.0);
        ros::Rate rate(10);
        
        ROS_INFO("Node %s running.", ros::this_node::getName().c_str());
        
        while(ros::ok()){
            
            ros::spinOnce();
            rate.sleep();
            
        };
        
    };
    
};
    
    
}