#include "ros/ros.h"
#include "project1/ParamMsg.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class Calibrator {
private:
    ros::NodeHandle n;
    
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose;
    message_filters::Subscriber<sensor_msgs::JointState> sub_wheels;
    
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::JointState> MySyncPolicy;
    
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
        
    

    ros::Publisher  output_publisher;
    
    double x_past, y_past, theta_past;
    double tick_past[4];
    double lw, Tr, N;
    
    ros::Time t_curr, t_prev;
    
    void Callback(const geometry_msgs::PoseStamped::ConstPtr& true_pose, const sensor_msgs::JointState wheels_tick) {
        
        double x_curr = true_pose->pose.position.x;
        double y_curr = true_pose->pose.position.y;

        double quatx= true_pose->pose.orientation.x;
        double quaty= true_pose->pose.orientation.y;
        double quatz= true_pose->pose.orientation.z;
        double quatw= true_pose->pose.orientation.w;
    
        tf2::Quaternion q(quatx, quaty, quatz, quatw);
        tf2::Matrix3x3 m(q);

        double roll, pitch, theta_curr;
        m.getRPY(roll, pitch, theta_curr);

        ROS_INFO("I heard true position [%f, %f, %f] and wheels' state [%f, %f, %f, %f]", x_curr, y_curr, theta_curr, tick_past[0], tick_past[1], tick_past[2], tick_past[3]);

    };
    
    void compute(){
        
        
        
        
    };
    
    
    
    void publish(){
        project1::ParamMsg msg;
        msg.lw = this->lw;
        msg.Tr = this->Tr;
        msg.N = this->N;
        output_publisher.publish(msg);
    };
    
public:
    
    Calibrator(){ // class constructor
      // all initializations here
        sub_pose.subscribe(n, "/robot/pose", 1);
        sub_wheels.subscribe(n, "/wheel_states",1);
        
        sync.reset(new Sync(MySyncPolicy(10), sub_pose, sub_wheels));
        sync->registerCallback(&Calibrator::Callback, this);
                
        this->output_publisher = this->n.advertise<project1::ParamMsg>("/est_params", 1);

        //inizialitation of class variables
        this->x_past = 0;
        this->y_past = 0;
        this->theta_past = 0;   
        for(int i = 0; i<4; i++)
            this->tick_past[i] = 0; 


        ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
    };
    
    void main_loop(){
        sleep(1.0);
        ros::Rate rate(10);
        
        ROS_INFO("Node %s running.", ros::this_node::getName().c_str());
        
        while(ros::ok()){
            
            ros::spinOnce();
            rate.sleep();
            
        };
        
    };
    
};
    



int main(int argc, char **argv){
    ros::init(argc, argv, "calibrator");
    
    Calibrator my_calibrator;
    
    my_calibrator.main_loop();
    
    ROS_INFO("node calibrator is shutting down");

  return 0;

}
    