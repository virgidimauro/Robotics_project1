#include "ros/ros.h"
#include "Robotics_project1/param.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class calibration {
private:
    ros::NodeHandle m;
    
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose;
    message_filters::Subscriber<sensor_msgs::JointState> sub_wheels;
    
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::JointState> MySyncPolicy;
    
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Publisher pub;
    
    double past_x, past_y, past_theta;
    double past_tick[4];
    double lw, Tr, N;
    
    ros::Time current_time, past_time;
    
    void Callback(const geometry_msgs::PoseStamped::ConstPtr& true_pose, const sensor_msgs::JointState wheels_tick) {
        
        double current_x = true_pose->pose.position.x;
        double current_y = true_pose->pose.position.y;

        double quaternion_x= true_pose->pose.orientation.x;
        double quaternion_y= true_pose->pose.orientation.y;
        double quaternion_z= true_pose->pose.orientation.z;
        double quaternion_w= true_pose->pose.orientation.w;
    
        tf2::Quaternion q(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
        tf2::Matrix3x3 mat(q);

        double roll, pitch, current_theta;
        m.getRPY(roll, pitch, current_theta);

        ROS_INFO("I heard true position [%f, %f, %f] and wheels' state [%f, %f, %f, %f]", current_x, current_y, current_theta, past_tick[0], past_tick[1], past_tick[2], past_tick[3]);

    };
    
    void compute(){
        
        
        
        
    };
    
    
    
    void publish(){
        Robotics_project1::param msg;
        msg.lw = this->lw;
        msg.Tr = this->Tr;
        msg.N = this->N;
        output_publisher.publish(msg);
    };
    
public:
    
    calibration(){ // class constructor
      // all initializations here
        sub_pose.subscribe(n, "/robot/pose", 1);
        sub_wheels.subscribe(n, "/wheel_states",1);
        
        sync.reset(new Sync(MySyncPolicy(10), sub_pose, sub_wheels));
        sync->registerCallback(&calibration::Callback, this);
                
        this->pub = this->n.advertise<Robotics_project1::ParamMsg>("/est_params", 1);

        //inizialitation of class variables
        this->past_x = 0;
        this->past_y = 0;
        this->past_theta = 0;   
        for(int i = 0; i<4; i++)
            this->past_tick[i] = 0; 


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
    ros::init(argc, argv, "calibration");
    
    calibration calibration_node;
    
    calibration_node.main_loop();
    
    ROS_INFO("node calibration is shutting down");

  return 0;

}
    