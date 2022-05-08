#include <ros/ros.h>
                  //#include <std_msgs/Time.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
               // #include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/parameters_calibrationConfig.h>

/* 		
		This node estimates the linear and the angular velocity of the robot and publishes them in the 
		topic /cmd_vel as TwistStamped messages. 
    It listens to the /wheel_states topic published by the bags (sensor_msgs/JointState messages).
*/

class vel{ //header of the class

  private:

    //ROS
    ros::NodeHandle n;

    /* ROS topics */
    ros::Publisher pub;
    ros::Subscriber sub;

    /* Node state variables */
    double last_time;
    float last_ticks[4] = {0, 0, 0, 0};
    double vel_xyz[3];
    double l, w, r, N, T;

    /*dyn reconfig server*/
    dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig> dynServer;

  public:

    vel(){ //constructor

      last_time = 0;

      sub = n.subscribe("/wheel_states", 1, &vel::CalcluateVelocityCallback, this);
      pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

      /*dynamic reconfigure*/
      dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig>::CallbackType f;
      f = boost::bind(&vel::parameters_Callback, this, _1, _2);
      this->dynServer.setCallback(f);

    }

    void update_values(const sensor_msgs::JointState& wheels_msg){

      last_time = ros::Time::now().toSec();

      /*reads msg and stores info for each wheel*/
      for (int i = 0; i < 4; i++) {
        last_ticks[i] = wheels_msg.position[i];
      }
      
    }
    
    void parameters_Callback(Robotics_project1::parameters_calibrationConfig &config,uint32_t level){

      //callback of the dynamic reconfigure server to set the parameters of the robot

      //ROS_INFO("Requested to reconfigure: l=%f, w=%f, r=%f, T=%d, N=%d - Level %d", config.l, config.w, config.r, config.T, config.N, level);
        this->l = config.l;
        this->w = config.w;
        this->r = config.r;
        this->T = config.T;
        this->N = config.N;
    };

    void vel_computation_Callback(const sensor_msgs::JointState& wheels_msg){

      //Computation not done at the beginning of the bag
      if(last_time != 0)
      {
        //computation of the velocity of the wheels from the encoder ticks and then the velocity of the robot
        float wheels_velocity[4];
        for (int i = 0; i < 4; i++) {
          wheels_velocity[i] = 2 * 3.14159 * ((wheels_msg.position[i] - last_ticks[i])/ (ros::Time::now().toSec() - last_time)) / N / T;
        }
        double matrix_fromwheelveltovel_xyz[3][4]={{r/4, r/4, r/4, r/4},{-r/4,r/4,r/4,-r/4},{(-r/4)/(l+w),(r/4)/(l+w),-(r/4)/(l+w),(r/4)/(l+w)}}; 
        for(int i=0;i<3;i++)
          vel_xyz[i]=0;
        for(int i=0;i<3;i++){
          for(int j=0;j<4;j++){
            this->vel_xyz[i]=this->vel_xyz[i]+matrix_fromwheelveltovel_xyz[i][j]*wheels_velocity[j];
          };
        };

        //Setup message out
        geometry_msgs::TwistStamped cmd_vel = geometry_msgs::TwistStamped();
        cmd_vel.header.seq = wheels_msg.header.seq;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = "base_link";
        cmd_vel.twist.linear.x = vel_xyz[0];
        cmd_vel.twist.linear.y = vel_xyz[1];
        cmd_vel.twist.angular.z = vel_xyz[2];

        //Publishing v and omega as topic cmd_vel
        pub.publish(cmd_vel);
      }
        update_values(wheels_msg);
    };
  };

  int main(int argc, char **argv){

    ros::init(argc, argv, "vel");
    vel vel_node;
    ros::spin();
  }