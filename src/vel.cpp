#include <ros/ros.h>
                  #include <std_msgs/Time.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
                #include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/parameters_calibrationConfig.h>

//  //======== PARAMETERS CALIBRATION =========
//#include <dynamic_reconfigure/server.h>
//#include <project1/parametersConfig.h>
//  //=========================================


class vel{

// ================ ATTRIBUTES ================
private:

  //ROS
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;

  //Calculations
  double last_time;
  float last_ticks[4] = {0, 0, 0, 0};
  double vel_xyz[3];
  double l, w, r, N, T;
  dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig> dynServer;

  //Utility
  //const int noise_remover = 2;
  //int skip_counter = 0;
  //float ticks_avg[4];
  //float r, l, w, N; //PARAMETERS RECALIBRATION

// =============== CONSTRUCTOR ===============
public:

  vel(){

    last_time = 0;

    sub = n.subscribe("/wheel_states", 1, &vel::CalcluateVelocityCallback, this);
    pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

    /*dynamic reconfigure*/
    dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig>::CallbackType f;
    f = boost::bind(&vel::parameters_Callback, this, _1, _2);
    this->dynServer.setCallback(f);

  }



// ================ METHODS ==================

  //void ResetTickAvg(){
  //  for(int i = 0; i < sizeof(ticks_avg)/sizeof(float); i++)
  //    ticks_avg[i] = 0;
  //}

  void UpdateLastValues(const sensor_msgs::JointState& wheels_msg){

    last_time = ros::Time::now().toSec();

    for (int i = 0; i < 4; i++) {
      last_ticks[i] = wheels_msg.position[i];
    }
    
  }

  //float CalculateVelocityFromTickDelta(float deltaTick, double deltaTime){
  //  return (2 * 3.14159 * (deltaTick / deltaTime) / N_ENCODER) / GEAR_RATIO;
  //}



// ================ CALLBACK ==================

  
  void parameters_Callback(Robotics_project1::parameters_calibrationConfig &config,uint32_t level){

    //callback of the dynamic reconfigure server to set the parameters of the robot

    //ROS_INFO("Requested to reconfigure: l=%f, w=%f, r=%f, T=%d, N=%d - Level %d", config.l, config.w, config.r, config.T, config.N, level);
      this->l = config.l;
      this->w = config.w;
      this->r = config.r;
      this->T = config.T;
      this->N = config.N;
  };

  void CalcluateVelocityCallback(const sensor_msgs::JointState& wheels_msg){

    // ============= PARAMETERS CALIBRATION =============
    // Sappiamo che è sconsigliato utilizzare il getParam all'interno dei callback
    // ma siccome e una parte di codice che abbiamo dovuto commentare e scommentare 
    // spesso abbiamo dato più importanza alla compattezza e alla leggibilità,
    // piuttosto che alle good practices, considerando anche il fatto che nella prodotto
    // finale queste righe sarebbero state rimosse
    //nh.getParam("/dynamic_reconfigure/r", r);
    //nh.getParam("/dynamic_reconfigure/l", l);
    //nh.getParam("/dynamic_reconfigure/w", w);
    //nh.getParam("/dynamic_reconfigure/N", N);
    //ROS_INFO("%f, %f, %f, %f", r, l, w, N);
    // ==================================================

    //============== Decrease Noise (unused) ============
    //Rimosso in quanto induce un ritardo indesiderato
    //if(skip_counter < noise_remover)
    //{
    //  for(int i = 0; i < sizeof(ticks_avg)/sizeof(float); i++)
    //    ticks_avg[i] = (ticks_avg[i] * skip_counter + msg_in.position[i]) / (skip_counter + 1); //avg di <noise_remover> ticks -> leggero smoothing
    //
    //  skip_counter++;
    //  return;
    //}
    //ResetTickAvg();
    //skip_counter = 0;
    //===================================================


    //Skipped only at the beginning of the bag
    if(last_time != 0)
    {
      //Calculate wheels angular velocities - Wheels order: fl, fr, rl, rr
      float wheels_velocity[4];
      for (int i = 0; i < 4; i++) {
        wheels_velocity[i] = 2 * 3.14159 * ((wheels_msg.position[i] - last_ticks[i])/ (ros::Time::now().toSec() - last_time)) / N / T;
        //CalculateVelocityFromTickDelta(msg_in.position[i] - last_ticks[i], ros::Time::now().toSec() - last_time);
        //wheels_velocity[i] = msg_in.velocity[i] / GEAR_RATE / 60; //SOLO DEBUG
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
      //cmd_vel.header.frame_id = wheels_msg.header.frame_id;
      cmd_vel.header.frame_id = "base_link";
      cmd_vel.twist.linear.x = vel_xyz[0];
      cmd_vel.twist.linear.y = vel_xyz[1];
      cmd_vel.twist.angular.z = vel_xyz[2];

      //Publish message out
      pub.publish(cmd_vel);

      //ROS_INFO("Vx: %f, Vy: %f, omega: %f", vx, vy, omega);
    }

      UpdateLastValues(wheels_msg);
  }
};

  int main(int argc, char **argv){

    ros::init(argc, argv, "vel");
    vel vel_node;
    //ROS_INFO("VELOCITY NODE");
    ros::spin();
  }