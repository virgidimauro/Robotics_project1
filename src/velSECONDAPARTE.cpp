%%%%%%%%%%%%%%%%%%%%## tutta prima parte Luci io da riga 46 %%%%%%%
void vel::Stop(void){
	ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
}

void vel::inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& wheels_msg) {
	this->current_pos[]=wheels_msg-> pos[];

	vel_computation();
}

void vel::vel_computation(void){
	this->current_time=ros::Time::now();
	double dt=(this->current_time-this->past_time).toSec();
	double tick[]=this->current_pos[]-this->past_pos[];
	double wheel_vel[4];

	wheel_vel[]=tick[]*2*3,14159/dt/N/T;
	this->vel[]=r/4*[1 1 1 1; -1 1 1 -1; -1/(l+w) 1/(l+w) -1/(l+w) 1/(l+w)]*wheel_vel[];

	this->past_time=this->current_time;
	this->past_pos=this->current_pos;
}