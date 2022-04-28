#include "odom/odom.h"

int main(int argc, char **argv){
	ros::init(argc, argv, NODE_NAME);

	odom odom_node;

	odom_node.Data();

	odom_node.RunPeriod();

	odom_node.Stop();

	return(0);
}