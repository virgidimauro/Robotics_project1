#include "vel/vel.h"

int main (int argc, char **argv) {
	ros::init(argc,argv,NODE_NAME);
	vel vel_node;
	vel_node.Data();
	vel_node.RunPeriod();
	vel_node.Stop();

	return (0);
}