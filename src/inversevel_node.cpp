#include "vel/inversevel.h"

int main (int argc, char **argv) {
	ros::init(argc,argv,"inversevel");
	inversevel inversevel_node;
	inversevel_node.Data();
	inversevel_node.RunPeriod();
	inversevel_node.Stop();

	return (0);
}