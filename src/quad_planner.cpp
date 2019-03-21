#include "quad_rrt_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_rrt_planner_node");
    ros::NodeHandle n;
    std::string sampling_param;
	// Load sampling type from parameter server
	if (n.getParam("/sampling_type", sampling_param))
		std::cout<<"Sampling type:="<<sampling_param<<std::endl;
	
    planner planner_object(n);
    std::cout<<"Planner Initialized"<<std::endl;
    ros::spin();
    return 0;
}
