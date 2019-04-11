//
// Created by niraj on 2/18/2019.
//

#include "quad_rrt_planner.h"

octomap::OcTree temp_tree(0.1);

void planner::init_start(void)
{
    if(!set_start)
        std::cout << "Initialized" << std::endl;
    set_start = true;
}
void planner::setStart(double x, double y, double z)
{
    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setXYZ(x,y,z);
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    pdef->clearStartStates();
    pdef->addStartState(start);
}
void planner::setGoal(double x, double y, double z)
{
    if(prev_goal[0] != x || prev_goal[1] != y || prev_goal[2] != z)
    {
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(x,y,z);
        prev_goal[0] = x;
        prev_goal[1] = y;
        prev_goal[2] = z;
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        pdef->clearGoal();
        pdef->setGoalState(goal);
        std::cout << "Goal point set to: " << x << " " << y << " " << z << std::endl;
        if(set_start)
            plan();
        
    }
}
ob::ValidStateSamplerPtr planner::allocOBValidStateSampler(const ob::SpaceInformation *si)
{
std::cout<<"sampler select"<<std::endl;
if (sampling_type=="uniform")
{   
	std::cout<<"Using Uniform Sampling"<<std::endl;
	return std::make_shared<ob::UniformValidStateSampler>(si);
}

else if(sampling_type=="obstacle")
{
	std::cout<<"Using Obstacle Based Sampling"<<std::endl;
	return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}
else if(sampling_type=="gaussian")
{
	std::cout<<"Using Gaussian Sampling"<<std::endl;
	return std::make_shared<ob::GaussianValidStateSampler>(si);
}
else if(sampling_type=="maxclearance")
{
	std::cout<<"Using Maximize Clearance Valid State Sampling"<<std::endl;
	return std::make_shared<ob::MaximizeClearanceValidStateSampler>(si);
}
else
{
	std::cout<<"Sampling Parameter not in {0,1,2}"<<std::endl;
	std::cout<<"Setting sampling type to Uniform\n";
	return std::make_shared<ob::UniformValidStateSampler>(si);
}
}

// Constructor
planner::planner(const ros::NodeHandle& n)
{
    nh=n;
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
    octomap_pub = nh.advertise<octomap_msgs::Octomap>("world/octomap", 1,true);

    octree_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1,&planner::octomapCallback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/ardrone/odometry_sensor1/odometry", 1, &planner::odomCb, this);
    goal_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &planner::goalCb, this);

    init_ros_params();

    Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.35, 0.35, 0.35));
    space = ob::StateSpacePtr(new ob::SE3StateSpace());

    // create a start state
    ob::ScopedState<ob::SE3StateSpace> start(space);
    
    // create a goal state
    ob::ScopedState<ob::SE3StateSpace> goal(space);

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);

    bounds.setLow(0,-10);
    bounds.setHigh(0,9);
    bounds.setLow(1,-7);
    bounds.setHigh(1,7);
    bounds.setLow(2,0);
    bounds.setHigh(2,2.5);

    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    start->setXYZ(7.22,-1.86,1.6);
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    // start.random();

    goal->setXYZ(-7,-5,1);
    prev_goal[0] = 0;
    prev_goal[1] = 0;
    prev_goal[2] = 0;
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    // goal.random();

    // set state validity checking for this space
    si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1 ));
    si->setStateValidityCheckingResolution(0.005);

    // create a problem instance
    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // set Optimizattion objective
    //pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));
    pdef->setOptimizationObjective(planner::getThresholdPathLengthObj(si));
    loadOctomapfile();
    std::cout << "Initialized: " << std::endl;

}

// Destructor
planner::~planner()
{
    delete tree;
}
void planner::init_ros_params(void)
{
    std::string sampling_param;
    bool display_param;
	// Load sampling type from parameter server
	if (nh.getParam("/sampling_type", sampling_param))
    {
        sampling_type=sampling_param;
		std::cout<<"Sampling type:="<<sampling_param<<std::endl;
    }
    if (nh.getParam("/display_octomap", display_param))
	{
        publish_octomap=display_param;
        std::cout<<"display_octomap:="<<display_param<<std::endl;
    }

}
void planner::loadOctomapfile(void)
{
    //loading octree from binary. Do rosservice call to make octomap bt file before doing this.
    std::string file_path = ros::package::getPath("path_planning")+"/octomap/";
    const std::string filename = "quad_world_octomap.bt";
    file_path=file_path+filename;
    
    //octomap::OcTree temp_tree(0.1);
    temp_tree.readBinary(file_path);
    tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
    // // Update the octree used for collision checking
    updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
    if(publish_octomap)
    {
        ros::Time begin = ros::Time::now();
        octomap_msgs::Octomap octo_map;
        octo_map.header.frame_id = "world";
        octo_map.header.stamp = ros::Time(begin.toSec(), begin.toNSec());
        if (!octomap_msgs::binaryMapToMsg(temp_tree, octo_map)) {
        ROS_ERROR("Error serializing OctoMap");
        }
        octomap_pub.publish(octo_map);
    }
}

void planner::replan(void)
{
    if(path_smooth != NULL && set_start)
    {
        std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
        if(path_smooth->getStateCount () <= 2)
            plan();
        else
        {
            for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
            {
                if(!replan_flag)
                    replan_flag = !isStateValid(path_smooth->getState(idx));
                else
                    break;

            }
            if(replan_flag)
                plan();
            else
                std::cout << "Replanning not required" << std::endl;
        }
    }
}

void planner::plan(void)
{
    // set sampling technique
  //  si->setValidStateSamplerAllocator(planner::allocOBValidStateSampler);
    // create a planner for the defined space
    og::InformedRRTstar* pRRT= new og::InformedRRTstar(si);
    
    ob::PlannerPtr plan(pRRT);
    // set the problem we are trying to solve for the planner
    plan->setProblemDefinition(pdef);

    // perform setup steps for the planner
    plan->setup();
    std::cout<<"Current planner range="<<pRRT->getRange()<<std::endl;
    pRRT->setRange(0.75);
    std::cout<<"Current planner range="<<pRRT->getRange()<<std::endl;
    // print the settings for this space
    si->printSettings(std::cout);
    //pRRT->setKNearest(true);
    //pRRT->setTreePruning(true);
    pRRT->setNumSamplingAttempts(1000000);
    pRRT->setRewireFactor(0.01);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = plan->solve(30);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        std::cout << "Found solution:" << std::endl;
        std::cout<<"Best solution cost"<< pRRT->bestCost()<<std::endl;
        ob::PathPtr path = pdef->getSolutionPath();
        og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
        pth->printAsMatrix(std::cout);
        // print the path to screen
        // path->print(std::cout);
        trajectory_msgs::MultiDOFJointTrajectory msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.joint_names.clear();
        msg.points.clear();
        msg.joint_names.push_back("Quadcopter");
        
        for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
        {
            const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
            const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            point_msg.time_from_start.fromSec(ros::Time::now().toSec());
            point_msg.transforms.resize(1);

            point_msg.transforms[0].translation.x= pos->values[0];
            point_msg.transforms[0].translation.y = pos->values[1];
            point_msg.transforms[0].translation.z = pos->values[2];

            point_msg.transforms[0].rotation.x = rot->x;
            point_msg.transforms[0].rotation.y = rot->y;
            point_msg.transforms[0].rotation.z = rot->z;
            point_msg.transforms[0].rotation.w = rot->w;

            msg.points.push_back(point_msg);


        }
        traj_pub.publish(msg);

        //Path smoothing using bspline
        og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
        path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
        pathBSpline->smoothBSpline(*path_smooth,3);
        // std::cout << "Smoothed Path" << std::endl;
        // path_smooth.print(std::cout);

        //Publish path as markers
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        vis_pub.publish(marker);

        for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
        {
                // cast the abstract state type to the type we expect
            const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
            const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
            
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            marker.ns = "path";
            marker.id = idx;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = pos->values[0];
            marker.pose.position.y = pos->values[1];
            marker.pose.position.z = pos->values[2];
            marker.pose.orientation.x = rot->x;
            marker.pose.orientation.y = rot->y;
            marker.pose.orientation.z = rot->z;
            marker.pose.orientation.w = rot->w;
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            vis_pub.publish(marker);
            ros::Duration(0.1).sleep();
            //std::cout << "Published marker: " << idx << std::endl;  
        }
        
        // Clear memory
        pdef->clearSolutionPaths();
        replan_flag = false;

    }
    else
        std::cout << "No solution found" << std::endl;
}
bool planner::isStateValid(const ob::State *state)
	{	
		//Checking state validity
		
		const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

	    // extract the first component of the state and cast it to what we expect
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	    // extract the second component of the state and cast it to what we expect
		const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

		fcl::CollisionObject treeObj(tree_obj);  // PROBLEM HERE
		fcl::CollisionObject aircraftObject(Quadcopter);

		//std::cout<<pos->values[0]<<" "<<pos->values[1]<<" "<<pos->values[2]<<std::endl;
		
        
	    // check validity of state defined by pos & rot
		fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
		fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
		aircraftObject.setTransform(rotation, translation);
		fcl::CollisionRequest requestType(1,false,1,false);
		fcl::CollisionResult collisionResult;
		
		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
		//std::cout<<!collisionResult.isCollision()<<" ***"<<std::endl;
		return(!collisionResult.isCollision());
	return true;
	}
void planner::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	
	// Update the octree used for collision checking
	updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
	replan();
}

void planner::odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	init_start();
}

void planner::goalCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	setGoal(msg->point.x, msg->point.y, msg->point.z);
}