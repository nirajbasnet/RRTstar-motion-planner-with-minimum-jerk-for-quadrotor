#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>

#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

int sampling_type=0;
int sampling_param, default_sampling_param;

// Define Sampling Technique
ob::ValidStateSamplerPtr allocUniformSampler(const ob::SpaceInformation *si)
{
	std::cout<<"\n***** Using Uniform Sampling *****"<<std::endl;
	return std::make_shared<ob::UniformValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocObstacleSampler(const ob::SpaceInformation *si)
{
	std::cout<<"\n***** Using Obstacle Based Sampling *****"<<std::endl;
	return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocGaussianSampler(const ob::SpaceInformation *si)
{
	std::cout<<"\n***** Using Gaussian Sampling *****"<<std::endl;
	return std::make_shared<ob::GaussianValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocMaximizeClearanceSampler(const ob::SpaceInformation *si)
{
	std::cout<<"\n***** Using Maximize Clearance Valid State Sampling *****"<<std::endl;
	return std::make_shared<ob::MaximizeClearanceValidStateSampler>(si);
}


//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;

octomap::OcTree temp_tree(0.1);


class planner {
public:
	void init_start(void)
	{
		if(!set_start)
			std::cout << "Initialized" << std::endl;
		set_start = true;
	}
	void setStart(double x, double y, double z)
	{
		ob::ScopedState<ob::SE3StateSpace> start(space);
		start->setXYZ(x,y,z);
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		pdef->clearStartStates();
		pdef->addStartState(start);
	}
	
	void updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
	{
		tree_obj = map;
	}
	// Constructor
	planner(void)
	{  
		Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.35, 0.35, 0.35));	
		plan_index=0;
	
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

		start->setXYZ(-7,-5,1);  //-7,-5,1
		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

		// start.random();

		goal->setXYZ(-2,4.5,1.5);   //7.22,-1.86,1.5
		prev_goal[0] = 0;
		prev_goal[1] = 0;
		prev_goal[2] = 0;
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		// goal.random();
		
	    // set state validity checking for this space
		si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1 ));
		// set sampling technique
		si->setValidStateSamplerAllocator(allocUniformSampler);

		si->setStateValidityCheckingResolution(0.005);

        og::SimpleSetup ss(si);
		ss.setStartAndGoalStates(start,goal);
		ss.setOptimizationObjective(planner::getThresholdPathLengthObj(si));
		

		// create a problem instance
		pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

		// set the start and goal states
		pdef->setStartAndGoalStates(start, goal);

	    // set Optimizattion objective
		//pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));
        pdef->setOptimizationObjective(planner::getThresholdPathLengthObj(si));
		

    //loading octree from binary. Do rosservice call to make octomap bt file before doing this.
		const std::string filename = "/home/niraj/quad_world_octomap.bt";
		//octomap::OcTree temp_tree(0.1);
		temp_tree.readBinary(filename);
		tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
		// // Update the octree used for collision checking
		updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));

		std::cout << "Initialized: " << std::endl;
		benchmark_project(ss);

	}
	// Destructor
	~planner()
	{
		delete tree;
	}


	void optionalPreRunEvent(const ob::PlannerPtr &plansys )
 	{
		 if(plansys->getName()=="uniform")
		 {
			 si->setValidStateSamplerAllocator(allocUniformSampler);
		 }
		 else  if(plansys->getName()=="obstacle")
		 {
			 si->setValidStateSamplerAllocator(allocObstacleSampler);
		 }
		 else  if(plansys->getName()=="gaussian")
		 {
			 si->setValidStateSamplerAllocator(allocGaussianSampler);
		 }
		 else  if(plansys->getName()=="maxclearance")
		 {
			 si->setValidStateSamplerAllocator(allocMaximizeClearanceSampler);
		 }
    }
 
    void optionalPostRunEvent(const ob::PlannerPtr &plansys , ompl::tools::Benchmark::RunProperties &run )
	{
 	}


    void benchmark_project(og::SimpleSetup &ss)
	{
		

		std::cout << "Initialized benchmarking: " << std::endl;

		og::InformedRRTstar* pRRT1= new og::InformedRRTstar(si);
		pRRT1->setRange(0.5);
		pRRT1->setNumSamplingAttempts(100000);
		pRRT1->setRewireFactor(0.05);
		pRRT1->setTreePruning(true);
		
		og::InformedRRTstar* pRRT2= new og::InformedRRTstar(si);
		pRRT2->setRange(0.5);
		pRRT2->setNumSamplingAttempts(100000);
		pRRT2->setRewireFactor(0.05);
		pRRT2->setTreePruning(true);

		og::InformedRRTstar* pRRT3= new og::InformedRRTstar(si);
		pRRT3->setRange(0.5);
		pRRT3->setNumSamplingAttempts(100000);
		pRRT3->setRewireFactor(0.05);
		pRRT3->setTreePruning(true);

		og::InformedRRTstar* pRRT4= new og::InformedRRTstar(si);
		pRRT4->setRange(0.5);
		pRRT4->setNumSamplingAttempts(100000);
		pRRT4->setRewireFactor(0.075);
		pRRT4->setTreePruning(true);
		ob::PlannerPtr plan1(pRRT1); 
		plan1->setName("uniform");     
		ob::PlannerPtr plan2(pRRT2);
		plan2->setName("obstacle");
		ob::PlannerPtr plan3(pRRT3);
		plan3->setName("gaussian");
		ob::PlannerPtr plan4(pRRT4);
		plan4->setName("maxclearance");


		// First we create a benchmark class:
        ompl::tools::Benchmark bench(ss, "Quadcopter path planning");


	
		bench.addPlanner(plan1);
		bench.addPlanner(plan2);
		bench.addPlanner(plan3);
		bench.addPlanner(plan4);

		ompl::tools::Benchmark::Request req;
	    bench.setPreRunEvent(std::bind(&planner::optionalPreRunEvent,this, std::placeholders::_1));
		bench.setPostRunEvent(std::bind(&planner::optionalPostRunEvent,this, std::placeholders::_1, std::placeholders::_2));
		req.maxTime = 10.0;
		req.maxMem = 3000.0;
		req.runCount = 10;
		req.displayProgress = true;
		
		std::cout << "Benchmark started: " << std::endl;
		bench.benchmark(req);
		bench.saveResultsToFile();
	}

    void setGoal(double x, double y, double z)
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
		
			
		}
	}

	void plan(void)
	{ 

	    // create a planner for the defined space
		og::InformedRRTstar* pRRT= new og::InformedRRTstar(si);
        
		ob::PlannerPtr plan(pRRT);
	    // set the problem we are trying to solve for the planner
		plan->setProblemDefinition(pdef);

	    // perform setup steps for the planner
		plan->setup();
        std::cout<<"Current planner range="<<pRRT->getRange()<<std::endl;
		pRRT->setRange(0.5);
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
private:
    
	// construct the state space we are planning in
	ob::StateSpacePtr space;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;
	ob::SpaceInformationPtr si11;
	ob::SpaceInformationPtr si22;
	ob::SpaceInformationPtr si33;
	ob::SpaceInformationPtr si44;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

	// goal state
	double prev_goal[3];

	int plan_index;

	og::PathGeometric* path_smooth = NULL;

	bool replan_flag = false;

	std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

	std::shared_ptr<fcl::CollisionGeometry> tree_obj;
	fcl::OcTree* tree;
	//octomap::OcTree temp_tree(0.1);//ADDED

	// Flag for initialization
	bool set_start = false;

	bool isStateValid(const ob::State *state)
	{	
		//std::cout<<"Checking state validity"<<std::endl;
		
	    // cast the abstract state type to the type we expect
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

	// Returns a structure representing the optimization objective to use
	// for optimal motion planning. This method returns an objective which
	// attempts to minimize the length in configuration space of computed
	// paths.
	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostThreshold(ob::Cost(25));
		return obj;
	}

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
		return obj;
	}

};


void odomCb(const nav_msgs::Odometry::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	planner_ptr->init_start();
}

void startCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z);
	planner_ptr->init_start();
}

void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_node");
    ros::NodeHandle n;

	// Load sampling type from parameter server
	if (n.getParam("/sampling_type", sampling_param))
	{
		sampling_type = sampling_param;
	}
	else
	{
		sampling_type = 0;
	}

    planner planner_object;
    std::cout<<"Planner Initialized"<<std::endl;
    // ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/ardrone/odometry_sensor1/odometry", 1, boost::bind(&odomCb, _1, &planner_object));
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));
   // ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/start/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));

    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
    ros::spin();
    return 0;
}
