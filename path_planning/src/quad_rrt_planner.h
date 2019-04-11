//
// Created by niraj on 2/18/2019.
//
#include "ros/ros.h"
#include <ros/package.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
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


#ifndef QUAD_RRT_PLANNER_H
#define QUAD_RRT_PLANNER_H

namespace ob = ompl::base;
namespace og = ompl::geometric;

class planner 
{
public:
	planner(const ros::NodeHandle& n);
	~planner();
	void init_ros_params(void);
	void init_start(void);
	void setStart(double x, double y, double z);
	void setGoal(double x, double y, double z);
	void setSamplingType(std::string samp_type){sampling_type=samp_type;}
	void loadOctomapfile(void);
	void updateMap(std::shared_ptr<fcl::CollisionGeometry> map){tree_obj = map;}
	ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si);
	void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
	void odomCb(const nav_msgs::Odometry::ConstPtr &msg);
	void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg);
	bool isStateValid(const ob::State *state);

	void replan(void);
	void plan(void);

private:
    ros::NodeHandle nh;
	ros::Publisher vis_pub;
	ros::Publisher traj_pub;
	ros::Publisher octomap_pub;

	ros::Subscriber octree_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber goal_sub;
	// construct the state space we are planning in
	ob::StateSpacePtr space;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

	// goal state
	double prev_goal[3];
	og::PathGeometric* path_smooth = NULL;
	bool replan_flag = false;
	std::shared_ptr<fcl::CollisionGeometry> Quadcopter;
	std::shared_ptr<fcl::CollisionGeometry> tree_obj;
	fcl::OcTree* tree;
	// Flag for initialization
	bool set_start = false;
	
	std::string sampling_type;
	bool publish_octomap=false;

	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostThreshold(ob::Cost(0.0));
		return obj;
	}

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
		return obj;
	}
};

#endif