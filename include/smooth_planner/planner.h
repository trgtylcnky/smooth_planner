#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include "dynamic_reconfigure/server.h"
#include "smooth_planner/SmoothPlannerConfig.h"
#include "Tree.h"

using std::string;

#ifndef PLANNER_H
#define PLANNER_H

namespace smooth_planner {

class SmoothPlanner : public nav_core::BaseGlobalPlanner {

	
	costmap_2d::Costmap2D* costmap_;
	ros::Publisher plan_pub_;
	ros::Publisher marker_pub;
	ros::Publisher plan_pub_2;
	ros::Publisher marker_pub2;

	int max_iterations;

	Tree tree;

	dynamic_reconfigure::Server<smooth_planner::SmoothPlannerConfig> *reconfigure_server;
	boost::recursive_mutex configuration_mutex_;

public:

 SmoothPlanner();
 ~SmoothPlanner();

 SmoothPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

 /** overridden classes from interface nav_core::BaseGlobalPlanner **/
 void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
 bool makePlan(const geometry_msgs::PoseStamped& start,
               const geometry_msgs::PoseStamped& goal,
               std::vector<geometry_msgs::PoseStamped>& plan
              );
 

 void dynamic_reconfigure_callback(smooth_planner::SmoothPlannerConfig &, uint32_t);
 };
};
#endif