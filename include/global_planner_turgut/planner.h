#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include "Tree.h"

using std::string;

#ifndef PLANNER_H
#define PLANNER_H

namespace global_planner_turgut {

class GlobalPlannerTurgut : public nav_core::BaseGlobalPlanner {

	
	costmap_2d::Costmap2D* costmap_;
	ros::Publisher plan_pub_;
	ros::Publisher marker_pub;
	ros::Publisher plan_pub_2;
	ros::Publisher marker_pub2;

	Tree tree;
public:

 GlobalPlannerTurgut();
 GlobalPlannerTurgut(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

 /** overridden classes from interface nav_core::BaseGlobalPlanner **/
 void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
 bool makePlan(const geometry_msgs::PoseStamped& start,
               const geometry_msgs::PoseStamped& goal,
               std::vector<geometry_msgs::PoseStamped>& plan
              );
 };
};
#endif