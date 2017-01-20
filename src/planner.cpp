#include <pluginlib/class_list_macros.h>
#include "global_planner_turgut/planner.h"


PLUGINLIB_EXPORT_CLASS(global_planner_turgut::GlobalPlannerTurgut, nav_core::BaseGlobalPlanner)

using namespace std;

namespace global_planner_turgut
{


	




	GlobalPlannerTurgut::GlobalPlannerTurgut()
	{

	}

	GlobalPlannerTurgut::GlobalPlannerTurgut(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		
		initialize(name, costmap_ros);
	}

	void GlobalPlannerTurgut::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		ros::NodeHandle nh;
		costmap_ = costmap_ros->getCostmap();

		marker_pub = nh.advertise<visualization_msgs::Marker>("tree_marker", 10);

		plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 10);

		marker_pub2 = nh.advertise<visualization_msgs::Marker>("tree_marker2", 10);

		plan_pub_2 = nh.advertise<nav_msgs::Path>("plan2", 10);


	}

	bool GlobalPlannerTurgut::makePlan(
		const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped &goal, 
		std::vector<geometry_msgs::PoseStamped>& plan )
	{

		ros::NodeHandle nh;

		
		
	
		tree.init_starting_pose(start.pose);
		

		
		tree.costmap_ = costmap_;
		
		tree.grid_astar(start.pose, goal.pose);

		
		visualization_msgs::Marker tree_marker;
		tree_marker.header.frame_id = "/map";
		tree_marker.type = visualization_msgs::Marker::LINE_LIST;
		tree_marker.scale.x = 0.002;
		tree_marker.color.b = 1.0f;
		tree_marker.color.r = 1.0f;
		tree_marker.color.a = 0.7f;

		/*

		marker_pub.publish(tree_marker);
*/
		nav_msgs::Path gui_path2;
		gui_path2.poses.resize(tree.grid_road.size());

		gui_path2.header.frame_id = "/map";
		gui_path2.header.stamp = ros::Time::now();

		    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
		for (unsigned int i = 0; i < tree.grid_road.size(); i++) {
			gui_path2.poses[i].header.stamp = ros::Time::now();
			gui_path2.poses[i].header.frame_id = "/map";
			gui_path2.poses[i].pose.position.x = tree.grid_road[i].first;
			gui_path2.poses[i].pose.position.y = tree.grid_road[i].second;
			gui_path2.poses[i].pose.position.z = 0;
			gui_path2.poses[i].pose.orientation.x = 0;
			gui_path2.poses[i].pose.orientation.y = 0;
			gui_path2.poses[i].pose.orientation.z = 0;
			gui_path2.poses[i].pose.orientation.w = 1;


		}

		plan_pub_2.publish(gui_path2);


		int best_start_to_end ;
		
		int lim = 0;
		char stat = 0;
		while(lim++<5000 )
		{
			
			best_start_to_end = tree.find_best_end(goal);

			

			int v = tree.expand_node(best_start_to_end, goal);
			

			if(v == -1 ) {
				stat = -1;
				break;
			}
			else if(v == 0) continue;
			else if(v == 1)
			{
				stat = 1;
				break;
			}
		}


		
		tree.create_road_to_final(plan);

		tree.create_line_list(tree_marker);
		
		marker_pub.publish(tree_marker);


		nav_msgs::Path gui_path;
		gui_path.poses.resize(plan.size());

		gui_path.header.frame_id = "/map";
		gui_path.header.stamp = ros::Time::now();

		    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
		for (unsigned int i = 0; i < plan.size(); i++) {
			gui_path.poses[i] = plan[i];
		}

		plan_pub_.publish(gui_path);


/*
		Tree t2;
		t2.init_starting_pose(start.pose);

		t2.costmap_ = costmap_;
		
		


		lim = 0;
		while(lim++<50000 && t2.expand_best_end(goal, plan) );

		

		



		t2.create_line_list(tree_marker2);

		*/
		


		if(stat == -1 || stat == 0) return false;
		else return true;
		

	}



};