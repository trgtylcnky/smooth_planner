#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>


namespace global_planner_turgut
{
	struct node
	{
	    int parent_id;

	    std::vector<int> children_ids;

	    float result_x;
	    float result_y;
	    float result_theta;

	    float cost;
	    float potential;


	    bool end;

	    int id;

	};
	#define NUM_OF_PRIMS 8
	/*
	const float prim_dist[NUM_OF_PRIMS] = {0.05, 0.05, 0.05,  0, 0};
	const float prim_theta[NUM_OF_PRIMS] = {0, -M_PI/12, M_PI/12,  -M_PI/6, M_PI/6};
	const float prim_cost[NUM_OF_PRIMS] = {0.05, 0.06, 0.06,  10.0, 10.0 };
	*/

	const float prim_dist[NUM_OF_PRIMS] = {0.1, 0.1, 0.1, -0.1, -0.1, -0.1, 0, 0};
	const float prim_theta[NUM_OF_PRIMS] = {0, -M_PI/15, M_PI/15, 0, -M_PI/15, M_PI/15, -M_PI/12, M_PI/12};
	const float prim_cost[NUM_OF_PRIMS] = {0.1, 0.12, 0.12, 0.10, 0.12, 0.12, 10, 10};


	class Tree
	{

	    std::vector<node> node_vector;


	    int final_node;

	    int path_point;

	    float map_width;
	    float map_height;

	    float map_origin_x;
	    float map_origin_y;

	    float grid_resolution_xy;
	    float grid_resolution_theta;

	    int grid_width_x;
	    int grid_width_y;
	    int grid_width_theta;

	    float turn_radius;
	    float wall_clearance;

	    float goal_thresh_trans;
	    float goal_thresh_rot;

	    float pursue_gain;
	    float angle_gain;
	    float guide_gain;

	    float best_found_angle;

	    std::vector<std::vector<std::vector<int> > > visited_map;

	    std::vector<std::vector<int> > nav_grid;


	public:

	    std::vector<std::pair<float, float> > grid_road;

	    costmap_2d::Costmap2D* costmap_;

	    Tree();

	    void set_grid_resolution(float, float );
	    void set_turn_radius(float);
	    void set_wall_clearance(float);
	    void initialize(costmap_2d::Costmap2D*);
	    void grid_astar(const geometry_msgs::Pose &, const geometry_msgs::Pose &);
	    void set_grid_road_from_path(const std::vector<geometry_msgs::PoseStamped> &);
	    void init_starting_pose(geometry_msgs::Pose);
	    void set_visited_map(float, float, float, int);
	    int get_visited_map(float, float, float);
	    float find_closest_wall_distance(float, float, float);
	    int expand_node(int, const geometry_msgs::PoseStamped &);
	    void remove_children(int );
	    void create_road_to_final(std::vector<geometry_msgs::PoseStamped> &);
	    int find_best_end();
	    void create_line_list( visualization_msgs::Marker &);
	};
}