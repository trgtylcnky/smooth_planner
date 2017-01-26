#include "global_planner_turgut/Tree.h"

namespace global_planner_turgut
{
	Tree::Tree()
	{
		final_node = 0;

		best_found_angle = 10;

		node starting_node;

		wall_clearance = 0.05;

		pursue_gain = 0.06;
		angle_gain = 0.3;
		guide_gain = 0.1;

		zig_zag_cost = 0.1;

		starting_node.parent_id = -1;
		starting_node.cost = 0;
		starting_node.result_theta = 0;
		starting_node.result_x = 0;
		starting_node.result_y = 0;
		starting_node.id = 0;

		starting_node.end = false;

		
		node_vector.push_back(starting_node);


		goal_thresh_trans = 0.05;
		goal_thresh_rot = 0.2;

		path_point = 0;

		starting_node.parent_id = -1;
		starting_node.cost = 0;
		starting_node.result_theta = 0;
		starting_node.result_x = 0;
		starting_node.result_y = 0;
		starting_node.id = 0;

		starting_node.end = false;
	}


	void Tree::set_grid_resolution(float grid_res_xy, float grid_res_theta)
	{
		grid_resolution_xy = grid_res_xy;
		grid_resolution_theta = grid_res_theta;
	}

	void Tree::set_turn_radius(float r)
	{
		turn_radius = r;
	}

	void Tree::set_wall_clearance(float c)
	{
		wall_clearance = c;
	}

	void Tree::set_goal_threshold(float trans, float rot)
	{
		goal_thresh_trans = trans;
		goal_thresh_rot = rot;
	}

	void Tree::set_gains(float p, float a, float g)
	{
		pursue_gain = p;
		angle_gain = a;
		guide_gain = g;
	}

	void Tree::set_zig_zag_cost(float z)
	{
		zig_zag_cost = z;
	}

	void Tree::initialize(costmap_2d::Costmap2D* costmap)
	{
		costmap_ = costmap;
		map_width = costmap_->getSizeInMetersX();
		map_height = costmap_->getSizeInMetersY();
		map_origin_x = costmap_->getOriginX();
		map_origin_y = costmap_->getOriginY();
		grid_resolution_xy = 0.05;
		grid_resolution_theta = M_PI/30;
		grid_width_x = map_width / grid_resolution_xy;
		grid_width_y = map_height / grid_resolution_xy;
		grid_width_theta = 2*M_PI/grid_resolution_theta;

		visited_map.resize(grid_width_x);
		for(int i = 0; i< grid_width_x; i++)
		{
			visited_map[i].resize(grid_width_y);
			for(int j = 0; j<grid_width_y; j++)
			{
				visited_map[i][j].resize(grid_width_theta);

				for(int k = 0; k<grid_width_theta; k++) visited_map[i][j][k] = -1;
			}
		}

		for(int i = 0; i<grid_width_x; i++)
		{
			std::vector<int> v;

			for(int j = 0; j<grid_width_y; j++) 
			{
				
				v.push_back(-1);	    			
			}
			nav_grid.push_back(v);
		}

	}




	void Tree::init_starting_pose(geometry_msgs::Pose p)
	{


		node_vector.clear();
		final_node = 0;

		node starting_node;

		

		starting_node.parent_id = -1;
		starting_node.cost = 0;
		starting_node.result_theta = 0;
		starting_node.result_x = 0;
		starting_node.result_y = 0;
		starting_node.id = 0;

		starting_node.end = false;

		
		node_vector.push_back(starting_node);

	    node_vector[0].result_theta = round(tf::getYaw(p.orientation)/(M_PI/15.0))*(M_PI/15.0);
	    node_vector[0].result_x = p.position.x;
	    node_vector[0].result_y = p.position.y;

	    for(int i = 0; i< grid_width_x; i++)
	    {
	    	for(int j = 0; j<grid_width_y; j++)
	    	{
	    		for(int k = 0; k<grid_width_theta; k++) visited_map[i][j][k] = -1;
	    	}
	    }


	    set_visited_map(node_vector[0].result_x, node_vector[0].result_y, node_vector[0].result_theta, 0);

	    node_vector[0].end = false;


	}

	void Tree::set_visited_map(float x, float y, float theta, int node_id)
	{
		int mx = (x - map_origin_x)/grid_resolution_xy;
		int my = (y - map_origin_y)/grid_resolution_xy;
		int mtheta = (theta + M_PI)/grid_resolution_theta;

		if(mx >=0 && mx < grid_width_x
			&& my >=0 && my < grid_width_y
			&& mtheta >=0 && mtheta < grid_width_theta
			)
			visited_map[mx][my][mtheta] = node_id;

	}

	int Tree::get_visited_map(float x, float y, float theta)
	{
		int mx = (x - map_origin_x)/grid_resolution_xy;
		int my = (y - map_origin_y)/grid_resolution_xy;
		int mtheta = (theta + M_PI)/grid_resolution_theta;

		//std::cout <<mx <<" "<<my << " "<<mtheta<<"\n";

		if(mx >=0 && mx < grid_width_x
			&& my >=0 && my < grid_width_y
			&& mtheta >=0 && mtheta < grid_width_theta
			)
			return visited_map[mx][my][mtheta];

		else return -1;

	}

	float Tree::find_closest_wall_distance(float x, float y, float max_search_radius)
	{
		unsigned int mx, my;

		unsigned int upright_mx, upright_my, downleft_mx, downleft_my;
		costmap_->worldToMap(x-max_search_radius, y-max_search_radius, downleft_mx, downleft_my);
		costmap_->worldToMap(x+max_search_radius, y+max_search_radius, upright_mx, upright_my);



		float closest = max_search_radius;
		for(mx = downleft_mx; mx <= upright_mx; mx++)
		{
			for(my = downleft_my; my < upright_my; my++)
			{
				if(costmap_->getCost(mx, my) >= 200)
				{
					double wx, wy;
					costmap_->mapToWorld(mx, my, wx, wy);
					float d = sqrt(pow(wx - x, 2) + pow(wy - y, 2));
					if(d < closest) closest = d;
				}
			}
		}

		return closest;

	}


	void Tree::remove_children(int node_id)
	{
		//std::cout<<"removing "<<node_id<<"\n";
		//std::cout<<"has " <<node_vector[node_id].children_ids.size()<< " children\n";
		for(int i = 0; i<node_vector[node_id].children_ids.size(); i++)
		{
			remove_children(node_vector[node_id].children_ids[i]);
		}
		node_vector[node_id].end = false;
	}

	void Tree::create_road_to_final(std::vector<geometry_msgs::PoseStamped> &path)
	{
	    path.clear();
	    int n = final_node;



	    while(1)
	    {
	    	if(n == -1) break;
	    	float node_x = node_vector[n].result_x;
	    	float node_y = node_vector[n].result_y;


	        geometry_msgs::PoseStamped p1;
	        p1.header.stamp = ros::Time::now();
	        p1.header.frame_id = "/map";
	        p1.pose.position.x = node_x;
	        p1.pose.position.y = node_y;
	        p1.pose.position.z = 0;
	        p1.pose.orientation.x = 0;
	        p1.pose.orientation.y = 0;
	        p1.pose.orientation.z = 0;
	        p1.pose.orientation.w = 1;


	        path.push_back(p1);

	        n = node_vector[n].parent_id;
	    }

	    std::reverse(path.begin(), path.end());
	}

	int Tree::find_best_end()
	{

	    float best_cost = -1000000;
	    int best_id = 0;

	    for(int i = 0; i<node_vector.size(); i++)
	    {	
	    	
	        if(node_vector[i].end) //if it is very end
	        {
	        	
	            if(node_vector[i].cost + node_vector[i].potential> best_cost) 
	            {
	                best_cost = node_vector[i].cost + node_vector[i].potential;
	                best_id = i;
	            }
	        }
	    }

	    return best_id;

	}

	void Tree::create_line_list( visualization_msgs::Marker &output)
	{
		output.points.clear();
	    for(int i = 0; i<node_vector.size(); i++)
	    {
	        geometry_msgs::Point parent_p;
	        parent_p.x = node_vector[i].result_x;
	        parent_p.y = node_vector[i].result_y;
	        parent_p.z = 0;

	        for(int j = 0; j<node_vector[i].children_ids.size(); j++)
	        {
	            geometry_msgs::Point child_p;
	            child_p.x = node_vector[node_vector[i].children_ids[j]].result_x;
	            child_p.y = node_vector[node_vector[i].children_ids[j]].result_y;
	            child_p.z = 0;

	            output.points.push_back(parent_p);
	            output.points.push_back(child_p);
	        }
	    }

	   
	}




}