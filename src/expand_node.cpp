#include "global_planner_turgut/Tree.h"

namespace global_planner_turgut
{
	int Tree::expand_node(int node_id, const geometry_msgs::PoseStamped &goal)
	{
		
	    if(!node_vector[node_id].children_ids.empty()) return -1;

	    unsigned int xi, yi;

	    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, xi, yi);

	    if(costmap_->getCost(xi,yi) > 252){
	        std::cout << "goal in bad position, interrupted\n";
	        return -1;
	    }

	    //std::cout << "\n\nnew children appending to node " << node_id <<"\n";

	    node_vector[node_id].end = false;

	    for(int i = 0; i<NUM_OF_PRIMS; i++)
	    {

	        node child;

	        child.parent_id = node_id;
	        child.result_theta = node_vector[node_id].result_theta + prim_theta[i];

	        //angle bound correction
	        if(child.result_theta > M_PI) child.result_theta-=2*M_PI;
	        else if(child.result_theta < -M_PI) child.result_theta+=2*M_PI;


	        child.result_x = node_vector[node_id].result_x + cos(child.result_theta)*prim_dist[i];
	        child.result_y = node_vector[node_id].result_y + sin(child.result_theta)*prim_dist[i];

	        float closest_wall = find_closest_wall_distance(child.result_x, child.result_y, 2*wall_clearance);
	        if(closest_wall < wall_clearance) continue;

	        child.cost = node_vector[node_id].cost - prim_cost[i];
	        child.id=node_vector.size();

	        child.end = true;

	        float theta1 = child.result_theta - tf::getYaw(goal.pose.orientation);
	    	if(theta1 > M_PI)  theta1-=2*M_PI;
	    	else if(theta1 < -M_PI) theta1+=2*M_PI;

	        int nearest_grid_road_waypoint=-1;
	        int second_nearest_grw = -1;
	        float distance_to_nearest_grid_road_waypoint= 9999999;
	        float distance_to_sngrw = 9999999;
	        for(int p = 0; p<grid_road.size(); p++)
	        {
	        	float dis = sqrt(pow(grid_road[p].first - child.result_x, 2) + pow(grid_road[p].second - child.result_y, 2));

	        	if(dis < distance_to_nearest_grid_road_waypoint)
	        	{
	        		second_nearest_grw = nearest_grid_road_waypoint;
	        		distance_to_sngrw = distance_to_nearest_grid_road_waypoint;
	        		nearest_grid_road_waypoint = p;
	        		distance_to_nearest_grid_road_waypoint = dis;
	        	}
	        }

	        child.potential = 0;


	        float r = turn_radius;
	        float goal_yaw = tf::getYaw(goal.pose.orientation);
	        float goal_cw_x = goal.pose.position.x + r*cos(goal_yaw - M_PI/2.0);
	        float goal_cw_y = goal.pose.position.y + r*sin(goal_yaw - M_PI/2.0);
	        float goal_ccw_x = goal.pose.position.x + r*cos(goal_yaw + M_PI/2.0);
	        float goal_ccw_y = goal.pose.position.y + r*sin(goal_yaw + M_PI/2.0);

	        float robot_cw_x = child.result_x + r*cos(child.result_theta - M_PI/2.0);
	        float robot_cw_y = child.result_y + r*sin(child.result_theta - M_PI/2.0);
	        float robot_ccw_x = child.result_x + r*cos(child.result_theta + M_PI/2.0);
	        float robot_ccw_y = child.result_y + r*sin(child.result_theta + M_PI/2.0);

	        float x1_cw = cos(child.result_theta)*(robot_cw_x - goal_cw_x) + sin(child.result_theta)*(robot_cw_y - goal_cw_y);
	        float y1_cw = -sin(child.result_theta)*(robot_cw_x - goal_cw_x) + cos(child.result_theta)*(robot_cw_y - goal_cw_y);

	        float x1_ccw = cos(child.result_theta)*(robot_ccw_x - goal_ccw_x) + sin(child.result_theta)*(robot_ccw_y - goal_ccw_y);
	        float y1_ccw = -sin(child.result_theta)*(robot_ccw_x - goal_ccw_x) + cos(child.result_theta)*(robot_ccw_y - goal_ccw_y);

	        float closest = sqrt(pow(x1_cw, 2) + pow(y1_cw, 2));
	        float closest_y1 = fabs(y1_cw);
	        float closest_x1 = fabs(x1_cw);
	        if(sqrt(pow(x1_ccw, 2) + pow(y1_ccw, 2)) < closest)
	        {
	        	closest_y1 = fabs(y1_ccw);
	        	closest_x1 = fabs(x1_ccw);
	        }

	        if(closest_y1 > 2*r) closest_y1 = 2*r;

	        if(closest_y1/closest_x1 > 1.1) child.potential-= -1;

	        float distance_to_goal = sqrt(
	        	pow(goal.pose.position.x - child.result_x, 2)
	        	+ pow(goal.pose.position.y - child.result_y, 2) 
	        	
	        	);

	        if(distance_to_goal < goal_thresh_trans)
	        {
	        	if(fabs(theta1) > M_PI/2) continue;
	        }

	        if(nearest_grid_road_waypoint != -1 && second_nearest_grw != -1)
	        {
	        	if(distance_to_nearest_grid_road_waypoint<1)
	        		child.potential += pursue_gain*( float(nearest_grid_road_waypoint)*distance_to_sngrw + float(second_nearest_grw)*distance_to_nearest_grid_road_waypoint)/(distance_to_sngrw+distance_to_nearest_grid_road_waypoint);
	        	else child.potential -= 1;

	        	child.potential -= guide_gain*distance_to_nearest_grid_road_waypoint;

	        	child.potential -= angle_gain*fabs(theta1)*(float(nearest_grid_road_waypoint + 1)/float(grid_road.size() ));

	        }
	        else child.potential = -distance_to_goal;

	        //Extra cost of zig-zag
	        if(node_id != 0)
	        {
	        	float x = node_vector[node_vector[node_id].parent_id].result_x;
	        	float y = node_vector[node_vector[node_id].parent_id].result_y;

	        	if(sqrt(pow(x-child.result_x, 2) + pow(y-child.result_y, 2)) < prim_dist[i]) child.cost -= zig_zag_cost;

	        }

	        


	        bool samethingness = false;
	        int vis = get_visited_map(child.result_x, child.result_y, child.result_theta);
	        if(vis>-1)
	        {
	        	//std::cout << "same with " << vis << "\n";
	        	if(node_vector[vis].cost < child.cost)
	        	{
	        		//std::cout << "replacing " << vis << "\n";

	        	    remove_children(vis);

	        	}
	        	else samethingness = true;
	        }






	        if(!samethingness)
	        {   


	            node_vector.push_back(child);
	            node_vector[node_id].children_ids.push_back(child.id);
	            //std::cout <<node_id<<">"<< child.id << " " << child.result_x << " x " << child.result_y <<"\n";

	            set_visited_map(child.result_x, child.result_y, child.result_theta, child.id);
	            
	            if(distance_to_goal < goal_thresh_trans) 
	            {
	            	double t = child.result_theta - tf::getYaw(goal.pose.orientation);
	            	if(t > M_PI) t = t - 2*M_PI;
	            	else if(t < -M_PI) t = t + 2*M_PI;
	            	if(fabs(t) < goal_thresh_rot)
	            	{


	            		final_node = child.id;
	                		//std::cout << "reached to goal, interrupted\n";
	            		return 1;
	            	}

	            	else
	            	{
	            		if(fabs(theta1) < best_found_angle)
	            		{
	            			best_found_angle = theta1;
	            			final_node = child.id;

	            		}
	            		return 2;
	            	}
	            }


	            //std::cout << child.id << " " << child.result_x << " x " << child.result_y <<"\n";
	        }
	    }

	    return 0;

	}

}