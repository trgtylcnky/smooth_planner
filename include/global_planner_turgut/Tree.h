
#include "visualization_msgs/Marker.h"


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
	/*theta1
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

	    std::vector<std::vector<std::vector<int> > > visited_map;

	    std::vector<std::vector<int> > nav_grid;


	public:

	    std::vector<std::pair<float, float> > grid_road;

	    costmap_2d::Costmap2D* costmap_;

	    Tree()
	    {

	        final_node = 0;

	        node starting_node;

	        wall_clearance = 0.05;

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

	    void set_grid_resolution(float grid_res_xy, float grid_res_theta)
	    {
	    	grid_resolution_xy = grid_res_xy;
	    	grid_resolution_theta = grid_res_theta;
	    }

	    void set_turn_radius(float r)
	    {
	    	turn_radius = r;
	    }

	    void set_wall_clearance(float c)
	    {
	    	wall_clearance = c;
	    }

	    void initialize(costmap_2d::Costmap2D* costmap)
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


	    //This function does a prior exploration
	    //Find a path to guide later path finding
	    void grid_astar(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal)
	    {
	    	grid_road.clear();	    	

	    	//Using a navigation grid for simplicity
	    	for(int i = 0; i<grid_width_x; i++)
	    	{
	    		for(int j = 0; j<grid_width_y; j++) 
	    		{
  					nav_grid[i][j] = -1;
	    		}

	    	}

	    	//using grid coordinated instead of world coordinates
	    	int goal_x =(goal.position.x - map_origin_x)/grid_resolution_xy;
	    	int goal_y =(goal.position.y - map_origin_y)/grid_resolution_xy;


	    	std::vector<node> nodes;

	    	//node for initial position
	    	node starting_node;
	    	starting_node.result_x = (start.position.x - map_origin_x)/grid_resolution_xy;
	    	starting_node.result_y = (start.position.y - map_origin_y)/grid_resolution_xy;
	    	starting_node.id = 0;
	    	starting_node.end = true;
	    	starting_node.cost = 0;
	    	starting_node.parent_id = -1;
	    	starting_node.potential = - sqrt(pow(goal_y - starting_node.result_y,2) + pow(goal_x - starting_node.result_x, 2)); 
	    	nodes.push_back(starting_node);
	    	nav_grid[int(starting_node.result_x)][int(starting_node.result_y)] = 0;

	    	
	    	const char x_step[8]={-1, 0, 1, -1, 1, -1, 0, 1};
	    	const char y_step[8]={1, 1, 1, 0, 0, -1, -1, -1};

	    	char stat = 0;
	    	int lim = 0;
	    	int reached_node = 0;
	    	while(lim++ <50000)
	    	{
	    		int best = -1;
	    		float best_p = -99999;
	    		for(int i = 0; i<nodes.size(); i++)
	    		{
	    			if(nodes[i].end)
	    			{
	    				if(nodes[i].cost + nodes[i].potential > best_p)
	    				{
	    					best = i;
	    					best_p = nodes[i].cost + nodes[i].potential;
	    				}
	    			}

	    		}

	    		//std::cout<<"best: "<<best<<"\n";

	    		if(best == -1)
	    		{
	    			reached_node = -1;
	    			stat = -1;
	    			break;
	    		}

	    		if(nodes[best].result_x == goal_x && nodes[best].result_y == goal_y)
	    		{
	    			std::cout << "found \n";
	    			stat = 1;
	    			reached_node = best;
	    			break;
	    		}

	    		nodes[best].end = 0;

	    		for(int i=0; i<8; i++)
	    		{
	    			int x = nodes[best].result_x + x_step[i];
	    			int y = nodes[best].result_y + y_step[i];

	    			float cost = nodes[best].cost - sqrt(pow(x_step[i], 2) + pow(y_step[i], 2));
	    			float potential = - sqrt(pow(goal_y -y,2) + pow(goal_x - x, 2));
	    			//float potential= 0;

	    			float x_world = x*grid_resolution_xy + map_origin_x;
	    			float y_world = y*grid_resolution_xy + map_origin_y;

	    			if(find_closest_wall_distance(x_world, y_world, 0.1) < 0.05) continue;

	    			if(x>=0 && x<grid_width_x && y>=0 && y<grid_width_y)
	    			{
	    				if(nav_grid[x][y] == -1 || nodes[nav_grid[x][y]].cost < cost)
	    				{
	    					
	    					node child;
	    					child.result_x = x;
	    					child.result_y = y;
	    					child.potential = potential;
	    					child.parent_id = best;	    					
	    					child.end = true;
	    					child.cost = cost;
	    					child.id = nodes.size();
	    					nodes.push_back(child);
	    					nav_grid[x][y] = child.id;
	    					nodes[best].children_ids.push_back(child.id);
	    					//std::cout<<best<<" > "<<child.id<<" "<< child.result_x << " " << child.result_y<<"\n";
	    				}
	    			}

	    		}

	    	}




	    	int n = reached_node;

	    	std::vector<std::pair<float, float> > temporary_road;

	    	if(stat == 1)
	    	{
	    		while(1)
	    		{
	    			if(n==-1) break;
	    			std::pair<float, float> coord;

	    			float x_world = (nodes[n].result_x)*grid_resolution_xy + map_origin_x;
	    			float y_world = (nodes[n].result_y)*grid_resolution_xy + map_origin_y;
	    			coord.first = x_world;
	    			coord.second = y_world;

	    			temporary_road.push_back(coord);

	    			n=nodes[n].parent_id;

	    		}

	    		std::reverse(temporary_road.begin(), temporary_road.end());

	    	}

	    	if(temporary_road.size() < 1) return;

	    	//post process
	    	int curr = 0;
	    	while(curr < temporary_road.size())
	    	{
	    		int last_checked = temporary_road.size()-1;
	    		bool seen = false;

	    		while(last_checked > curr+1)
	    		{
	    			float dist = sqrt(pow(temporary_road[curr].first - temporary_road[last_checked].first, 2)
	    				+ pow(temporary_road[curr].second - temporary_road[last_checked].second, 2));
	    			int num = dist/costmap_->getResolution();
	    			seen = true;
	    			for(int inc = 0; inc < num; inc++)
	    			{
	    				float x = temporary_road[curr].first + inc*(temporary_road[last_checked].first - temporary_road[curr].first)/float(num);
	    				float y = temporary_road[curr].second + inc*(temporary_road[last_checked].second - temporary_road[curr].second)/float(num);

	    				unsigned int mx, my;
	    				costmap_->worldToMap(x, y, mx, my);
	    				if(costmap_->getCost(mx, my)>200)
	    				{
	    					seen = false;
	    					break;
	    				}
	    			}
	    			if(!seen)
	    				last_checked--;
	    			else
	    				break;
	    		}
	    		if(seen&&last_checked>curr+1)
	    		{
	    			temporary_road.erase(temporary_road.begin()+curr+1, temporary_road.begin()+last_checked);
	    			
	    		}
	    		curr++;
	    	}

	    	for(int i = 0; i< temporary_road.size()-1; i++)
	    	{
	    		float dist = sqrt(pow(temporary_road[i].first - temporary_road[i+1].first, 2)
	    			+ pow(temporary_road[i].second - temporary_road[i+1].second, 2));
	    		float num = dist/0.05;
	    		float x_inc = (temporary_road[i+1].first - temporary_road[i].first)/num;
	    		float y_inc = (temporary_road[i+1].second - temporary_road[i].second)/num;

	    		for(int j = 0; j < num; j++)
	    		{
	    			std::pair<float, float> p;
	    			p.first = temporary_road[i].first + x_inc*j;
	    			p.second = temporary_road[i].second + y_inc*j;

	    			grid_road.push_back(p);
	    		}
	    	}
	    }

	    void set_grid_road_from_path(const std::vector<geometry_msgs::PoseStamped> &path)
	    {
	    	grid_road.clear();

	    	for(int i = 0; i< path.size(); i++)
	    	{
	    		std::pair<float, float> p;
	    		p.first = path[i].pose.position.x;
	    		p.second = path[i].pose.position.y;
	    	}
	    }

	    void init_starting_pose(geometry_msgs::Pose p)
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


	    void set_visited_map(float x, float y, float theta, int node_id)
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

	    int get_visited_map(float x, float y, float theta)
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

	    float find_closest_wall_distance(float x, float y, float max_search_radius)
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


	    int expand_node(int node_id, const geometry_msgs::PoseStamped &goal)
	    {
	    	
	        if(!node_vector[node_id].children_ids.empty()) return -1;

	        unsigned int xi, yi;

	        costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, xi, yi);

	        if(costmap_->getCost(xi,yi) > 252){
	            //std::cout << "goal in bad position, interrupted\n";
	            return -1;
	        }


	        if(fabs(node_vector[node_id].result_x - goal.pose.position.x) < 0.05
	            &&fabs(node_vector[node_id].result_y - goal.pose.position.y) < 0.05
	            ) 
	            {
	            	double t = node_vector[node_id].result_theta - tf::getYaw(goal.pose.orientation);
	            	if(t > M_PI) t = t - 2*M_PI;
	            	else if(t < -M_PI) t = t + 2*M_PI;
	            	if(fabs(t) < 0.2)
	            	{	                
	            		final_node = node_id;
	            		//std::cout << "reached to goal, interrupted\n";
	            		return 1;
	            	}
	            }
	        //std::cout << "\n\nnew children appending to node " << node_id <<"\n";

	        node_vector[node_id].end = false;

	        for(int i = 0; i<NUM_OF_PRIMS; i++)
	        {

	            node child;

	            child.parent_id = node_id;

	            child.result_theta = node_vector[node_id].result_theta + prim_theta[i];

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
	            for(int i = 0; i<grid_road.size(); i++)
	            {
	            	float dis = sqrt(pow(grid_road[i].first - child.result_x, 2) + pow(grid_road[i].second - child.result_y, 2));

	            	if(dis < distance_to_nearest_grid_road_waypoint)
	            	{
	            		second_nearest_grw = nearest_grid_road_waypoint;
	            		distance_to_sngrw = distance_to_nearest_grid_road_waypoint;
	            		nearest_grid_road_waypoint = i;
	            		distance_to_nearest_grid_road_waypoint = dis;
	            	}
	            }

	            child.potential = 0;


	            if(nearest_grid_road_waypoint != -1 && second_nearest_grw != -1)
	            {
	            	if(distance_to_nearest_grid_road_waypoint<1)
	            		child.potential += 0.1*( float(nearest_grid_road_waypoint)*distance_to_sngrw + float(second_nearest_grw)*distance_to_nearest_grid_road_waypoint)/(distance_to_sngrw+distance_to_nearest_grid_road_waypoint);
	            	else child.potential -= 1;
	            }

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

	            child.potential -= closest_y1*float(nearest_grid_road_waypoint)/float(grid_road.size());
	           

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

	            float distance_to_goal = sqrt(
	            	pow(goal.pose.position.x - child.result_x, 2)
	            	+ pow(goal.pose.position.y - child.result_y, 2) 
	            	
	            	);


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
	                }


	                //std::cout << child.id << " " << child.result_x << " x " << child.result_y <<"\n";
	            }
	        }

	        return 0;

	    }

	    void remove_children(int node_id)
	    {
	    	//std::cout<<"removing "<<node_id<<"\n";
	    	//std::cout<<"has " <<node_vector[node_id].children_ids.size()<< " children\n";
	    	for(int i = 0; i<node_vector[node_id].children_ids.size(); i++)
	    	{
	    		remove_children(node_vector[node_id].children_ids[i]);
	    	}
	    	node_vector[node_id].end = false;
	    }

	    void create_road_to_final(std::vector<geometry_msgs::PoseStamped> &path)
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

	    int find_best_end()
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



	    void create_line_list( visualization_msgs::Marker &output)
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


	};
}