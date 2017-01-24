
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

	    float xy_samethingness_threshold;
	    float theta_samethingness_threshold;

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

	    bool approach;

	    std::vector<std::vector<std::vector<int> > > visited_map;

	    std::vector<std::vector<int> > nav_grid;


	public:

	    std::vector<std::pair<float, float> > grid_road;

	    costmap_2d::Costmap2D* costmap_;

	    Tree()
	    {



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

	        xy_samethingness_threshold = 0.05;
	        theta_samethingness_threshold = 0.05;

	        path_point = 0;

	        starting_node.parent_id = -1;
	        starting_node.cost = 0;
	        starting_node.result_theta = 0;
	        starting_node.result_x = 0;
	        starting_node.result_y = 0;
	        starting_node.id = 0;

	        starting_node.end = false;
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
	    void grid_astar(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal)
	    {
	    	grid_road.clear();	    	

	    	for(int i = 0; i<grid_width_x; i++)
	    	{
	    		for(int j = 0; j<grid_width_y; j++) 
	    		{
  					nav_grid[i][j] = -1;
	    		}

	    	}



	    	int goal_x =(goal.position.x - map_origin_x)/grid_resolution_xy;
	    	int goal_y =(goal.position.y - map_origin_y)/grid_resolution_xy;

	    	std::vector<node> nodes;
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
	    			unsigned int xi, yi;
	    			costmap_->worldToMap(x_world, y_world, xi, yi);

	    			if(costmap_->getCost(xi, yi) > 100) continue;

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

	    			grid_road.push_back(coord);

	    			n=nodes[n].parent_id;

	    		}

	    		std::reverse(grid_road.begin(), grid_road.end());

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
	    	approach = false;

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

	    	xy_samethingness_threshold = 0.05;
	    	theta_samethingness_threshold = 0.025;

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


	            costmap_->worldToMap(child.result_x, child.result_y, xi, yi);
	            if(costmap_->getCost(xi,yi) > 100
	            	|| costmap_->getCost(xi+2,yi+2) > 100
	            	|| costmap_->getCost(xi-2,yi-2) > 100
	            	|| costmap_->getCost(xi+2,yi-2) > 100
	            	|| costmap_->getCost(xi-2,yi+2) > 100
	            	){
	            	// int parent=node_vector[node_id].parent_id;
	            	// for(char p=0; p<5; p++)
	            	// {
	            	//  	if(parent==-1) break;
	            	//  	else
	            	//  	{
	            	//  		node_vector[parent].cost-=0.5*(pow(0.5,  p));
	            	//  		parent=node_vector[parent].parent_id;
	            	//  	}

	            	//  }
	            	
	            	continue;
	            }




	            child.cost = node_vector[node_id].cost - prim_cost[i];
	            child.id=node_vector.size();

	            child.end = true;

	            float theta1 = child.result_theta - tf::getYaw(goal.pose.orientation);
	        	if(theta1 > M_PI)  theta1-=2*M_PI;
	        	else if(theta1 < -M_PI) theta1+=2*M_PI;

	        	float dx = child.result_x - goal.pose.position.x;
	        	float dy = child.result_y - goal.pose.position.y;
	        	float y1 = -sin(child.result_theta)*dx + cos(child.result_theta)*dy;

	            float distance_to_goal = sqrt(
	            	pow(goal.pose.position.x - child.result_x, 2)
	            	+ pow(goal.pose.position.y - child.result_y, 2) 
	            	
	            	);

	            if(distance_to_goal < 0.5 && !approach) 
	            {
	            	approach = true;
	            	//for(int asd = 0; asd < node_vector.size(); asd++) node_vector[asd].end = false;

	            }

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
	            //child.potential = -distance_to_goal ;

	            float r = 0.27;
	            float goal_yaw = tf::getYaw(goal.pose.orientation);
	            float c1_x = goal.pose.position.x + cos(goal_yaw + M_PI/2.0)*r;
	            float c1_y = goal.pose.position.y + sin(goal_yaw + M_PI/2.0)*r;
	            float c2_x = goal.pose.position.x + cos(goal_yaw - M_PI/2.0)*r;
	            float c2_y = goal.pose.position.y + sin(goal_yaw - M_PI/2.0)*r;

	            

	            if(nearest_grid_road_waypoint != -1 && second_nearest_grw != -1)
	            {
	            	if(distance_to_nearest_grid_road_waypoint<0.5)
	            		child.potential += 0.1*( nearest_grid_road_waypoint*distance_to_sngrw + second_nearest_grw*distance_to_nearest_grid_road_waypoint)/(distance_to_sngrw+distance_to_nearest_grid_road_waypoint);
	            	else child.potential -= 1;
	            }

	            //child.potential += 1/(1+distance_to_goal*fabs(theta1));
	            
	            // float look_x;

	            // for(look_x = 0; look_x< 0.25; look_x+=0.05)
	            // {

	            // 	costmap_->worldToMap(child.result_x + cos(yaw)*look_x, child.result_y + sin(yaw)*look_x, xi, yi);
	            // 	if(costmap_->getCost(xi,yi) > 100) 
	            // 		{
	            // 			child.potential += 2*look_x/distance_to_goal;
	            // 			break;
	            // 		}
	            // }

	            // for(look_x = 0; look_x> -0.25; look_x-=0.05)
	            // {

	            // 	costmap_->worldToMap(child.result_x + cos(yaw)*look_x, child.result_y + sin(yaw)*look_x, xi, yi);
	            // 	if(costmap_->getCost(xi,yi) > 100) 
	            // 		{
	            // 			child.potential -= 2*look_x/distance_to_goal;
	            // 			break;
	            // 		}
	            // }



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
	                
	                if(fabs(child.result_x - goal.pose.position.x) < 0.05
	                	&&fabs(child.result_y - goal.pose.position.y) < 0.05
	                	) 
	                {
	                	double t = child.result_theta - tf::getYaw(goal.pose.orientation);
	                	if(t > M_PI) t = t - 2*M_PI;
	                	else if(t < -M_PI) t = t + 2*M_PI;
	                	if(fabs(t) < 0.2)
	                	{	                
	                		final_node = node_id;
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

	    void recalculate_node_cost(float difference, int node_id)
	    {
	    	node_vector[node_id].cost += difference;
	    	if(!node_vector[node_id].children_ids.empty())
	    	{
	    		for(int i = 0; i<node_vector[node_id].children_ids.size(); i++)
	    		{
	    			recalculate_node_cost(difference, node_vector[node_id].children_ids[i]);
	    		}
	    	}
	    }

	    float compute_arc_and_line(float c1_x, float c1_y, float c2_x, float c2_y, float a1, float a2, float r, bool c1_cw, bool c2_cw)
	    {



	    	float distance_between_centers = sqrt(pow(c1_x - c2_x, 2) + pow(c1_y - c2_y, 2));
	    		
	    	while(distance_between_centers < 2*r)
	    	{
	    		c2_x += 0.01*cos(a2);
	    		c2_y += 0.01*sin(a2);

	    		distance_between_centers = sqrt(pow(c1_x - c2_x, 2) + pow(c1_y - c2_y, 2));
	    	}

	    	float line_length;
	    	if((c1_cw && c2_cw) || (!c1_cw && !c2_cw)) line_length = distance_between_centers;
	    	else
	    	{
	    		line_length = sqrt(pow(distance_between_centers, 2) - 4*r*r);
	    	}

	    	float line_angle;
	    	if((c1_cw && c2_cw) || (!c1_cw && !c2_cw)) line_angle = atan((c2_y - c1_y)/(c2_x - c1_x));
	    	else
	    	{
	    		line_angle = atan((c2_y - c1_y)/(c2_x - c1_x)) - asin(2*r / distance_between_centers);
	    	}

	    	float arc1, arc2;
	    	if(c1_cw) arc1 = (a1-line_angle);
	    	else  arc1 = (line_angle - a1);

	    	if(c2_cw) arc2 = (line_angle - a2);
	    	else  arc2 = (a2 - line_angle);

	    	while(arc1 < 0) arc1 += 2*M_PI;
	    	while(arc1 > 2*M_PI) arc1-=2*M_PI;

	    	while(arc2 < 0) arc2 += 2*M_PI;
	    	while(arc2 > 2*M_PI) arc2-=2*M_PI;
	    	
	    	return (arc1+arc2)*r + line_length;


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


	    void get_node_pose(int node_id, geometry_msgs::PoseStamped &pose)
	    {
	    	pose.pose.position.x = node_vector[node_id].result_x;
	    	pose.pose.position.y = node_vector[node_id].result_y;
	    	pose.pose.position.z = 0;
	    	pose.pose.orientation = tf::createQuaternionMsgFromYaw(node_vector[node_id].result_theta);
	    }

	};
}