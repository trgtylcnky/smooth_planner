
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

	#define GRID_WH 200
	#define GRID_TH 30

	class Tree
	{

	    std::vector<node> node_vector;

	    float xy_samethingness_threshold;
	    float theta_samethingness_threshold;

	    int final_node;

	    int path_point;


	    int visited_map[GRID_WH][GRID_WH][GRID_TH];



	public:

	    float *potential_array;
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

	    void init_starting_pose(geometry_msgs::Pose p)
	    {

	    	for(int i = 0; i< GRID_WH; i++)
	    		for(int j = 0; j<GRID_WH; j++)
	    			for(int k = 0; k<GRID_TH; k++) visited_map[i][j][k] = -1;

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
	    	theta_samethingness_threshold = 0.05;



	        node_vector[0].result_theta = round(tf::getYaw(p.orientation)/(M_PI/15.0))*(M_PI/15.0);
	        node_vector[0].result_x = p.position.x;
	        node_vector[0].result_y = p.position.y;

/*	        //add one front and one back
	        node second_step;

	        second_step.parent_id = 0;
	        second_step.id=1;
	        second_step.end = true;

	        second_step.result_theta = node_vector[0].result_theta;
	        second_step.result_x = node_vector[0].result_x + cos(second_step.result_theta)*prim_dist[0];
	        second_step.result_y = node_vector[0].result_y + sin(second_step.result_theta)*prim_dist[0];
	        second_step.cost = node_vector[0].cost + prim_cost[0];

	        node_vector[0].children_ids.push_back(1);
	        node_vector.push_back(second_step);

	        node second_step2;

	        second_step2.parent_id = 0;
	        second_step2.id=2;
	        second_step2.end = true;

	        second_step2.result_theta = node_vector[0].result_theta;
	        second_step2.result_x = node_vector[0].result_x + cos(second_step2.result_theta)*prim_dist[3];
	        second_step2.result_y = node_vector[0].result_y + sin(second_step2.result_theta)*prim_dist[3];
	        second_step2.cost = node_vector[0].cost + prim_cost[3];

	        node_vector[0].children_ids.push_back(2);
	        node_vector.push_back(second_step2);*/

	        node_vector[0].end = false;


	    }

	    void set_visited_map(float x, float y, float theta, int node_id)
	    {
	    	int mx = (x/10.0)*GRID_WH + GRID_WH/2;
	    	int my = (y/10.0)*GRID_WH + GRID_WH/2;
	    	int mtheta = (theta/(2*M_PI))*GRID_TH + GRID_TH/2;

	    	if(mx >=0 && mx < GRID_WH
	    		&& my >=0 && my < GRID_WH
	    		&& mtheta >=0 && mtheta < GRID_TH
	    		)
	    		visited_map[mx][my][mtheta] = node_id;

	    }

	    int get_visited_map(float x, float y, float theta)
	    {
	    	int mx = (x/10.0)*GRID_WH + GRID_WH/2;
	    	int my = (y/10.0)*GRID_WH + GRID_WH/2;
	    	int mtheta = (theta/(2*M_PI))*GRID_TH + GRID_TH/2;

	    	//std::cout <<mx <<" "<<my << " "<<mtheta<<"\n";

	    	if(mx >=0 && mx < GRID_WH
	    		&& my >=0 && my < GRID_WH
	    		&& mtheta >=0 && mtheta < GRID_TH
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
	            	//if(fabs(t) < 0.05)
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
	            if(costmap_->getCost(xi,yi) > 100){
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



	            float distance_to_goal = sqrt(
	            	pow(goal.pose.position.x - child.result_x, 2)
	            	+ pow(goal.pose.position.y - child.result_y, 2) 
	            	
	            	);

	            double yaw;
	            yaw = child.result_theta;
	           

	            child.potential = -distance_to_goal ;

	            //child.potential-= fabs(theta1);


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

/*	            for(int i=0; i<node_vector.size(); i++)
	            {
	                if(fabs(node_vector[i].result_x - child.result_x) < xy_samethingness_threshold
	                    && fabs(node_vector[i].result_y - child.result_y) < xy_samethingness_threshold
	                    && fabs(node_vector[i].result_theta - child.result_theta) < theta_samethingness_threshold) {
	                    if(node_vector[i].cost < child.cost)
	                    {

	                        remove_children(i);

	                    }
	                    else samethingness = true;
	                }
	            }*/

	            if(!samethingness)
	            {   

	                node_vector.push_back(child);
	                node_vector[node_id].children_ids.push_back(child.id);
	                //std::cout <<node_id<<">"<< child.id << " " << child.result_x << " x " << child.result_y <<"\n";

	                set_visited_map(child.result_x, child.result_y, child.result_theta, child.id);
	                
	                
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
	    		c2_x -= 0.01*cos(a2);
	    		c2_y -= 0.01*sin(a2);

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
	        	if(n == 0) break;
	        	float node_x = node_vector[n].result_x;
	        	float node_y = node_vector[n].result_y;
	        	float parent_x = node_vector[node_vector[n].parent_id].result_x;
	        	float parent_y = node_vector[node_vector[n].parent_id].result_y;

	        	float x1 = node_x - 0.25*(node_x-parent_x);
	        	float y1 = node_y - 0.25*(node_y-parent_y);

	        	float x2 = node_x - 0.75*(node_x - parent_x);
	        	float y2 = node_y - 0.75*(node_y - parent_y);

	            geometry_msgs::PoseStamped p1;
	            p1.header.stamp = ros::Time::now();
	            p1.header.frame_id = "/map";
	            p1.pose.position.x = x1;
	            p1.pose.position.y = y1;
	            p1.pose.position.z = 0;
	            p1.pose.orientation.x = 0;
	            p1.pose.orientation.y = 0;
	            p1.pose.orientation.z = 0;
	            p1.pose.orientation.w = 1;

	            geometry_msgs::PoseStamped p2;
	            p2.header.stamp = ros::Time::now();
	            p2.header.frame_id = "/map";
	            p2.pose.position.x = x2;
	            p2.pose.position.y = y2;
	            p2.pose.position.z = 0;
	            p2.pose.orientation.x = 0;
	            p2.pose.orientation.y = 0;
	            p2.pose.orientation.z = 0;
	            p2.pose.orientation.w = 1;

	            path.push_back(p1);
	            //path.push_back(p2);

	            if(n == 0 ) break;
	            else n = node_vector[n].parent_id;
	        }

	        std::reverse(path.begin(), path.end());
	    }
	    int find_best_end(const geometry_msgs::PoseStamped &goal)
	    {
	    	std::vector<geometry_msgs::PoseStamped> path;

	    	path.push_back( goal);
	    	return find_best_end(path);

	    }
	    int find_best_end(const std::vector<geometry_msgs::PoseStamped>& path)
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

	    bool expand_best_end(const geometry_msgs::PoseStamped &goal)
	    {
	        return expand_node(find_best_end(goal), goal);

	    }

	    bool expand_best_end(const geometry_msgs::PoseStamped &goal, const std::vector<geometry_msgs::PoseStamped>& path)
	    {
	    	return expand_node(find_best_end(path), goal);
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

	    void create_line_list_to_final( visualization_msgs::Marker &output)
	    {
	    	output.points.clear();
	        int n = final_node;

	        while(n != 0)
	        {
	            int i = node_vector[n].parent_id;

	            geometry_msgs::Point child_p;
	            child_p.x = node_vector[n].result_x;
	            child_p.y = node_vector[n].result_y;
	            child_p.z = 0;

	            geometry_msgs::Point parent_p;
	            parent_p.x = node_vector[i].result_x;
	            parent_p.y = node_vector[i].result_y;
	            parent_p.z = 0;

	            //std::cout << "Child: " << n << " Parent: " << i << "\n";

	            output.points.push_back(parent_p);
	            output.points.push_back(child_p);

	            n = i;
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