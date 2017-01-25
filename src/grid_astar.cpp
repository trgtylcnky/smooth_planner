#include "global_planner_turgut/Tree.h"

namespace global_planner_turgut
{
		    //This function does a prior exploration
		    //Find a path to guide later path finding
	void Tree::grid_astar(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal)
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

	void Tree::set_grid_road_from_path(const std::vector<geometry_msgs::PoseStamped> &path)
	{
		grid_road.clear();

		for(int i = 0; i< path.size(); i++)
		{
			std::pair<float, float> p;
			p.first = path[i].pose.position.x;
			p.second = path[i].pose.position.y;
		}
	}
}