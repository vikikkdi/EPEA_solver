#ifndef EPEA_HPP
#define EPEA_HPP

#include <stack>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <deque>
#include <chrono>

#include <algorithm>

typedef std::pair<int, int> pair_1;

template<typename Mapf>
class OSF{
private:
	std::vector<std::vector<std::vector<int> > > h;
	std::vector<pair_1> goals;
	int x;
	int y;
	Mapf obj;
	std::vector<std::map<pair_1, std::vector<std::pair<pair_1, int> > > > agent_osfs;
public:
	OSF(){}
	OSF(Mapf mapf){	
		obj = mapf;
		goals = mapf.get_goals();
		x = mapf.get_x();
		y = mapf.get_y();
		h = get_true_distance_heuristics();
		agent_osfs = populate_agent_osf();
	}

	std::vector<std::vector<std::vector<int> > > get_true_distance_heuristics(){
		std::vector<std::vector<std::vector<int> > > heu;
		for(int i=0; i<goals.size(); i++){
			pair_1 this_goal = goals[i];
			heu.push_back(true_distance_bfs(this_goal));
		}
		return heu;
	}

	std::vector<std::vector<int> > true_distance_bfs(pair_1 goal){
		std::vector<std::vector<int> > b(x, std::vector<int>(y, 0));
		std::deque<std::pair<pair_1, int> > q;
		q.push_back({goal, 0});

		std::unordered_set<int> visited;
		visited.insert((goal.first*y) + goal.second);

		while(!q.empty()){
			std::pair<pair_1, int> node = q.front();
			q.pop_front();
			b[node.first.first][node.first.second] = node.second;
			std::vector<pair_1> children = obj.get_graph().get_neighbor(node.first.first, node.first.second);
			children.push_back(node.first);

			for(auto i:children){
				if(visited.find((i.first*y) + i.second) == visited.end()){
					visited.insert((i.first*y) + i.second);
					q.push_back({i, node.second+1});
				}
			}
		}
		return b;
	}

	int manhattan_distance(pair_1 a, pair_1 b){
		return abs(a.first - b.first) + abs(a.second - b.second);
	}

	std::vector<std::map<pair_1, std::vector<std::pair<pair_1, int> > > > populate_agent_osf(){
		std::vector<std::map<pair_1, std::vector<std::pair<pair_1, int> > > > temp;
		for(int i=0; i<goals.size(); i++){
			temp.push_back(get_one_agent_osf(i));
		}
		return temp;
	}

	std::map<pair_1, std::vector<std::pair<pair_1, int> > > get_one_agent_osf(int agent_no){
		std::map<pair_1, std::vector<std::pair<pair_1, int> > > agent_osf;

		std::vector<pair_1> obstacles = obj.get_obstacles();
		std::unordered_set<int> processed_obstacles;
		for(auto it = obstacles.begin(); it != obstacles.end(); ++it){
			int i = it->first;
			int j = it->second;
			int new_xy = i*y + j;
			processed_obstacles.insert(new_xy);
		}

        for(int i=0; i<x; i++){
        	for(int j=0; j<y; j++){
        		std::vector<std::pair<pair_1, int> > good_ops;
        		if(processed_obstacles.find((i*y)+(j)) == processed_obstacles.end()){
        			std::vector<pair_1> children = obj.get_graph().get_neighbor(i, j);
        			children.push_back({i, j});
        			for(auto it:children){
        				pair_1 op = {it.first-i, it.second-j};
        				good_ops.push_back({op, h[agent_no][it.first][it.second]});
        			}
        		}
        		agent_osf[{i, j}] = good_ops;
        	}
        }
        return agent_osf;
	}

};

namespace EPEA{
	template<typename Mapf>
	class EPEAStar{
	public:
		EPEAStar()	{}
		bool search(Mapf mapf, std::vector<pair_1> starts, std::pair<int, std::vector<std::vector<pair_1> > > *solution){
			OSF<Mapf> osf(mapf);
			return true;
		}

	};
}

#endif
