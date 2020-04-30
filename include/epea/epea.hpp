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

#include <boost/heap/fibonacci_heap.hpp>

typedef std::pair<int, int> pair_1;
typedef std::tuple<int, int, int, int> priority_tuple;

typedef struct pq_node
{
	std::vector<pair_1> agent_locs;
	int g;
	int h;
	int small_f;
	int big_f;
	struct pq_node *parent;	

	pq_node(std::vector<pair_1> a, int b, int c, int d, int e, struct pq_node* p){
		agent_locs = a;
		g = b;
		h = c;
		small_f = d;
		big_f = e;
		parent = p;
	}
} pq_node;

typedef std::pair<priority_tuple, pq_node> heap_node;

struct VectorHashByElements {
public:
	size_t operator()(const std::vector<pair_1> & v) const {
		int size = v.size();
		return std::hash<int>()(size);
	}
};

struct VectorCompareByElements {
public:
	bool operator()(const std::vector<pair_1> & v1, const std::vector<pair_1> & v2) const {
 
		if (v1.size() == v2.size()){
			for(int i=0; i<v1.size(); i++){
				if(v1[i].first == v2[i].first && v1[i].second == v2[i].second)	continue;
				return false;
			}
			return true;
		}
		else{
			return false;
		}
	}
};

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

	int get_size(){	return goals.size();	}

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

	int list_of_locations_to_heuristic(std::vector<pair_1> locs){
		int val = 0;
		for(int i=0; i<locs.size(); i++){
			val += h[i][locs[i].first][locs[i].second];
		}
		return val;
	}
	std::pair<std::vector<std::vector<pair_1> >, int> get_children_and_next_F(pq_node current_node){

	}

	std::pair<std::vector<pair_1>, int> select_operators(pq_node node){
		std::vector<pair_1> agent_locs = node.agent_locs;
		int big_f = node.big_f;
		int h = node.h;
		int g = node.g;
		int small_f = h + g;

		int requested_row = big_f - small_f;

	}

};

struct compare_node
{
    bool operator()(const heap_node & n1, const heap_node & n2) const
    {
        return n1.first > n2.first;
    }
};

namespace EPEA{
	template<typename Mapf>
	class EPEAStar{
	public:
		EPEAStar()	{}
		bool search(Mapf mapf, std::vector<pair_1> starts, std::pair<int, std::vector<std::vector<pair_1> > > *solution){
			OSF<Mapf> osf(mapf);
			std::unordered_set<std::vector<pair_1>, VectorHashByElements, VectorCompareByElements> visited;

			int mycounter = 0; //counter used to break ties in the priority queue
			int g = 0;
			int h = osf.list_of_locations_to_heuristic(starts);
			int n_agents = osf.get_size();

			pq_node start_node(starts, 0, h, g+h, g+h, NULL);

			priority_tuple pq_tuple = {g+h, -g, h, mycounter};

			boost::heap::fibonacci_heap<heap_node, boost::heap::compare<compare_node> > heap;
			heap.push({pq_tuple, start_node});

			mycounter += 1;
			int nodes_expanded = 0;

			while(!heap.empty()){
				heap_node t = heap.top();
				heap.pop();
				priority_tuple pq_tuple = t.first;
				pq_node current_node = t.second;

				if(current_node.agent_locs == goals){
					std::cout<<"PATH FOUND ENJOY, EPEA* WORK IS DONE(JUST OUTPUT CREATION IS REMAINING"<<std::endl;
					return true;
				}
			}
			return true;
		}

	};
}

#endif
