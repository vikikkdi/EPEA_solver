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
#include <climits>

#include <algorithm>

#include <boost/heap/fibonacci_heap.hpp>

typedef std::pair<int, int> pair_1;
typedef std::tuple<int, int, int, int> priority_tuple;

typedef struct pq_node{
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

typedef struct op_table_type{
	int delta_h;
	std::vector<std::vector<std::pair<pair_1, int> > > operators;

	bool is_data_present;

	op_table_type(){
		is_data_present = false;
		operators = std::vector<std::vector<std::pair<pair_1, int> > >();
	}
} op_table_type;

typedef std::pair<priority_tuple, pq_node*> heap_node;

struct VectorHashBySize {
public:
	size_t operator()(const std::vector<pair_1> & v) const {
		int size = v.size();
		return std::hash<int>()(size);
	}
};

struct VectorHashByElements {
public:
	size_t operator()(const std::vector<pair_1> & v) const {
		int size = v.size();
		int val = 0;
		for(int i=0; i<size; i++){
			val += ((v[i].first*(i+1)) + (v[i].second*(i+10000)));
		}
		return std::hash<int>()(val);
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
	std::vector<pair_1> obstacles;

	std::vector<std::vector<bool> > temp_map;
	std::vector<pair_1> op = {{0,0}, {-1,0}, {0,1}, {1,0}, {0,-1}};

	int x;
	int y;
	Mapf obj;
	std::vector<std::map<pair_1, std::vector<std::pair<pair_1, int> > > > agent_osfs;
	std::unordered_map<std::vector<pair_1>, std::vector<op_table_type>, VectorHashByElements> osf_table;

public:
	OSF(){}
	OSF(Mapf mapf){	
		obj = mapf;
		goals = mapf.get_goals();
		obstacles = mapf.get_obstacles();
		x = mapf.get_x();
		y = mapf.get_y();

		temp_map.resize(x);
		for(int i=0; i<x; i++)	temp_map[i].resize(y, false);

		for(int i=0; i<obstacles.size(); i++){
			temp_map[obstacles[i].first][obstacles[i].second] = true;
		}

		h = get_true_distance_heuristics();
		agent_osfs = populate_agent_osf();
	}

	int get_size(){	return goals.size();	}

	std::vector<pair_1>  get_goals(){	return goals;	}

	std::vector<std::vector<std::vector<int> > > get_true_distance_heuristics(){
		std::vector<std::vector<std::vector<int> > > heu;
		for(int i=0; i<goals.size(); i++){
			pair_1 this_goal = goals[i];
			heu.push_back(true_distance_bfs(this_goal));
		}
		return heu;
	}

	std::vector<std::vector<int> > true_distance_bfs(pair_1 goal){
		std::vector<std::vector<int> > bfs_result(x, std::vector<int>(y, 0));
		std::deque<pair_1> dq;
		std::unordered_set<int> vis;

		dq.push_back({(goal.first*y)+goal.second, 0});
		vis.insert((goal.first*y)+goal.second);

		while(!dq.empty()){
			std::pair<int, int> a = dq.front();
			dq.pop_front();

			int x_val = a.first / y;
			int y_val = a.first % y;

			bfs_result[x_val][y_val] = a.second;

			for(int i=0; i<op.size(); i++){
				int new_x = x_val + op[i].first;
				int new_y = y_val + op[i].second;

				if(new_x >= 0 && new_x < x && new_y >=0 && new_y < y && temp_map[new_x][new_y] == false && vis.find((new_x*y)+new_y) == vis.end()){
					vis.insert((new_x*y)+new_y);
					dq.push_back({(new_x*y)+new_y, a.second+1});
				}
			}
		}
		return bfs_result;
	}

	int manhattan_distance(pair_1 a, pair_1 b){
		return abs(a.first - b.first) + abs(a.second - b.second);
	}

	std::vector<std::vector<std::pair<pair_1, int> > > cart_product (const std::vector<std::vector<std::pair<pair_1, int> > >& v) {
		std::vector<std::vector<std::pair<pair_1, int> > > s = {{}};
		for (const auto& u : v) {
			std::vector<std::vector<std::pair<pair_1, int> > > r;

			for (const auto& x : s) {
				for (const auto y : u) {
					r.push_back(x);
					r.back().push_back(y);
				}
			}
			s = std::move(r);

		}
		
		return s;
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
        			for(int t=0; t<op.size(); t++){
        				int new_x = i + op[t].first;
        				int new_y = j + op[t].second;
        				if(new_x >= 0 && new_x < x && new_y >=0 && new_y < y && temp_map[new_x][new_y] == false)
	        				good_ops.push_back({op[t], h[agent_no][new_x][new_y]});
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

	std::pair<std::vector<std::vector<pair_1> >, int> get_children_and_next_F(pq_node *current_node){
		std::pair<std::vector<std::vector<pair_1> >, int> temp = select_operators(current_node);
		std::vector<std::vector<pair_1> > operators = temp.first;
		int next_big_F = temp.second;

		if(operators.size() == 0){
			return {operators, next_big_F};
		}

		std::vector<std::vector<pair_1> > new_child_nodes = get_new_children(current_node->agent_locs, operators);
		return {new_child_nodes, next_big_F};
	}

	std::vector<pair_1> get_new_locations(const std::vector<pair_1> &ops, const std::vector<pair_1> &agent_locs){
		std::vector<pair_1> new_locs;
		for(int i=0; i<ops.size(); i++){
			int new_x = agent_locs[i].first + ops[i].first;
			int new_y = agent_locs[i].second + ops[i].second;
			if(new_x >= 0 && new_x < x && new_y >= 0 && new_y < y)
				new_locs.push_back({new_x, new_y});
		}
		return new_locs;
	}

	bool has_edge_collisions(const std::vector<pair_1> &this_locs, const std::vector<pair_1> &next_locs){
		std::vector<std::pair<pair_1, pair_1> > forward;
		std::vector<std::pair<pair_1, pair_1> > backward;

		for(int i=0; i<this_locs.size(); i++){
			if(this_locs[i] != next_locs[i]){
				forward.push_back({this_locs[i], next_locs[i]});
			}
		}
		for(int i=0; i<this_locs.size(); i++){
			if(this_locs[i] != next_locs[i]){
				backward.push_back({next_locs[i], this_locs[i]});
			}
		}

		std::set<std::pair<pair_1, pair_1> > set_forward(forward.begin(), forward.end());
		std::set<std::pair<pair_1, pair_1> > set_backward(backward.begin(), backward.end());
		
		std::vector<std::pair<pair_1, pair_1>> common_data;
		std::set_intersection(set_forward.begin(), set_forward.end(), set_backward.begin(), set_backward.end(), std::back_inserter(common_data));
		return common_data.size() > 0;
	}

	bool move_invalid(const std::vector<pair_1> &this_locs, const std::vector<pair_1> &next_locs){
		std::set<pair_1> set_locs(next_locs.begin(), next_locs.end());
		bool vertex_collision = (set_locs.size() != next_locs.size());
		bool edge_collision = has_edge_collisions(this_locs, next_locs);
		return (vertex_collision || edge_collision);
	}

	std::vector<std::vector<pair_1> > get_new_children(const std::vector<pair_1> &agent_locs, const std::vector<std::vector<pair_1> > &group_ops){
		std::vector<std::vector<pair_1> > children;

		for(auto op:group_ops){
			std::vector<pair_1> new_locs = get_new_locations(op, agent_locs);
			if(move_invalid(agent_locs, new_locs) == false){
				children.push_back(new_locs);
			}
		}

		return children;
	}

	int get_heuristics_from_op(std::vector<std::pair<pair_1, int> > individual_ops){
		int heuristic_value = 0;
		for(int i=0; i<individual_ops.size(); i++){
			heuristic_value +=  individual_ops[i].second;
		}
		return heuristic_value;
	}

	std::vector<op_table_type> get_op_table(const std::vector<std::vector<std::pair<pair_1, int> > > &all_possible_ops, const std::vector<pair_1> &agent_locs, int small_f, int h, int g){
		int num_agents = agent_locs.size();
		int max_rows = (2*num_agents) + 1;

		std::vector<op_table_type> op_table(max_rows);

		for(auto it:all_possible_ops){//type of it is std::vector<std::pair<pair_1, int> >
			int this_h = get_heuristics_from_op(it);
			int this_g = g + num_agents;
			int this_small_f = this_h + this_g;
			int delta_small_f = this_small_f - small_f;

			if(!op_table[delta_small_f].is_data_present){
				op_table_type new_op_table_row;
				new_op_table_row.delta_h = this_h - h;
				new_op_table_row.is_data_present = true;
				std::vector<std::vector<std::pair<pair_1, int> > > tt;	tt.push_back(it);
                new_op_table_row.operators = std::move(tt);
                op_table[delta_small_f] = new_op_table_row;
			} else {
				op_table[delta_small_f].operators.push_back(it);
			}
		}

		return op_table;
	}

	int get_delta_big_F_next(const std::vector<op_table_type> &op_table, int requested_row){
		if(requested_row > op_table.size())	return INT_MAX;

        int my_index = requested_row;
        if(op_table[requested_row].is_data_present == false){
        	my_index = -1;
        }
        int next_start_index = my_index + 1;
        int delta_big_F_next = INT_MAX;
        for(int i=next_start_index; i<op_table.size(); i++){
        	if(op_table[i].is_data_present){
        		return i;
        	}
        }
        return delta_big_F_next;
	}

	std::pair<std::vector<std::vector<pair_1> >, int> select_operators(pq_node *node){
		std::vector<pair_1> agent_locs = node->agent_locs;
		int big_f = node->big_f;
		int h = node->h;
		int g = node->g;
		int small_f = h + g;

		int requested_row = big_f - small_f;

		std::vector<op_table_type> op_table;

		if(osf_table.find(agent_locs) != osf_table.end()){
			op_table = osf_table[agent_locs];
		} else {
			std::vector<std::vector<std::pair<pair_1, int> > > ops_to_cross_prod;
			for(int i=0; i<agent_locs.size(); i++){
				ops_to_cross_prod.push_back(agent_osfs[i][{agent_locs[i].first, agent_locs[i].second}]);
			}
			std::vector<std::vector<std::pair<pair_1, int> > > all_possible_ops = cart_product(ops_to_cross_prod);
			op_table = get_op_table(all_possible_ops, agent_locs, small_f, h, g);
			if(op_table.size()){
				osf_table[agent_locs] = op_table;
			}
		}
		int delta_big_F_next = get_delta_big_F_next(op_table, requested_row);

        int next_big_F = small_f + delta_big_F_next;
        if(requested_row > op_table.size() || op_table[requested_row].is_data_present == false){
        	return {std::vector<std::vector<pair_1> >(), next_big_F};
        }

        std::vector<std::vector<std::pair<pair_1, int> > > all_ops = op_table[requested_row].operators;
        std::vector<std::vector<pair_1> > good_ops;
        for(auto it:all_ops){
        	std::vector<pair_1> just_ops;
        	for(auto u:it){
        		just_ops.push_back(u.first);
        	}
        	good_ops.push_back(just_ops);
        }

		return {good_ops, next_big_F};
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
			std::unordered_set<std::vector<pair_1>, VectorHashBySize, VectorCompareByElements> visited;

			int mycounter = 0; //counter used to break ties in the priority queue
			int g = 0;
			int h = osf.list_of_locations_to_heuristic(starts);
			int n_agents = osf.get_size();

			pq_node *start_node = new pq_node(starts, 0, h, g+h, g+h, NULL);

			priority_tuple pq_tuple = {g+h, -g, h, mycounter};

			boost::heap::fibonacci_heap<heap_node, boost::heap::compare<compare_node> > heap;
			heap.push({pq_tuple, start_node});

			mycounter += 1;
			int nodes_expanded = 0;

			while(!heap.empty()){
				heap_node t = heap.top();
				heap.pop();
				priority_tuple pq_tuple = t.first;
				pq_node *current_node = t.second;

				if(current_node->agent_locs == mapf.get_goals()){
					find_solution(current_node, mapf.get_goals(), solution);
					return true;
				}
				std::pair<std::vector<std::vector<pair_1> >, int> temp = osf.get_children_and_next_F(current_node);
				std::vector<std::vector<pair_1> > new_child_nodes = temp.first;
				int next_big_f = temp.second;

				for(auto child:new_child_nodes){
					pq_node *child_node = get_child_node(child, current_node, osf);
					if(visited.find(child) == visited.end()){
						visited.insert(child);
						pq_tuple = {child_node->big_f, child_node->h, -child_node->g, mycounter};
						heap.push({pq_tuple, child_node});
						mycounter++;
					}
				}

				if(next_big_f == INT_MAX){
					visited.insert(current_node->agent_locs);
				}else{
					current_node->big_f = next_big_f;
					pq_tuple = {current_node->big_f, current_node->h, -current_node->g, mycounter};
					heap.push({pq_tuple, current_node});
					mycounter++;
				}
				nodes_expanded++;
			}
			return false;
		}

		void find_solution(pq_node *node, std::vector<pair_1> goals, std::pair<int, std::vector<std::vector<pair_1> > > *solution){
			std::vector<std::vector<pair_1> > paths;
			paths.push_back(goals);

			while(node->parent){
				paths.push_back(node->parent->agent_locs);
				node = node->parent;
			}

			std::reverse(paths.begin(), paths.end());
			
			std::vector<std::vector<pair_1> > processed_path(goals.size(), std::vector<pair_1>());
			for(int i=0; i<paths.size(); i++){
				for(int j=0; j<paths[i].size(); j++){
					if(processed_path[j].size() > 0 &&  processed_path[j][processed_path.size()-1] == goals[j])	continue;
					processed_path[j].push_back(paths[i][j]);
				}
			}
			solution->second = processed_path;

			int val = 0;
			for(int i=0; i<processed_path.size(); i++){
				std::set<pair_1> set_path(processed_path[i].begin(), processed_path[i].end());
				val += (set_path.size() - 1);
			}
			solution->first = val;
		}

		pq_node* get_child_node(const std::vector<pair_1> &child, pq_node *parent_node, OSF<Mapf> osf){
			int h_val = osf.list_of_locations_to_heuristic(child);
			int num_of_agents_not_at_goal = 0;

			std::vector<pair_1> goals = osf.get_goals();
			for(int i=0; i<child.size(); i++){
				if(child[i] != goals[i])	num_of_agents_not_at_goal++;
			}

			int g = parent_node->g + num_of_agents_not_at_goal;
			int small_f = g + h_val;
			int big_f = small_f;

			pq_node *temp_node = new pq_node(child, g, h_val, small_f, big_f, parent_node);
			return temp_node;
		}

	};
}

#endif
