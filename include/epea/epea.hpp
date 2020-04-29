#ifndef EPEA_HPP
#define EPEA_HPP

#include <stack>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <chrono>

#include <algorithm>

typedef std::pair<int, int> pair_1;

namespace EPEA{
	template<typename Mapf>
	class EPEAStar{
	public:
		EPEAStar()	{}
		bool search(Mapf mapf, std::vector<pair_1> starts, std::pair<int, std::vector<std::vector<pair_1> > > *solution){
			std::cout<<"INSIDE THE SEARCH FUNCTION OF EPEAStar CLASS"<<std::endl;
			return true;
		}

	};
}

#endif
