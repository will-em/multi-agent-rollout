#include "coop-astar.hpp"
#include <iostream>


ReservationTable::ReservationTable() {
	cells = std::unordered_set<TimeNode, TimeNodeHash>();
	fronts = std::unordered_set<std::pair<TimeNode, TimeNode>, PairTimeNodeHash>();
}

bool ReservationTable::action_is_valid(TimeNode node, TimeNode next_node) {
	if (cells.find(next_node) != cells.end()) {
		return true;
	}

	std::pair<TimeNode, TimeNode> jump(next_node, node);

	if (fronts.find(jump) != fronts.end()) {
		return false;
	}

	return true;
}

int ReservationTable::reserve_path(std::vector<TimeNode> path) {

	TimeNode last_node = path[0];
	this->cells.insert(path[0]);

	for (int i = 1; i< path.size(); i++) {
		this->cells.insert(path[i]);
		this->fronts.insert({last_node, path[i]});

		last_node = path[i];
	}

	return 1;
}


int AStarFinder::expand_next_in_queue() {
	// Obtain best node in queue
	NodeInQueue parent_node = this->queue.top();
	this->queue.pop();

	// Add it to the expanded_nodes


	// Change stuff

}


int main() {
	std::unordered_set<std::pair<int,int>, NodeHash> my_set = {{4, 6}, {8, 12}};
	for (auto const &entry: my_set) {
		std::cout << entry.first << " " << entry.second << std::endl;
	}
}
