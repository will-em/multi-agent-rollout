#include "../coop-astar.hpp"
#include "assert.h"
#include <iostream>


void print_path(std::vector<TimeNode> & path) {
	for (int i = 0; i < path.size(); i++ ) {
		printf("- (%i, (%i, %i)) -", path[i].turn, path[i].node.x, path[i].node.y);
	}
	printf("\n");
}

int main() {
	ReservationTable table;
	std::vector<TimeNode> path = {
		{0, {0, 0}}, {1, {0,1}}, {2, {0,2}}, {3, {1,2}}
	};

	table.reserve_path(path);

	
	// Cell collisions
	std::cout << "Testing cell collisions" << std::endl;
	assert(!table.action_is_valid({0, {0,2}}, {1, {0,1}}));
	assert(!table.action_is_valid({1, {1,2}}, {2, {0,2}}));
	assert(!table.action_is_valid({2, {0,2}}, {3, {1,2}}));


	// Front collisions
	std::cout << "Testing front collisions" << std::endl;
	assert(!table.action_is_valid({0, {0,1}}, {1, {0,0}}));
	assert(!table.action_is_valid({1, {0,2}}, {2, {0,1}}));
	assert(!table.action_is_valid({2, {1,2}}, {3, {0,2}}));


	// Available actions
	std::cout << "Testing accceptable actions" << std::endl;
	assert(table.action_is_valid({0, {0,0}}, {1, {1,0}}));
	assert(table.action_is_valid({0, {0,1}}, {1, {0,2}}));


	// A*
	TimeNode initial_node(0, Node(3,4));
	Node final_node(6,6);
	std::vector<TimeNode> optimal_path = compute_optimal_path(initial_node, final_node, 10);

	printf("Here comes the PATH:\n");
	print_path(optimal_path);


	std::cout << "Tests passed successfully" << std::endl;
}
