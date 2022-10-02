#include "../coop-astar.hpp"
#include "assert.h"
#include <iostream>


void print_path(std::vector<TimeNode> & path) {
	for (int i = 0; i < path.size(); i++ ) {
		printf("- (%i, (%i, %i)) -", path[i].turn, path[i].node.x, path[i].node.y);
	}
	printf("\n");
}

bool check_action_is_valid(ReservationTable & table, TimeNode node_a, TimeNode node_b) {
	return table.action_is_valid(node_a, node_b);
}

void reservation_table_checks() {
	ReservationTable table;
	std::vector<TimeNode> path = {
		{0, {0, 0}}, {1, {0,1}}, {2, {0,2}}, {3, {1,2}}
	};

	table.reserve_path(path);

	
	// Cell collisions
	std::cout << "Testing cell collisions" << std::endl;
	assert(!check_action_is_valid(table, {0, {0,2}}, {1, {0,1}}));
	assert(!check_action_is_valid(table, {1, {1,2}}, {2, {0,2}}));
	assert(!check_action_is_valid(table, {2, {0,2}}, {3, {1,2}}));


	// Front collisions
	std::cout << "Testing front collisions" << std::endl;
	assert(!check_action_is_valid(table, {0, {0,1}}, {1, {0,0}}));
	assert(!check_action_is_valid(table, {1, {0,2}}, {2, {0,1}}));
	assert(!check_action_is_valid(table, {2, {1,2}}, {3, {0,2}}));


	// Available actions
	std::cout << "Testing accceptable actions" << std::endl;
	assert(check_action_is_valid(table, {0, {0,0}}, {1, {1,0}}));
	assert(check_action_is_valid(table, {0, {0,1}}, {1, {0,2}}));
}

void no_collision_pathfinding() {
	ReservationTable reservation_table;

	TimeNode initial_node(0, Node(3,4));
	Node final_node(6,6);
	std::vector<TimeNode> optimal_path = compute_optimal_path(
			initial_node,
			final_node,
			&reservation_table,
			10);

	assert(optimal_path.size() == 6);
}

void corridor_1() {
	// 2 . . a .
	// 1 o . . xb
	// 0 . . . c
	//   0 1 2 3
	//

	ReservationTable reservation_table;

	std::vector<TimeNode> path_1 = {
		{0, {2, 2} }, {1, {1, 2}}, {2, {0, 2}}, {3, {0, 2}}
	};
	std::vector<TimeNode> path_2 = {
		{0, {2, 2} }, {1, {2, 2}}, {2, {2, 2}}, {3, {1, 2}}, {3, {0, 2}}
	};
	std::vector<TimeNode> path_3 = {
		{0, {3, 1} }, {1, {2, 1}}, {2, {1, 1}}, {3, {0, 1}}
	};
	std::vector<TimeNode> path_4 = {
		{0, {3, 0} }, {1, {2, 0}}, {2, {1, 0}}, {3, {0, 0}}
	};

	TimeNode initial_node(0, {0,1});
	Node final_node(3,1);

	reservation_table.reserve_path(path_1);
	reservation_table.reserve_path(path_2);
	reservation_table.reserve_path(path_3);
	reservation_table.reserve_path(path_4);

	std::vector<TimeNode> optimal_path = compute_optimal_path(
			initial_node,
			final_node,
			&reservation_table,
			10);
	
	std::vector<TimeNode> expected_path = {
		{0, {0,1}}, {1, {1,1}}, {2, {1,2}}, {3, {1,1}}, {4, {2,1}}, {5, {3,1}}
	};

	print_path(expected_path);
	print_path(optimal_path);

	assert(optimal_path.size() == expected_path.size());

	for (int i=0; i < optimal_path.size(); i++) {
		TimeNode node_a = expected_path[i];
		TimeNode node_b = optimal_path[i];

		assert(node_a == node_b);
	}
}


int main() {
	reservation_table_checks();

	// A*

	no_collision_pathfinding();

	corridor_1();


	std::cout << "Tests passed successfully" << std::endl;
}
