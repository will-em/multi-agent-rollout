#include "../coop-astar.hpp"
#include "assert.h"
#include <iostream>


void print_path(std::vector<TimeNode> & path) {
	for (int i = 0; i < path.size(); i++ ) {
		printf("- (%i, (%i, %i)) -", path[i].turn, path[i].node.x, path[i].node.y);
	}
	printf("\n");
}

bool paths_are_equal(std::vector<TimeNode> & path_a, std::vector<TimeNode> & path_b) {
	if (path_a.size() != path_b.size()) {
		return false;
	}

	for (int i=0; i < path_a.size(); i++) {
		TimeNode node_a = path_a[i];
		TimeNode node_b = path_b[i];

		if (node_a != node_b){
			return false;
		}
	}

	return true;
};

bool check_action_is_valid(ReservationTable & table, TimeNode node_a, TimeNode node_b) {
	Node clearance_node(-1,-1);
	return table.action_is_valid(node_a, node_b, clearance_node);
}

void reservation_table_checks() {
	std::pair<int, int> size = {10, 10};

	ReservationTable table(size, std::vector<Node>());

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
	ReservationTable reservation_table({10,10}, std::vector<Node>());

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

	ReservationTable reservation_table({10,10}, std::vector<Node>());

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

	assert(
		paths_are_equal(
			expected_path,
			optimal_path
		)
	);



}

void corridor_2() {
	//
	// 1 # . # #
	// 0 o . . x 
	//   0 1 2 3

	std::vector<Node> static_obstacles = {
		{0,1}, {2,1}, {3,1}, {3,0}
	};

	ReservationTable reservation_table({4,2}, static_obstacles);

	std::vector<TimeNode> path_1 = {
		{0, {3,0}}, {1, {2,0}}, {2, {1,0}}, {3, {0,0}}
	};

	reservation_table.reserve_path(path_1);

	TimeNode initial_node(0, Node(0,0));
	Node final_node(3,0);

	std::vector<TimeNode> optimal_path = compute_optimal_path(
		initial_node,
		final_node,
		&reservation_table,
		10
	);

	std::vector<TimeNode> expected_path = {
		{0, {0,0}}, {1, {1,0}}, {2, {1,1}}, {3, {1,0}}, {4, {2,0}}, {5, {3,0}}
	};

	// print_path(optimal_path);

	assert(
		paths_are_equal(
			expected_path,
			optimal_path
		)
	);
}


int main() {
	reservation_table_checks();

	// A*

	no_collision_pathfinding();

	corridor_1();
	corridor_2();


	std::cout << "Tests passed successfully" << std::endl;
}
