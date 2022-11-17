#include "../CoopAstar.hpp"
#include "assert.h"
#include <iostream>


void print_path(std::vector<TimeNode> & path) {
	for (int i = 0; i < path.size(); i++ ) {
		printf("- (%i, (%i, %i)) -", path[i].turn, path[i].node.x, path[i].node.y);
	}
	printf("\n");
}

template <typename T> bool paths_are_equal(std::vector<T> & path_a, std::vector<T> & path_b) {
	if (path_a.size() != path_b.size()) {
		return false;
	}

	for (int i=0; i < path_a.size(); i++) {
		T node_a = path_a[i];
		T node_b = path_b[i];

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
	std::pair<int, int> size = {3, 3};

	ReservationTable table(size, std::vector<Node>());

	std::vector<TimeNode> path = {
		{0, {0, 0}}, {1, {0,1}}, {2, {0,2}}, {3, {1,2}}
	};

	table.reserve_path(path);

	std::cout << "\nTesting reservation table" << std::endl;

	// Out of bounds
	std::cout << "\tTesting out of bounds" << std::endl;
	assert(!check_action_is_valid(table, {0, {0,0}}, {1, {0,-1}}));
	assert(!check_action_is_valid(table, {0, {0,0}}, {1, {-1,0}}));
	assert(!check_action_is_valid(table, {0, {1,2}}, {1, {1,3}}));
	assert(!check_action_is_valid(table, {0, {2,2}}, {1, {3,2}}));
	
	// Cell collisions
	std::cout << "\tTesting cell collisions" << std::endl;
	assert(!check_action_is_valid(table, {0, {0,2}}, {1, {0,1}}));
	assert(!check_action_is_valid(table, {1, {1,2}}, {2, {0,2}}));
	assert(!check_action_is_valid(table, {2, {0,2}}, {3, {1,2}}));


	// Front collisions
	std::cout << "\tTesting front collisions" << std::endl;
	assert(!check_action_is_valid(table, {0, {0,1}}, {1, {0,0}}));
	assert(!check_action_is_valid(table, {1, {0,2}}, {2, {0,1}}));
	assert(!check_action_is_valid(table, {2, {1,2}}, {3, {0,2}}));


	// Available actions
	std::cout << "\tTesting accceptable actions" << std::endl;
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

void no_collision_pathfinding_many_turns() {
	ReservationTable reservation_table({100,100}, std::vector<Node>());

	TimeNode initial_node(000, Node(3,4));
	Node final_node(68,39);
	std::vector<TimeNode> optimal_path = compute_optimal_path(
			initial_node,
			final_node,
			&reservation_table,
			100);

	std::cout << optimal_path.size() << std::endl;
}

void corridor_1() {
	// 0 . . .
	// 1 . . .
	// 2 . . .
	// 3 . . .
	//   0 1 2


	ReservationTable reservation_table({4,3}, std::vector<Node>());

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

	// print_path(optimal_path);

	assert(
		paths_are_equal(
			expected_path,
			optimal_path
		)
	);
}

void corridor_2() {
	// 0 o #
	// 1 . #
	// 2 . .
	// 3 x #
	//   0 1

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


void pathfinder_checks() {
	std::cout << "\nPathfinder checks" << std::endl;

	std::cout << "\tEmpty environment" << std::endl;
	no_collision_pathfinding();

	std::cout << "\tCrowded wide corridor" << std::endl;
	corridor_1();

	std::cout << "\tSmall corridor with hole" << std::endl;
	corridor_2();
}


void complete_simple_corridor() {
	// 0 # # . #
	// 1 a . . b
	//   0 1 2 3

	std::vector<Node> obstacles = {
		{0,0}, {0,1}, {0,3}
	};
	std::vector<Node> initial_positions = {
		{1,0}, {1,3}

	};
	std::vector<Node> final_positions = {
		{1,3}, {1,0}
	};

	std::vector<std::vector<int>> optimal_controls = compute_controls(
		2,
		4,
		obstacles,
		initial_positions,
		final_positions
	);

	std::vector<int> expected_path_a = {4, 4, 4};
	std::vector<int> expected_path_b = {3, 1, 2, 3, 3};

	assert(
		paths_are_equal(expected_path_a, optimal_controls[0])
	);

	assert(
		paths_are_equal(expected_path_b, optimal_controls[1])
	);
}

void complete_corridor_back_and_forth() {
	// 0 a # . .
	// 1 b # # .
	// 2 . . . .
	//   0 1 2 3
	//
	//   This test will fail if the agents do not disappear after reaching their targets

	std::vector<Node> obstacles = {
		{0,1}, {1,1}, {1,2}
	};
	std::vector<Node> initial_positions = {
		{0,0}, {1,0}

	};
	std::vector<Node> final_positions = {
		{0,3}, {0,0}
	};

	std::vector<std::vector<int>> optimal_controls = compute_controls(
		3,
		4,
		obstacles,
		initial_positions,
		final_positions
	);

	std::vector<int> expected_path_a = {2, 2, 4, 4, 4, 1, 1};
	std::vector<int> expected_path_b = {2, 4, 4, 4, 1, 1, 3, 4, 2, 2, 3, 3, 3, 1, 1};

	assert(
		paths_are_equal(expected_path_a, optimal_controls[0])
	);

	assert(
		paths_are_equal(expected_path_b, optimal_controls[1])
	);
}


void check_complete_function() {
	std::cout << "\nComplete Cooperative A* tests:" << std::endl;

	std::cout << "\tSimple corridor problem:" << std::endl;
	complete_simple_corridor();


	std::cout << "\tSlighlty more cumbersome corridor problem:" << std::endl;
 	complete_corridor_back_and_forth();
}

void empty_warehouse_turn_limit() {
	std::vector<Node> obstacles;
	ReservationTable reservation_table({10, 10}, obstacles);

	std::vector<TimeNode> optimal_path_0 = compute_optimal_path(
		{0, {3,2}},
		{8, 8},
		&reservation_table,
		5
	);
	assert(optimal_path_0.size() == 0);

	std::vector<TimeNode> optimal_path_1 = compute_optimal_path(
		{0, {3,2}},
		{8, 8},
		&reservation_table,
		10
	);
	assert(optimal_path_1.size() == 0);

	std::vector<TimeNode> optimal_path_2 = compute_optimal_path(
		{0, {3,2}},
		{8, 8},
		&reservation_table,
		11
	);
	assert(optimal_path_2.size() == 12);

	std::vector<TimeNode> optimal_path_3 = compute_optimal_path(
		{0, {3,2}},
		{8, 8},
		&reservation_table,
		20
	);
	assert(optimal_path_3.size() == 12);
}

void limit_turn_checks() {
	std::cout << "\nChecking turn limit:" << std::endl;
	empty_warehouse_turn_limit();
}

void path_to_action_check() {
	std::cout << "\nPath-to-action check" << std::endl;

	std::vector<TimeNode> path = {
		{0, {1,1}}, // Down  (2)
		{1, {2,1}}, // Right (4)
		{1, {2,2}}, // Up	 (1)
		{1, {1,2}},	// Left  (3)
		{1, {1,1}}, // Stay  (0)
		{1, {1,1}},
	};


	std::vector<int> actions = path_to_actions(path);

	std::vector<int> expected = {
		2, 4, 1, 3, 0
	};

	assert(paths_are_equal(expected, actions));
}


int main() {
	reservation_table_checks();

	// A*
	
	pathfinder_checks();

	no_collision_pathfinding();

	corridor_1();
	corridor_2();
 	
	no_collision_pathfinding_many_turns();

	path_to_action_check();

	// TODO: Test complete function
	limit_turn_checks();

	std::cout << "Tests passed successfully" << std::endl;
}
