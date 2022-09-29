#include "coop-astar.hpp"
#include "assert.h"


int main() {
	ReservationTable table;
	std::vector<TimeNode> path = {
		{0, {0, 0}}, {1, {0,1}}, {2, {0,2}}, {3, {1,2}}
	};

	table.reserve_path(path);

	
	// Cell collisions
	assert(!table.action_is_valid({0, {0,2}}, {1, {0,1}}));
	assert(!table.action_is_valid({1, {1,2}}, {2, {0,2}}));
	assert(!table.action_is_valid({2, {0,2}}, {3, {1,2}}));


	// Front collisions
	assert(!table.action_is_valid({0, {0,1}}, {1, {0,0}}));
	assert(!table.action_is_valid({1, {0,2}}, {2, {0,1}}));
	assert(!table.action_is_valid({2, {1,2}}, {3, {0,2}}));


	// Available actions
	assert(table.action_is_valid({0, {0,0}}, {1, {1,0}}));
	assert(table.action_is_valid({0, {0,1}}, {1, {0,2}}));
}
