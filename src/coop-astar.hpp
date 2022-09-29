#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <vector>
#include <utility>
#include "math.h"

class Node {
	public:
		int x;
		int y;
		
		Node(int x, int y);
};

class TimeNode {
	public:
		int turn;
		Node node;

		TimeNode(int turn, Node node);
};


// Hash functions for using sets of nodes and time nodes

class NodeHasher {
	public:
		std::size_t operator() (const Node &node) const;
};

class TimeNodeHasher {
	public:
		std::size_t operator() (const TimeNode &time_node) const;
};

class PairTimeNodeHasher {
	public:
		std::size_t operator() (const std::pair<TimeNode, TimeNode> &pair) const;
};



// Structure for holding all the information of the paths already
// reserved by other agents

class ReservationTable {
	public:
		std::unordered_set<TimeNode, TimeNodeHasher> cells;
		std::unordered_set<std::pair<TimeNode,TimeNode>, PairTimeNodeHasher> fronts;

		ReservationTable();

		// Takes a path a inserts elements in the reservation table so any other path
		// that avoids the table will not collide with this path
		int reserve_path(std::vector<TimeNode> path);

		// Check whther a movement from a node in time to another does not conflict in
		// the reservation table
		bool action_is_valid(TimeNode node, TimeNode next_node);
};


// Code for A* with time dimensions

class NodeInQueue {
	public:
		TimeNode node;
		int distance_to_target;

		NodeInQueue(TimeNode node, int distance_to_target);
};

bool operator<(const NodeInQueue &n1, const NodeInQueue &n2) {
  return n1.node.turn + n1.distance_to_target > n2.node.turn + n2.distance_to_target;
}
bool operator>(const NodeInQueue &n1, const NodeInQueue &n2) {
  return n1.node.turn + n1.distance_to_target < n2.node.turn + n2.distance_to_target;
}

class AStarFinder {
	public:
  		std::priority_queue<NodeInQueue> queue;
		std::unordered_map<TimeNode, TimeNode, TimeNodeHasher> expanded_nodes;
		Node target;

		int expand_next_in_queue();
		int generate_path(Node origin);
};
