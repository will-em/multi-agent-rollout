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

int compute_manhattan_distance(Node &node_a, Node &node_b);

class IterationStatus {
	public:
		int status;
		int expansion_turn;
		IterationStatus(int status, int expansion_turn);
};

class NodeInQueue {
	public:
		TimeNode node;
		int distance_to_target;

		NodeInQueue(TimeNode node, int distance_to_target);
};

std::vector<TimeNode> compute_optimal_path(TimeNode initial_node, Node final_node, int max_turns);

class AStarFinder {
	public:
  		std::priority_queue<NodeInQueue> queue;
		int queue_length;
		std::unordered_map<TimeNode, TimeNode, TimeNodeHasher> expanded_nodes;
		Node target;

		AStarFinder(TimeNode origin, Node target);
		IterationStatus expand_next_in_queue();
		std::vector<TimeNode> generate_path(TimeNode initial_node);
};
