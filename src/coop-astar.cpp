#include "coop-astar.hpp"
#include <iostream>

Node::Node(int x, int y): x(x), y(y) {};
TimeNode::TimeNode(int turn, Node node): turn(turn), node(node) {};

bool operator==(const Node &n1, const Node &n2) {
	return n1.x == n2.x && n1.y == n2.y;
};

ReservationTable::ReservationTable() {
	cells = std::unordered_set<TimeNode, TimeNodeHasher>();
	fronts = std::unordered_set<std::pair<TimeNode, TimeNode>, PairTimeNodeHasher>();
}

bool operator==(const TimeNode &n1, const TimeNode &n2) {
	return n1.turn == n2.turn && n1.node == n2.node;
};

bool operator<(const NodeInQueue &n1, const NodeInQueue &n2) {
  return n1.node.turn + n1.distance_to_target > n2.node.turn + n2.distance_to_target;
}
bool operator>(const NodeInQueue &n1, const NodeInQueue &n2) {
  return n1.node.turn + n1.distance_to_target < n2.node.turn + n2.distance_to_target;
}


std::size_t NodeHasher::operator() (const Node &node) const {
	return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
};

std::size_t TimeNodeHasher::operator() (const TimeNode &time_node) const {
	return std::hash<int>()(time_node.turn) ^ NodeHasher()(time_node.node);
}

std::size_t PairTimeNodeHasher::operator() (const std::pair<TimeNode, TimeNode> &pair) const {
	return TimeNodeHasher()(pair.first) ^ TimeNodeHasher()(pair.second);
}


bool ReservationTable::action_is_valid(TimeNode node, TimeNode next_node) {
	if (cells.find(next_node) != cells.end()) {
		return false;
	}

	std::pair<TimeNode, TimeNode> jump(node, next_node);

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
		this->fronts.insert(
			{
				TimeNode(last_node.turn, path[i].node),
				TimeNode(path[i].turn, last_node.node),
			}
		);

		last_node = path[i];
	}

	return 1;
}




//
//
//		A*
//
//

int AStarFinder::expand_next_in_queue() {
	// Obtain best node in queue
	TimeNode parent_node = this->queue.top().node;


	// Check reference is null, and return -2 in that case


	this->queue.pop();

	// Add it to the expanded_nodes
	std::pair<int, int> delta_array[] = {{0,0}, {0,1}, {1,0}, {0,-1}, {-1,0}};

	for (int i=0; i<5; i++) {
		std::pair<int,int> delta = delta_array[i];


		// TODO: Check that the node is valid. If not, continue loop

		Node new_node(parent_node.node.x + delta.first, parent_node.node.y + delta.second);
		TimeNode new_tnode(parent_node.turn + 1, new_node);

		int heuristic_distance = 0;


		this->queue.push(
			NodeInQueue(new_tnode, heuristic_distance)
		);
	}

	if (parent_node.node == this->target) {
		return parent_node.turn;
	}


	// Change stuff

	return -1;
}

NodeInQueue::NodeInQueue(TimeNode node, int distance_to_target) :
	node(node), distance_to_target(distance_to_target) {};


