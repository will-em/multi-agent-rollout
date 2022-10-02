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


bool ReservationTable::action_is_valid(TimeNode & node, TimeNode & next_node) {
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

int compute_manhattan_distance(Node &node_a, Node &node_b) {
	return std::abs(node_a.x - node_b.x) + std::abs(node_a.y - node_b.y);
}



std::vector<TimeNode> compute_optimal_path(
		TimeNode initial_node,
		Node final_node,
		ReservationTable * reservation_table,
		int max_turns) {

	AStarFinder a_star(initial_node, final_node, reservation_table);

	bool finished = false;
	bool failed_search = false;
	int arrival_turn;

	while (!finished) {
		IterationStatus last_iter_status = a_star.expand_next_in_queue();

		if (last_iter_status.status == -1) {
			failed_search = true;
			finished = true;

		} else if (last_iter_status.status == 1) {
			finished = true;
			arrival_turn = last_iter_status.expansion_turn;

		} else if (last_iter_status.expansion_turn >= max_turns) {
			failed_search = true;
			finished = true;
		}
	}

	if (failed_search) {
		return std::vector<TimeNode>();
	} else {
		return a_star.generate_path(TimeNode(arrival_turn, final_node));
	}

	return std::vector<TimeNode>();
}


IterationStatus::IterationStatus(int status, int expansion_turn):
	status(status), expansion_turn(expansion_turn) {};



AStarFinder::AStarFinder(
		TimeNode origin,
		Node new_target,
		ReservationTable * reservation_table
		) : reservation_table(reservation_table), target(new_target) {

	queue = std::priority_queue<NodeInQueue>();
	NodeInQueue q_node = NodeInQueue(origin, 0);
	queue.push(q_node);
	queue_length = 1;
	expanded_nodes = std::unordered_map<TimeNode, TimeNode, TimeNodeHasher>();

	std::pair<TimeNode, TimeNode> first_insertion(origin, origin);

	expanded_nodes.insert(first_insertion);
}

IterationStatus AStarFinder::expand_next_in_queue() {
	// Obtain best node in queue
	if (this->queue_length == 0) {
		return IterationStatus(-1, 0);
	};

	TimeNode parent_node = this->queue.top().node;


	this->queue.pop();


	// Add it to the expanded_nodes
	std::pair<int, int> delta_array[] = {{0,0}, {0,1}, {1,0}, {0,-1}, {-1,0}};

	for (int i=0; i<5; i++) {
		std::pair<int,int> delta = delta_array[i];


		Node new_node(parent_node.node.x + delta.first, parent_node.node.y + delta.second);
		TimeNode new_tnode(parent_node.turn + 1, new_node);

		// TODO: Check that the node is valid. If not, continue loop
		if (!this->reservation_table->action_is_valid(parent_node, new_tnode)) {
			continue;
		}
		// Still consider static obstacles outside the reservation table


		int heuristic_distance = compute_manhattan_distance(new_node, this->target);


		this->queue.push(
			NodeInQueue(new_tnode, heuristic_distance)
		);


		this->expanded_nodes.insert({new_tnode, parent_node});

		if (parent_node.node == this->target) {
			return IterationStatus(1, parent_node.turn);
		}
	}

	// Change stuff

	return IterationStatus(0, parent_node.turn);
}

std::vector<TimeNode> AStarFinder::generate_path(TimeNode final_node) {
	std::vector<TimeNode> reverse_path = std::vector<TimeNode>();
	reverse_path.push_back(final_node);

	while (true) {
		TimeNode last_path_node = reverse_path[reverse_path.size()-1];

		TimeNode prev_node = this->expanded_nodes.at(last_path_node);

		if (prev_node == reverse_path[reverse_path.size()-1]) {
			break;
		}

		reverse_path.push_back(prev_node);

	}

	std::vector<TimeNode> path = std::vector<TimeNode>();

	int path_length = reverse_path.size();

	for (int i = 0; i < path_length ; i++) {
		path.push_back(reverse_path[reverse_path.size()-1]);
		reverse_path.pop_back();
	}

	return path;
}

NodeInQueue::NodeInQueue(TimeNode node, int distance_to_target) :
	node(node), distance_to_target(distance_to_target) {};


