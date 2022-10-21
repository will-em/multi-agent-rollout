#include "coop-astar.hpp"
#include <iostream>

Node::Node(int y, int x): y(y), x(x) {}
TimeNode::TimeNode(int turn, Node node): turn(turn), node(node) {}

bool operator==(const Node &n1, const Node &n2) {
	return n1.x == n2.x && n1.y == n2.y;
}

bool operator!=(const Node &n1, const Node &n2) {
	return !(n1 == n2);
}


ReservationTable::ReservationTable(std::pair<int,int> table_size, std::vector<Node> obstacles) {
	cells = std::unordered_set<TimeNode, TimeNodeHasher>();
	fronts = std::unordered_set<std::pair<TimeNode, TimeNode>, PairTimeNodeHasher>();
	static_cells = std::unordered_set<Node, NodeHasher>();
	size = table_size;

	for (int i=0; i<obstacles.size(); i++) {
		static_cells.insert(obstacles[i]);
	}
}

ReservationTable::ReservationTable(){
	cells = std::unordered_set<TimeNode, TimeNodeHasher>();
	fronts = std::unordered_set<std::pair<TimeNode, TimeNode>, PairTimeNodeHasher>();
	static_cells = std::unordered_set<Node, NodeHasher>();
}

ReservationTable global_reservation_table;
int max_turns;
bool operator==(const TimeNode &n1, const TimeNode &n2) {
	return n1.turn == n2.turn && n1.node == n2.node;
}

bool operator!=(const TimeNode &n1, const TimeNode &n2) {
	return !(n1 == n2);
}


bool operator<(const NodeInQueue &n1, const NodeInQueue &n2) {
  return n1.node.turn + n1.distance_to_target > n2.node.turn + n2.distance_to_target;
}
bool operator>(const NodeInQueue &n1, const NodeInQueue &n2) {
  return n1.node.turn + n1.distance_to_target < n2.node.turn + n2.distance_to_target;
}


std::size_t NodeHasher::operator() (const Node &node) const {
	//std::cout << (10032 * node.x + 20079 * node.y) << '\n'; 
	//return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
	return 10032 * node.x + 20079 * node.y;
}

std::size_t TimeNodeHasher::operator() (const TimeNode &time_node) const {
	//std::cout << (NodeHasher()(time_node.node) ^ (time_node.turn * 30080)) << '\n';
	return (NodeHasher()(time_node.node) ^ (time_node.turn * 30080));
	//return std::hash<int>()(time_node.turn) ^ NodeHasher()(time_node.node);
}

std::size_t PairTimeNodeHasher::operator() (const std::pair<TimeNode, TimeNode> &pair) const {
	return TimeNodeHasher()(pair.first) ^ TimeNodeHasher()(pair.second);
}


bool ReservationTable::action_is_valid(TimeNode & node, TimeNode & next_node, Node & clearance_node) {
	if (next_node.node.x < 0
		|| next_node.node.x >= size.second
		|| next_node.node.y < 0
		|| next_node.node.y >= size.first) {

		return false;
	}

	if(node.node.x != next_node.node.x || node.node.y != next_node.node.y){
		if (static_cells.find(next_node.node) != static_cells.end()
				&& next_node.node != clearance_node) {
			return false;
		}
	}

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

	for (int i = 1; i < path.size(); i++) {
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

	max_turns = max_turns + initial_node.turn;

	// Creates an A* pathfinder
	AStarFinder a_star(initial_node, final_node, reservation_table);

	bool finished = false;
	bool failed_search = false;
	int arrival_turn;

	while (!finished) {

		IterationStatus last_iter_status = a_star.expand_next_in_queue();

		if (last_iter_status.status == -1) {
			// The queue is empty, so the search finishes with no results
			failed_search = true;
			finished = true;

		} else if (last_iter_status.status == 1) {
			// The target node was reached
			finished = true;
			arrival_turn = last_iter_status.expansion_turn;

		} else if (last_iter_status.expansion_turn >= max_turns) {
			// A node was expanded, but its turn is over the maximum one so the search
			// finishes unsuccesfully
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
	status(status), expansion_turn(expansion_turn) {}



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

		Node new_node(parent_node.node.y + delta.first, parent_node.node.x + delta.second);
		TimeNode new_tnode(parent_node.turn + 1, new_node);
		if (this->expanded_nodes.count(new_tnode) != 0) { continue; }

		if (!this->reservation_table->action_is_valid(parent_node, new_tnode, this->target)) {
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
	node(node), distance_to_target(distance_to_target) {}




std::vector<std::vector<int>> compute_controls(
		int height,
		int width,
		std::vector<Node> obstacles, 
		std::vector<Node> initial_positions,
		std::vector<Node> target_positions) {

	std::vector<std::vector<int>> optimal_actions;

	int max_turns = (height + width)*3;

	ReservationTable reservation_table({height, width}, obstacles);


	for (int i=0; i<initial_positions.size(); i++) {
		TimeNode initial_node = {0, initial_positions[i]};
		Node target_node = target_positions[i];

		std::vector<TimeNode> optimal_path = compute_optimal_path(
			initial_node,
			target_node,
			&reservation_table,
			max_turns
		);

		std::vector<int> agent_actions = path_to_actions(optimal_path);

		optimal_actions.push_back(agent_actions);

		reservation_table.reserve_path(optimal_path);

	}

	return optimal_actions;
}

std::vector<int> compute_controls_for_single_agent(
		Node initial_position,
		std::vector<Node> &target_positions,
		int curr_time,
		int agent_idx) {

	std::vector<std::vector<int>> optimal_actions;

	TimeNode initial_node = {curr_time, initial_position};
	Node target_node = target_positions[agent_idx];

	std::vector<TimeNode> optimal_path = compute_optimal_path(
		initial_node,
		target_node,
		&global_reservation_table,
		max_turns
	);

	global_reservation_table.reserve_path(optimal_path);
	std::vector<int> agent_actions = path_to_actions(optimal_path);

	std::reverse(agent_actions.begin(), agent_actions.end());

	return agent_actions;
}

void init_reservation_table(int height, int width, std::vector<Node> obstacles){
	global_reservation_table = ReservationTable({height, width}, obstacles);

	max_turns = (height + width);
}


std::unordered_set<Node, NodeHasher> & ReservationTable::static_cells_ref(){
	return static_cells;
}

void remove_from_obstacles(Node to_be_removed){
	auto static_cells = global_reservation_table.static_cells_ref();
	static_cells.erase(to_be_removed);

}

std::vector<int> path_to_actions(std::vector<TimeNode> & path) {
	std::vector<int> actions;

	for (int i=0; i < path.size()-1; i++) {

		// 0: stand still
		// 1: up
		// 2: down
		// 3: left
		// 4: right

		int turn_action;

		Node current_node = path[i].node;
		Node next_node = path[i+1].node;

		if (current_node == next_node) {
			turn_action = 0;

		} else if (next_node.y == current_node.y && next_node.x == current_node.x + 1) {
			turn_action = 4;

		} else if (next_node.y == current_node.y && next_node.x == current_node.x - 1) {
			turn_action = 3;

		} else if (next_node.y == current_node.y + 1 && next_node.x == current_node.x) {
			turn_action = 2;

		} else if (next_node.y == current_node.y - 1 && next_node.x == current_node.x) {
			turn_action = 1;

		}

		actions.push_back(turn_action);
	}

	return actions;
}
