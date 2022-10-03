#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <vector>
#include <utility>
#include "math.h"

// A node represents a position in 2 dimensions:x and y
class Node {
	public:
		int x;
		int y;
		
		Node(int x, int y);
};

// A time-node is a node with an additional coordinate: time. The time is given in turns
class TimeNode {
	public:
		int turn;
		Node node;

		TimeNode(int turn, Node node);
};

bool operator==(const Node &n1, const Node &n2);
bool operator==(const TimeNode &n1, const TimeNode &n2);



// Hash functions for using sets of nodes and time nodes. These classes are necessary for creating
// sets and maps that use nodes, time-nodes and pairs of time-nodes as keys.

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
// reserved by other agents.

class ReservationTable {
	private:
		std::unordered_set<TimeNode, TimeNodeHasher> cells;
		std::unordered_set<std::pair<TimeNode,TimeNode>, PairTimeNodeHasher> fronts;

	public:
		// A reservation table is generated completely empty by default
		ReservationTable();

		// Takes a path a inserts elements in the reservation table so any other path
		// that avoids the table will not collide with this path
		int reserve_path(std::vector<TimeNode> path);

		// Check whther a movement from a node in time to another does not conflict in
		// the reservation table. Returns true if the actions does not conflict with
		// the reservation table
		bool action_is_valid(TimeNode & node, TimeNode & next_node);
};


// Code for A* with time dimensions

// The Manhattan distance is a simple heuristic that provides a very fast lower bound
// of the distance between two cells in a 2D space.
//
// Its result is the minimum distance between two nodes if there were neither static nor
// dynamic obstacle in the warehouse.
int compute_manhattan_distance(Node &node_a, Node &node_b);




// Nodes are stored in the priority queue with their heuristic distance to the target node.
class NodeInQueue {
	public:
		TimeNode node;
		int distance_to_target;

		NodeInQueue(TimeNode node, int distance_to_target);
};

// Nodes in the queue must be comparable, so a priority can exist.
bool operator<(const NodeInQueue &n1, const NodeInQueue &n2);
bool operator>(const NodeInQueue &n1, const NodeInQueue &n2);



// Obtains the optimal path between two nodes
std::vector<TimeNode> compute_optimal_path(
		TimeNode initial_node,
		Node final_node,
		ReservationTable * reservation_table,
		int max_turns
	);



// Output of the "expand_next_in_queue" method in the "AStarFinder" class. Returns information about
// how to process once this methods has been called.
class IterationStatus {
	public:
		// The status returns the basic information of the iteration. It can take values of -1, 0 or 1.
		// 	-1:
		// 		The queue is empty, so no more nodes can iterations can be performed. The search of
		// 		an optimal path should be aborted, as all options have been exhausted.
		//
		//   0:
		//   	A normal iteration, with none of the expanded nodes being the target one. Then, more
		//   	iteration should be exectuted.
		//
		//   1:
		//   	The target node was expanded, so the "generate_path" method should be called to obtain
		//   	the optimal path
		int status;

		// Only for status 0 or 1 (that is, the queue is not empty and nodes were expanded), returns
		// the turn of the expanded nodes. This is necessary to know at which turn the target was reached
		// (for status 1) or to stop the search if the nodes being expanded have a very large value of time.
		int expansion_turn;

		// Simple constructor
		IterationStatus(int status, int expansion_turn);
};

// An structure for obtaining an optimal path in a 2D warehouse, with static and dynamic obstacles.
//
// A particularity of this algorithm is that it considers a node "expanded" as soon as it is reached.
class AStarFinder {
	public:
		ReservationTable * reservation_table;
  		std::priority_queue<NodeInQueue> queue;
		int queue_length;
		std::unordered_map<TimeNode, TimeNode, TimeNodeHasher> expanded_nodes;
		Node target;


		// The class is constructed by providing the initial node, and the target one. The search 
		// has an initial turn, but not a fixed final one, so the target is a node and not a 
		// time-node.
		AStarFinder(TimeNode origin, Node target, ReservationTable * reservation_table);

		// Performs one iteration of the A* algorithm, taking the best node from the queue, expanding
		// all connected nodes and adding them to the queue.
		IterationStatus expand_next_in_queue();

		// Generates a path that goes from the initial node to the final one.
		std::vector<TimeNode> generate_path(TimeNode final_node);
};
