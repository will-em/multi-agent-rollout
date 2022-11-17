#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <vector>
#include <utility>
#include <algorithm>
#include "math.h"

/*
	A node represents a position in the warehouse, and is defined by two coordinates: i and j.
	Both j and i must be positive, and the position (0,0) is the upper left corner of the warehouse.
*/
class Node {
	public:
		// Vertical position of the node, with 0 as the upper row
		int i;
		// Horizontal position of the node, with 0 as the left column
		int j;
		
		// Generates a new node with the specified coordinates
		Node(int i, int j);
};

/*
	A TimeNode represents a position in space but also in time, given as the
	number of turns since the initialization of the problem. This allows
	the representation of dynamic elements in the warehouse, such as robots.

	The main purpose of TimeNodes is to represent the path of a robot through
	the warehouse. 
*/
class TimeNode {
	public:
		// Time dimension of the TimeNode
		int turn;
		// Position dimension of the TimeNode
		Node node;

		// Creates a TimeNode with at the specified turn and position
		TimeNode(int turn, Node node);
};

// Equality and inequality operators for Node and TimeNode
bool operator==(const Node &n1, const Node &n2);
bool operator==(const TimeNode &n1, const TimeNode &n2);
bool operator!=(const Node &n1, const Node &n2);
bool operator!=(const TimeNode &n1, const TimeNode &n2);



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

/*
	A reservation table holds the information required to generate collision-free
	paths throught a warehouse with static and dynamic obstacles. These obstacles
	are stored in three different HashSets so the time required to determine a
	collision is constant.
*/
class ReservationTable {
	private:
		// Nodes that are occupied at a specific turn
		std::unordered_set<TimeNode, TimeNodeHasher> cells;
		// Edges between two nodes that are occupied at a specific turn
		std::unordered_set<std::pair<TimeNode,TimeNode>, PairTimeNodeHasher> fronts;
		// Nodes that are occupied during the whole problem, regardless of the turn
		std::unordered_set<Node, NodeHasher> static_cells;

	public:
		// Size of the warehouse to which this rerservation table corresponds.
		std::pair<int, int> size;

		/*
			Generates an empty reservation table with the specified size and
			static obstacles.

			@param size Height and width of the warehouse on which the reservation
						table is used
			@param obstacles Vector with the coordinates of static obstacles in the
							 warehouse
			@returns An instance of a reservation table with the specified static
				obstacles and no dynamic obstacles
		*/
		ReservationTable(std::pair<int,int> size, std::vector<Node> obstacles);


		/*
			Generates an empty reservation table with no specified size and no
			static obstacles.
			
			@param size Height and width of the warehouse on which the reservation
				table is used
			@param obstacles Vector with the coordinates of static obstacles in the
				warehouse
			@returns An instance of a reservation table with no static obstacles
				and no dynamic obstacles
		*/
		ReservationTable();

		/*
			Fills the reservation table with the given path, so a new path that is
			generated  using this table is guaranteed to not collide with the given path.

			@param path Vector of TimeNodes representing the path of a robot through the
				warehouse
			@returns 1 if the operation is succesful, -1 otherwise
		*/
		int reserve_path(std::vector<TimeNode> path);

		/*
			Check whether an action of an agent does not conflict with any path
			that has been added to the reservation table.

			@param node The node in which the agent is located
			@param next_node The node to which the agent wants to move
			@param node clearance_node Location which the agent is able to
				access even if there is a static obstacle there.
			@returns True if the action does not conflict with the reservation
				table, false otherwise
		
		*/
		bool action_is_valid(TimeNode & node, TimeNode & next_node, Node & clearance_node);

		/*
			Obtains a reference to the hashset with the static cells in the
			reservation table

			@returns A reference to the hashset of static cells in the reservation table
		*/
	    std::unordered_set<Node, NodeHasher> * static_cells_ref();	
};


/*
	Remove a static obstacle from the static cells of the reservation table

	@params to_be_removed Coordinates of the static obstacle that is being removed
*/
void remove_from_obstacles(Node to_be_removed);

// Code for A* with time dimensions

// The Manhattan distance is a simple heuristic that provides a very fast lower bound
// of the distance between two cells in a 2D space.
//
// Its result is the minimum distance between two nodes if there were neither static nor
// dynamic obstacle in the warehouse.
/*
	Obtain the Manhattan distance between two nodes. This distance is computed as:
	abs(x_a - x_b) + abs(y_a - y_b).

	@param node_a First node
	@param node_b Second node
	@returns Manhattan distance between the nodes provided
*/
int compute_manhattan_distance(Node &node_a, Node &node_b);




/*
	TimeNode stored in the queue of an instance of AStarFinder. It represents
	a single TimeNode, along with its precomputed manhattan distance to the
	target
*/
class NodeInQueue {
	public:
		TimeNode node;
		int distance_to_target;

		// Creates a NodeInQueue
		NodeInQueue(TimeNode node, int distance_to_target);
};

// Nodes in the queue must be comparable, so a priority can exist.
bool operator<(const NodeInQueue &n1, const NodeInQueue &n2);
bool operator>(const NodeInQueue &n1, const NodeInQueue &n2);



// Obtains the optimal path between two nodes, respecting the reservation table
/*
	Obtains a sequence of TimeNodes that represents the shortest path from one node
	to another respecting the reservation table, that is, without colliding with any
	of the obstacles represented by this table.

	The function also takes a maximum number of turns. If the algorithm detects that
	the agent is not able to reach the target in that time, the search is cancelled,
	and an empty vector is returned.

	@param initial_node Position and turn in which an agent starts the pathfinding
	@param final_node Target node that must be reached as early as possible by the agent
	@param reservation_table The table that contains the obstacles inside the warehouse
	@param max_turns The maximum number of turns the agent is allowed to take to reach the target
	@returns A sequence of TimeNodes representing the optimal path of the agent. If no path exists,
		or takes longer than max_turns, an empty vector is returned.
*/
std::vector<TimeNode> compute_optimal_path(
		TimeNode initial_node,
		Node final_node,
		ReservationTable * reservation_table,
		int max_turns
	);



// Output of the expand_next_in_queue method in the AStarFinder class. Returns information about
// how to process once this methods has been called.
class IterationStatus {
	public:
		/*
			The status returns the basic information of the iteration. It can take values of -1, 0 or 1.
				-1:
					The queue is empty, so no more nodes can iterations can be performed. The search of
					an optimal path should be aborted, as all options have been exhausted.
		
			  0:
			  	A normal iteration, with none of the expanded nodes being the target one. Then, more
			  	iteration should be exectuted.
		
			  1:
			  	The target node was expanded, so the "generate_path" method should be called to obtain
			  	the optimal path
		*/
		int status;

		/*
			Only for status 0 or 1 (that is, the queue is not empty and nodes were expanded), returns
			the turn of the expanded nodes. This is necessary to know at which turn the target was reached
			(for status 1) or to stop the search if the nodes being expanded have a very large value of time.
		*/
		int heuristic_arrival_turn;

		// Simple constructor
		IterationStatus(int status, int heuristic_arrival_turn);
};

/*
	An structure for obtaining an optimal path in a 2D warehouse, with static and dynamic obstacles.

	A particularity of this algorithm is that it considers a node "expanded" as soon as it is reached.
*/
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

/*
	Takes a sequence of time-nodes and converts it to a sequence of actions,
	such that a robot executing them would occupy the nodes given by the input
	at each turn

	@param path The sequence of TimeNodes to be occupied by the agent
	@returns The sequence of actions that will move the agent along the provided path
*/
std::vector<int> path_to_actions(std::vector<TimeNode> & path);


/*
	Take an environment size, a vector of obstacles and the initial and final positions
	for several agents and compute their optimal paths using Cooperative A*

	@param height The height of the warehouse in wich the pathfinding takes place
	@param width The with of the warehouse in wich the pathfinding takes place
	@param obstacles The static obstacles of the warehouse
	@param initial_positions Vector with the initial position of each agent
	@param target_positions Vector with the target position of each agent
	@returns A vector containing the sequence of actions to be performed by each agent so
		they reach their respetive targets
*/
std::vector<std::vector<int>> compute_controls(
		int height,
		int width,
		std::vector<Node> obstacles, 
		std::vector<Node> initial_positions,
		std::vector<Node> target_positions);

/*
	Compute the optimal controls for a single agent in a warehouse,
	so it reaches its target in the minimum time while avoiding the obstacles
	given by the global variable global_reservation_table.

	@param initial_positino Initial position of the agent
	@param target_positions A vector with the target position of each agent
	@param curr_time The turn in which the agent begins the pathfinding
	@param agent_idx The index of the agent
	@returns The sequence of instructions that takes the agent to the target
		in the minimum time possible while avoiding obstacles
*/
std::vector<int> compute_controls_for_single_agent(
		Node initial_position,
		std::vector<Node> &target_positions,
		int curr_time,
		int agent_idx);

/*
	Initializes the global reservation table

    @param height The height of the warehouse for this table
	@param width The width of the warehouse for this table
	@param obstacles A vector with the positions of all static obstacles in
		the warehouse
*/
void init_reservation_table(int height, int width, std::vector<Node> obstacles);

