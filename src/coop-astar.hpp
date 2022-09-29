#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <vector>
#include <utility>

typedef std::pair<int, int> Node;
typedef std::pair<int, Node> TimeNode;


// Hash functions for using sets of nodes and time nodes

struct NodeHash {
	std::size_t operator() (const std::pair<int, int> &pair) const {
		return std::hash<int>()(pair.first) ^ std::hash<int>()(pair.second);
	}
};

struct TimeNodeHash {
	std::size_t operator() (const std::pair<int, Node> &pair) const {
		return std::hash<int>()(pair.first) ^ NodeHash()(pair.second);
	}
};

struct PairTimeNodeHash {
	std::size_t operator() (const std::pair<TimeNode, TimeNode> &pair) const {
		return TimeNodeHash()(pair.first) ^ TimeNodeHash()(pair.second);
	}
};



// Structure for holding all the information of the paths already
// reserved by other agents

struct ReservationTable {
	public:
		std::unordered_set<TimeNode, TimeNodeHash> cells;
		std::unordered_set<std::pair<TimeNode,TimeNode>, PairTimeNodeHash> fronts;

		ReservationTable();
		~ReservationTable();

		// Takes a path a inserts elements in the reservation table so any other path
		// that avoids the table will not collide with this path
		int reserve_path(std::vector<TimeNode> path);

		// Check whther a movement from a node in time to another does not conflict in
		// the reservation table
		bool action_is_valid(TimeNode node, TimeNode next_node);
};


// Code for A* with time dimensions

struct NodeInQueue {
	public:
		TimeNode node;
		int distance_to_target;
};

bool operator<(const NodeInQueue &n1, const NodeInQueue &n2) {
  return n1.node.first + n1.distance_to_target > n2.node.first + n2.distance_to_target;
}

struct AStarFinder {
	public:
  		std::priority_queue<NodeInQueue> queue;
		std::unordered_map<TimeNode, TimeNode, TimeNodeHash> expanded_nodes;
		Node target;

		int expand_next_in_queue();
		int generate_path(Node origin);
};
