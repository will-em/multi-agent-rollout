#include "BasePolicy.hpp"
#include "IndexToPair.hpp"
#include "coop-astar.hpp"

std::vector<int> basePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, int agentIdx, int iteration){ 
    int height = env.getHeight();
    int width = env.getWidth();
    int* matrix = env.getMatPtr();

	std::vector<Node> target_positions;

    for(auto el : targets)
        target_positions.push_back(Node(el.first, el.second));

    std::pair<int,int> position = indexToPair(env.getMatrixIndex(agentIdx), width);
    Node node_pos(position.first, position.second);
    remove_from_obstacles(node_pos); 
    return compute_controls_for_single_agent(node_pos, target_positions, iteration, agentIdx);
}
