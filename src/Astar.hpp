// represents a single pixel
class Node {
  public:
	int turn;	 // turn associated with the node
    int idx;     // index in the flattened grid
    float cost;  // cost of traversing this pixel

    Node(int t, int i, float c) : turn(t),idx(i),cost(c) {}
};

// the top of the priority queue is the greatest element by default,
// but we want the smallest, so flip the sign
bool operator<(const Node &n1, const Node &n2);


bool operator==(const Node &n1, const Node &n2);

// See for various grid heuristics:
// http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#S7
// L_\inf norm (diagonal distance)
float linf_norm(int i0, int j0, int i1, int j1);

// L_1 norm (manhattan distance)
float l1_norm(int i0, int j0, int i1, int j1);

// weights:        flattened h x w grid of costs
// h, w:           height and width of grid
// start, goal:    index of start/goal in flattened grid
// diag_ok:        if true, allows diagonal moves (8-conn.)
// paths (output): for each node, stores previous node in path
extern "C" bool astar(
      const float* weights, const int h, const int w,
      const int start, const int goal, bool time_dimension,
      int* paths);