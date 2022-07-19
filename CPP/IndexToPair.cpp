#include "IndexToPair.hpp"

std::pair<int, int> indexToPair(int i, const int width){
    int x = (int) floor(i / width);
    int y = i % width; 

    std::pair target = {x, y};
    return target;
}