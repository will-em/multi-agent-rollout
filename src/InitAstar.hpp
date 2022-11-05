#pragma once
#include "Environment.hpp"
#include <vector>

void reservePath(char* paths, float* obstacles, int height, int width, int startIdx, int goalIdx, int numOfTargets, int targetIdx);
void initAstar(char* paths, Environment &env, std::vector<std::pair<int, int>> boxes, std::vector<std::pair<int, int>> dropOffPoints);