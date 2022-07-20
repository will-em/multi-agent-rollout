#pragma once
#include <iostream>
/**
    Converts a flattened position index into to a 2D-coordinate

    @param i Flattened index
    @param width Width of environment 
    @return The corresponding 2D-coordinate
*/
std::pair<int, int> indexToPair(int i, const int width);