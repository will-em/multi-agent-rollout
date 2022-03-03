#include "Environment.h"
#include <iostream>

Environment::Environment() : m_stepCount(0) {
    m_matrix = std::vector< std::vector <int> > {{1, 2}, {3, 4}};
}

void Environment::printMatrix(){
    for(auto &row : m_matrix){
        for(auto &el : row){
            std::cout << el << " ";
        }
        std::cout << '\n';
    }
}