#include <iostream>
#include "Environment.hpp"

int main(){
    Environment env(3, 2, 5, 2);
    env.printMatrix();

    std::vector<uint8_t> actions = {4, 2};

    std::pair<unsigned int, unsigned int> target1(0, 0);
    std::pair<unsigned int, unsigned int> target2(3, 3);
    std::vector<std::pair<unsigned int, unsigned int>> targets = {target1, target2};

    std::cout << env.step(actions, targets) << std::endl;

    env.printMatrix();
    targets[1] = {2, 3};
    std::cout << env.step(actions, targets) << std::endl;
    env.printMatrix();

    return 0;
}