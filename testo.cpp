#include <iostream>
#include <utility>
#include <algorithm>
#include <functional>

int main() {
	for (int i=0; i<20; i++) {
		std::cout << i << " --> " << std::hash<int>(i) << std::endl;
	}
}
