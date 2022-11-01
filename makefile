all:
	g++ -std=c++17 -O2 src/main.cpp src/IndexToPair.cpp src/Environment.cpp src/BasePolicy.cpp src/CostsToControl.cpp src/BoxPicker.cpp src/UpdateTargets.cpp src/UpdateBasePolicy.cpp src/ControlPicker.cpp src/Simulate.cpp src/Astar.cpp src/InitAstar.cpp -o rollout

coop:
	g++ -std=c++17 -O2 src/mainCoop.cpp src/IndexToPair.cpp src/Environment.cpp src/BoxPicker.cpp src/UpdateTargets.cpp  src/SimulateCoop.cpp src/coop-astar.cpp -o coop

test:
	g++ -std=c++17 src/coop-astar.cpp src/testing/main.cpp -o test

clean:
	rm -f rollout
	rm -f coop
	rm -f test
