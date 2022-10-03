all:
	g++ -std=c++17 -O2 src/main.cpp src/IndexToPair.cpp src/Environment.cpp src/BasePolicy.cpp src/CostsToControl.cpp src/BoxPicker.cpp src/UpdateTargets.cpp src/UpdateBasePolicy.cpp src/ControlPicker.cpp src/Simulate.cpp -o rollout

tests:
	g++ -std=c++17 -O2 src/runTests.cpp src/IndexToPair.cpp src/Environment.cpp src/BasePolicy.cpp src/CostsToControl.cpp src/BoxPicker.cpp src/UpdateTargets.cpp src/UpdateBasePolicy.cpp src/ControlPicker.cpp src/Simulate.cpp -o test 
