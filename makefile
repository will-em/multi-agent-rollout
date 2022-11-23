all:
	g++ -std=c++17 -O2 src/Rollout.cpp src/IndexToPair.cpp src/Environment.cpp src/BasePolicy.cpp src/CostsToControl.cpp src/BoxPicker.cpp src/UpdateTargets.cpp src/UpdateBasePolicy.cpp src/ControlPicker.cpp src/Simulate.cpp src/Astar.cpp src/InitAstar.cpp src/drawer.cpp -o rollout

CA-star:
	g++ -std=c++17 -O2 src/Coop.cpp src/IndexToPair.cpp src/Environment.cpp src/BoxPicker.cpp src/UpdateTargets.cpp  src/SimulateCoop.cpp src/CoopAlgorithm.cpp -o coop

test:
	g++ -std=c++17 src/coop-astar.cpp src/testing/coop.cpp -o coopTest

mp4:
	ffmpeg -r 2 -i video/frames/frame%04d.svg -r 24 video/video.mp4 && \
	rm video/frames/*

smooth_mp4:
	ffmpeg -r 24 -i video/frames/frame%04d.svg -r 24 video/video.mp4 && \
	rm video/frames/*
	
clean_frames:
	rm video/frames/*

clean:
	rm -f rollout
	rm -f coop
	rm -f coopTest 
