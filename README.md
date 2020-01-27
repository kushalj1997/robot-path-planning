### Robot Path Planning with Djikstra's Algorithm

# Usage
```console
./rpp_build # Uses cmake to build the project and execute
./rpp_debug # Builds project with Debug Mode on for use with gdb on the command line
./rpp_xcode # Creates an Xcode project with cmake
```

# Goals
Generate map for a circular robot to traverse.
- Find the optimal path from a given start location to given end location.
Approaches
- Use BFS or Dijkstra's to calculate an optimal path.
- Use A* to generate the probabilistic model of graphs to traverse through with BB8

# Deliverables
- Create an interface to build up a map for robot.
- Place obstacles on the map.
- Visualize the map and the optimal path that robot will traverse.
- Save and load the map from a file.
- Provide example usage of the library.

# Interface

## Map Class
This should contain a binary grid of the map with each spot containing distances
to the closest obstacle.
- Takes in initial parameters of map size (M x N) and robot radius.

Have a function to add obstacles to the map.
- This should take in as parameter a list of circular obstacles

Class should create the best safe path given a start and end location on the map.
- Generate the configuration space within which to plan a path.
- Use Dijkstra's algorithm.
- Use A* algorithm.
- Compare the performance of these two approaches.

## Obstacle Class
Holds an obstacle with given center and radius. Simplified to a struct, change to a class if more complexity is needed.
