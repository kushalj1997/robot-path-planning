// Robot Path Planning Project
// Kushal Jaligama

// TODO: give some mass and propulsion dynamics to robot

#include <fstream>
#include <random>
#include <string>
#include "RobotMapPlan.h"

using cv::Point2i;

Obstacle randomObstacleGenerator(int min, int max, int maxRad)
{
  std::random_device rd;
  std::mt19937 mt(rd());
  // location distribution
  std::uniform_int_distribution<int> locDist(min, max);
  // radius distribution
  std::uniform_int_distribution<int> radDist(1, maxRad);

  return Obstacle{Point2i(locDist(mt), locDist(mt)), radDist(mt)};
}

int randomNumberGenerator(int min, int max)
{
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_int_distribution<int> dist(0, 1);

  return dist(mt);
}

void writeMapTextFile(size_t rows, size_t cols, size_t num_obstacles,
                      size_t robot_x, size_t robot_y, size_t robot_rad,
                      size_t end_x, size_t end_y)
{
  std::ofstream ofs;
  ofs.open("map.txt");

  // Add map dimensions to file
  ofs << rows << " " << cols << std::endl;
  // Add robot start location and radius to file
  ofs << robot_x << " " << robot_y << " " << robot_rad << std::endl;
  // Add end location to file
  ofs << end_x << " " << end_y << std::endl;

  Obstacle o;

  for (size_t i = 0; i < num_obstacles; ++i)
  {
    o = randomObstacleGenerator(40, 880, 30);
    ofs << o.center.x << " " << o.center.y << " " << o.radius << std::endl;
  }
  ofs.close();
}

void testTextFileConstructor()
{
  // Generate a random map text file for testing the RobotMapPlan class map
  // creation via loading a file
  writeMapTextFile(1000, 1000, 100, 21, 21, 20, 900, 900);
  // Insantiate map from file, robot location, radius, and target end location
  // map.txt is a 100 by 100 map
  std::ifstream ifs;
  ifs.open("map.txt");
  RobotMapPlan rmp(ifs);
  ifs.close();

  // Verify the file writer member function produces what we started with
  rmp.writeMapFile("map2.txt");

  // Plan optimal path given obstacles
  rmp.planPathDijkstra();

  // Visualize the map and path
  rmp.visualize();
}

void testNormalConstructor()
{
  // Instantiate map with map size, robot location, radius and target end
  // location
  RobotMapPlan rmp(1000, 1000, Point2i(21, 21), 20, Point2i(900, 900));
  // Add obstacles to map
  for (size_t i = 0; i < 100; ++i)
  {
    // constrain the obstacles to be placed outside of the robot and end
    // location + robot radius, take into account the size of obstacles
    rmp.addObstacle(randomObstacleGenerator(70, 870, 30));
  }

  // Plan optimal path given obstacles
  rmp.planPathDijkstra();

  // Visualize the map and path
  rmp.visualize();
}

void testHorizontalPath()
{
  RobotMapPlan rmp(100, 100, Point2i(10, 10), 5, Point2i(10, 90));

  rmp.planPathDijkstra();

  rmp.visualize();
}

void testDiagonalPath()
{
  RobotMapPlan rmp(100, 100, Point2i(10, 10), 5, Point2i(90, 90));

  rmp.planPathDijkstra();

  rmp.visualize();
}

int main(int argc, char **argv)
{
  testNormalConstructor();
  // testTextFileConstructor();
  // testHorizontalPath();
  // testDiagonalPath();
  return 0;
}
