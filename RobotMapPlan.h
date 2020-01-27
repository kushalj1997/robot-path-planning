// RobotMapPlan.h
// Kushal Jaligama

#include <vector>
#include "Obstacle.h"
#include <limits>
#include <string>
#include <fstream>

const int NULL_PARENT = -1;

struct Tile
{
  int par_x = NULL_PARENT;
  int par_y = NULL_PARENT;
  size_t x = 0;
  size_t y = 0;
  bool visited = false;
  double cost = std::numeric_limits<double>::infinity();
  double distToNearestObstacle = std::numeric_limits<double>::infinity();
};

class RobotMapPlan
{
public:
  RobotMapPlan(size_t map_rows, size_t map_cols, cv::Point2i robotStart,
               int robotRadius, cv::Point2i end);

  RobotMapPlan(std::ifstream &is);

  void drawRobot(cv::Point2i robotStart, int robotRadius);

  void drawEndLoc(cv::Point2i endLoc);

  void addObstacle(Obstacle o);

  void planPathDijkstra();

  void visualize();

  void writeMapFile(std::string file_name);

private:
  void buildInitialMap();
  bool isValidPoint(int row, int col);

  cv::Mat theVisualMap;
  cv::Mat theConfigurationSpace;

  std::vector<std::vector<Tile>> theCostMap;
  std::vector<cv::Point2i> path;

  std::vector<Obstacle> obstacles;

  Obstacle robot;

  cv::Point2i startLoc;
  cv::Point2i endLoc;

  size_t map_size_rows;
  size_t map_size_cols;
};
