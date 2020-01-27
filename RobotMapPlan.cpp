// RobotMapPlan.cpp
// Kushal Jaligama

#include "RobotMapPlan.h"
#include <cmath>
#include <iostream>
#include <opencv2/highgui/highgui.hpp> // imshow()
#include <opencv2/imgproc/imgproc.hpp> // cv::circle
#include <queue>
#include <vector>

using cv::Point2i;

const uchar ROBOT_GREY = 255;
const cv::Scalar ROBOT_COLOR(0, 255, 0);
const uchar OBSTACLE_GREY = 128;
const cv::Scalar OBSTACLE_COLOR(0, 0, 255);
const uchar END_GREY = 255;
const cv::Scalar END_COLOR(255, 255, 255);
const uchar PATH_GREY = 200;
const cv::Scalar PATH_COLOR(255, 0, 0);
const uchar TEXT_GREY = 255;
const cv::Scalar TEXT_COLOR(255, 255, 255);

RobotMapPlan::RobotMapPlan(size_t map_rows, size_t map_cols,
                           Point2i robotStart, int robotRadius, Point2i end)
    : theVisualMap(cv::Mat::zeros(cv::Size(map_rows, map_cols), CV_8U)),
      theConfigurationSpace(
          cv::Mat::zeros(cv::Size(map_rows, map_cols), CV_8U)),
      theCostMap(map_rows, std::vector<Tile>(map_cols)),
      robot(Obstacle{robotStart, robotRadius}),
      startLoc(robotStart),
      endLoc(end),
      map_size_rows(map_rows),
      map_size_cols(map_cols)
{
  buildInitialMap();
}

RobotMapPlan::RobotMapPlan(std::ifstream &is)
{
  is >> map_size_rows >> map_size_cols;
  is >> startLoc.x >> startLoc.y;
  robot.center.x = startLoc.x;
  robot.center.y = startLoc.y;
  is >> robot.radius;
  is >> endLoc.x >> endLoc.y;

  theVisualMap = cv::Mat::zeros(cv::Size(map_size_rows, map_size_cols), CV_8U);
  theConfigurationSpace =
      cv::Mat::zeros(cv::Size(map_size_rows, map_size_cols), CV_8U);
  theCostMap.resize(map_size_rows);
  for (size_t i = 0; i < map_size_rows; ++i)
  {
    theCostMap[i].resize(map_size_cols);
  }

  int r, c, rad;

  while (is >> r >> c >> rad)
  {
    addObstacle(Obstacle{Point2i(r, c), rad});
  }

  buildInitialMap();
}

void RobotMapPlan::buildInitialMap()
{
  // Function to add the walls, robot, and end location to the map
  // Fill in the coordinates of the occupancy grid and also
  for (size_t i = 0; i < map_size_rows; ++i)
  {
    for (size_t j = 0; j < map_size_cols; ++j)
    {
      theCostMap[i][j].x = i;
      theCostMap[i][j].y = j;
    }
  }

  // pad the borders of the configuration space by the robot's radius
  // so that it cannot go out of bounds
  for (size_t r = 0; r < robot.radius; ++r)
  {
    for (size_t c = 0; c < map_size_cols; ++c)
    {
      theConfigurationSpace.at<uchar>(r, c) = OBSTACLE_GREY;
    }
  }

  for (size_t r = 0; r < map_size_rows; ++r)
  {
    for (size_t c = 0; c < robot.radius; ++c)
    {
      theConfigurationSpace.at<uchar>(r, c) = OBSTACLE_GREY;
    }
  }

  for (size_t r = map_size_rows - robot.radius; r < map_size_rows; ++r)
  {
    for (size_t c = 0; c < map_size_cols; ++c)
    {
      theConfigurationSpace.at<uchar>(r, c) = OBSTACLE_GREY;
    }
  }

  for (size_t r = 0; r < map_size_rows; ++r)
  {
    for (size_t c = map_size_cols - robot.radius; c < map_size_cols; ++c)
    {
      theConfigurationSpace.at<uchar>(r, c) = OBSTACLE_GREY;
    }
  }

  // Draw the robot at the start location
  drawRobot(startLoc, robot.radius);
  // Draw the end location
  drawEndLoc(endLoc);
}

void RobotMapPlan::drawRobot(Point2i robotStart, int robotRadius)
{
  // Initialize the robot's starting position and size
  robot = Obstacle{robotStart, robotRadius};
  startLoc = robotStart;
  cv::circle(theVisualMap, robotStart, robotRadius, ROBOT_GREY, CV_FILLED);
  // Robot is just a point mass in the configuration space (radius 1)
  cv::circle(theConfigurationSpace, robotStart, 1, ROBOT_GREY, CV_FILLED);

  cv::putText(theVisualMap, "ROBOT", robotStart, cv::FONT_HERSHEY_SIMPLEX, .5,
              TEXT_GREY);

  std::cout << "Added robot to the map." << std::endl;
}

void RobotMapPlan::drawEndLoc(Point2i end)
{
  endLoc = end;
  cv::circle(theVisualMap, end, robot.radius, END_GREY);
  cv::circle(theConfigurationSpace, end, robot.radius, END_GREY);
  cv::putText(theVisualMap, "END", end, cv::FONT_HERSHEY_SIMPLEX, .5,
              TEXT_GREY);

  std::cout << "Set the target location." << std::endl;
}

void RobotMapPlan::addObstacle(Obstacle o)
{
  obstacles.push_back(o);
  cv::circle(theVisualMap, o.center, o.radius, OBSTACLE_GREY, CV_FILLED);

  // maintain obstacles in the configuration space for planning
  // scale the robot down to point mass
  // increase obstacle size by robot radius
  cv::circle(theConfigurationSpace, o.center, o.radius + robot.radius,
             OBSTACLE_GREY, CV_FILLED);

  cv::putText(theVisualMap, "OBSTACLE", o.center, cv::FONT_HERSHEY_SIMPLEX, 0.5,
              TEXT_GREY);

  // std::cout << "Added obstacle " << obstacles.size() << std::endl;
}

static double euclidDist(Tile a, Point2i endLoc)
{
  return sqrt((a.x - endLoc.x) * (a.x - endLoc.x) +
              (a.y - endLoc.y) * (a.y - endLoc.y));
}

static double euclidDist(Tile a, Tile b)
{
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Comparator used for priority queue in path planning algorithms
// Want to prioritize Tiles with lower cost, so must make a > comparator
// since STL pq uses in an opposite manner
// Alternative is to use
struct TileComparator
{
  bool operator()(Tile a, Tile b) { return (a.cost > b.cost); }
};

void RobotMapPlan::planPathDijkstra()
{
  std::cout << "Planning path from robot location to target end location with "
               "Dijkstra's Algorithm..."
            << std::endl;
  // function to find a path from startloc to endloc

  // perform Dijkstra's algorithm on the configuration space vector
  // implementation based on
  // https://en.wikipedia.org/wiki/Dijkstra's_algorithm#Algorithm
  // also referenced Computerphile's YouTube video on Dijkstra's
  std::priority_queue<Tile, std::vector<Tile>, TileComparator> candidates;
  // set cost of initial start location to zero
  theCostMap[startLoc.x][startLoc.y].cost = 0;
  // add the start location to pq
  candidates.push(theCostMap[startLoc.x][startLoc.y]);

  bool pathExists = false;

  while (!candidates.empty())
  {
    Tile curr = candidates.top();
    candidates.pop();

    // terminate Dijkstra's if the node we're visiting now is end location
    // with a valid state (path is found)
    if (curr.x == endLoc.x && curr.y == endLoc.y)
    {
      pathExists = true;
      break;
    }

    // terminate Dijkstra's if smallest cost in pq is infinity
    // with an invalid state (path is not found)
    if (curr.cost == std::numeric_limits<double>::infinity())
    {
      pathExists = false;
      break;
    }

    // add neighbors (n, s, e, w, nw, ne, se, sw)
    for (int row = -1; row <= 1; ++row)
    {
      for (int col = -1; col <= 1; ++col)
      {
        int neighbor_row = curr.x + row;
        int neighbor_col = curr.y + col;
        // ensure neighbor is unvisited, within bounds, and not an obstacle
        if (isValidPoint(neighbor_row, neighbor_col))
        {
          const int diagonal_neighbor = 2;
          const int vertical_neighbor = 1;
          int cost = 0;

          if (abs(row) == 1 && abs(col) == 1)
          {
            cost = diagonal_neighbor;
          }
          else
          {
            cost = vertical_neighbor;
          }

          // closest obstacles
          double minDist = std::numeric_limits<double>::infinity();
          for (size_t i = 0; i < obstacles.size(); ++i)
          {
            double dist = euclidDist(curr, obstacles[i].center);
            if (euclidDist(theCostMap[neighbor_row][neighbor_col],
                           obstacles[i].center) < minDist)
            {
              minDist = dist;
            }
          }

          // assign the cost of going to this tile (every node has cost of 1) if
          // current cost of node is greater than new cost
          double tentative_cost = curr.cost + cost + minDist;
          if (theCostMap[neighbor_row][neighbor_col].cost > tentative_cost)
          {
            theCostMap[neighbor_row][neighbor_col].cost = tentative_cost;
            // set the parent of the neighbor
            theCostMap[neighbor_row][neighbor_col].par_x = curr.x;
            theCostMap[neighbor_row][neighbor_col].par_y = curr.y;
            // add valid neighbors to the pq only if we have a new cost for this
            // tile
            candidates.push(theCostMap[neighbor_row][neighbor_col]);
          }
        }
      }
    }
    // mark current node as visited
    theCostMap[curr.x][curr.y].visited = true;
  } // end while

  if (pathExists)
  {
    std::cout << "Path exists, building and storing..." << std::endl;
    // backtrack through from end location to start to get the path
    Tile it = theCostMap[endLoc.x][endLoc.y];
    while (it.par_x != -1 && it.par_y != -1)
    {
      Point2i p(it.x, it.y);
      path.push_back(p);
      it = theCostMap[it.par_x][it.par_y];
    }
    // push on the start tile
    path.push_back(Point2i(startLoc.x, startLoc.y));
  }
  else
  {
    std::cout << "No path found from start (" << startLoc.x << ", "
              << startLoc.y << ") to end " << endLoc.x << ", " << endLoc.y
              << ")" << std::endl;
  }
}

void RobotMapPlan::visualize()
{
  std::cout << "Drawing path onto the map and visualizing..." << std::endl;
  size_t pathDist = 0;
  if (!path.empty())
  {
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
      pathDist++;
      cv::line(theVisualMap, path[i], path[i + 1], PATH_GREY);
      cv::line(theConfigurationSpace, path[i], path[i + 1], PATH_GREY);
    }
  }

  std::string distText = "Path Distance: " + std::to_string(pathDist);
  Point2i distTextPoint(map_size_cols / 2 - 100, map_size_rows - 50);

  cv::putText(theVisualMap, distText, distTextPoint, cv::FONT_HERSHEY_SIMPLEX,
              1, TEXT_GREY);

  cv::imwrite("Path on Map.jpg", theVisualMap);

  // Pop up a graph that shows the 2D map
  cv::imshow("theVisualMap", theVisualMap);
  cv::imshow("theConfigurationSpace", theConfigurationSpace);
  cv::waitKey(0);
}

void RobotMapPlan::writeMapFile(std::string file_name)
{
  std::ofstream ofs;
  ofs.open(file_name);

  // Add map dimensions to file
  ofs << map_size_rows << " " << map_size_cols << std::endl;
  // Add robot start location and radius to file
  ofs << robot.center.x << " " << robot.center.y << " " << robot.radius
      << std::endl;
  // Add end location to file
  ofs << endLoc.x << " " << endLoc.y << std::endl;

  for (size_t i = 0; i < obstacles.size(); ++i)
  {
    ofs << obstacles[i].center.x << " " << obstacles[i].center.y << " "
        << obstacles[i].radius << std::endl;
  }
  ofs.close();
}

bool RobotMapPlan::isValidPoint(int row, int col)
{
  // ensure neighbor is unvisited, within bounds, and not an obstacle
  // OpenCV Mat::at function takes the column as first argument
  return (row >= 0 && row < map_size_rows && col >= 0 && col < map_size_cols &&
          !theCostMap[row][col].visited &&
          theConfigurationSpace.at<uchar>(col, row) != OBSTACLE_GREY);
}
