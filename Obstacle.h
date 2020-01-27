// Obstacle.h
// Kushal Jaligama

// Obstacles should be circular with a center location and radius

#include <opencv2/core/core.hpp> // cv::Point

struct Obstacle
{
  cv::Point2i center;
  int radius;
};

// class Obstacle {
//   Obstacle(int n) {
//     if (n < 3) {
//       // constructu circle
//       Circle()
//     }
//   }

//   Obstacle(cv::Point2i, radius) {

//   }

// };
