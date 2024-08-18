#include "grid_map.h"

using namespace std;


GridMap::GridMap():
  Grid_<uint8_t>(0, 0) {
    reset(Vector2f(0,0), 1);
  }


GridMap::GridMap(float resolution_, int rows_, int cols_):
  Grid_<uint8_t>(rows_, cols_){
    reset(Vector2f(0,0), resolution_);
}


float GridMap::scanRay(const Vector2f& origin,
                        const Vector2f& direction,
                        const float max_range) const {
  float range = 0;

  while (range < max_range) {
    Vector2i int_endpoint=world2grid(origin + direction * range).cast<int>();
    
    if (!inside(int_endpoint))
      return max_range;

    if ((*this)(int_endpoint) < 127)
      return range;

    range +=resolution();
  }
  
  return max_range;
}


void GridMap::loadFromImage(const char* filename, float res) {
  cerr << "loading [" << filename << "]" << endl;
  cv::Mat m = cv::imread(filename);
  if (m.rows == 0) {
    throw std::runtime_error("unable to load image");
  }
  cv::Mat loaded_image;
  cv::cvtColor(m, loaded_image, cv::COLOR_BGR2GRAY);
  resize(loaded_image.rows, loaded_image.cols);
  reset(Vector2f(0,0), res);
  memcpy(&cells[0], loaded_image.data, cells.size());
}


void GridMap::loadFromOccupancyGrid(nav_msgs::OccupancyGrid grid) {
  int rows = grid.info.height;
  int cols = grid.info.width;
  float res = grid.info.resolution;
  Vector2f origin(grid.info.origin.position.x, grid.info.origin.position.y);
  std::vector<int8_t> int_vector = grid.data;
  std::vector<uint8_t> uint_vector(int_vector.begin(), int_vector.end());
  cv::Mat m(rows, cols, CV_8UC1, uint_vector.data());
  cv::Mat negative_image = 255 - m*2;
  cv::Mat flipped_image;
  cv::flip(negative_image, flipped_image, 0);
  resize(flipped_image.rows, flipped_image.cols);
  reset(origin, res);
  memcpy(&cells[0], flipped_image.data, cells.size());
}