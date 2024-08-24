#include "draw_helpers.h"


void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, uint8_t color) {
  cv::line(dest, cv::Point(p0[0], p0[1]), cv::Point(p1[0], p1[1]), cv::Scalar(color, color, color), 1);
}


void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, cv::viz::Color color) {
  cv::line(dest, cv::Point(p0[0], p0[1]), cv::Point(p1[0], p1[1]), color, 1);
}


void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius, cv::Scalar(color, color, color));
}


void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, cv::viz::Color color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius, color);
}


void drawFilledCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius, cv::Scalar(color, color, color), cv::FILLED);
}


void drawFilledCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, cv::viz::Color color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius, color, cv::FILLED);
}


void drawPath(Canvas& dest, const std::vector<Eigen::Vector2f>& path, cv::viz::Color color) {
  int lenght = path.size();
  for(int i = 0; i < lenght-1; i++) {
    Eigen::Vector2f p1 = path.at(i);
    Eigen::Vector2f p2 = path.at(i+1);
    drawLine(dest, p1, p2, color);
  }
}


int showCanvas(Canvas& canvas, int timeout_ms) {
  cv::imshow("canvas", canvas);
  int key = cv::waitKey(timeout_ms);
  if (key == 27)  // exit on ESC
    exit(0);
  // cerr << "key" << key << endl;
  return key;
}
