#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

using Canvas = cv::Mat;

// draws a line from p0 to p1
void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, uint8_t color);
void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, cv::viz::Color color);
void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color);
void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, cv::viz::Color color);
void drawFilledCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color);
void drawFilledCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, cv::viz::Color color);
void drawPath(Canvas& dest, const std::vector<Eigen::Vector2f>& path, cv::viz::Color color);
int showCanvas(Canvas& canvas, int timeout_ms);
