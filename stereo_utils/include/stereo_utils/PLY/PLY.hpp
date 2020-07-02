//
// Created by yyhu on 4/12/19.
//

#ifndef STEREO_UTILS_PLY_HPP
#define STEREO_UTILS_PLY_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace stereo_utils
{

/**
 * Write a PLY file based on the disparity and color image. The reprojection matrix is Q which
 * is 4x4 and row-major.
 *
 * This functio only handles specific Mat types.
 * disp must be CV_32FC1, color must be CV_8UC3.
 *
 * @param fn The output filename.
 * @param disp The disparity image, should be one channel.
 * @param color The color image, should be three channels.
 * @param Q The reprojection matrix. Should be 4x4.
 * @param binary Set to true if write a binary PLY file.
 */
void write_ply_with_color(const std::string& fn, const cv::Mat& disp, const cv::Mat& color,
        const Eigen::MatrixXf &Q, bool flip, const float farLimit=10000.0f, bool binary=true);

} // namespace stereo_utils

#endif //STEREO_UTILS_PLY_HPP
