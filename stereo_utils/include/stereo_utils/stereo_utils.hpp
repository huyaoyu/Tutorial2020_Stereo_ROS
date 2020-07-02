
#ifndef __STEREO_UTILS__STEREO_UTILS_HPP__
#define __STEREO_UTILS__STEREO_UTILS_HPP__

#include <string>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include "stereo_utils.hpp"

namespace stereo_utils
{

void convert_eigen_matrix_2_mat( 
	const Eigen::MatrixXf &e, 
	cv::Mat &m );

void load_true_disparity(
    const std::string &fn, 
    cv::Mat &trueDisp);

struct DiffStat {
	float mean;
	float minVal;
	float maxVal;
};

DiffStat compute_diff_statistics( 
	const cv::Mat &diff, 
    const cv::Mat &mask=cv::Mat() );

void compare_with_true_disparity( 
	const cv::Mat &trueDisp, 
	const cv::Mat &predDisp, 
	cv::Mat &diff );

void save_float_image_self_normalize( 
	const std::string fn, 
	const cv::Mat &img );

} // namespace stereo_utils

#endif // __STEREO_UTILS__STEREO_UTILS_HPP__