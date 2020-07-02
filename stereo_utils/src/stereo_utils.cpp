
#define TINYPLY_IMPLEMENTATION

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "Filesystem.hpp"
#include "JSONHelper.hpp"
#include "NumPyIO.hpp"

#include "stereo_utils.hpp"

using namespace stereo_utils;
using JSON = nlohmann::json;

std::shared_ptr<JSON> stereo_utils::read_json( const std::string &fn ) {
    std::shared_ptr<JSON> pJson ( new JSON );

    std::ifstream ifs(fn);

    if ( !ifs.good() ) {
        std::stringstream ss;
        ss << fn << " not good. ";
        throw std::runtime_error( ss.str() );
    }

    ifs >> *pJson;

    return pJson;
}

void stereo_utils::convert_eigen_matrix_2_mat( 
	const Eigen::MatrixXf &e, 
	cv::Mat &m ) {
	cv::eigen2cv( e, m );
}

void stereo_utils::load_true_disparity(const std::string &fn, cv::Mat &trueDisp) {
	Eigen::MatrixXf eTrueDisp;
	read_npy_2_eigen_matrix(fn, eTrueDisp);
	convert_eigen_matrix_2_mat( eTrueDisp, trueDisp );
}

DiffStat stereo_utils::compute_diff_statistics( 
	const cv::Mat &diff, const cv::Mat &mask ) {

	if ( 1 != diff.channels() ) {
		throw std::runtime_error("diff must be 1 channel. ");
	}

	float acc      = 0.f;
	int   n        = 0;

	DiffStat ds;
	ds.mean   = 0.f;
	ds.minVal = diff.cols * 2;
	ds.maxVal = 0.f;

	if ( !mask.empty() ) {
		assert( diff.rows == mask.rows );
		assert( diff.cols == mask.cols );

		for ( int i = 0; i < diff.rows; ++i ) {
			const float *pd   = diff.ptr<float>(i);
			const uint8_t *pm = mask.ptr<uint8_t>(i);
			for ( int j = 0; j < diff.cols; ++j ) {
				const uint8_t m = pm[j];
				if ( m > 0 ) {
					const float d = pd[j];

					if ( d > ds.maxVal ) {
						ds.maxVal = d;
					}

					if ( d < ds.minVal ) {
						ds.minVal = d;
					}

					acc += d;
					n++;
				}
			}
		}

		if ( n == 0 ) {
			n = 1;
		}

		ds.mean = acc / n;
	} else {
		for ( int i = 0; i < diff.rows; ++i ) {
			const float *pd   = diff.ptr<float>(i);
			for ( int j = 0; j < diff.cols; ++j ) {
				const float d = pd[j];

				if ( d > ds.maxVal ) {
					ds.maxVal = d;
				}

				if ( d < ds.minVal ) {
					ds.minVal = d;
				}

				acc += d;
				n++;
			}
		}

		if ( n == 0 ) {
			n = 1;
		}

		ds.mean = acc / n;
	}

	return ds;
}

void stereo_utils::compare_with_true_disparity( 
	const cv::Mat &trueDisp, 
	const cv::Mat &predDisp, 
	cv::Mat &diff ) {

	cv::Mat maskFloat;
	cv::threshold( trueDisp, maskFloat, 0, 255, cv::THRESH_BINARY);

	cv::Mat mask;
	maskFloat.convertTo(mask, CV_8UC1);

	// std::cout << "mask.type() = " << mask.type() << std::endl;
	
	// It is assumed that the data type of trueDisp and 
	// predDisp is floating point.
	diff = cv::abs( trueDisp - predDisp );
	
	DiffStat ds = compute_diff_statistics( diff, mask );

	std::cout << "diff mean = " << ds.mean << ", "
	          << "diff min = "  << ds.minVal << ", "
			  << "diff max = "  << ds.maxVal << ". \n";
}

void stereo_utils::save_float_image_self_normalize( 
	const std::string fn, 
	const cv::Mat &img ) {
	
	const int type = img.type() % 8;
	assert( type == 5 || type == 6 );
	assert( 1 == img.channels() );

	test_directory_by_filename(fn);

	// Normalize the image.
	double minVal, maxVal;
	cv::minMaxLoc(img, &minVal, &maxVal);

	cv::Mat normalized = (img - minVal) / ( maxVal - minVal ) * 255;

	cv::imwrite(fn, normalized);
}