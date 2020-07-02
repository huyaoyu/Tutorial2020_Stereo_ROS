/*
    Copyright (C) 2014  Koichiro Yamaguchi

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define TINYPLY_IMPLEMENTATION

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "SPSStereo.h"
#include "defParameter.h"

#include "Filesystem.hpp"
#include "JSONHelper.hpp"
#include "MatrixIO.hpp"
#include "NumPyIO.hpp"
#include "PLY/PLY.hpp"
#include "SimpleTime.hpp"

using JSON = nlohmann::json;

struct CaseDescription {
	std::string name   = "";
	std::string fn0    = "";
	std::string fn1    = "";
	std::string fnD    = "";
	std::string outDir = "";
	std::string fnQ    = "";
	float qFactor      = 1.0f;
	float dOffs        = 0.0f;
};

void makeSegmentBoundaryImage(const cv::Mat & inputImage,
							  const /*png::image<png::gray_pixel_16>*/ cv::Mat & segmentImage,
							  std::vector< std::vector<int> >& boundaryLabels,
							  cv::Mat& segmentBoundaryImage)
{
	int width = static_cast<int>(inputImage.cols);
	int height = static_cast<int>(inputImage.rows);
	int boundaryTotal = static_cast<int>(boundaryLabels.size());

	segmentBoundaryImage.create(height, width, CV_8UC3);
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			segmentBoundaryImage.at<cv::Vec3b>(y,x) = inputImage.at<cv::Vec3b>(y, x);
		}
	}

	int boundaryWidth = 2;
	for (int y = 0; y < height - 1; ++y) {
		for (int x = 0; x < width - 1; ++x) {
			int pixelLabelIndex = segmentImage.at<uint16_t>(y, x);

			if (segmentImage.at<uint16_t>(y, x + 1) != pixelLabelIndex) {
				for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (x - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y, x - w) = cv::Vec3b(128, 128, 128);
				}
				for (int w = 1; w < boundaryWidth; ++w) {
					if (x + w < width) segmentBoundaryImage.at<cv::Vec3b>(y, x + w) =  cv::Vec3b(128, 128, 128);
				}
			}
			if (segmentImage.at<uint16_t>( y + 1, x) != pixelLabelIndex) {
				for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (y - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y - w, x) =  cv::Vec3b(128, 128, 128);
				}
				for (int w = 1; w < boundaryWidth; ++w) {
					if (y + w < height) segmentBoundaryImage.at<cv::Vec3b>(y + w, x) = cv::Vec3b(128, 128, 128);
				}
			}
		}
	}

	boundaryWidth = 7;
	for (int y = 0; y < height - 1; ++y) {
		for (int x = 0; x < width - 1; ++x) {
			int pixelLabelIndex = segmentImage.at<uint16_t>(y, x);

			if (segmentImage.at<uint16_t>(y, x + 1) != pixelLabelIndex) {
				cv::Vec3b negativeSideColor, positiveSideColor;
				int pixelBoundaryIndex = -1;
				for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
					if ((boundaryLabels[boundaryIndex][0] == pixelLabelIndex && boundaryLabels[boundaryIndex][1] == segmentImage.at<uint16_t>(y, x + 1))
						|| (boundaryLabels[boundaryIndex][0] == segmentImage.at<uint16_t>(y, x + 1) && boundaryLabels[boundaryIndex][1] == pixelLabelIndex))
					{
						pixelBoundaryIndex = boundaryIndex;
						break;
					}
				}
				if (boundaryLabels[pixelBoundaryIndex][2] == 3) continue;
				else if (boundaryLabels[pixelBoundaryIndex][2] == 2) {
					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 225;  negativeSideColor.val[0] = 0;
					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 225;  positiveSideColor.val[0] = 0;
				} else if (pixelLabelIndex == boundaryLabels[pixelBoundaryIndex][boundaryLabels[pixelBoundaryIndex][2]]) {
					negativeSideColor.val[2] = 225;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 0;
					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 225;
				} else {
					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 225;
					positiveSideColor.val[2] = 225;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 0;
				}

				for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (x - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y, x - w) = negativeSideColor;
				}
				for (int w = 1; w < boundaryWidth; ++w) {
					if (x + w < width) segmentBoundaryImage.at<cv::Vec3b>(y, x + w) = positiveSideColor;
				}
			}
			if (segmentImage.at<uint16_t>(y + 1, x) != pixelLabelIndex) {
                cv::Vec3b negativeSideColor, positiveSideColor;
				int pixelBoundaryIndex = -1;
				for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
					if ((boundaryLabels[boundaryIndex][0] == pixelLabelIndex && boundaryLabels[boundaryIndex][1] == segmentImage.at<uint16_t>(y + 1, x))
						|| (boundaryLabels[boundaryIndex][0] == segmentImage.at<uint16_t>(y + 1, x) && boundaryLabels[boundaryIndex][1] == pixelLabelIndex))
					{
						pixelBoundaryIndex = boundaryIndex;
						break;
					}
				}
				if (boundaryLabels[pixelBoundaryIndex][2] == 3) continue;
				else if (boundaryLabels[pixelBoundaryIndex][2] == 2) {
					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 225;  negativeSideColor.val[0] = 0;
					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 225;  positiveSideColor.val[0] = 0;
				} else if (pixelLabelIndex == boundaryLabels[pixelBoundaryIndex][boundaryLabels[pixelBoundaryIndex][2]]) {
					negativeSideColor.val[2] = 225;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 0;
					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 225;
				} else {
					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 225;
					positiveSideColor.val[2] = 225;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 0;
				}

				for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (y - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y - w, x) = negativeSideColor;
				}
				for (int w = 1; w < boundaryWidth; ++w) {
					if (y+ w < height) segmentBoundaryImage.at<cv::Vec3b>(y + w, x) = positiveSideColor;
				}
			}
		}
	}
}

void writeDisparityPlaneFile(const std::vector< std::vector<double> >& disparityPlaneParameters, const std::string outputDisparityPlaneFilename) {
	std::ofstream outputFileStream(outputDisparityPlaneFilename.c_str(), std::ios_base::out);
	if (outputFileStream.fail()) {
		std::cerr << "error: can't open file (" << outputDisparityPlaneFilename << ")" << std::endl;
		exit(0);
	}

	int segmentTotal = static_cast<int>(disparityPlaneParameters.size());
	for (int segmentIndex = 0; segmentIndex < segmentTotal; ++segmentIndex) {
		outputFileStream << disparityPlaneParameters[segmentIndex][0] << " ";
		outputFileStream << disparityPlaneParameters[segmentIndex][1] << " ";
		outputFileStream << disparityPlaneParameters[segmentIndex][2] << std::endl;
	}

	outputFileStream.close();
}

void writeBoundaryLabelFile(const std::vector< std::vector<int> >& boundaryLabels, const std::string outputBoundaryLabelFilename) {
	std::ofstream outputFileStream(outputBoundaryLabelFilename.c_str(), std::ios_base::out);
	if (outputFileStream.fail()) {
		std::cerr << "error: can't open output file (" << outputBoundaryLabelFilename << ")" << std::endl;
		exit(1);
	}

	int boundaryTotal = static_cast<int>(boundaryLabels.size());
	for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
		outputFileStream << boundaryLabels[boundaryIndex][0] << " ";
		outputFileStream << boundaryLabels[boundaryIndex][1] << " ";
		outputFileStream << boundaryLabels[boundaryIndex][2] << std::endl;
	}
	outputFileStream.close();
}

void convert_eigen_matrix_2_mat( 
	const Eigen::MatrixXf &e, 
	cv::Mat &m ) {
	cv::eigen2cv( e, m );
}

void load_true_disparity(const std::string &fn, cv::Mat &trueDisp) {
	Eigen::MatrixXf eTrueDisp;
	read_npy_2_eigen_matrix(fn, eTrueDisp);
	convert_eigen_matrix_2_mat( eTrueDisp, trueDisp );
}

struct DiffStat {
	float mean;
	float minVal;
	float maxVal;
};

static DiffStat compute_diff_statistics( 
	const cv::Mat &diff, const cv::Mat &mask=cv::Mat() ) {

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

void compare_with_true_disparity( 
	const cv::Mat &trueDisp, 
	const cv::Mat &predDisp, 
	cv::Mat &diff ) {

	cv::Mat maskFloat;
	cv::threshold( trueDisp, maskFloat, 0, 255, cv::THRESH_BINARY);

	cv::Mat mask;
	maskFloat.convertTo(mask, CV_8UC1);

	std::cout << "mask.type() = " << mask.type() << std::endl;
	
	// It is assumed that the data type of trueDisp and 
	// predDisp is floating point.
	diff = cv::abs( trueDisp - predDisp );
	
	DiffStat ds = compute_diff_statistics( diff, mask );

	std::cout << "diff mean = " << ds.mean << ", "
	          << "diff min = "  << ds.minVal << ", "
			  << "diff max = "  << ds.maxVal << ". \n";
}

void save_float_image_self_normalize( 
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

void process( const CaseDescription &cd ) {
	std::string leftImageFilename  = cd.fn0;
    std::string rightImageFilename = cd.fn1;
    std::string outDir = cd.outDir + "/" + cd.name;

	std::cout << "Procesing : " << leftImageFilename << ". ";
	cv::Mat leftImage = cv::imread(leftImageFilename, cv::IMREAD_UNCHANGED);
	cv::Mat rightImage = cv::imread(rightImageFilename, cv::IMREAD_UNCHANGED);
	std::cout << "Image size (HxW): " << leftImage.rows << "x" << leftImage.cols << "\n";

	SPSStereo sps;
	sps.setIterationTotal(outerIterationTotal, innerIterationTotal);
	sps.setWeightParameter(lambda_pos, lambda_depth, lambda_bou, lambda_smo);
	sps.setInlierThreshold(lambda_d);
	sps.setPenaltyParameter(lambda_hinge, lambda_occ, lambda_pen);

	cv::Mat segmentImage;
	cv::Mat disparityImage;
	std::vector< std::vector<double> > disparityPlaneParameters;
	std::vector< std::vector<int> > boundaryLabels;

	QUICK_TIME_START(teCompute)
	sps.compute(superpixelTotal, leftImage, rightImage, segmentImage, disparityImage, disparityPlaneParameters, boundaryLabels);
	QUICK_TIME_SHOW(teCompute, "sps-stereo compute")

	cv::Mat segmentBoundaryImage;
	makeSegmentBoundaryImage(leftImage, segmentImage, boundaryLabels, segmentBoundaryImage);

	test_directory(outDir);

	std::string outputBaseFilename           = outDir + "/res";
	std::string outputDisparityImageFilename = outputBaseFilename + "_left_disparity.png";
	std::string outputSegmentImageFilename   = outputBaseFilename + "_segment.png";
	std::string outputBoundaryImageFilename  = outputBaseFilename + "_boundary.png";
	std::string outputDisparityPlaneFilename = outputBaseFilename + "_plane.txt";
	std::string outputBoundaryLabelFilename  = outputBaseFilename + "_label.txt";

	cv::imwrite(outputDisparityImageFilename, disparityImage);
	cv::imwrite(outputSegmentImageFilename, segmentImage);
	cv::imwrite(outputBoundaryImageFilename, segmentBoundaryImage);
	writeDisparityPlaneFile(disparityPlaneParameters, outputDisparityPlaneFilename);
	writeBoundaryLabelFile(boundaryLabels, outputBoundaryLabelFilename);

	cv::Mat predDisp;
	disparityImage.convertTo(predDisp, CV_32FC1);
	predDisp /= 256.f;

	// Compare with the trud disparity.
	if ( cd.fnD != "" ) {
		cv::Mat trueDisp;
		load_true_disparity(cd.fnD, trueDisp);

		cv::Mat diff;
		compare_with_true_disparity(trueDisp, predDisp, diff);

		// // Save the true disparity as image for debug.
		// std::string trueDispFn = outDir + "/TrueDisp.png";
		// cv::imwrite(trueDispFn, trueDisp);

		// Save the difference as an image.
		std::string diffImgFn = outDir + "/Diff.png";
		save_float_image_self_normalize( diffImgFn, diff );
	}

	if ( cd.fnQ != "" ) {
		// Load the Q matrix.
		Eigen::MatrixXf Q;
		read_matrix(cd.fnQ, 4, 4, " ", Q);

		Q(0, 3) *= cd.qFactor;
		Q(1, 3) *= cd.qFactor;
		Q(2, 3) *= cd.qFactor;

		std::string plyFn = outDir + "/Cloud.ply";
		const bool flagFlip   = true;
		const bool flagBinary = true;

		cv::Mat predDispOffs = predDisp + cd.dOffs;
		write_ply_with_color(plyFn, predDispOffs, leftImage, Q, flagFlip, 20, flagBinary);
	}
}

int main(int argc, char* argv[]) {
	if ( argc < 2 ) {
		std::cerr << "Must specify the input JSON file. \n";
		throw std::runtime_error("Must specify the input JSON file. ");
	}

	// Read the JSON file.
	std::shared_ptr<JSON> pJSON = read_json(argv[1]);
	auto& cases = (*pJSON)["cases"];

	const int N = cases.size();

	// Process all the cases.
	for ( int i = 0; i < N; ++i ) {
		auto cd = CaseDescription();
		cd.fn0  = cases[i]["fn0"]; // Filename of image 0.
		cd.fn1  = cases[i]["fn1"]; // Filename of image 1.

		// True disparity, if exists.
		if ( cases[i].find("fnD") != cases[i].end() ) {
			cd.fnD = cases[i]["fnD"];
		} else {
			cd.fnD = "";
		}

		cd.outDir = cases[i]["outDir"]; // Output directory.
		cd.name   = cases[i]["name"];   // Case name.

		if ( cases[i].find("fnQ") != cases[i].end() ) {
			cd.fnQ     = cases[i]["fnQ"];
			cd.qFactor = cases[i]["QF"];
			cd.dOffs   = cases[i]["dOffs"];
		} else {
			cd.fnQ     = "";
			cd.qFactor = 1.0f;
			cd.dOffs   = 0.0f;
		}

		process( cd );
	}
    
    return 0;
}