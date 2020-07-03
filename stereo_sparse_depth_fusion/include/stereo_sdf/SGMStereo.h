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

/*
   * Original code by Koichiro Yamaguchi, modified by:
   *    Shreyas S. Shivakumar,
   *    University of Pennsylvania.
   *    sshreyas@seas.upenn.edu
   */

#pragma once
#include <png++/png.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

class SGMStereo {
public:
  SGMStereo();

  void setDisparityTotal(const int disparityTotal);
  void setDisparityFactor(const double disparityFactor);
  void setDataCostParameters(const int sobelCapValue,
                 const int censusWindowRadius,
                 const double censusWeightFactor,
                 const int aggregationWindowRadius);
  void setSmoothnessCostParameters(const int smoothnessPenaltySmall, const int smoothnessPenaltyLarge);
  void setConsistencyThreshold(const int consistencyThreshold);

  void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat1i &X, cv::Mat1i &Y);
  void meshgridInit(const cv::Range &xgv, const cv::Range &ygv, cv::Mat1i &X, cv::Mat1i &Y);

  /*
   * The following functions were added by Shreyas S. Shivakumar
   * details can be found in the corresponding paper
   */

  /* Neighborhood Support Method (NS) */
  void updateCostVolumes_NS(unsigned short *leftCostVolume,
                            unsigned short * rightCostVolume,
                            cv::Mat leftImage,
                            cv::Mat weightImage);

  /* Naive Range Fusion (NRF) */
  void updateCostVolumes_NRF(unsigned short *leftCostVolume,
                             unsigned short * rightCostVolume,
                             cv::Mat leftImage,
                             cv::Mat weightImage);

  /* Diffusion Based Fusion (DB) */
  void updateCostVolumes_DB(unsigned short *leftCostVolume,
                            unsigned short * rightCostVolume,
                            cv::Mat leftImage,
                            cv::Mat weightImage);

  /* Modified original stereo compute function */
  void compute(const png::image<png::rgb_pixel>& leftImage,
               const png::image<png::rgb_pixel>& rightImage,
               float* disparityImage,
               int STEREO_MODE,
               std::string paramFile,
               cv::Mat depthImage,
               int FUSE_FLAG,
               cv::Mat leftImageFuse,
               cv::Mat weightImage);

private:
  void initialize(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage);
  void setImageSize(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage);
  void allocateDataBuffer();
  void freeDataBuffer();
  void computeCostImage(const png::image<png::rgb_pixel>& leftImage, const png::image<png::rgb_pixel>& rightImage);
  void convertToGrayscale(const png::image<png::rgb_pixel>& leftImage,
              const png::image<png::rgb_pixel>& rightImage,
              unsigned char* leftGrayscaleImage,
              unsigned char* rightGrayscaleImage) const;
  void computeLeftCostImage(const unsigned char* leftGrayscaleImage, const unsigned char* rightGrayscaleImage);
  void computeCappedSobelImage(const unsigned char* image, const bool horizontalFlip, unsigned char* sobelImage) const;
  void computeCensusImage(const unsigned char* image, int* censusImage) const;
  void calcTopRowCost(unsigned char*& leftSobelRow, int*& leftCensusRow,
            unsigned char*& rightSobelRow, int*& rightCensusRow,
            unsigned short* costImageRow);
  void calcRowCosts(unsigned char*& leftSobelRow, int*& leftCensusRow,
            unsigned char*& rightSobelRow, int*& rightCensusRow,
            unsigned short* costImageRow);
  void calcPixelwiseSAD(const unsigned char* leftSobelRow, const unsigned char* rightSobelRow);
  void calcHalfPixelRight(const unsigned char* rightSobelRow);
  void addPixelwiseHamming(const int* leftCensusRow, const int* rightCensusRow);
  void computeRightCostImage();
  void performSGM(unsigned short* costImage, unsigned short* disparityImage);
  void speckleFilter(const int maxSpeckleSize, const int maxDifference, unsigned short* image) const;
  void enforceLeftRightConsistency(unsigned short* leftDisparityImage, unsigned short* rightDisparityImage) const;

  // Parameter
  int disparityTotal_;
  double disparityFactor_;
  int sobelCapValue_;
  int censusWindowRadius_;
  double censusWeightFactor_;
  int aggregationWindowRadius_;
  int smoothnessPenaltySmall_;
  int smoothnessPenaltyLarge_;
  int consistencyThreshold_;

  // Data
  int width_;
  int height_;
  int widthStep_;
  unsigned short* leftCostImage_;
  unsigned short* rightCostImage_;
  unsigned char* pixelwiseCostRow_;
  unsigned short* rowAggregatedCost_;
  unsigned char* halfPixelRightMin_;
  unsigned char* halfPixelRightMax_;
  int pathRowBufferTotal_;
  int disparitySize_;
  int pathTotal_;
  int pathDisparitySize_;
  int costSumBufferRowSize_;
  int costSumBufferSize_;
  int pathMinCostBufferSize_;
  int pathCostBufferSize_;
  int totalBufferSize_;
  short* sgmBuffer_;

  /* Range measurement to be fused with stereo */
  cv::Mat_<double> rangeMeasurement_;
};
