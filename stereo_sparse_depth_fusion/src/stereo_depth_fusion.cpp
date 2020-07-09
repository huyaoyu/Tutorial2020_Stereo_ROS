#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <dirent.h>
#include <iostream>
#include <sstream>
#include <string>

#include "stereo_sdf/SGMStereo.h"
#include "stereo_sdf/utils.h"

#include <Eigen/Dense>

// Headers from package stereo_utils.
#include <stereo_utils/stereo_utils.hpp>
#include <stereo_utils/Common.hpp>

using namespace std;

using JSON = nlohmann::json;
namespace su = stereo_utils;

/* -------------------------- SET THESE PARAMETERS --------------------------- */
/* stereo options */
#define STEREO_FULL_PIPELINE -1
#define STEREO_LEFT_ONLY      0

/* cost volume update options */
#define VOLUME_UPDATE_NAIVE   3
#define VOLUME_UPDATE_DIFFB   2
#define VOLUME_UPDATE_NEIGH   1
#define VOLUME_UPDATE_NONE   -1

/* image save flag */
#define SAVE_IMAGES           1

/* ground truth sampling option */
double SAMPLING_FRACTION   = 0.50;

/* disparity scaling factor (256 for KITTI) */
double SCALING_FACTOR      = 256.0;

/* --------------------------------------------------------------------------- */

/* ------------ Type definition for interfacing the JSON file. --------------- */

struct CaseDescription {
	std::string name    = "";   // Name of the test case.
	std::string fn0     = "";   // Filename of image Ref.
	std::string fn1     = "";   // Filename of image Tst.
	std::string fnD     = "";   // Filename of the true disparity map.
	std::string outDir  = "";   // Output directory.
	std::string fnQ     = "";   // Filename of the Q matrix.
	float qFactor       = 1.0f; // Scale factor of Q.
	float dOffs         = 0.0f; // Disparity offset. Non-zero for Middlebury dataset. Zero for other datasets.
    float distLimit     = 20.f; // The distance limit for point cloud generation.
    double gtSampleFrac = 0.5;  // Ground truth sampling option.

    SGMParams sgmParams = SGMParams();
};

/* --------------------------------------------------------------------------- */

void SemiGlobalMatching(const cv::Mat &leftImage,
                        const cv::Mat &rightImage,
                        cv::Mat &dispImage,
                        int STEREO_PIPELINE_MODE,
                        const std::string cameraParamFile,
                        cv::Mat depthImage,
                        cv::Mat weightImg,
                        int FUSE_FLAG,
                        const SGMParams &sgmParams)
{
    png::image<png::rgb_pixel> leftImageSGM, rightImageSGM;
    Utils utilities;
    utilities.convertCVMatToPNG(leftImage, leftImageSGM);
    utilities.convertCVMatToPNG(rightImage, rightImageSGM);
    size_t width = leftImageSGM.get_width();
    size_t height = leftImageSGM.get_height();
    if (width != rightImageSGM.get_width() ||
        height != rightImageSGM.get_height())
    {
        dispImage = cv::Mat1w();
        return;
    }
    float* dispImageFloat = (float*)malloc(width*height*sizeof(float));
    SGMStereo sgm;

    // Configure the SGM method.
    std::cout << "P1 = " << sgmParams.P1 << ", "
              << "P2 = " << sgmParams.P2 << ". \n";
    sgm.setSmoothnessCostParameters( sgmParams.P1, sgmParams.P2 );
    sgm.setDisparityTotal( sgmParams.total );

    cv::Mat leftImageGray;
    cv::cvtColor(leftImage, leftImageGray, CV_RGB2GRAY);
    sgm.compute(leftImageSGM,
                rightImageSGM,
                dispImageFloat,
                STEREO_PIPELINE_MODE,
                cameraParamFile,
                depthImage,
                FUSE_FLAG,
                leftImageGray,
                weightImg);
    dispImage = utilities.convertFloatToCVMat(width, height, dispImageFloat);
    free(dispImageFloat);
}

void displayMinMax(cv::Mat array)
{
    double min, max;
    cv::minMaxLoc(array, &min, &max);
    std::cout << "Minimum: " << min << " | Maximum: " << max << std::endl;
}

static void save_ply( 
    const std::string &fn,
    const Eigen::MatrixXf &Q,
    const cv::Mat &dispFloat, 
    const cv::Mat &color,
    float distLimit=20.f,
    float dOffs=0.f ) {

    cv::Mat dispFloat32;
    dispFloat.convertTo(dispFloat32, CV_32FC1);

    const bool flagFlip       = true; // Flip the point cloud so that the y-axis is upwards.
    const bool flagBinary     = true; // Write binary PLY files.
    const float distanceLimit = 20.f; // All points with depth greater than this value will be ignored.
    cv::Mat predDispOffs      = dispFloat32 + dOffs; // This is required by Middlebury dataset. dOffs will be zero for other dataset.
    su::write_ply_with_color(fn, 
        predDispOffs, color, 
        Q, flagFlip, distanceLimit, flagBinary);
}

static void process( const CaseDescription &cd ) {
    /* input and output directories */
    // std::string repo_dir = argv[1];
    // std::string left_image_uri = repo_dir + "imgs/stereo_left.png";
    // std::string right_image_uri = repo_dir + "imgs/stereo_right.png";
    // std::string left_depth_uri = repo_dir + "imgs/gt_disparity.png";
    // std::string save_dir = repo_dir + "results/";

    std::string left_image_uri  = cd.fn0;
    std::string right_image_uri = cd.fn1;
    std::string left_depth_uri  = cd.fnD;
    std::string save_dir        = cd.outDir + "/";

    su::test_directory(save_dir);

    Eigen::MatrixXf Q;
    if ( cd.fnQ != "" ) {
        su::read_matrix(cd.fnQ, 4, 4, " ", Q);

		// Update the Q matrix according to the scale factor.
		Q(0, 3) *= cd.qFactor;
		Q(1, 3) *= cd.qFactor;
		Q(2, 3) *= cd.qFactor;
    }

    Utils utilities;

    std::cout << "DATA DETAILS: " << std::endl;
    std::cout << "--- Left Image: " << left_image_uri << std::endl;
    std::cout << "--- Right Image: " << right_image_uri << std::endl;
    std::cout << "--- Disparity: (GT) " << left_depth_uri << std::endl;

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    cv::Mat left_image_clr = cv::imread(left_image_uri);
    cv::Mat right_image_clr = cv::imread(right_image_uri);

    cv::Mat_<double> disp_image = cv::imread(left_depth_uri,
                                             cv::IMREAD_ANYDEPTH);
    disp_image = disp_image / SCALING_FACTOR;
    std::cout << "{read status:} successfully read input images.." << std::endl;

    /* EVALUATION A: SEMI GLOBAL MATCHING */
    std::cout << "\n{EVALUATION A:} -- SEMI GLOBAL MATCHING -- " << std::endl;
    cv::Mat disparity_image;
    SemiGlobalMatching(left_image_clr,
                       right_image_clr,
                       disparity_image,
                       STEREO_LEFT_ONLY,
                       "no_params_needed",
                       cv::Mat(),
                       cv::Mat(),
                       VOLUME_UPDATE_NONE,
                       cd.sgmParams);
    if (SAVE_IMAGES) {
        std::string save_file_name = + "sgm_default.png";
        std::string save_url = save_dir + save_file_name;
        std::cout << "{SGM} saving image to: " << save_url << std::endl;
        cv::imwrite(save_url, disparity_image, compression_params);
    }

    cv::Mat_<double> disp_SGM = disparity_image / SCALING_FACTOR;
    /* evaluate SGM */
    cv::Mat_<double> error_image_sgm;
    double average_error_sgm;
    cv::Mat sample_mask_sgm = cv::Mat::zeros(disp_SGM.rows,
                                             disp_SGM.cols,
                                             CV_32FC1);
    utilities.calculateAccuracy(disp_SGM,
                                disp_image,
                                average_error_sgm,
                                error_image_sgm,
                                sample_mask_sgm);
    std::cout << "{SGM} avg error: " << average_error_sgm << std::endl;

    cv::Mat sample_mask;
    // utilities.generateRandomSamplingMask(disp_image,
    //                                      sample_mask,
    //                                      SAMPLING_FRACTION);
    utilities.generateRandomSamplingMask(disp_image,
                                         sample_mask,
                                         cd.gtSampleFrac);

    if (SAVE_IMAGES) {
        std::string save_file_name = "sparse_mask.png";
        std::string save_url = save_dir + save_file_name;
        std::cout << "{MASK} saving image to: " << save_url << std::endl;
        cv::imwrite(save_url, sample_mask, compression_params);
    }
    cv::Mat masked_depth;
    disp_image.copyTo(masked_depth, sample_mask);

    // Save PLY file.
    if ( cd.fnQ != "" ) {
        std::string plyFn = save_dir + "Cloud_SGM.ply";
        save_ply(plyFn, Q, 
            disp_SGM, left_image_clr, 
            cd.distLimit, cd.dOffs );
        std::cout << "Point cloud saved to " << plyFn << "\n";
    }

    /* EVALUATION B: USE SPARSE LIDAR POINTS FOR NAIVE FUSION */
    std::cout << "\n{EVALUATION B:} -- NAIVE LIDAR FUSION -- " << std::endl;
    cv::Mat disparity_image_sl_naive;
    SemiGlobalMatching(left_image_clr,
                       right_image_clr,
                       disparity_image_sl_naive,
                       STEREO_LEFT_ONLY,
                       "no_params_needed",
                       masked_depth,
                       cv::Mat(),
                       VOLUME_UPDATE_NAIVE,
                       cd.sgmParams);
    if (SAVE_IMAGES) {
        std::string save_file_name = "fuse_naive.png";
        std::string save_url = save_dir + save_file_name;
        std::cout << "{Naive Fusion} saving image to: " << save_url << std::endl;
        cv::imwrite(save_url, disparity_image_sl_naive, compression_params);
    }
    cv::Mat_<double> disp_NF = disparity_image_sl_naive / SCALING_FACTOR;
    /* evaluate naive fusion */
    cv::Mat_<double> error_image_nf;
    double average_error_nf;
    utilities.calculateAccuracy(disp_NF,
                                disp_image,
                                average_error_nf,
                                error_image_nf, sample_mask);
    std::cout << "{NAIVE FUSION} avg error: " << average_error_nf << std::endl;

    // Save PLY file.
    if ( cd.fnQ != "" ) {
        std::string plyFn = save_dir + "Cloud_Naive.ply";
        save_ply(plyFn, Q, 
            disp_NF, left_image_clr, 
            cd.distLimit, cd.dOffs );
        std::cout << "Point cloud saved to " << plyFn << "\n";
    }

    /*EVALUATION C: USE DIFFUSION BASED METHOD */
    std::cout << "\n{EVALUATION C:} -- DIFFUSION BASED -- " << std::endl;
    cv::Mat disparity_image_db;
    SemiGlobalMatching(left_image_clr,
                       right_image_clr,
                       disparity_image_db,
                       STEREO_LEFT_ONLY,
                       "no_params_needed",
                       masked_depth,
                       cv::Mat(),
                       VOLUME_UPDATE_DIFFB, 
                       cd.sgmParams);
    if (SAVE_IMAGES) {
        std::string save_file_name = "fuse_diffusionbased.png";
        std::string save_url = save_dir + save_file_name;
        std::cout << "{DB} saving image to: " << save_url << std::endl;
        cv::imwrite(save_url, disparity_image_db, compression_params);
    }
    cv::Mat_<double> disp_DB = disparity_image_db / SCALING_FACTOR;
    /* evaluate diffusion based confidence propagation method */
    cv::Mat_<double> error_image_db;
    double average_error_db;
    utilities.calculateAccuracy(disp_DB,
                                disp_image,
                                average_error_db,
                                error_image_db,
                                sample_mask);
    std::cout << "{DB} avg error: " << average_error_db << std::endl;

    // Save PLY file.
    if ( cd.fnQ != "" ) {
        std::string plyFn = save_dir + "Cloud_Diffusion.ply";
        save_ply(plyFn, Q, 
            disp_DB, left_image_clr, 
            cd.distLimit, cd.dOffs );
        std::cout << "Point cloud saved to " << plyFn << "\n";
    }

    /*EVALUATION D: USE BASIC BILATERAL COST UPDATE */
    std::cout << "\n{EVALUATION D:} -- NEIGHBORHOOD SUPPORT -- " << std::endl;
    cv::Mat disparity_image_ns;
    SemiGlobalMatching(left_image_clr,
                       right_image_clr,
                       disparity_image_ns,
                       STEREO_LEFT_ONLY,
                       "no_params_needed",
                       masked_depth,
                       cv::Mat(),
                       VOLUME_UPDATE_NEIGH, 
                       cd.sgmParams);
    if (SAVE_IMAGES) {
        std::string save_file_name = "fuse_neighborhoodsupport.png";
        std::string save_url = save_dir + save_file_name;
        std::cout << "{NS} saving image to: " << save_url << std::endl;
        cv::imwrite(save_url, disparity_image_ns, compression_params);
    }
    cv::Mat_<double> disp_NS = disparity_image_ns / SCALING_FACTOR;
    /* evaluate neighborhood support method */
    cv::Mat_<double> error_image_ns;
    double average_error_ns;
    utilities.calculateAccuracy(disp_NS,
                                disp_image,
                                average_error_ns,
                                error_image_ns,
                                sample_mask);
    std::cout << "{NS} avg error: " << average_error_ns << std::endl;

    // Save PLY file.
    if ( cd.fnQ != "" ) {
        std::string plyFn = save_dir + "Cloud_Neighbor.ply";
        save_ply(plyFn, Q, 
            disp_NS, left_image_clr, 
            cd.distLimit, cd.dOffs );
        std::cout << "Point cloud saved to " << plyFn << "\n";
    }
}

int main(int argc, char** argv)
{
    if ( argc < 2 ) {
		std::cerr << "Must specify the input JSON file. \n";
		throw std::runtime_error("Must specify the input JSON file. ");
	}

	// Read the JSON file.
	std::shared_ptr<JSON> pJSON = su::read_json(argv[1]);
	auto& cases = (*pJSON)["cases"];
	const int N = cases.size();

	// Process all the cases.
	for ( int i = 0; i < N; ++i ) {
        if ( true != cases[i]["enable"] )
            continue;

		auto cd = CaseDescription();
		cd.fn0  = cases[i]["fn0"]; // Filename of image 0.
		cd.fn1  = cases[i]["fn1"]; // Filename of image 1.

		// True disparity if exists.
		cd.fnD = cases[i]["fnD"];

		cd.name         = cases[i]["name"];         // Case name.
        cd.gtSampleFrac = cases[i]["gtSampleFrac"]; // The true data sample fraction.

        std::stringstream ss;
        std::string tempDir = cases[i]["outDir"]; // This strips the double quotes.
        ss << tempDir << "/" << cd.name << "_" << cd.gtSampleFrac;
        cd.outDir = ss.str(); // Output directory.

		// Q matrix if exists.
		if ( cases[i].find("fnQ") != cases[i].end() ) {
			cd.fnQ       = cases[i]["fnQ"];
			cd.qFactor   = cases[i]["QF"];
			cd.dOffs     = cases[i]["dOffs"];
            cd.distLimit = cases[i]["distLimit"];
		} else {
			cd.fnQ       = "";
			cd.qFactor   = 1.f;
			cd.dOffs     = 0.f;
            cd.distLimit = 20.f;
		}

        // SGM parameters.
        cd.sgmParams.P1    = cases[i]["sgm"]["P1"];
        cd.sgmParams.P2    = cases[i]["sgm"]["P2"];
        cd.sgmParams.total = cases[i]["sgm"]["total"];

		std::cout << "\n========== Procesing " << cd.name << "_" << cd.gtSampleFrac << ". ==========\n\n";
		process( cd );
	}

	std::cout << "SPS-Stereo done. \n";

    return 0;
}
