#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

#define EPS 1e-5
// apriltag width and height in mm
#define AT_WIDTH 26
#define AT_HEIGHT 26

int warpUsingRot(cv::Mat image, cv::Mat_<double> R, int fx, int fy)
{
	int imh = image.rows;
	int imw = image.cols;

	//cv::Mat_<double> R = M.rowRange(0, 3).colRange(0, 3);
	//cv::Mat trans = M.rowRange(0, 3).col(3);

	R.push_back(cv::Mat::zeros(1, 3, CV_64F));
	cv::hconcat(R, cv::Mat::zeros(4, 1, CV_64F), R);
	R(3, 3) = 1.0;
	
	// Warp image using rotation
	// "Projection" Matrix
	cv::Mat A1 = (cv::Mat_<double>(4,3) <<
			1, 0, -imw/2.0,
			0, 1, -imh/2.0,
			0, 0,    0,
			0, 0,    1);
	// Intrinsics Matrix
	cv::Mat K = (cv::Mat_<double>(3,4) <<
			fx, 0, imw/2.0, 0,
			0, fy, imh/2.0, 0,
			0, 0,   1, 0);
	// translate
	cv::Mat T = (cv::Mat_<double>(4, 4) <<
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, fx,
			0, 0, 0, 1);

	// overall transformation matrix
	cv::Mat transfo = K * (T * (R * A1));

	cv::Mat warp_im = cv::Mat::zeros(image.size(), image.type());
	cv::warpPerspective(image, warp_im, transfo, cv::Size(imw, imh),
			cv::WARP_INVERSE_MAP);
	
	//transformTool(image, transfo);
	cv::imwrite("rot_warp_im.png", warp_im);
	
	return 0;
}

int main(int argc, char **argv)
{
	if(argc < 2)
	{
		printf("Usage: %s imname\n", argv[0]);
		exit(-1);
	} 
	
	// Initialize detector
	AprilTags::TagDetector* m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);

	cv::Mat image = cv::imread(argv[1]);
	int imw = image.rows;
	int imh = image.cols;
	
	// Grayscale for apriltag
	cv::Mat grayim;
   	cv::cvtColor(image, grayim, cv::COLOR_BGR2GRAY);

	// Detect apriltag
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(grayim);

	double fx = 391.096, fy = 463.098;
	
	//Eigen::Matrix4d T = 
	//	detections[0].getRelativeTransform(0.277, 
	//			fx, fy, imw/2.0, imh/2.0);

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detections[0].getRelativeTranslationRotation(0.277, fx, fy, 
			imw/2.0, imh/2.0, translation, rotation);
    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;

	cv::Mat_<double> R = cv::Mat::zeros(3, 3, CV_64F);
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			R(i, j) = fixed_rot(i, j);

	std::cout << "rotation mat " << rotation << std::endl;
	warpUsingRot(image, R, fx, fy);

	return 0;
}
