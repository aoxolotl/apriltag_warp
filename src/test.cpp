#include "apriltag/apriltag.h"
#include "apriltag_utils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define EPS 1e-5
// apriltag width and height in mm
#define AT_WIDTH 26
#define AT_HEIGHT 26

int transformTool(cv::Mat source, cv::Mat R1)
{
	int alpha_=90., beta_=90., gamma_=90.;
	int f_ = 500, dist_ = 500;
	
	cv::Mat destination;
	
	std::string wndname1 = "Source: ";
	std::string wndname2 = "WarpPerspective: ";
	std::string tbarname1 = "Alpha";
	std::string tbarname2 = "Beta";
	std::string tbarname3 = "Gamma";
	std::string tbarname4 = "f";
	std::string tbarname5 = "Distance";
	cv::namedWindow(wndname1, CV_WINDOW_NORMAL);
	cv::namedWindow(wndname2, CV_WINDOW_NORMAL);
	cv::createTrackbar(tbarname1, wndname2, &alpha_, 180);
	cv::createTrackbar(tbarname2, wndname2, &beta_, 180);
	cv::createTrackbar(tbarname3, wndname2, &gamma_, 180);
	cv::createTrackbar(tbarname4, wndname2, &f_, 2000);
	cv::createTrackbar(tbarname5, wndname2, &dist_, 2000);
	
	cv::imshow(wndname1, source);
	cv::resizeWindow(wndname1, 800, 400);
	cv::resizeWindow(wndname2, 800, 400);
	while(true) {
	    double f, dist;
	    double alpha, beta, gamma;
	    alpha = ((double)alpha_ - 90.) * 3.1416/180;
	    beta = ((double)beta_ - 90.) * 3.1416/180;
	    gamma = ((double)gamma_ - 90.) * 3.1416/180;
	    f = (double) f_;
	    dist = (double) dist_;
	
		cv::Size taille = source.size();
	    double w = (double)taille.width, h = (double)taille.height;
	
	    // Projection 2D -> 3D matrix
		cv::Mat A1 = (cv::Mat_<double>(4,3) <<
	        1, 0, -w/2,
	        0, 1, -h/2,
	        0, 0,    0,
	        0, 0,    1);
	
	    // Rotation matrices around the X,Y,Z axis
		cv::Mat RX = (cv::Mat_<double>(4, 4) <<
	        1,          0,           0, 0,
	        0, cos(alpha), -sin(alpha), 0,
	        0, sin(alpha),  cos(alpha), 0,
	        0,          0,           0, 1);
	
		cv::Mat RY = (cv::Mat_<double>(4, 4) <<
	        cos(beta), 0, -sin(beta), 0,
	                0, 1,          0, 0,
	        sin(beta), 0,  cos(beta), 0,
	                0, 0,          0, 1);
	
		cv::Mat RZ = (cv::Mat_<double>(4, 4) <<
	        cos(gamma), -sin(gamma), 0, 0,
	        sin(gamma),  cos(gamma), 0, 0,
	        0,          0,           1, 0,
	        0,          0,           0, 1);
	
	    // Composed rotation matrix with (RX,RY,RZ)
		cv::Mat R = RX * RY * RZ;
	
	    // Translation matrix on the Z axis change dist will change the height
	    cv::Mat T = (cv::Mat_<double>(4, 4) <<
	        1, 0, 0, 0,
	        0, 1, 0, 0,
	        0, 0, 1, dist,
	        0, 0, 0, 1);
	
	    // Camera Intrisecs matrix 3D -> 2D
	    cv::Mat A2 = (cv::Mat_<double>(3,4) <<
	        f, 0, w/2, 0,
	        0, f, h/2, 0,
	        0, 0,   1, 0);
	
	    // Final and overall transformation matrix
		cv::Mat transfo = A2 * (T * (R * A1));
		std::cout << "Trans from main" << R1 << std::endl;
		std::cout << "transfo" << transfo << std::endl;
	
	    // Apply matrix transformation
		cv::warpPerspective(source, destination, transfo, taille, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
	
		cv::imshow(wndname2, destination);
		cv::waitKey(30);
	}
}

int main(int argc, char **argv)
{
	if(argc < 2)
	{
		printf("Usage: %s imname\n", argv[0]);
		exit(-1);
	} 
	
	// Initialize detector
	apriltag_family_t *tf = tag36h11_create();
	apriltag_detector_t *td = apriltag_detector_create();

//void apriltag_init(apriltag_detector *td, apriltag_family *tf,
//	float decimate, float blur, int num_threads, 
//	int debug, int ref_edg, int ref_dec, int ref_pose)  
	apriltag_init(td, tf,  1.0f, 0.0f, 2, 0, 0, 0, 0);

	// Load colour image
	cv::Mat image = cv::imread(argv[1]);
	
	// Grayscale for apriltag
	cv::Mat grayim;
   	cv::cvtColor(image, grayim, cv::COLOR_BGR2GRAY);
	int imh = grayim.rows;
	int imw = grayim.cols;

	// Refer: https://github.com/openmv/apriltag/blob/master/example/opencv_demo.cc#L114 
	image_u8_t im = {
		.width =	grayim.cols,
		.height =	grayim.rows,
		.stride =	grayim.cols,
		.buf = 		grayim.data};

	// Detect apriltag
	zarray_t *detections = apriltag_detector_detect(td, &im);
	if(!zarray_size(detections))
	{
		printf("Not detected\n");
		return -1;
	}

	apriltag_detection_t *det;
	zarray_get(detections, 0, &det);

	// Convert homography to Mat for warpPerspective
	cv::Mat H = cv::Mat(3, 3, CV_64F, det->H->data);

	double t[9] = {
		1, 0,	-det->c[0],
		0, 1,	-det->c[1],
		0, 0,	1};
	cv::Mat Tr = cv::Mat(3,3, CV_64F, t); /**< Translation mat to get the H in image frame**/
	
	double sx = fabs(det->p[0][0] - det->p[1][0]) / 2;
	double sy = fabs(det->p[0][1] - det->p[2][1]) / 2;
	double s[9] = {
		1/sx,	0, 		0,
		0,		1/sy, 	0,
		0, 		0, 		1};
	cv::Mat Sc = cv::Mat(3, 3, CV_64F, s); /**< Scale factor to keep it in image scale**/

	H = H * Sc;
	H = H * Tr;

	// Warp image
	cv::Mat warp_im = cv::Mat::zeros(image.size(), image.type());
	cv::warpPerspective(image, warp_im, H, image.size(), cv::WARP_INVERSE_MAP);

	// Decompose Homography
	double fx = 391.096, fy = 463.098;
	cv::Mat M = getExtrinsics(&det, fx, fx, fabs(det->p[0][0] - det->p[1][0]));
	cv::Mat_<double> rot;
	cv::Mat_<double> R = M.rowRange(0, 3).colRange(0, 3);
	cv::Rodrigues(R, rot);
	cv::Mat trans = M.rowRange(0, 3).col(3);

	R.push_back(cv::Mat::zeros(1, 3, CV_64F));
	cv::hconcat(R, cv::Mat::zeros(4, 1, CV_64F), R);
	R(3, 3) = 1.0;
	
	// Detect apriltag again for measurements
   	cv::cvtColor(warp_im, grayim, cv::COLOR_BGR2GRAY);
	image_u8_t im2 = {
		.width =	grayim.cols,
		.height =	grayim.rows,
		.stride =	grayim.cols,
		.buf = 		grayim.data};

	detections = apriltag_detector_detect(td, &im2);
	if(!zarray_size(detections)) { printf("Not detected in warped\n"); return -1; }
	zarray_get(detections, 0, &det);

	// Measurement scale in x and y
	double meas_x = AT_WIDTH / fabs(det->p[0][0] - det->p[1][0]);
	double meas_y = AT_HEIGHT / fabs(det->p[0][1] - det->p[2][1]); 
	printf("m_x:%lf, m_y:%lf\n", meas_x, meas_y);

	// Write out warped image
	cv::imwrite("warp_im.png", warp_im);
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
			0, 0, 1, 400,
			0, 0, 0, 1);

	// overall transformation matrix
	cv::Mat transfo = K * (T * (R * A1));

	cv::warpPerspective(image, warp_im, transfo, cv::Size(imw, imh),
			cv::WARP_INVERSE_MAP);
	//transformTool(image, transfo);
	cv::imwrite("rot_warp_im.png", warp_im);
	return 0;
}
