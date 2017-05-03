#include "apriltag/apriltag.h"
#include "apriltag_utils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define EPS 1e-5
// apriltag width and height in mm
#define AT_WIDTH 26
#define AT_HEIGHT 26

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
	apriltag_init(td, tf,  1.0f, 0.0f, 4, 0, 0, 0, 0);

	// Load colour image
	cv::Mat image = cv::imread(argv[1]);
	
	// Grayscale for apriltag
	cv::Mat grayim;
   	cv::cvtColor(image, grayim, cv::COLOR_BGR2GRAY);

	// Refer: https://github.com/openmv/apriltag/blob/master/example/opencv_demo.cc#L114 
	image_u8_t im = {
		.width =	grayim.cols,
		.height =	grayim.rows,
		.stride =	grayim.cols,
		.buf = 		grayim.data};

	// Detect apriltag
	zarray_t *detections = apriltag_detector_detect(td, &im);
	int num_det = zarray_size(detections);
	std::cout << "Detection count:" << num_det << std::endl;
	if(!num_det)
	{
		printf("Not detected\n");
		return -1;
	}

	apriltag_detection_t *det;

	cv::Mat H_av = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
	int i = 0;
	for(i = 0; i < num_det; ++i)
	{
		zarray_get(detections, i, &det);

		// Convert homography to Mat for warpPerspective
		cv::Mat H = cv::Mat(3, 3, CV_64F, det->H->data);

		double t[9] = {
			1, 0,	-det->c[0],
			0, 1,	-det->c[1],
			0, 0,	1};
		cv::Mat Tr = cv::Mat(3,3, CV_64F, t); /**< Translation mat to get the H in image frame**/

		double sx = abs(det->p[0][0] - det->p[1][0]) / 2;
		double sy = abs(det->p[0][1] - det->p[2][1]) / 2;
		double s[9] = {
			1/sx,	0, 		0,
			0,		1/sy, 	0,
			0, 		0, 		1};
		cv::Mat Sc = cv::Mat(3, 3, CV_64F, s); /**< Scale factor to keep it in image scale**/

		H = H * Sc;
		H = H * Tr;

		std::cout << H << std::endl;
		std::cout << "" << std::endl;
		H_av += H;
	}

	H_av /= i;
	std::cout << H_av << std::endl;

	// Warp image
	cv::Mat warp_im = cv::Mat::zeros(image.size(), image.type());
	cv::warpPerspective(image, warp_im, H_av, image.size(), cv::WARP_INVERSE_MAP);
	
	// Detect apriltag again for measurements
   	cv::cvtColor(warp_im, grayim, cv::COLOR_BGR2GRAY);
	image_u8_t im2 = {
		.width =	grayim.cols,
		.height =	grayim.rows,
		.stride =	grayim.cols,
		.buf = 		grayim.data};

	detections = apriltag_detector_detect(td, &im2);
	zarray_get(detections, 0, &det);

	// Measurement scale in x and y
	double meas_x = AT_WIDTH / fabs(det->p[0][0] - det->p[1][0]);
	double meas_y = AT_HEIGHT / fabs(det->p[0][1] - det->p[2][1]); 
	printf("m_x:%lf, m_y:%lf\n", meas_x, meas_y);

	//Write out warped image
	cv::imwrite("warp_im.png", warp_im);

	return 0;
}
