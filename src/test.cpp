#include "apriltag/apriltag.h"
#include "apriltag_utils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define EPS 1e-3


void on_mouse(int e, int x, int y, int d, void *ptr)
{
	if(e == CV_EVENT_LBUTTONDOWN)
	{
		std::vector<cv::Point> *p = (std::vector<cv::Point> *) ptr;
		p->push_back(cv::Point(x, y));
	}
}

/**
 * @brief Get intersection points formed by two lines from 4 points
 * @params a1, a2 points from first tag array of x, y
 * @params b1, b2 points from second tag array of x, y
 * @params vp_out ouput of x and y 
 */
void get_intersection_points(double *a1, double *a2, double *b1, double *b2, double *vp_out)
{
	double slope1 = (a1[1] - a2[1]) / 
		(((a1[0] - a2[0]) != 0) ? (a1[0] - a2[0]) : EPS);

	double slope2 = (b1[1] - b2[1]) / 
		(((b1[0] - b2[0]) != 0) ? (b1[0] - b2[0]) : EPS);

	// Horizontal vp
	double vpx_h = b1[1] - a1[1] + (slope1 * a1[0]) - (slope2 * b1[0]);
	vpx_h /= (slope1 - slope2);
	double vpy_h = (slope1 * (vpx_h - a1[0])) + a1[1];

	vp_out[0] = vpx_h;
	vp_out[1] = vpy_h;
}

int main(int argc, char **argv)
{
	if(argc < 2)
	{
		printf("Usage: %s imname\n", argv[0]);
		exit(-1);
	} 

	apriltag_family_t *tf = tag36h11_create();
	apriltag_detector_t *td = apriltag_detector_create();

//void apriltag_init(apriltag_detector *td, apriltag_family *tf,
//	float decimate, float blur, int num_threads, 
//	int debug, int ref_edg, int ref_dec, int ref_pose)  
	apriltag_init(td, tf,  1.0f, 0.0f, 4, 0, 0, 0, 0);

	image_u8_t *im = image_u8_create_from_pnm(argv[1]);

	zarray_t *detections = apriltag_detector_detect(td, im);
	cv::Mat image = cv::imread(argv[1]);
	
	int num_det = zarray_size(detections);
	if(!num_det)
		printf("Not detected\n");
	else
		printf("Num det:%d\n", num_det);

	apriltag_detection_t dets[2];
	apriltag_detection_t *det;

	for(int d = 0; d < num_det; ++d)
	{

		zarray_get(detections, d, &det);
		dets[d] = *det;
	}
		
	double vph[2], vpv[2];
	double vph2[2], vpv2[2];
	// Horizontal vp
	get_intersection_points(dets[0].p[0], dets[0].p[1], 
			dets[1].p[0], dets[1].p[1], vph);

	// Vertical vp
	get_intersection_points(dets[0].p[0], dets[0].p[3], 
			dets[1].p[0], dets[1].p[3], vpv);

	printf("H: %lf, %lf V:%lf, %lf\n", vph[0], vph[1], 
			vpv[0], vpv[1]);
	
	// Horizontal vp2
	get_intersection_points(dets[0].p[2], dets[0].p[3], 
			dets[1].p[2], dets[1].p[3], vph2);

	// Vertical vp2
	get_intersection_points(dets[0].p[1], dets[0].p[2], 
			dets[1].p[1], dets[1].p[2], vpv2);

	printf("H2: %lf, %lf V2:%lf, %lf\n", vph2[0], vph2[1], 
			vpv2[0], vpv2[1]);

	// Average of two vps
	double vph_av[2], vpv_av[2];
	vph_av[0] = (vph[0] + vph2[0]) / 2.0;
	vph_av[1] = (vph[1] + vph2[1]) / 2.0;
	vpv_av[0] = (vpv[0] + vpv2[0]) / 2.0;
	vpv_av[1] = (vpv[1] + vpv2[1]) / 2.0;

	// Get user input
	std::vector<cv::Point> p;
	
	cv::namedWindow("warppts", CV_WINDOW_NORMAL);
	cv::imshow("warppts", image);
	cv::resizeWindow("warppts", 1600, 900);
	cv::setMouseCallback("warppts", on_mouse, (void *)&p);
	cv::waitKey(0);

	if(p.size() != 2)
	{
		printf("Exactly 2 points to mark a region\n");
		return -1;
	}
	// top-left, bottom-right convert to array
	double tl[2] = {p[0].x, p[0].y};
	double br[2] = {p[1].x, p[1].y};
	// top-right and bottom-left from intersections
	double tr[2], bl[2]; get_intersection_points(tl, vph_av, br, vpv_av, tr); get_intersection_points(tl, vpv_av, br, vph_av, bl);

	std::cout << "top-left:" << tl[0] << ", " << tl[1]
		<< "  top-right:" << tr[0] << ", " << tr[1]
		<< "  bottom-right:" << br[0] << ", " << br[1]
		<< "  bottom-left:" << bl[0] << ", " << bl[1]
		<< std::endl;

	cv::line(image, p[0], cv::Point((int ) tr[0], (int ) tr[1]), cv::Scalar(0, 255, 255), 3);
	cv::line(image, p[1], cv::Point((int ) bl[0], (int ) bl[1]), cv::Scalar(0, 255, 255), 3);
	cv::line(image, p[0], cv::Point((int ) bl[0], (int ) bl[1]), cv::Scalar(0, 255, 255), 3);
	cv::line(image, p[1], cv::Point((int ) tr[0], (int ) tr[1]), cv::Scalar(0, 255, 255), 3);

	cv::Mat M;
	cv::Point2f src[4], dst[4];
	src[0] = cv::Point2f(tl[0], tl[1]);
	src[1] = cv::Point2f(tr[0], tr[1]);
	src[2] = cv::Point2f(br[0], br[1]);
	src[3] = cv::Point2f(bl[0], bl[1]);

	dst[0] = cv::Point2f(0.0, 0.0);
	dst[1] = cv::Point2f(image.cols, 0.0);
	dst[2] = cv::Point2f(image.cols, image.rows);
	dst[3] = cv::Point2f(0.0, image.rows);

	M = cv::getPerspectiveTransform(src, dst);
	std::cout << M << std::endl;

	cv::Mat warp_image;
	cv::warpPerspective(image, warp_image, M, 
			cv::Size(image.cols, image.rows));

	image_u8 warp_im ={
		.width = warp_image.cols,
		.height = warp_image.rows,
		.stride = warp_image.cols,
		.buf = warp_image.data};

	zarray_get(detections, 0, &det);
	// Measurement scale in x and y
	double tag_width = 16.2;
	double tag_height = 16.2;

	double meas_x = tag_width / 
		fabs(det->p[0][0] - det->p[1][0]);
	double meas_y = tag_height / 
		fabs(det->p[0][1] - det->p[2][1]); 
	printf("width:%lf, height:%lf\n",
		   	warp_image.cols * meas_x, warp_image.rows * meas_y);

	cv::imwrite("Crop.png", warp_image);
	
	return 0;
}
