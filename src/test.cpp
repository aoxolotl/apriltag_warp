#include "apriltag/apriltag.h"
#include "apriltag_utils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define EPS 1e-3

void on_mouse(int e, int x, int y, int d, void *pt)
{
	cv::Point *p = (cv::Point *)pt;
	p->x = x;
	p->y = y;
}

/**
 * @brief Get vanishing points formed by two lines from 4 points
 * @params a1, a2 points from first tag array of x, y
 * @params b1, b2 points from second tag array of x, y
 * @params vp_out ouput of x and y 
 */
void get_vanishing_points(double *a1, double *a2, double *b1, double *b2, double *vp_out)
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

	for(int d = 0; d < num_det; ++d)
	{
		apriltag_detection_t *det;

		zarray_get(detections, d, &det);
		dets[d] = *det;
	}
		
	double vph[2], vpv[2];
	double vph2[2], vpv2[2];
	// Horizontal vp
	get_vanishing_points(dets[0].p[0], dets[0].p[1], 
			dets[1].p[0], dets[1].p[1], vph);

	// Vertical vp
	get_vanishing_points(dets[0].p[0], dets[0].p[3], 
			dets[1].p[0], dets[1].p[3], vpv);

	printf("H: %lf, %lf V:%lf, %lf\n", vph[0], vph[1], 
			vpv[0], vpv[1]);
	
	// Horizontal vp2
	get_vanishing_points(dets[0].p[2], dets[0].p[3], 
			dets[1].p[2], dets[1].p[3], vph2);

	// Vertical vp2
	get_vanishing_points(dets[0].p[1], dets[0].p[2], 
			dets[1].p[1], dets[1].p[2], vpv2);

	printf("H2: %lf, %lf V2:%lf, %lf\n", vph2[0], vph2[1], 
			vpv2[0], vpv2[1]);

	cv::Point crop_pts[2];
	crop_pts[0].x = dets[0].p[0][0];
	crop_pts[0].y = dets[0].p[0][1];
	crop_pts[1].x = 380;
	crop_pts[1].y = 200;
	//// Lines from vanishing points to top-left and bottom-right corner

	int colour_change = 70 + 70;

	// Draw lines to see how parallel they are
	cv::line(image, cv::Point((int ) round(vph[0]), (int ) round(vph[1])), crop_pts[0], cv::Scalar(0, colour_change, colour_change), 3);
	cv::line(image, cv::Point((int ) round(vph[0]), (int ) round(vph[1])), crop_pts[1], cv::Scalar(0, colour_change, colour_change), 3);
	//cv::line(image, cv::Point((int ) round(v2px_v), (int ) round(vp2y_v)), crop_pts[0], cv::Scalar(0, 255, 255), 3);
	cv::line(image, cv::Point((int ) round(vpv[0]), (int ) round(vpv[1])), crop_pts[1], cv::Scalar(255, 255, 0), 3);

	cv::imwrite("Crop.png", image);

	return 0;
}
