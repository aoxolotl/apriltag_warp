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
	apriltag_init(td, tf,  1.0f, 0.0f, 4, 1, 0, 0, 0);

	image_u8_t *im = image_u8_create_from_pnm(argv[1]);

	zarray_t *detections = apriltag_detector_detect(td, im);
	cv::Mat image = cv::imread(argv[1]);
	
	int num_det = zarray_size(detections);
	if(!num_det)
		printf("Not detected\n");
	else
		printf("Num det:%d\n", num_det);

	double vpx_h[2], vpy_h[2], vpx_v[2], vpy_v[2];

	for(int d = 0; d < num_det; ++d)
	{
		apriltag_detection_t *det;

		zarray_get(detections, d, &det);
		
		double slope1 = (det->p[0][1] - det->p[1][1]) / (((det->p[0][0] - det->p[1][0]) != 0) ? (det->p[0][0] - det->p[1][0]) : EPS);
		double slope2 = (det->p[2][1] - det->p[3][1]) / (((det->p[2][0] - det->p[3][0]) != 0) ? (det->p[2][0] - det->p[3][0]) : EPS);
		double slope3 = (det->p[0][1] - det->p[3][1]) / (((det->p[0][0] - det->p[3][0]) != 0) ? (det->p[0][0] - det->p[3][0]) : EPS);
		double slope4 = (det->p[2][1] - det->p[1][1]) / (((det->p[2][0] - det->p[1][0]) != 0) ? (det->p[2][0] - det->p[1][0]) : EPS);

		// Horizontal vp
		vpx_h[d] = det->p[2][1] - det->p[0][1] + (slope1 * det->p[0][0]) - (slope2 * det->p[2][0]);
		vpx_h[d] /= (slope1 - slope2);
		vpy_h[d] = (slope1 * (vpx_h[d] - det->p[0][0])) + det->p[0][1];

		// Vertical vp
		vpx_v[d] = det->p[2][1] - det->p[0][1] + (slope3 * det->p[0][0]) - (slope4 * det->p[2][0]);
		vpx_v[d] /= (slope3 - slope4);
		vpy_v[d] = (slope3 * (vpx_v[d] - det->p[0][0])) + det->p[0][1];

		printf("H: %lf, %lf V:%lf, %lf\n", vpx_h[d], vpy_h[d], vpx_v[d], vpy_v[d]);

		
		cv::Point crop_pts[2];
		crop_pts[0].x = det->p[0][0];
		crop_pts[0].y = det->p[0][1];
		crop_pts[1].x = 3000;
		crop_pts[1].y = 2100;
		//// Lines from vanishing points to top-left and bottom-right corner

		int colour_change = d * 70 + 70;

		cv::line(image, cv::Point((int ) round(vpx_h[d]), (int ) round(vpy_h[d])), crop_pts[0], cv::Scalar(0, colour_change, colour_change), 3);
		cv::line(image, cv::Point((int ) round(vpx_h[d]), (int ) round(vpy_h[d])), crop_pts[1], cv::Scalar(0, colour_change, colour_change), 3);
		//cv::line(image, cv::Point((int ) round(vpx_v), (int ) round(vpy_v)), crop_pts[0], cv::Scalar(0, 255, 255), 3);
		//cv::line(image, cv::Point((int ) round(vpx_v), (int ) round(vpy_v)), crop_pts[1], cv::Scalar(255, 255, 0), 3);
		
	}
	cv::imwrite("Crop.png", image);

	return 0;
}
