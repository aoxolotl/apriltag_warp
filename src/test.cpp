#include "apriltag/apriltag.h"
#include "apriltag_utils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define EPS 1e-5

void on_mouse(int e, int x, int y, int d, void *pt)
{
	cv::Point *p = (cv::Point *)pt;
	p->x = x;
	p->y = y;
}

int main(int argc, char **argv)
{
	if(argc < 3)
	{
		printf("Usage: %s imname dumpfile\n", argv[0]);
		exit(-1);
	} apriltag_family_t *tf = tag36h11_create();
	apriltag_detector_t *td = apriltag_detector_create();

//void apriltag_init(apriltag_detector *td, apriltag_family *tf,
//	float decimate, float blur, int num_threads, 
//	int debug, int ref_edg, int ref_dec, int ref_pose)  
	apriltag_init(td, tf,  1.0f, 0.0f, 4, 0, 0, 0, 0);

	image_u8_t *im = image_u8_create_from_pnm(argv[1]);

	zarray_t *detections = apriltag_detector_detect(td, im);
	cv::Mat image = cv::imread(argv[1]);
	
	if(!zarray_size(detections))
		printf("Not detected\n");

	//for(int d = 0 ; d < zarray_size(detections); ++d)
	//{
		apriltag_detection_t *det;

		zarray_get(detections, 0, &det);
		
		FILE *fp = fopen(argv[2], "w");
		FILE *fp2 = fopen(argv[3], "w");
		FILE *fp3 = fopen(argv[4], "w");

		fprintf(fp2, "%f %f\n", det->c[0], det->c[1]);
		fclose(fp2);

		double rot[9], trans[3];

		getExtrinsics(det, 762, 771, rot, trans);


		// Rotation matrix
		cv::Mat R = cv::Mat(3, 3, CV_32F, rot);
                double ang_x, ang_y, ang_z;

                getEulerAngles(rot, &ang_x, &ang_y, &ang_z);
                ang_x = ang_x * 180 / 3.1416;
                ang_y = ang_y * 180 / 3.1416;
                ang_z = ang_z * 180 / 3.1416;
                printf("Angles: %lf, %lf, %lf\n", ang_x, ang_y, ang_z);

		
		   for(int i = 0; i < 9; ++i)
		   fprintf(fp, "%lf ", det->H->data[i]);
		   fprintf(fp, "\n");
		   fclose(fp);

		   for(int i = 0; i < 4; ++i)
		   fprintf(fp3, "%lf %lf ", det->p[i][0], det->p[i][1]);
		   fprintf(fp3, "\n");
		   fclose(fp3);

		   cv::Mat warp_im = cv::Mat::zeros(image.size(), CV_32F);
		   cv::warpPerspective(image, warp_im, R, image.size());

		   cv::imwrite("warp_im.png", warp_im);
	//}

	return 0;
}
