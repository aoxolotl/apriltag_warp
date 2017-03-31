#ifndef APRILTAG_UTILS_H
#define APRILTAG_UTILS_H

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include <opencv2/core/core.hpp>

void apriltag_init(apriltag_detector *td, apriltag_family *tf,
	float decimate, float blur, int num_threads, 
	int debug, int ref_edg, int ref_dec, int ref_pose);

cv::Mat getExtrinsics(apriltag_detection *det, double fx, double fy,
					double tagSize);

void getEulerAngles(double *rot, double *ang_x, double *ang_y, double *ang_z);
#endif //APRILTAG_UTILS_H
