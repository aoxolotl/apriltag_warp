#include "apriltag_utils.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#define SQ(a) (a * a)

void apriltag_init(apriltag_detector *td, apriltag_family *tf,
	float decimate, float blur, int num_threads, 
	int debug, int ref_edg, int ref_dec, int ref_pose)  
{
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = decimate;
    td->quad_sigma = blur; 
    td->nthreads = num_threads;
    td->debug = debug;
    td->refine_edges = ref_edg;
    td->refine_decode = ref_dec;
    td->refine_pose = ref_pose;
}

// Refer: https://github.com/swatbotics/apriltags-cpp/blob/master/src/CameraUtil.cpp#L68
cv::Mat getExtrinsics(apriltag_detection **det, double fx, double fy,
		double tagSize)
{
	cv::Mat_<double> h = cv::Mat(3, 3, CV_64F, (*det)->H->data);
	cv::Mat_<double> F = cv::Mat::eye(3, 3, CV_64F);
	F(1,1) = F(2,2) = -1;
	h = F * h;
	std::cout << "tagSize:" << tagSize << std::endl;
	
	bool openGLStyle = false;

	cv::Mat_<double> M(4,4);
	M(0,0) =  h(0,0) / fx;
	M(0,1) =  h(0,1) / fx;
	M(0,3) =  h(0,2) / fx;
	M(1,0) =  h(1,0) / fy;
	M(1,1) =  h(1,1) / fy;
	M(1,3) =  h(1,2) / fy;
	M(2,0) =  h(2,0);
	M(2,1) =  h(2,1);
	M(2,3) =  h(2,2);

	// Compute the scale. The columns of M should be made to be
	// unit vectors. This is over-determined, so we take the
	// geometric average.
	double scale0 = sqrt(SQ(M(0,0)) + SQ(M(1,0)) + SQ(M(2,0)));
	double scale1 = sqrt(SQ(M(0,1)) + SQ(M(1,1)) + SQ(M(2,1)));
	double scale = sqrt(scale0*scale1);

	M /= scale;

	// recover sign of scale factor by noting that observations must
	// occur in front of the camera.
	if (M(2,3) > 0)
	{
		M *= -1;
	}

	// The bottom row should always be [0 0 0 1].  
	M(3,0) = 0;
	M(3,1) = 0;
	M(3,2) = 0;
	M(3,3) = 1;

	// recover third rotation vector by crossproduct of the other two
	// rotation vectors
	cv::Vec<double, 3> a( M(0,0), M(1,0), M(2,0) );
	cv::Vec<double, 3> b( M(0,1), M(1,1), M(2,1) );
	cv::Vec<double, 3> ab = a.cross(b);

	M(0,2) = ab[0];
	M(1,2) = ab[1];
	M(2,2) = ab[2];

	// pull out just the rotation component so we can normalize it.

	cv::Mat_<double> R(3,3);
	for (int i=0; i<3; ++i) 
	{
		for (int j=0; j<3; ++j) 
		{
			R(i,j) = M(i,j);
		}
	}

	// polar decomposition, R = (UV')(VSV')

	cv::SVD svd(R);
	cv::Mat_<double> MR = svd.u * svd.vt;

	if (!openGLStyle) 
	{ 
		MR = F * MR; 
	}

	for (int i=0; i<3; ++i) 
	{
		for (int j=0; j<3; ++j) 
		{
			M(i,j) = MR(i,j);
		}
	}

	// Scale the results based on the scale in the homography. The
	// homography assumes that tags span from -1 to +1, i.e., that
	// they are two units wide (and tall).
	for (int i = 0; i < 3; i++) 
	{
		double scl = openGLStyle ? 1 : F(i,i);
		M(i,3) = M(i,3) * scl * tagSize / 2.0;
	}

	return M;
}    

void getExtrinsics_old(apriltag_detection *det, double fx, double fy,
					double *rot, double *trans)
{
	double *homography = det->H->data;

	double scale_factor, sf1, sf2;

	sf1 = sqrt(SQ(homography[0] / fx) 
				+ SQ(homography[3] / fy)
				+ SQ(homography[6]));


	sf2 = sqrt(SQ(homography[1] / fx) 
				+ SQ(homography[4] / fy)
				+ SQ(homography[7]));

	scale_factor = sqrt(sf1 * sf2);

	rot[0] = homography[0] / (scale_factor * fx);
	rot[1] = homography[3] / (scale_factor * fy);
	rot[2] = homography[6] / scale_factor;

	rot[3] = homography[1] / (scale_factor * fx);
	rot[4] = homography[4] / (scale_factor * fy);
	rot[5] = homography[7] / scale_factor;

	rot[6] = rot[1] * rot[5] - (rot[4] * rot[2]);
	rot[7] = rot[3] * rot[2] - (rot[0] * rot[5]);
	rot[8] = rot[0] * rot[4] - (rot[3] * rot[1]);

	trans[0] = homography[2] / (scale_factor * fx);
	trans[1] = homography[5] / (scale_factor * fy);
	trans[2] = homography[8] / scale_factor;
}

void getEulerAngles(double *r, double *ang_x, double *ang_y, double *ang_z)
{
	*ang_x = atan2(r[7], r[8]);
	*ang_y = atan2(-r[6], sqrt(SQ(r[0]) + SQ(r[3])));
	*ang_z = atan2(r[3], r[0]);
}

