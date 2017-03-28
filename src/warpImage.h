#include <opencv2/core/core.hpp>

void warpMatrix(cv::Size   sz,
                double theta,
                double phi,
                double gamma,
                double scale,
                double fovy,
                cv::Mat&   M,
                std::vector<cv::Point2f>* corners);

void warpImage(const cv::Mat &src,
               double    theta,
               double    phi,
               double    gamma,
               double    scale,
               double    fovy,
               cv::Mat&      dst,
               cv::Mat&      M,
               std::vector<cv::Point2f> &corners);
