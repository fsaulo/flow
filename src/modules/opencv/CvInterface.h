#ifndef FLOW_OPENCV_INTERFACE_H
#define FLOW_OPENCV_INTERFACE_H

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace flow {
namespace opencv {

template<typename T, int N>
std::vector<T> cvVecToStdVector(const cv::Vec<T, N>& vect);

template<typename T, int N>
void saveCvVecToFile(const std::string& filename, const cv::Vec<T, N>& vect);
void applyBarrelDistortion(cv::Mat& image, float k);

bool isValidRotationMatrix(const cv::Mat& rotationMatrix);

cv::Vec4f rotationMatrixToQuaternion(const cv::Mat& rotationMatrix);
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R);


} // opencv
} // flow

#endif // FLOW_OPENCV_INTERFACE_H
