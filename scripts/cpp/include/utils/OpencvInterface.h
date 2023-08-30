#ifndef OPENCV_INTERFACE_H
#define OPENCV_INTERFACE_H

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>

template<typename T, int N>
std::vector<T> cvVecToStdVector(const cv::Vec<T, N>& vect) { 
    return std::vector<T>(vect.val, vect.val + N);
}

template<typename T, int N>
void saveCvVecToFile(std::string& filename, const cv::Vec<T, N>& vect);
void applyBarrelDistortion(cv::Mat& image, float k);

bool isValidRotationMatrix(const cv::Mat& rotationMatrix);

cv::Vec4f rotationMatrixToQuaternion(const cv::Mat& rotationMatrix);
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R);

#endif // OPENCV_INTERFACE_H
