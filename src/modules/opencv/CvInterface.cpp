#include <fstream>
#include <iterator>

#include "CvInterface.h"

namespace flow {
namespace opencv {

template<typename T, int N>
std::vector<T> cvVecToStdVector(const cv::Vec<T, N>& vect) { 
    return std::vector<T>(vect.val, vect.val + N);
}

template<typename T, int N = 4>
void saveCvVecToFile(const std::string& filename, const cv::Vec<T, N>& vect)
{
    std::vector<float> vectorData = cvVecToStdVector(vect);
    std::ofstream dataFile(filename, std::ios::app);
    if (!dataFile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::copy(vectorData.begin(), vectorData.end() - 1, std::ostream_iterator<T>(dataFile, ","));
    dataFile << vectorData.back() << std::endl;
    dataFile.close();
}

template void saveCvVecToFile(const std::string& filename, const cv::Vec<float, 3>& vect);
template void saveCvVecToFile(const std::string& filename, const cv::Vec<float, 4>& vect);

bool isValidRotationMatrix(const cv::Mat& rotationMatrix) {
    // Check matrix size
    if (rotationMatrix.rows != 3 || rotationMatrix.cols != 3) {
        std::cout << "Invalid rotation matrix size." << std::endl;
        return false;
    }

    // Check orthogonality
    cv::Mat transpose = rotationMatrix.t();
    cv::Mat identity = rotationMatrix * transpose;
    cv::Mat diff = identity - cv::Mat::eye(3, 3, rotationMatrix.type());
    double norm = cv::norm(diff, cv::NORM_L2);

    if (norm > 1e-6) {
        std::cout << "Rotation matrix is not orthogonal." << std::endl;
        return false;
    }

    // Check determinant
    double determinant = cv::determinant(rotationMatrix);
    if (std::abs(determinant - 1.0) > 1e-6) {
        std::cout << "Rotation matrix determinant is not equal to 1." << std::endl;
        return false;
    }

    return true;
}

cv::Vec4f rotationMatrixToQuaternion(const cv::Mat& rotationMatrix)
{
    Eigen::Matrix3d eigenRotationMatrix;
    cv::cv2eigen(rotationMatrix, eigenRotationMatrix);

    Eigen::Quaterniond quaternion(eigenRotationMatrix);
    cv::Vec4f result;
    result[0] = static_cast<float>(quaternion.x());
    result[1] = static_cast<float>(quaternion.y());
    result[2] = static_cast<float>(quaternion.z());
    result[3] = static_cast<float>(quaternion.w());
    return result;
}

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R)
{
    // assert(isValidRotationMatrix(R));
 
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6;
 
    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return cv::Vec3f(x, y, z);
}

void applyBarrelDistortion(cv::Mat& image, float k)
{
    int width = image.cols;
    int height = image.rows;

    cv::Mat distortedImage = cv::Mat::zeros(image.size(), image.type());
    cv::Point2f center(static_cast<float>(width / 2), static_cast<float>(height / 2));

    cv::Mat mapX, mapY;
    mapX.create(image.size(), CV_32FC1);
    mapY.create(image.size(), CV_32FC1);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            float nx = (x - center.x) / center.x;
            float ny = (y - center.y) / center.y;
            float r = sqrt(nx * nx + ny * ny);

            float theta = atan(r);
            float distortion = theta * k;

            float srcX = center.x + distortion * center.x * nx;
            float srcY = center.y + distortion * center.y * ny;

            mapX.at<float>(y, x) = srcX;
            mapY.at<float>(y, x) = srcY;
        }
    }

    cv::remap(image, distortedImage, mapX, mapY, cv::INTER_LINEAR);

    image = distortedImage;
}

} //opencv
} // flow
