#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>

#define PI 3.14159265358979323846

class DCOffsetRemover {
public:
  DCOffsetRemover(int windowSize) : windowSize_(windowSize), sum_(0) {}

  double removeOffset(double sample) {
    // Add the new sample to the sum
    sum_ += sample;

    // Add the sample to the window buffer
    windowBuffer_.push_back(sample);

    // If the window buffer is larger than the specified window size,
    // remove the oldest sample from the sum and the buffer
    if (windowBuffer_.size() > windowSize_) {
      sum_ -= windowBuffer_.front();
      windowBuffer_.pop_front();
    }

    // Calculate the average and subtract it from the sample
    double average = sum_ / windowBuffer_.size();
    return sample - average;
  }

private:
  int windowSize_;
  double sum_;
  std::deque<double> windowBuffer_;
};  

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

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
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

void saveVec3fToFile(std::string filename, cv::Vec3f data)
{
    std::ofstream dataFile(filename, std::ios::app);
    if (dataFile.is_open()) {
        dataFile << data[0] << "," << data[1] << "," << data[2] << std::endl;
        dataFile.close();
    }
} 

void saveQuatfToFile(std::string filename, cv::Vec4f data)
{
    std::ofstream dataFile(filename, std::ios::app);
    if (dataFile.is_open()) {
        dataFile << data[0] << "," << data[1] << "," << data[2]  << "," << data[3] << std::endl;
        dataFile.close();
    }
} 

void clearFile(std::string filename)
{
    std::ofstream dataFile(filename, std::ios::trunc);
    while (dataFile.is_open()) 
        dataFile.close();
}

int main(int argc, char* argv[])
{

    const std::string keys =
        "{ h help |      | print this help message }"
        "{ @image | vtest.avi | path to image file }";

    cv::CommandLineParser parser(argc, argv, keys);
    std::string pipeline = parser.get<std::string>("@image");
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    cv::VideoCapture video(pipeline, cv::CAP_GSTREAMER);
    if (!video.isOpened())
    {
        std::cout << "Failed to open the video file!\n";
        return -1;
    }

    cv::Mat prevFrame, currFrame;

    std::vector<cv::Point2f> prevPoints, currPoints;
    std::vector<uchar> status;
    std::vector<float> error;
    std::vector<cv::Vec3f> values;

    video >> prevFrame;

    int target_width, target_height;
    target_width = target_height = 640;

    float imratio = target_width / prevFrame.size().width;
    cv::Size size = cv::Size(target_width, int(prevFrame.size().height * imratio));
    cv::Mat distorted;

    cv::resize(prevFrame, prevFrame, size, 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(prevFrame, prevPoints, 500, 0.01, 10);

    clearFile("angular_velocity.csv");
    clearFile("linear_velocity.csv");

    double cx = 320.5;
    double cy = 240.5;
    double focalLength = 277.191356;

    cv::Mat priorPose = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        focalLength, 0, cx,
        0, focalLength, cy,
        0, 0, 1);

    double maxVelocityThreshold = 10;
    double prevX = target_width / 2.0;
    double prevY = target_height / 2.0;
    double x, y;
    double dt = 0.033;
    double hfov = 1.047;

    bool pause = false;
    bool filterByStatus = true;
    bool filterByVelocity = false;

    x = y = prevX;
    std::vector<cv::Point2f> filteredPrevPts, filteredCurrPts;
    cv::Mat trajectory = cv::Mat::zeros(target_width, target_height, CV_8UC3);
    cv::Mat accVelocity = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat priorR = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat lastVec = cv::Mat::zeros(3, 1, CV_64F);

    int loop_counter = 0;
    int scaleFactor = 2;
    int scaleHeight = 100;
    int imx, imy = (int) prevX;

    auto start = std::chrono::high_resolution_clock::now();

    DCOffsetRemover xVelFilter(100); 
    DCOffsetRemover yVelFilter(100); 
    DCOffsetRemover zVelFilter(100); 

    while (video.read(currFrame))
    {
        auto runtime = std::chrono::high_resolution_clock::now();

        try {
            cv::resize(currFrame, currFrame, size, 0, 0, cv::INTER_LINEAR);
            cv::cvtColor(currFrame, currFrame, cv::COLOR_BGR2GRAY);
            cv::calcOpticalFlowPyrLK(prevFrame, currFrame, prevPoints, currPoints, status, error);

            double sumMagnitude = 0.0;
            int count = 0;
            for (size_t i = 0; i < status.size(); ++i) {
                double magnitude = cv::norm(prevPoints[i] - currPoints[i]);
                sumMagnitude += magnitude;
                count++;
            }

            double averageMagnitude = sumMagnitude / count;
            for (size_t i = 0; i < status.size(); ++i) {
                if (!status[i]) {
                    continue;
                }

                double velocity = cv::norm(prevPoints[i] - currPoints[i]);
                if (velocity > averageMagnitude + maxVelocityThreshold && filterByVelocity) {
                    continue;
                }

                filteredPrevPts.push_back(prevPoints[i]);
                filteredCurrPts.push_back(currPoints[i]);
            }

            if (filteredCurrPts.size() < 4 || filteredPrevPts.size() != filteredPrevPts.size()) {
                continue;
            }

            std::vector<cv::Mat> rvec_matrices, tvec_tvectors, nvec_normals;
            cv::Mat H = cv::findHomography(filteredPrevPts, filteredCurrPts, cv::RANSAC);

            // homography normalization
            double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) +
            H.at<double>(1,0)*H.at<double>(1,0) +
            H.at<double>(2,0)*H.at<double>(2,0));
            H /= norm;

            cv::Mat c1 = H.col(0);
            cv::Mat c2 = H.col(1);
            cv::Mat c3 = c1.cross(c2);

            cv::Mat tvec = H.col(2);
            cv::Mat R(3, 3, CV_64F);

            for (int i = 0; i < 3; i++) {
                R.at<double>(i,0) = c1.at<double>(i,0);
                R.at<double>(i,1) = c2.at<double>(i,0);
                R.at<double>(i,2) = c3.at<double>(i,0);
            }

            cv::Mat_<double> W, U, Vt;
            cv::SVDecomp(R, W, U, Vt);
            R = U*Vt;
            double det = cv::determinant(R);

            if (det < 0) {
                for (int i = 0; i < 3; i++)
                    Vt.at<double>(2, i) = Vt.at<double>(2, i) - 1;
                R = U*Vt;
            }
            
            // moving average filtering
            tvec = (tvec + lastVec) * 0.5;

            cv::Mat cam_R_img = K.inv() * R * K;
            cv::Mat cam_t_img = K.inv() * (tvec);
            cv::Mat cam_T_img = cv::Mat::eye(4, 4, CV_64F);

            cam_t_img.at<double>(0) = xVelFilter.removeOffset(cam_t_img.at<double>(0)) * -1;
            cam_t_img.at<double>(1) = yVelFilter.removeOffset(cam_t_img.at<double>(1));
            cam_t_img.at<double>(2) = zVelFilter.removeOffset(cam_t_img.at<double>(2));

            cam_R_img.copyTo(cam_T_img(cv::Rect(0, 0, 3, 3)));
            cam_t_img.copyTo(cam_T_img(cv::Rect(3, 0, 1, 3)));

            cv::Mat camPose = cam_T_img * priorPose;
            cv::Mat camRot = camPose(cv::Rect(0, 0, 3, 3)).clone();

            accVelocity = camPose.rowRange(0, 3).col(3);

            cv::Vec3f xyzAngles = rotationMatrixToEulerAngles(cam_R_img);
            cv::Vec3f xyzVelocity = accVelocity;

            saveVec3fToFile("angular_velocity.csv", xyzAngles);
            saveVec3fToFile("linear_velocity.csv", xyzVelocity);

            std::cout << camPose << std::endl;
            std::cout << xyzVelocity << std::endl;

            x = camPose.at<double>(0, 3) * scaleFactor + target_width / 2;
            y = camPose.at<double>(1, 3) * scaleFactor + target_height / 2;
            imx = (int) x;
            imy = (int) y;

            priorPose = camPose;
            priorR = R;
            lastVec = tvec;

            float px, py;
            px = -scaleHeight * tan(xyzAngles(0) - tvec.at<double>(0) * hfov / target_width);
            py =  scaleHeight * tan(xyzAngles(2) - tvec.at<double>(1) * hfov / target_height);

            // imx = (int) px + target_height / 2;
            // imy = (int) py + target_height / 2;

            std::cout << xyzAngles << std::endl;

            // std::cout << "(" << px << ", " << py << ")" << std::endl;
            // std::cout << "(" << x << ", " << y << ")" << std::endl;
        } catch (const cv::Exception& e) {
            std::cout << e.what()<< std::endl;
        }

        cv::Mat flowDisplay = currFrame.clone();
        cv::Mat coloredFlowDisplay;

        cv::cvtColor(flowDisplay, coloredFlowDisplay, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < prevPoints.size(); ++i) {
            if (status[i]) {
                cv::arrowedLine(coloredFlowDisplay, prevPoints[i], currPoints[i], cv::Scalar(0, 0, 255), 2);
                cv::arrowedLine(coloredFlowDisplay, filteredPrevPts[i], filteredCurrPts[i], cv::Scalar(30, 150, 0), 3);
            }
        }

        filteredPrevPts.clear();
        filteredCurrPts.clear();

        cv::line(trajectory, cv::Point(prevX, prevY), cv::Point(imx, imy), cv::Scalar(0, 0, 255), 2);
        prevX = imx;
        prevY = imy;

        if (!pause) {
            cv::imshow("Optical Flow", coloredFlowDisplay);
            cv::imshow("Trajectory", trajectory);
        }

        int keyboard = cv::waitKey(1);
        if (keyboard == 'q' || keyboard == 27)
            break;
        if (keyboard == 'p')
            pause = !pause;

        // Update frames and feature points
        std::swap(prevFrame, currFrame);
        std::swap(prevPoints, currPoints);
        cv::goodFeaturesToTrack(prevFrame, prevPoints, 500, 0.01, 10);

        loop_counter++;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::chrono::duration<double> step = end - runtime;

        dt = step.count();
        
        if (duration.count() >= 1.0) {
            loop_counter = 0;
            start = std::chrono::high_resolution_clock::now();
        }
    }

    cv::destroyAllWindows();
    video.release();

    return 0;
}
