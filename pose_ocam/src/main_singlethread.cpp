/**
 * @file main_singlethread.cpp
 * @author interiit-11.0: team 43
 * @brief used for benchmarking the frame drop wrt threaded pose_estimation
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include "withrobot_camera.hpp" /* withrobot camera API */
#include "client.hpp"

double MARKER_SIZE = 5.3;
double DIST_CEILING = 233;
const char *devPath = "/dev/video2";

socket_communication::Client client1("127.0.0.1", 5002);
socket_communication::Client client2("127.0.0.1", 5003);

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

// global variables
bool quit = false;

/**
 * @brief Estimate pose in cv::Mat using cv/aruco
 *
 * @param frame
 * @param camMat
 * @param dist_coeff
 * @return std::vector<double>
 */
std::vector<double> pose_estimate(cv::Mat &frame, cv::Mat &camMat, cv::Mat &dist_coeff)
{
    std::vector<double> b = {-1000, -1000, -1000, -1000, -1000, -1000};
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<int> markerIds;

    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    if (markerIds.size() != 0)
    {
        // printf("Detected %d markers\n", (int)markerIds.size());
        std::vector<cv::Vec3d> rvec, tvec;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, MARKER_SIZE, camMat, dist_coeff, rvec, tvec);
        int i = 0;
        for (auto &it : markerIds)
        {
            if (it == 0) // ID for 1st drone is 0
            {
                b[0] = tvec[i][0];
                b[1] = tvec[i][1];
                b[2] = DIST_CEILING - tvec[i][2];
            }
            else if (it == 10) // ID for 2nd drone is 10
            {
                b[3] = tvec[i][0];
                b[4] = tvec[i][1];
                b[5] = DIST_CEILING - tvec[i][2];
            }
            i++;
        }
    }
    return b;
}

/**
 * @brief Socket communication
 *
 * @param pose
 * @param n
 */
void send_pose(std::vector<double> pose, int n)
{
    std::string msg = "";
    msg += std::to_string(pose[0]) + ",";
    msg += std::to_string(pose[1]) + ",";
    msg += std::to_string(pose[2]);
    client1.Send(msg);

    if (n == 2)
    {
        msg = "";
        msg += std::to_string(pose[3]) + ",";
        msg += std::to_string(pose[4]) + ",";
        msg += std::to_string(pose[5]);
        client2.Send(msg);
    }
}

/**
 *	Main
 */
int main(int argc, char **argv)
{
    int n = atoi(argv[1]);
    Withrobot::camera_format camFormat;
    Withrobot::Camera camera(devPath);

    camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat('G', 'R', 'E', 'Y'), 1, 60);

    camera.get_current_format(camFormat);
    camFormat.print(); // print camera details

    int gain = camera.get_control("Gain");
    int exposure = camera.get_control("Exposure (Absolute)");

    printf("Current Exposure: %d\n", exposure);
    printf("Current Gain: %d\n", gain);
    exposure = 200, gain = 240;
    camera.set_control("Gain", gain);
    camera.set_control("Exposure (Absolute)", exposure);
    printf("New Exposure: %d\n", exposure);
    printf("New Gain: %d\n", gain);

    if (!camera.start())
    {
        perror("Failed to start.");
        exit(0);
    }

    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::Mat unhistImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);

    // camera matrices parameters
    cv::Mat intrinsic = cv::Mat_<double>(3, 3);
    cv::Mat distCoeffs = cv::Mat_<double>(1, 5);
    double D[5] = {-0.411342, 0.158145, 0.000504, 0.000315, 0.000000};
    double K[9] = {963.304385, 0.000000, 637.039646, 0.000000, 965.910111, 479.796409, 0.000000, 0.000000, 1.000000};
    for (int i = 0; i < 5; i++)
        distCoeffs.at<double>(i) = D[i];
    int k = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            intrinsic.at<double>(i, j) = K[k++];

    std::vector<double> poses(6);
    // Main Loop
    while (quit)
    {
        int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
        if (size == -1)
        {
            printf("error number: %d\n", errno);
            perror("Cannot get image from camera");
            camera.stop();
            camera.start();
            continue;
        }

        cv::undistort(srcImg, unhistImg, intrinsic, distCoeffs);
        poses = pose_estimate(unhistImg, intrinsic, distCoeffs);

        send_pose(poses, n); // forward pose
    }
    camera.stop();

    return 0;
}
