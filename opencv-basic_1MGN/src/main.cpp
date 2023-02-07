#include <stdio.h>
#include <errno.h>
#include <sys/time.h>
#include <string>
#include <vector>
struct timeval tv;

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include "withrobot_camera.hpp" /* withrobot camera API */
#include "client.hpp"

double MARKER_SIZE = 5.3;
double DIST_CEILING = 233;

socket_communication::Client client1("127.0.0.1", 5002);
socket_communication::Client client2("127.0.0.1", 5003);

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

std::vector<double> pose_estimate(cv::Mat frame, cv::Mat &camMat, cv::Mat &dist_coeff)
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
            // cv::polylines(frame, markerCorners[i], true, (0, 255, 255));
            // cv::drawFrameAxes(frame, camMat, dist_coeff, rvec, tvec, 4.0, 4.0);
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

void send_pose(std::vector<double> pose)
{
    std::string msg = "";
    msg += std::to_string(pose[0]) + ",";
    msg += std::to_string(pose[1]) + ",";
    msg += std::to_string(pose[2]);
    client1.Send(msg);

    msg = "";
    msg += std::to_string(pose[3]) + ",";
    msg += std::to_string(pose[4]) + ",";
    msg += std::to_string(pose[5]);
    client2.Send(msg);
}

/*

 *	Main
 */
int main(int argc, char *argv[])
{
    int ct = 0;
    const char *devPath = "/dev/video0";

    Withrobot::Camera camera(devPath);

    /* USB 3.0 */
    /* 8-bit Greyscale 1280 x 720 60 fps */
    camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat('G', 'R', 'E', 'Y'), 1, 60);

    Withrobot::camera_format camFormat;
    camera.get_current_format(camFormat);

    /*
     * Print infomations
     */
    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();

    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf("----------------- Current format informations -----------------\n");
    camFormat.print();
    printf("---------------------------------------------------------------\n");

    int Gain = camera.get_control("Gain");
    int exposure = camera.get_control("Exposure (Absolute)");
    int wb_blue = 164;
    int wb_red = 152;
    camera.set_control("White Balance Blue Component", wb_blue);
    camera.set_control("White Balance Red Component", wb_red);

    printf("Current Exposure: %d\n", exposure);
    exposure = 200;
    Gain = 240;

    camera.set_control("Gain", Gain);
    camera.set_control("Exposure (Absolute)", exposure);

    if (!camera.start())
    {
        perror("Failed to start.");
        exit(0);
    }

    // std::string windowName = camName + " " + camSerialNumber;
    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1), undistImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);

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

    // cv::namedWindow(windowName.c_str(), cv::WINDOW_KEEPRATIO | cv::WINDOW_AUTOSIZE);

    // Main Loop
    bool quit = false;
    bool undistort = true;
    std::vector<double> poses;
    while (!quit)
    {
        // blocking
        int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
        cv::undistort(srcImg, undistImg, intrinsic, distCoeffs);
        if (undistort)
        {
            poses = pose_estimate(undistImg, intrinsic, distCoeffs);
        }
        else
        {
            poses = pose_estimate(srcImg, intrinsic, distCoeffs);
        }

        send_pose(poses);

        // If the error occured, restart the camera.
        if (size == -1)
        {
            printf("error number: %d\n", errno);
            perror("Cannot get image from camera");
            camera.stop();
            camera.start();
            continue;
        }

        // if (undistort)
        //     cv::imshow(windowName.c_str(), undistImg);
        // else
        //     cv::imshow(windowName.c_str(), srcImg);
        // char key = cv::waitKey(10);

        /* Keyboard options
        switch (key)
        {
        case '/':
            undistort = !undistort;
            break;
        case 'q':
            quit = true;
            break;
        case '[':
            exposure = camera.get_control("Exposure (Absolute)");
            camera.set_control("Exposure (Absolute)", --exposure);
            break;
        case ']':
            exposure = camera.get_control("Exposure (Absolute)");
            camera.set_control("Exposure (Absolute)", ++exposure);
            break;
        case '-':
            exposure = camera.get_control("Gain");
            camera.set_control("Gain", --Gain);
            break;
        case '=':
            exposure = camera.get_control("Gain");
            camera.set_control("Gain", ++Gain);
            break;
        case 'a':
            wb_blue = camera.get_control("White Balance Blue Component");
            camera.set_control("White Balance Blue Component", --wb_blue);
            break;
        case 's':
            wb_blue = camera.get_control("White Balance Blue Component");
            camera.set_control("White Balance Blue Component", ++wb_blue);
            break;
        case 'z':
            wb_red = camera.get_control("White Balance Red Component");
            camera.set_control("White Balance Red Component", --wb_red);
            break;
        case 'x':
            wb_red = camera.get_control("White Balance Red Component");
            camera.set_control("White Balance Red Component", ++wb_red);
            break;

        default:
            break;
        }
        */
    }

    // cv::destroyAllWindows();
    camera.stop();

    printf("Done.\n");

    return 0;
}
