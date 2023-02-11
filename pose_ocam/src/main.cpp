/**
 * @file main.cpp
 * @author interiit-11.0: team 43
 * @brief pose-estimation node that sends pose feedback to python nodes via socket communication
 * Note that if you observe frame drops below @40fps, adjust exposure with "[","]" and gain with "-","="
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "utils.hpp"
#include "withrobot_camera.hpp" /* withrobot camera API */
#include "client.hpp"

double MARKER_SIZE = 5.3;
double DIST_CEILING = 233;
const char *devPath = "/dev/video2";

socket_communication::Client client1("127.0.0.1", 7000);
socket_communication::Client client2("127.0.0.1", 8000);

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

// global variables
bool quit = false;
cv::Mat global_image;
bool ready_image = false;
bool ready_camFormat = false;
Withrobot::camera_format camFormat;

// camera matrices parameters
cv::Mat intrinsic = cv::Mat_<double>(3, 3);
cv::Mat distCoeffs = cv::Mat_<double>(1, 5);

// mutex
pthread_mutex_t mutex_img, mutex_format;
// thread
pthread_t img_thread;

/**
 * @brief Handling SIGINT call
 *
 * @param _
 */
void sig_handler(int _)
{
    (void)_;
    quit = true;
    pthread_cancel(img_thread);
    pthread_mutex_destroy(&mutex_format);
    pthread_mutex_destroy(&mutex_img);
    deb("Done");
    exit(0);
}

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
    // static tictoc timeCheck("pose");

    std::vector<double> b = {-1000, -1000, -1000, -1000, -1000, -1000};
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<int> markerIds;

    // parameters->

    // timeCheck.tic();
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    // timeCheck.toc_print();

    if (markerIds.size() != 0)
    {
        std::vector<cv::Vec3d> rvec, tvec;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, MARKER_SIZE, camMat, dist_coeff, rvec, tvec);
        int i = 0;
        for (auto &it : markerIds)
        {
            if (it == 17) // ID for 1st drone is 0
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
    if (n == 1 or n == 3)
    {
        msg = "";
        msg += std::to_string(pose[0]) + ",";
        msg += std::to_string(pose[1]) + ",";
        msg += std::to_string(pose[2]);
        client1.Send(msg);
        if (VERBOSE)
            printf("[pose1]: %s\n", msg.c_str());
    }

    if (n == 2 or n == 3)
    {
        msg = "";
        msg += std::to_string(pose[3]) + ",";
        msg += std::to_string(pose[4]) + ",";
        msg += std::to_string(pose[5]);
        client2.Send(msg);
        if (VERBOSE)
            printf("[pose2]: %s\n", msg.c_str());
    }
    
}

/**
 * @brief Function running in another thread to obtain images from oCam
 *
 * @param args
 * @return void*
 */
void *image_thread(void *args)
{
    deb("Thread started");
    Withrobot::Camera camera(devPath);

    static tictoc timeCheck("image_thread");

    camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat('G', 'R', 'E', 'Y'), 1, 60);

    pthread_mutex_lock(&mutex_format);
    {
        camera.get_current_format(camFormat);
        ready_camFormat = true;
    }
    pthread_mutex_unlock(&mutex_format);
    deb("camFormat ready");

    camFormat.print(); // print camera details

    int gain = camera.get_control("Gain");
    int exposure = camera.get_control("Exposure (Absolute)");
    // int gain, exposure;

    if (VERBOSE)
    {
        printf("Current Exposure: %d\n", exposure);
        printf("Current Gain: %d\n", gain);
        exposure = 100, gain = 133;
        camera.set_control("Gain", gain);
        camera.set_control("Exposure (Absolute)", exposure);
        printf("New Exposure: %d\n", exposure);
        printf("New Gain: %d\n", gain);
    }

    if (!camera.start())
    {
        perror("Failed to start.");
        exit(0);
    }

    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::Mat undistImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);

    // main loop that acquires new frames and copies it to global_image
    while (1)
    {
        timeCheck.tic();
        int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
        cv::undistort(srcImg, undistImg, intrinsic, distCoeffs);
        if (size == -1)
        {
            printf("error number: %d\n", errno);
            perror("Cannot get image from camera");
            camera.stop();
            camera.start();
            continue;
        }
        pthread_mutex_lock(&mutex_img);
        {
            if (!ready_image) // image is not ready already
            {
                // ready the image
                undistImg.copyTo(global_image);
                ready_image = true;
            }
        }
        pthread_mutex_unlock(&mutex_img);
        cv::imshow("img", undistImg);
        char key = cv::waitKey(1);
        switch (key)
        {
        case 'q':
            sig_handler(1);
            break;
        case '[':
            exposure = camera.get_control("Exposure (Absolute)");
            camera.set_control("Exposure (Absolute)", --exposure);
            printf("New Exposure: %d\n", exposure);
            break;
        case ']':
            exposure = camera.get_control("Exposure (Absolute)");
            camera.set_control("Exposure (Absolute)", ++exposure);
            printf("New Exposure: %d\n", exposure);
            break;
        case '-':
            exposure = camera.get_control("Gain");
            camera.set_control("Gain", --gain);
            printf("New Gain: %d\n", gain);
            break;
        case '=':
            exposure = camera.get_control("Gain");
            camera.set_control("Gain", ++gain);
            printf("New Gain: %d\n", gain);
            break;
        }

        usleep(1000);
        // timeCheck.toc_print();
    }
    camera.stop();
    return NULL;
}

/**
 *	Main
 */
int main(int argc, char **argv)
{
    // signal(SIGINT, sig_handler);
    int n = atoi(argv[1]);
    if (argc == 3)
        VERBOSE = atoi(argv[2]);
    double D[5] = {-0.411342, 0.158145, 0.000504, 0.000315, 0.000000};
    double K[9] = {963.304385, 0.000000, 637.039646, 0.000000, 965.910111, 479.796409, 0.000000, 0.000000, 1.000000};
    for (int i = 0; i < 5; i++)
        distCoeffs.at<double>(i) = D[i];
    int k = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            intrinsic.at<double>(i, j) = K[k++];

    deb("At start");
    tictoc timeCheck("main");

    // threading initialisation
    pthread_mutex_init(&mutex_img, NULL);
    pthread_mutex_init(&mutex_format, NULL);
    pthread_create(&img_thread, NULL, image_thread, &n);
    // deb("Thread created");

    bool loc_ready_camFormat = false;
    while (!loc_ready_camFormat)
    {
        pthread_mutex_lock(&mutex_format);
        {
            loc_ready_camFormat = ready_camFormat;
        }
        pthread_mutex_unlock(&mutex_format);
        // do we need to add sleep here for other thread to have time to grab it?
    }

    deb("Acquired camFormat");

    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);

    std::vector<double> poses(6);
    int skipped = 0;
    double skipped_time = 0;
    bool initial_time = true;
    // Main Loop
    while (1)
    {
        if (initial_time)
        {
            timeCheck.tic();
            initial_time = false;
        }
        pthread_mutex_lock(&mutex_img);
        {
            if (!ready_image) // if image not ready, retry
            {
                pthread_mutex_unlock(&mutex_img);
                skipped++;
                skipped_time += timeCheck.toc();
                continue;
            }
            global_image.copyTo(srcImg);
            ready_image = false;
        }
        pthread_mutex_unlock(&mutex_img);
        poses = pose_estimate(srcImg, intrinsic, distCoeffs);

        send_pose(poses, n); // forward pose
        usleep(1000);

        // timeCheck.toc_print();
        initial_time = true;
        skipped = 0;
        skipped_time = 0;
    }

    return 0;
}
