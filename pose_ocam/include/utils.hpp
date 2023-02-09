#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>
#include <string>
#include <vector>
#include <signal.h>

#include <pthread.h>
#include <boost/shared_ptr.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

bool VERBOSE = false;

#define deb(x)   \
    if (VERBOSE) \
        printf("[DBG]: %s\n", x);

class tictoc
{
public:
    tictoc(std::string prefix)
    {
        prefix_ = prefix;
    }
    void tic(void)
    {
        t1 = std::chrono::steady_clock::now();
    }

    float toc(void)
    {
        t2 = std::chrono::steady_clock::now();
        time_ms = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000.0;
        return time_ms;
    }

    void toc_print(std::string suffix = "")
    {
        t2 = std::chrono::steady_clock::now();
        time_ms = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000.0;
        if (VERBOSE)
            printf("[%s]: Time elapsed: %fms. %s\n", prefix_.c_str(), time_ms, suffix.c_str());
    }
    void toc_frequency(void)
    {
        t2 = std::chrono::steady_clock::now();
        time_ms = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000.0;
        if (VERBOSE)
            printf("[%s]: Frequency: %f\n", prefix_.c_str(), 1000.0 / time_ms);
    }

private:
    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    float time_ms;
    std::string prefix_;
};