#include <vector>
#include <string>
#include <iostream>

int main()
{
    std::vector<double> pose(6);
    pose[0] = 1.0;
    pose[1] = 2.0;
    pose[2] = 3.0;
    pose[3] = 4.0;
    pose[4] = 5.0;
    pose[5] = 6.0;
    std::string msg = std::to_string(pose[0]) + "," + std::to_string(pose[1]) + "," + std::to_string(pose[2]) + "," + std::to_string(pose[3]) + "," + std::to_string(pose[4]) + "," + std::to_string(pose[5]);
    std::cout << msg << std::endl;
}