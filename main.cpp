#include <iostream>
#include "lidarManager.h"

int main() {
    Lidar lidar;

    std::cout << "Press enter to start motor" << std::endl;
    getchar();
    lidar.start();

    lidar.scan();
    for(int i = 0; i < 100; i++) {
        std::cout << "Index: " << i << "  Angle: " << lidar.getAngle(i) << "  Dist: " << lidar.getDist(i) << std::endl;
    }

    std::cout << "Motor started." << std::endl;
    std::cout << "Press enter to stop motor" << std::endl;
    getchar();
    lidar.stop();

    std::cout << "Press enter to end program." << std::endl;
    getchar();
    return 0;
}
