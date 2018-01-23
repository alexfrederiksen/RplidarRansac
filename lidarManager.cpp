#include <stdio.h>
#include "lidarManager.h"

#include <iostream>
#include <array>
#include <algorithm>

#include "rplidar.h"

using namespace rp::standalone::rplidar;

// Constructors
Lidar::Lidar(std::string _comPath) 
    : comPath(_comPath)
    , drv(RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT)) {
        if(!drv) {
            std::cerr << "insufficient memory, bye" << comPath << std::endl;
            throw "insufficient memory";
        }
        //connect
        if(IS_FAIL(drv->connect(comPath.c_str(), baudrate))) {
            std::cerr << "Error: Cannot bind to serial port " << comPath << std::endl;
            throw "can't bind it";
        }

        if(!checkHealth()) {
            throw "unhealth";
        }

}

Lidar::Lidar() : Lidar("/dev/ttyUSB0") {}

Lidar::~Lidar() {
    stop();
    RPlidarDriver::DisposeDriver(drv);
}

/////////////////////////////////////////////

void Lidar::start() {
    drv->startMotor();
    drv->startScan();
}

void Lidar::stop() {
    drv->stop();
    drv->stopMotor();
}

void Lidar::scan() {
    rplidar_response_measurement_node_t _nodes[360*2];
    size_t nodeSize = 360*2;

    drv->grabScanData(_nodes, nodeSize);
    std::copy(std::begin(_nodes), std::end(_nodes), std::begin(nodes));
    
}

float Lidar::getAngle(int i) {
    return (nodes[i].angle_q6_checkbit >> 1) / 64.0f;
}

float Lidar::getDist(int i) {
    return nodes[i].distance_q2/4.0f;
}

bool Lidar::checkHealth() {
    u_result result;
    rplidar_response_device_health_t healthInfo;

    result = drv->getHealth(healthInfo);
    if(IS_OK(result)) {
        std::cout << "Health: " << healthInfo.status << std::endl;
        if(healthInfo.status == RPLIDAR_STATUS_ERROR) {
            std::cerr << "Error: RPlidar unhealthy. Reboot and try again." << std::endl;
            return false;
        } else {
            return true;
        }
    } else {
        std::cerr << "Error: Cannot retrieve lidar health" << std::endl;
        return false;
    }
}


